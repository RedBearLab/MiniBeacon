/*

  Copyright (c) 2013 RedBearLab

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal 
  in the Software without restriction, including without limitation the rights 
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"

#include "peripheral.h"
#include "gapbondmgr.h"

#include "MiniBeacon.h"
#include "iBeaconService.h"

#include "i2c.h"
#include "eeprom.h"
#include "button.h"
#include "hal_flash.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   5000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
/*static*/ uint8 MiniBeacon_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;
static uint8 ledState = 1;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  11,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'M', 'i', 'n', 'i', 'B', 'e', 'a', 'c', 'o', 'n',
    
  // service UUID, to notify central devices what services are included
  // in this peripheral
  17,   // length of this data
  GAP_ADTYPE_128BIT_MORE,
  UUID_BASE_TAIL,
  LO_UINT16( IBEACON_SERV_UUID ),
  HI_UINT16( IBEACON_SERV_UUID ),
  UUID_BASE_HEAD,
};

static uint8 advertData[] =
{
  0x02, // size
  0x01,0x1A,
  
  0x1A, // size
   0xFF,0x4C,0x00,0x02, // 0xFF == GAP_ADTYPE_MANUFACTURER_SPECIFIC
   0x15, // size
    IBEACON_UUID, // UUID
    0x18,0x88,0x01,0x01, // Major, Minor
    0xC6,
};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "MiniBeacon";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void MiniBeacon_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void iBeaconChangeCB( uint8 paramID );

#if defined( CC2540_MINIDK )
static void MiniBeacon_HandleKeys( uint8 shift, uint8 keys );
#endif  

#pragma location="CHAR_VALUE"
const uint8 charValue[30] = 
{ 
  'I','B','0','1',
  0x01,  // Flag
  0xe2, 0xc5, 0x6d, 0xb5, 0xdf, 0xfb, 0x48, 0xd2, 0xb0, 0x60, 0xd0, 0xf5, 0xa7, 0x10, 0x96, 0xe0,//UUID
  0x00, 0x00,  //Major 
  0x00, 0x00,  //Minor   
  0xC9,  //RSSI
  0x01,  //LED state, default: on
  0x00,250, //Advertising_interval, default: 250ms
  HCI_EXT_TX_POWER_0_DBM,  //TX Power, default: 0dbm
};
#pragma required=charValue     
  
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t MiniBeacon_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t MiniBeacon_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// iBeacon Callbacks
static iBeaconCBs_t MiniBeacon_iBeaconCBs =
{
  iBeaconChangeCB    // Charactersitic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      MiniBeacon_Init
 *
 * @brief   Initialization function for the MiniBeacon App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void MiniBeacon_Init( uint8 task_id )
{
  MiniBeacon_TaskID = task_id;

  // Setup the GAP Peripheral Role Profile
  {
    // For other hardware platforms, device starts advertising upon initialization
    uint8 initial_advertising_enable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;
    
    uint8 new_advertising_event = GAP_ADTYPE_ADV_NONCONN_IND;
    
    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &new_advertising_event );
        
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    
    i2c_init(); 
    
    uint16 page = 112;
    uint8 buf[30];
    HalFlashRead(112, 0, buf, 30);
    if(0x01 == buf[4])
    {
      for(int i=0; i<25; i++)
      {
        eeprom_write(i, buf[i+5]);
      }

      uint8 writebuf[4];  
  
      writebuf[0] = 0x00;
      writebuf[1] = buf[5];  
      writebuf[2] = buf[6];  
      writebuf[3] = buf[7];      
      uint16 addr = page*(HAL_FLASH_PAGE_SIZE/HAL_FLASH_WORD_SIZE) + 1;
      HalFlashWrite(addr, writebuf, 1);
    }
    for (int i = 0; i < 21; i++)
    {
      advertData[9+i] = eeprom_read(i);
    }
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );        
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );  

  // Set advertising interval
  {
    uint8 adv_buf[2];
    uint16 advInt;
    adv_buf[0] = eeprom_read(22);
    adv_buf[1] = eeprom_read(23);
    advInt = adv_buf[0] * 256 + adv_buf[1];
    advInt = (advInt / 5) * 5;
    advInt = (uint16)(advInt / 0.625);
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }
  
  // Set TX Power
  {
    uint8 txpwr;
    txpwr = eeprom_read(24);
    HCI_EXT_SetTxPowerCmd(txpwr);
  } 

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  {
    GGS_AddService( GATT_ALL_SERVICES );            // GAP
    GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes                          
    iBeacon_AddService( GATT_ALL_SERVICES );
  }

  // Setup the iBeacon Characteristic Values
  {
    uint8 charValue1[16] = {IBEACON_UUID};
    uint8 charValue2[2] = {00, 00};
    uint8 charValue3[2] = {00, 00};
    uint8 charValue4[1] = {0xC5};
    uint8 charValue5[1] = {0x01};
    uint8 charValue6[2] = {0x02, 0x80};
    uint8 charValue7[1] = {0x02};
    
    for (int i = 0; i < 16; i++)
    {
      charValue1[i] = advertData[9+i];
    }
    charValue2[0] = advertData[25];
    charValue2[1] = advertData[26];
    charValue3[0] = advertData[27];
    charValue3[1] = advertData[28];
    charValue4[0] = advertData[29];
    charValue5[0] = eeprom_read(21);
    charValue6[0] = eeprom_read(22);
    charValue6[1] = eeprom_read(23);
    charValue7[0] = eeprom_read(24);

    iBeacon_SetParameter( IBEACON_CHAR1, IBEACON_CHAR1_LEN, charValue1 );
    iBeacon_SetParameter( IBEACON_CHAR2, IBEACON_CHAR2_LEN, charValue2 );
    iBeacon_SetParameter( IBEACON_CHAR3, IBEACON_CHAR3_LEN, charValue3 );
    iBeacon_SetParameter( IBEACON_CHAR4, IBEACON_CHAR4_LEN, charValue4 );
    iBeacon_SetParameter( IBEACON_CHAR5, IBEACON_CHAR5_LEN, charValue5 );
    iBeacon_SetParameter( IBEACON_CHAR6, IBEACON_CHAR6_LEN, charValue6 );
    iBeacon_SetParameter( IBEACON_CHAR7, IBEACON_CHAR7_LEN, charValue7 );
    
    uint8 version[] = "MiniBeacon_20140121 ";
    iBeacon_SetParameter( IBEACON_CHAR8, IBEACON_CHAR8_LEN, version );
  }

  P0SEL = 0x00;  // Configure Port 0 as GPIO
  P1SEL = 0x30; // Configure Port 1 as GPIO
  P2SEL = 0x00; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons), all others (P0.2-P0.7) as output
  P1DIR = 0xCB; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output

  P0 = 0x81; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0x07;   // All pins on port 1 to low
  P2 = 0x00;   // All pins on port 2 to low

  i2c_init(); 
  
  // Register callback with iBeacon service
  VOID iBeacon_RegisterAppCBs( &MiniBeacon_iBeaconCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );
  
  ButtonInit();

  // Set LED State
  {
    P1SEL &= 0xFD; //P1.1 is GPIO
    P1DIR |= 0x02; //P1.1 is Output
    uint8 state;
    state = eeprom_read(21);
    if(state == 0x01)
    {
      P1_1 = 1;
      ledState = 1;
    }
    else
    {     
      P1_1 = 0;
      ledState = 0;
    }
  }
  
  // Setup a delayed profile startup
  osal_start_timerEx( MiniBeacon_TaskID, SBP_START_DEVICE_EVT,500 );
}

/*********************************************************************
 * @fn      MiniBeacon_ProcessEvent
 *
 * @brief   MiniBeacon Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 MiniBeacon_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( MiniBeacon_TaskID )) != NULL )
    {
      MiniBeacon_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &MiniBeacon_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &MiniBeacon_BondMgrCBs );

    // Set timer for first periodic event
    osal_start_timerEx( MiniBeacon_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

    return ( events ^ SBP_START_DEVICE_EVT );
  }

  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
    {
      osal_start_timerEx( MiniBeacon_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }

    // Perform periodic application task
    performPeriodicTask();

    return (events ^ SBP_PERIODIC_EVT);
  }
  
  if ( events & SBP_BUTTON_DOWN_EVT )
  {
    // There is indeed a button-down action  
    if(0 == P1_2)
    {     
      uint8 adv_enable = FALSE;
      uint8 new_advertising_event = GAP_ADTYPE_ADV_NONCONN_IND;
      
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &adv_enable );
      GAPRole_GetParameter( GAPROLE_ADV_EVENT_TYPE, &new_advertising_event );
      if(GAP_ADTYPE_ADV_NONCONN_IND == new_advertising_event)
      {
        new_advertising_event = GAP_ADTYPE_ADV_IND ;
        P0_7 = 0;
        if(ledState == 1)
        {
          P1_1 = 0;
        }
        uint8 advData[] =
        {
          0x02, // size
          0x01,0x1A,
        };
        GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advData ), advData ); 
        
        uint16 advInt = 338;//211.25ms
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
      }
      else if(GAP_ADTYPE_ADV_IND == new_advertising_event)
      {
        new_advertising_event = GAP_ADTYPE_ADV_NONCONN_IND ;
        P0_7 = 1;
        if(ledState == 1)
        {
          P1_1 = 1;
        }
        GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData ); 
        
        uint8 adv_buf[2];
        uint16 advInt;
        adv_buf[0] = eeprom_read(22);
        adv_buf[1] = eeprom_read(23);
        advInt = adv_buf[0] * 256 + adv_buf[1];
        advInt = (advInt / 5) * 5;
        advInt = (uint16)(advInt / 0.625);
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
      }
      GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &new_advertising_event );
      
      if (gapProfileState == GAPROLE_CONNECTED)
      {
        GAPRole_TerminateConnection();
      }
      
      osal_start_timerEx( MiniBeacon_TaskID, SBP_RE_ADVERT_EVT, 500 );
    }
    
    return (events ^ SBP_BUTTON_DOWN_EVT);
  }
  
  if ( events & SBP_RE_ADVERT_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

    return (events ^ SBP_RE_ADVERT_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      MiniBeacon_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void MiniBeacon_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      MiniBeacon_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  #endif // #if defined( CC2540_MINIDK )

  default:
    // do nothing
    break;
  }
}

#if defined( CC2540_MINIDK )
/*********************************************************************
 * @fn      MiniBeacon_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void MiniBeacon_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;

  VOID shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_SW_1 )
  {
    SK_Keys |= SK_KEY_LEFT;
  }

  if ( keys & HAL_KEY_SW_2 )
  {

    SK_Keys |= SK_KEY_RIGHT;

    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
    if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }

      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
    }
  }
}
#endif // #if defined( CC2540_MINIDK )

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  static bool flag = 0;
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {

      }
      break;

    case GAPROLE_ADVERTISING:
      {        
        if(1 == flag)
        {
          uint8 adv_enable = FALSE;
          uint8 new_advertising_event = GAP_ADTYPE_ADV_NONCONN_IND;
          
          flag = 0;
          GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &adv_enable );
          GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &new_advertising_event );
          P0_7 = 1;
          if(ledState == 1)
          {
            P1_1 = 1;
          }
          GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
          
          uint8 adv_buf[2];
          uint16 advInt;
          adv_buf[0] = eeprom_read(22);
          adv_buf[1] = eeprom_read(23);
          advInt = adv_buf[0] * 256 + adv_buf[1];
          advInt = (advInt / 5) * 5;
          advInt = (uint16)(advInt / 0.625);
          GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
          GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
          GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
          GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
          
          osal_start_timerEx( MiniBeacon_TaskID, SBP_RE_ADVERT_EVT, 500 );
        }
      }
      break;

    case GAPROLE_CONNECTED:
      {        
        flag = 1;
        
        uint8 adv_enable = FALSE;
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &adv_enable );
      }
      break;

    case GAPROLE_WAITING:
      {         
        if (gapProfileState == GAPROLE_CONNECTED)
        {
          uint8 adv_enable = TRUE;
          GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &adv_enable );
        }
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {         
          uint8 adv_enable = TRUE;
          GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &adv_enable );
      }
      break;

    case GAPROLE_ERROR:
      {
        
      }
      break;

    default:
      {

      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif
}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. 
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask( void )
{
  
}

/*********************************************************************
 * @fn      iBeaconChangeCB
 *
 * @brief   Callback from iBeacon service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void iBeaconChangeCB( uint8 paramID )
{ 
  switch( paramID )
  {
    case IBEACON_CHAR1:
      {        

      }
      break;

    case IBEACON_CHAR2:
      {

      }
      break;

   case IBEACON_CHAR3:
     {

     }
     break;

    case IBEACON_CHAR4:
      {

      }
      break;
      
    case IBEACON_CHAR5:
      {
        uint8 newValue;
        iBeacon_GetParameter( IBEACON_CHAR5, &newValue );
        if(0x01 == newValue)
        {
          ledState = 1;
        }
        else
        {
          ledState = 0;
        }
      }
      break;
      
    case IBEACON_CHAR6:
      {
        uint8 newValue[2];
        uint16 advInt;
        iBeacon_GetParameter( IBEACON_CHAR6, newValue );
        advInt = newValue[0] * 256 + newValue[1];
        advInt = (advInt / 5) * 5;
        if((advInt >= 100) && (advInt <= 10000))   //20ms -- 1000ms
        {
          advInt = (uint16)(advInt / 0.625);
        }
        else if(advInt < 100)   
        {
          advInt = 160;
        }
        else if(advInt > 10000)
        {
          advInt = 16000;
        }
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
      }
      break;
      
    case IBEACON_CHAR7:
      {
        uint8 newValue;
        iBeacon_GetParameter( IBEACON_CHAR7, &newValue );
        if(newValue < 4)
        {
          HCI_EXT_SetTxPowerCmd( newValue );
        }  
      }
      break;
      
    default:
      // should not reach here!
      break;
  }

  uint8 data[25];
  iBeacon_GetParameter( IBEACON_CHAR1, data );
  iBeacon_GetParameter( IBEACON_CHAR2, &data[16] );
  iBeacon_GetParameter( IBEACON_CHAR3, &data[18] );
  iBeacon_GetParameter( IBEACON_CHAR4, &data[20] );
  iBeacon_GetParameter( IBEACON_CHAR5, &data[21] );
  iBeacon_GetParameter( IBEACON_CHAR6, &data[22] );
  iBeacon_GetParameter( IBEACON_CHAR7, &data[24] );
  
  eeprom_write_25(data);

  for (int i = 0; i < 21; i++)
    advertData[9+i] = data[i];
  GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

}

/*********************************************************************
*********************************************************************/