/*
 
 Copyright (c) 2014 RedBearLab
 
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

#import "RBLViewController.h"
#import <CoreBluetooth/CoreBluetooth.h>
#import "RBLService.h"

#define DEFAULT_UUID @"E2C56DB5-DFFB-48D2-B060-D0F5A71096E0"
#define DEFAULT_MAJOR 0
#define DEFAULT_MINOR 0
#define DEFAULT_MEASURED -55
#define DEFAULT_LED 1
#define DEFAULT_INTERVAL 250
#define DEFAULT_TX 2

@interface RBLViewController () <CBCentralManagerDelegate, CBPeripheralDelegate>
{
    UInt16 advertisingInterval;
    bool isFailed;
    int characteristicCount;
    
    CBCharacteristic *iBeaconUUIDCharacteristic;
    CBCharacteristic *majorCharacteristic;
    CBCharacteristic *minorCharacteristic;
    CBCharacteristic *measuredPowerCharacteristic;
    CBCharacteristic *ledCharacteristic;
    CBCharacteristic *intervalCharacteristic;
    CBCharacteristic *txPowerCharacteristic;
}

@property (strong, nonatomic) CBCentralManager      *centralManager;
@property (strong, nonatomic) CBPeripheral          *discoveredPeripheral;

@property (weak, nonatomic) IBOutlet UIButton *defaultButton;
- (IBAction)defaultClick:(id)sender;

@property (weak, nonatomic) IBOutlet UIButton *saveButton;
- (IBAction)saveClick:(id)sender;

@property (weak, nonatomic) IBOutlet UIView *deviceView;

@property (weak, nonatomic) IBOutlet UILabel *deviceUUID;
@property (weak, nonatomic) IBOutlet UILabel *deviceiBeaconUUID;
@property (weak, nonatomic) IBOutlet UILabel *deviceMajor;
@property (weak, nonatomic) IBOutlet UILabel *deviceMinor;
@property (weak, nonatomic) IBOutlet UILabel *deviceMeasuredPower;
@property (weak, nonatomic) IBOutlet UILabel *deviceLED;
@property (weak, nonatomic) IBOutlet UILabel *deviceInterval;
@property (weak, nonatomic) IBOutlet UILabel *deviceTXPower;

@property (weak, nonatomic) IBOutlet UITextField *UUIDText;
@property (weak, nonatomic) IBOutlet UITextField *majorText;
@property (weak, nonatomic) IBOutlet UITextField *minorText;
@property (weak, nonatomic) IBOutlet UITextField *measuredPowerText;
@property (weak, nonatomic) IBOutlet UISwitch *LEDSwitch;
@property (weak, nonatomic) IBOutlet UISegmentedControl *TXSegment;
@property (weak, nonatomic) IBOutlet UILabel *intervalLabel;
@property (weak, nonatomic) IBOutlet UISlider *intervalSlider;
- (IBAction)intervalChanged:(id)sender;
@property (weak, nonatomic) IBOutlet UIStepper *intervalStepper;
- (IBAction)intervalStepPressed:(id)sender;

@end

@implementation RBLViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
	// Do any additional setup after loading the view, typically from a nib.

   [self showDeviceDetails:false];
    _centralManager = [[CBCentralManager alloc] initWithDelegate:self queue:nil];
}

- (void)viewWillDisappear:(BOOL)animated
{
    // Don't keep it going while we're not showing.
    [self.centralManager stopScan];
    NSLog(@"Scanning stopped");
    
    [super viewWillDisappear:animated];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)showDeviceDetails:(bool) show
{
    self.deviceView.hidden = !show;
    self.defaultButton.hidden = !show;
    self.saveButton.hidden = !show;
}

#pragma mark - Central Methods

- (void)centralManagerDidUpdateState:(CBCentralManager *)central
{
    if (central.state != CBCentralManagerStatePoweredOn) {
        return;
    }
    
    // The state must be CBCentralManagerStatePoweredOn...
    // ... so start scanning
    [self scan];
    
}

- (void)scan
{
    [self.centralManager scanForPeripheralsWithServices:[NSArray arrayWithObject:[CBUUID UUIDWithString:RBL_IBSERVICE_UUID]] options:nil];

    NSLog(@"Scanning started");
}

- (void)centralManager:(CBCentralManager *)central didDiscoverPeripheral:(CBPeripheral *)peripheral advertisementData:(NSDictionary *)advertisementData RSSI:(NSNumber *)RSSI
{
    
    NSLog(@"Discovered %@ at %@", peripheral.name, RSSI);
    
    if (self.discoveredPeripheral != peripheral) {
        
        // Save a local copy of the peripheral, so CoreBluetooth doesn't get rid of it
        self.discoveredPeripheral = peripheral;
        
        // And connect
        NSLog(@"Connecting to peripheral %@", peripheral.UUID);
        [self.centralManager connectPeripheral:peripheral options:nil];
    }
}


- (void)centralManager:(CBCentralManager *)central didFailToConnectPeripheral:(CBPeripheral *)peripheral error:(NSError *)error
{
    NSLog(@"Failed to connect to %@. (%@)", peripheral, [error localizedDescription]);
    [self cleanup];
}


- (void)cleanup
{
    // Don't do anything if we're not connected
    if (!self.discoveredPeripheral.state == CBPeripheralStateConnected) {
        return;
    }
    
    // If we've got this far, we're connected, but we're not subscribed, so we just disconnect
    [self.centralManager cancelPeripheralConnection:self.discoveredPeripheral];
}


- (void)centralManager:(CBCentralManager *)central didConnectPeripheral:(CBPeripheral *)peripheral
{
    NSLog(@"Peripheral Connected");
    
    // Stop scanning
    [self.centralManager stopScan];
    NSLog(@"Scanning stopped");
    
    // Make sure we get the discovery callbacks
    peripheral.delegate = self;
    
    [peripheral discoverServices:nil];
}


- (void)centralManager:(CBCentralManager *)central didDisconnectPeripheral:(CBPeripheral *)peripheral error:(NSError *)error
{
    NSLog(@"Peripheral Disconnected");
    self.discoveredPeripheral = nil;
    
    [self showDeviceDetails:false];
    
    // We're disconnected, so start scanning again
    [self scan];
}

- (void)peripheral:(CBPeripheral *)peripheral didDiscoverServices:(NSError *)error
{
    if (error) {
        NSLog(@"Error discovering services: %@", [error localizedDescription]);
        [self cleanup];
        return;
    }
    
    for (CBService *service in peripheral.services) {
        if ([service.UUID isEqual:[CBUUID UUIDWithString:RBL_IBSERVICE_UUID]]) {
            characteristicCount = 0;
            [peripheral discoverCharacteristics:nil forService:service];
        }
    }
}

- (void)peripheral:(CBPeripheral *)peripheral didDiscoverCharacteristicsForService:(CBService *)service error:(NSError *)error
{
    // Deal with errors (if any)
    if (error) {
        NSLog(@"Error discovering characteristics: %@", [error localizedDescription]);
        [self cleanup];
        return;
    }
    
    [self.deviceUUID setText:[NSString stringWithFormat:@"%@", [[peripheral identifier] UUIDString]]];
    for (CBCharacteristic *characteristic in service.characteristics) {
        
        [peripheral readValueForCharacteristic:characteristic];
        
    }
    
}

- (void)peripheral:(CBPeripheral *)peripheral didUpdateValueForCharacteristic:(CBCharacteristic *)characteristic error:(NSError *)error
{
    if (error) {
        NSLog(@"Error discovering characteristics: %@", [error localizedDescription]);
        return;
    }
    
    if ([characteristic.UUID isEqual:[CBUUID UUIDWithString:RBL_CHARACTERISTIC_IBEACON_UUID]]) {
        
        iBeaconUUIDCharacteristic = characteristic;

        NSString *temp = [self getHexString:characteristic.value];
        
        NSRange r1 = NSMakeRange(8, 4);
        NSRange r2 = NSMakeRange(12, 4);
        NSRange r3 = NSMakeRange(16, 4);
        
        NSString *ibeacon;
        
        ibeacon = [temp substringToIndex:8];
        ibeacon = [ibeacon stringByAppendingString:@"-"];
        ibeacon = [ibeacon stringByAppendingString:[temp substringWithRange:r1]];
        ibeacon = [ibeacon stringByAppendingString:@"-"];
        ibeacon = [ibeacon stringByAppendingString:[temp substringWithRange:r2]];
        ibeacon = [ibeacon stringByAppendingString:@"-"];
        ibeacon = [ibeacon stringByAppendingString:[temp substringWithRange:r3]];
        ibeacon = [ibeacon stringByAppendingString:@"-"];
        ibeacon = [ibeacon stringByAppendingString:[temp substringFromIndex:20]];
        ibeacon = [ibeacon uppercaseString];
        
        [self.deviceiBeaconUUID setText:[NSString stringWithFormat:@"%@", ibeacon]];
        [self.UUIDText setText:ibeacon];
        
        characteristicCount++;
    }
    else if ([characteristic.UUID isEqual:[CBUUID UUIDWithString:RBL_CHARACTERISTIC_MAJOR_UUID]]) {
        
        majorCharacteristic = characteristic;
        
        unsigned char data[2];
        [characteristic.value getBytes:data length:2];
        
        UInt16 major = data[0] << 8 | data[1];
        
        
        [self.deviceMajor setText:[NSString stringWithFormat:@"Major: %d", major]];
        [self.majorText setText:[NSString stringWithFormat:@"%d", major]];
        
        characteristicCount++;
    }
    else if ([characteristic.UUID isEqual:[CBUUID UUIDWithString:RBL_CHARACTERISTIC_MINOR_UUID]]) {
        
        minorCharacteristic = characteristic;
        
        unsigned char data[2];
        [characteristic.value getBytes:data length:2];
        
        UInt16 minor = data[0] << 8 | data[1];
        
        [self.deviceMinor setText:[NSString stringWithFormat:@"Minor: %d", minor]];
        [self.minorText setText:[NSString stringWithFormat:@"%d", minor]];
        
        characteristicCount++;
    }
    else if ([characteristic.UUID isEqual:[CBUUID UUIDWithString:RBL_CHARACTERISTIC_MEASURED_UUID]]) {
        
        measuredPowerCharacteristic = characteristic;
        
        unsigned char data[1];
        [characteristic.value getBytes:data length:1];
        
        int measured = data[0];
        
        if (measured == 0)
        {
            [self.deviceMeasuredPower setText:[NSString stringWithFormat:@"Measured Power: 0"]];
            [self.measuredPowerText setText:[NSString stringWithFormat:@"0"]];
        }
        else
        {
            [self.deviceMeasuredPower setText:[NSString stringWithFormat:@"Measured Power: %d", measured - 256]];
            [self.measuredPowerText setText:[NSString stringWithFormat:@"%d", measured - 256]];
        }
        
        characteristicCount++;
    }
    else if ([characteristic.UUID isEqual:[CBUUID UUIDWithString:RBL_CHARACTERISTIC_LED_UUID]]) {
        
        ledCharacteristic = characteristic;
        
        unsigned char data[1];
        [characteristic.value getBytes:data length:1];
        
        int led = data[0];
        
        //[self.deviceLED set]];
        
        if (led == 1)
        {
            [self.deviceLED setText:[NSString stringWithFormat:@"Status LED: ON"]];
            [self.LEDSwitch setOn:YES];
        }
        else
        {
            [self.deviceLED setText:[NSString stringWithFormat:@"Status LED: OFF"]];
            [self.LEDSwitch setOn:NO];
        }
        
        characteristicCount++;
    }
    else if ([characteristic.UUID isEqual:[CBUUID UUIDWithString:RBL_CHARACTERISTIC_INTERVAL_UUID]]) {
        
        intervalCharacteristic = characteristic;
        
        unsigned char data[2];
        [characteristic.value getBytes:data length:2];
        
        advertisingInterval = data[0] << 8 | data[1];
        
        self.intervalSlider.value = advertisingInterval/50;
        self.intervalStepper.value = advertisingInterval/5;
        [self.intervalLabel setText:[NSString stringWithFormat:@" %dms", (unsigned int)advertisingInterval]];
        [self.deviceInterval setText:[NSString stringWithFormat:@"Advertising Interval: %dms", (unsigned int)advertisingInterval]];
        
        characteristicCount++;
    }
    else if ([characteristic.UUID isEqual:[CBUUID UUIDWithString:RBL_CHARACTERISTIC_TX_UUID]]) {
    
        txPowerCharacteristic = characteristic;
        
        unsigned char data[1];
        [characteristic.value getBytes:data length:1];
        
        int tx = data[0];
        
        if (tx > 3)
        {
            tx = DEFAULT_TX;
        }
        [self.TXSegment setSelectedSegmentIndex:tx];
        NSString *txPower = @"TX Power: ";
        switch (tx)
        {
            case 0:
                txPower = [txPower stringByAppendingString:@"-23dbm"];
                break;
            case 1:
                txPower = [txPower stringByAppendingString:@"-6dbm"];
                break;
            case 2:
                txPower = [txPower stringByAppendingString:@"0dbm"];
                break;
            case 3:
                txPower = [txPower stringByAppendingString:@"+4dbm"];
                break;
        }
        [self.deviceTXPower setText:txPower];
        
        characteristicCount++;
    }

    if (characteristicCount == 7)
    {
        [self showDeviceDetails:true];
    }
}


- (void)peripheral:(CBPeripheral *)peripheral didWriteValueForCharacteristic:(CBCharacteristic *)characteristic error:(NSError *)error
{
    if (isFailed)
        return;
    
    if (error) {
        isFailed = true;
        NSLog(@"Error writing characteristics: %@", [error localizedDescription]);
        UIAlertView *alertView = [[UIAlertView alloc]initWithTitle:nil message:[NSString stringWithFormat:@"There is an error when saving, please try again."] delegate:self cancelButtonTitle:@"OK" otherButtonTitles:nil];
        [alertView show];
        return;
    }
    
    characteristicCount ++;

    if (characteristicCount == 7)
    {
        [_centralManager cancelPeripheralConnection:self.discoveredPeripheral];
        
        UIAlertView *alertView = [[UIAlertView alloc]initWithTitle:nil message:[NSString stringWithFormat:@"Update successful!"] delegate:self cancelButtonTitle:@"OK" otherButtonTitles:nil];
        [alertView show];
    }
    
}


-(NSString*)convertCBUUIDToString:(CBUUID*)uuid {
    NSData *data = uuid.data;
    NSUInteger bytesToConvert = [data length];
    const unsigned char *uuidBytes = [data bytes];
    NSMutableString *outputString = [NSMutableString stringWithCapacity:16];
    
    for (NSUInteger currentByteIndex = 0; currentByteIndex < bytesToConvert; currentByteIndex++)
    {
        switch (currentByteIndex)
        {
            case 3:
            case 5:
            case 7:
            case 9:[outputString appendFormat:@"%02x-", uuidBytes[currentByteIndex]]; break;
            default:[outputString appendFormat:@"%02x", uuidBytes[currentByteIndex]];
        }
        
    }
    
    NSString *result = [outputString uppercaseString];
    
    return result;
}

-(NSString*)getHexString:(NSData*)data {
    NSUInteger dataLength = [data length];
    NSMutableString *string = [NSMutableString stringWithCapacity:dataLength*2];
    const unsigned char *dataBytes = [data bytes];
    for (NSInteger idx = 0; idx < dataLength; ++idx) {
        [string appendFormat:@"%02x", dataBytes[idx]];
    }
    return string;
}


- (IBAction)intervalChanged:(id)sender {
    int ms = [self.intervalSlider value];
    advertisingInterval = ms * 50;
    self.intervalSlider.value = ms;
    self.intervalStepper.value = ms * 10;
    [self.intervalLabel setText:[NSString stringWithFormat:@" %dms", (unsigned int)advertisingInterval]];
}

- (IBAction)intervalStepPressed:(id)sender {
    int ms = [self.intervalStepper value];
    advertisingInterval = ms * 5;
    self.intervalSlider.value = ms / 10;
    [self.intervalLabel setText:[NSString stringWithFormat:@" %dms", (unsigned int)advertisingInterval]];
}


- (IBAction)defaultClick:(id)sender {
    
    [self.deviceView endEditing:YES];
    [self.UUIDText setText:DEFAULT_UUID];
    [self.majorText setText:[NSString stringWithFormat:@"%d", DEFAULT_MAJOR]];
    [self.minorText setText:[NSString stringWithFormat:@"%d", DEFAULT_MINOR]];
    [self.measuredPowerText setText:[NSString stringWithFormat:@"%d", DEFAULT_MEASURED]];
    [self.LEDSwitch setOn:DEFAULT_LED];
    advertisingInterval = DEFAULT_INTERVAL;
    self.intervalSlider.value = advertisingInterval/50;
    self.intervalStepper.value = advertisingInterval/5;
    [self.intervalLabel setText:[NSString stringWithFormat:@" %dms", (unsigned int)advertisingInterval]];
    [self.TXSegment setSelectedSegmentIndex:DEFAULT_TX];
}

- (IBAction)saveClick:(id)sender {
    
    [self.view endEditing:YES];
    
    CBUUID *uuid;
    
    @try {
        uuid = [CBUUID UUIDWithString:[self.UUIDText text]];
        [self.UUIDText setText:[self convertCBUUIDToString:uuid]];
        
    }
    @catch (NSException *exception) {
        UIAlertView *alertView = [[UIAlertView alloc]initWithTitle:nil message:[NSString stringWithFormat:@"UUID string not valid!"] delegate:self cancelButtonTitle:@"OK" otherButtonTitles:nil];
        [alertView show];
        return;
    }
    
    int major = [[self.majorText text] intValue];
    if ((major < 0) || (major > 65535))
    {
        UIAlertView *alertView = [[UIAlertView alloc]initWithTitle:nil message:[NSString stringWithFormat:@"Major number not valid!"] delegate:self cancelButtonTitle:@"OK" otherButtonTitles:nil];
        [alertView show];
        return;
    }
    [self.majorText setText:[NSString stringWithFormat:@"%d", major]];
    
    int minor = [[self.minorText text] intValue];
    if ((minor < 0) || (minor > 65535))
    {
        UIAlertView *alertView = [[UIAlertView alloc]initWithTitle:nil message:[NSString stringWithFormat:@"Minor number not valid!"] delegate:self cancelButtonTitle:@"OK" otherButtonTitles:nil];
        [alertView show];
        return;
    }
    [self.minorText setText:[NSString stringWithFormat:@"%d", minor]];

    int measured = [[self.measuredPowerText text] intValue];
    if ((measured > 0) || (measured < -100))
    {
        UIAlertView *alertView = [[UIAlertView alloc]initWithTitle:nil message:[NSString stringWithFormat:@"Measured Power not valid!"] delegate:self cancelButtonTitle:@"OK" otherButtonTitles:nil];
        [alertView show];
        return;
    }
    [self.measuredPowerText setText:[NSString stringWithFormat:@"%d", measured]];
    
    characteristicCount = 0;
    isFailed = false;
    
    NSData *data = uuid.data;
    [self.discoveredPeripheral writeValue:data forCharacteristic:iBeaconUUIDCharacteristic type:CBCharacteristicWriteWithResponse];
    
    uint8_t buf[] = {0x00 , 0x00};
    buf[1] =  (unsigned int) (major & 0xff);
    buf[0] =  (unsigned int) (major>>8 & 0xff);
    data = [[NSData alloc] initWithBytes:buf length:2];
    [self.discoveredPeripheral writeValue:data forCharacteristic:majorCharacteristic type:CBCharacteristicWriteWithResponse];
    
    buf[1] =  (unsigned int) (minor & 0xff);
    buf[0] =  (unsigned int) (minor>>8 & 0xff);
    data = [[NSData alloc] initWithBytes:buf length:2];
    [self.discoveredPeripheral writeValue:data forCharacteristic:minorCharacteristic type:CBCharacteristicWriteWithResponse];
    
    if (measured > 0)
        measured = measured + 256;
    buf[0] = measured;
    data = [[NSData alloc] initWithBytes:buf length:1];
    [self.discoveredPeripheral writeValue:data forCharacteristic:measuredPowerCharacteristic type:CBCharacteristicWriteWithResponse];
    
    buf[0] = self.LEDSwitch.on;
    data = [[NSData alloc] initWithBytes:buf length:1];
    [self.discoveredPeripheral writeValue:data forCharacteristic:ledCharacteristic type:CBCharacteristicWriteWithResponse];
    
    buf[1] =  (unsigned int) (advertisingInterval & 0xff);
    buf[0] =  (unsigned int) (advertisingInterval>>8 & 0xff);
    data = [[NSData alloc] initWithBytes:buf length:2];
    [self.discoveredPeripheral writeValue:data forCharacteristic:intervalCharacteristic type:CBCharacteristicWriteWithResponse];
    
    buf[0] = self.TXSegment.selectedSegmentIndex;
    data = [[NSData alloc] initWithBytes:buf length:1];
    [self.discoveredPeripheral writeValue:data forCharacteristic:txPowerCharacteristic type:CBCharacteristicWriteWithResponse];
    
}
@end
