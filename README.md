MiniBeacon
==========

MiniBeacon is an iBeacon (designed by Apple) implementation with CC2540 SoC.


Requirement
===========

This firmware is designed for and tested on RedBearLab's BLE Mini development boards, but it should work on other CC2540 platforms with some modifications (since we store beacon data in BLE Mini's EEPROM).


QuickStart
==========

The BLE Mini can load the MiniBeacon firmware (MiniBeacon-YYYYMMDD.bin) via the USB update. After that, you can use Apple's AirLocate App to see the beacon.

You need to compile the AirLocate App using XCode, but you can search for other 3rd party Apps in the AppStore.

We provided an App for MiniBeacon maintenance, such as setting the beacon UUID, marjor and minor values, etc.

*** Note that, the USB update is not compatible with Windows 8.1 ***


Compilation
===========

If you want to modify the source code, then you need to compile it. You need IAR 8051 C compiler (version 8.20.2) and TI CC254x BLE SDK (version 1.4). They are only available on Windows platform.


License
=======

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
