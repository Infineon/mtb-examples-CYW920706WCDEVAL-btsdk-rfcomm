-------------------------------------------------------------------------------
SPP app
-------------------------------------------------------------------------------

Overview
--------

SPP application uses SPP profile library to establish, terminate, send and receive SPP
data over BR/EDR. This sample supports a single SPP connection.

See chip specific readme for more information about the BT SDK.

Features demonstrated
---------------------
Use of Bluetooth SPP library

Instructions
------------
To demonstrate the app, work through the following steps.
1. Build and download the application to the WICED board.
2. Use standard terminal emulation application such as Term Term to open the WICED Peripheral UART, use
   baud rate of 115200.
3. On Windows 10 PCs, right click on the Bluetooth icon in the system tray and
   select 'Add a Bluetooth Device'. Find and pair with the spp app. That should create an incoming and an outgoing
   COM port on your computer. Right click on the Bluetooth icon in the system tray and
   select 'Open Settings', scroll down and select "More Bluetooth options" and then
   select the 'COM Ports' tab.
4. Use application such as Term Term to open the outgoing COM port. Opening the port
   will create the SPP connection.
5. By default the spp application sends data on a timer to the peer application.
6. Type any key on the terminal of the outgoing COM port, the spp application will receive the key.
7. Press the application button on the WICED board to send 1 MB data to the Windows.


Notes
-----
See the spp.c file for compile flag options for different modes of testing.
