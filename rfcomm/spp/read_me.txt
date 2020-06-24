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
2. Open the BT/BLE Profile Client Control application and open the port for WICED HCI for the device.
   Default baud rate configured in the application is defined by the BSP HCI_UART_DEAULT_BAUD #define,
   usually either 3M or 115200 depending on board UART capabilities.
3. Run the BTSpy program to view protocol and application traces.
   See "BT/BLE Profile Client Control" and "BT Spy" in chip-specifc readme.txt for more information about these apps.
4. On Windows 10 PCs, right click on the Bluetooth icon in the system tray and
   select 'Add a Bluetooth Device'. Find and pair with the spp app. That should create an incoming and an outgoing
   COM port on your computer. Right click on the Bluetooth icon in the system tray and
   select 'Open Settings', scroll down and select "More Bluetooth options" and then
   select the 'COM Ports' tab.
5. Use application such as Term Term to open the outgoing COM port. Opening the port
   will create the SPP connection.
6. Type any key on the terminal of the outgoing COM port, the spp application will receive the key.
7. By default, (SEND_DATA_ON_INTERRUPT=1) the application sends 1 MB data to the peer application on every
   App button press on the WICED board.
8. If desired, edit the spp.c file to configure the application to send data on a timer to the peer application by
   setting SEND_DATA_ON_INTERRUPT=0 and SEND_DATA_ON_TIMEOUT=1


Notes
-----
See the spp.c file for compile flag options for different modes of testing.
CYW9M2BASE-43012BT does not support SEND_DATA_ON_INTERRUPT because the platform does not have button connected to BT board.
