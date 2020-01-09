-------------------------------------------------------------------------------
Object Push Profile (OPP) Server app
-------------------------------------------------------------------------------

Overview
--------
The Object Push Profile server is used to receive object files(e.g vCard,
Image, Text, ...). Send object files form OPP client(mobile phone or PC).

See chip specific readme for more information about the BT SDK.

Features demonstrated
---------------------
 *  - WICED BT OPP server APIs
 *  - Handling of the UART WICED protocol
 *  - SDP configuration
 *  - Setting of the Local Bluetooth Device address from the host MCU

Instructions
------------
To demonstrate the app, work through the following steps.
1. Plug the WICED eval board into your computer.
2. Build and download the application (to the WICED board)
3. Launch Client Control application.
4. From Client Control UI, Open port to connect to the eval board.
5. From peer appliction such as Windows PC or mobile phone, find and pair
   with the OPP Server app.
6. From peer application, send a file to OPP Server app.
7. Received files are saved to the folder configured in ClientControl.

-------------------------------------------------------------------------------
