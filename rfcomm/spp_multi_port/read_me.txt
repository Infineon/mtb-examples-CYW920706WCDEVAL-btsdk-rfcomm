-------------------------------------------------------------------------------
SPP Multi Port Application
-------------------------------------------------------------------------------


Overview
--------
 SPP multi port application uses SPP profile library to establish, terminate,
 send and receive SPP data over BR/EDR. This sample supports up to two SPP
 connections and interfaces to an external host using WICED HCI. When a data
 packet is received over HCI it is formatted and sent over the appropriate SPP
 session. Similarly SPP packets are decoded by the device and sent to the host
 as data chunks.

See chip specific readme for more information about the BT SDK.

Instructions
------------
 To demonstrate the app, work through the following steps -

 1. Build and download the application to the WICED board
 2. Use ClientControl to conect to the HCI UART COM port
 3. Enable Discoverability and Connectability via ClientContol
 4. Use the computer's 'Add a Bluetooth Device' menu to pair with spp_multi_port
    app. That should create an incoming and outgoing COM ports on your computer,
    see 'More Bluetooth options'
 5. Use application such as Tera Term to open the BT COM port
 6. Use a BT SPP test app on your smart phone to connect to the 2nd SPP port
 7. You can use ClientControl to send data you input via the GUI or specify send from
    file. You can select between the two devices using the device drop down menu in
    ClientContol
 8. Type any keys on the terminal of the outgoing COM port, the spp application will
    receive the keys.

-------------------------------------------------------------------------------
