/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 *
 * SPP Multi Port Application for 20XXX devices.
 *
 * SPP multi port application uses SPP profile library to establish, terminate, send and receive SPP
 * data over BR/EDR. This sample supports up to two SPP connections and interfaces to an external host
 * using WICED HCI. When a data packet is received over HCI it is formatted and sent over
 * the appropriate SPP session. Similarly SPP packets are decoded by the device and sent
 * to the host as data chunks.
 *
 * Following compilation flags are important for testing
 *  HCI_TRACE_OVER_TRANSPORT - configures HCI traces to be routed to the WICED HCI interface
 *  WICED_BT_TRACE_ENABLE    - enables WICED_BT_TRACEs.  You can also modify makefile.mk to build
 *                             with _debug version of the library
 *
 * To demonstrate the app, work through the following steps.
 * 1. Build and download the application to the WICED board
 * 2. Use ClientControl to conect to the HCI UART COM port
 * 3. Enable Discoverability and Connectability via ClientContol
 * 4. Use the computer's 'Add a Bluetooth Device' menu to pair with spp_multi_port app. That should
 *    create an incoming and outgoing COM ports on your computer, see 'More Bluetooth options'
 * 5. Use application such as Tera Term to open the BT COM port
 * 6. Use a BT SPP test app on your smart phone to connect to the 2nd SPP port
 * 7. You can use ClientControl to send data you input via the GUI or specify send from file. You can
 *    select between the two devices using the device drop down menu in ClientContol
 * 8. Type any keys on the terminal of the outgoing COM port, the spp application will receive the keys.
 *
 * Features demonstrated
 *  - Use of SPP library using multiple ports
 *  - WICED HCI
 *
 * The sample Windows ClientControl application is provided.
 *
 */

#include "sparcommon.h"
#include "wiced.h"
#include "wiced_gki.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_hal_nvram.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_sdp.h"
#include "hci_control_api.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_rfcomm.h"
#include "wiced_hci.h"
#include "wiced_memory.h"
#include "wiced_timer.h"
#include "wiced_transport.h"
#include "wiced_platform.h"
#include "string.h"
#include "wiced_bt_stack.h"
#include "wiced_hal_wdog.h"
#include "spp_multi_port.h"

#define HCI_TRACE_OVER_TRANSPORT            1   // If defined HCI traces are send over transport/WICED HCI interface


#define SDP_ATTR_CLASS_ID_128(uuid)                                      \
    SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST), SDP_ATTR_SEQUENCE_1(17), \
        ((UUID_DESC_TYPE << 3) | SIZE_SIXTEEN_BYTES), uuid

#define SDP_ATTR_UUID128(uuid)                ((UUID_DESC_TYPE << 3) | SIZE_SIXTEEN_BYTES), uuid

#define WICED_EIR_BUF_MAX_SIZE                264
#define TRANS_UART_BUFFER_SIZE                1024

#define KEY_INFO_POOL_BUFFER_SIZE             145 //Size of the buffer used for holding the peer device key info
#define KEY_INFO_POOL_BUFFER_COUNT            10  //Correspond's to the number of peer devices

#define SPP_MULTI_PORT_FIRST_VALID_NVRAM_ID   WICED_NVRAM_VSID_START
#define SPP_MULTI_PORT_INVALID_NVRAM_ID       0x00

wiced_transport_buffer_pool_t *rfcomm_trans_pool;   // Trans pool for sending the RFCOMM data to host

/*****************************************************************************
**  Structures
*****************************************************************************/
typedef struct
{
    void    *p_next;
    uint16_t nvram_id;
    uint8_t  chunk_len;
    uint8_t  data[1];
} spp_multi_port_nvram_chunk_t;

spp_multi_port_nvram_chunk_t *p_nvram_first = NULL;

wiced_transport_buffer_pool_t* host_trans_pool;
wiced_bt_buffer_pool_t*        p_key_info_pool;//Pool for storing the  key info

const uint8_t spp_multi_port_sdp_db[] = // Define SDP database
{

    SDP_ATTR_SEQUENCE_2(219),

    SDP_ATTR_SEQUENCE_1(70),                                                // 2 bytes
            SDP_ATTR_RECORD_HANDLE(0x10001),                                    // 8 bytes
            SDP_ATTR_CLASS_ID(UUID_SERVCLASS_SERIAL_PORT),                      // 8
            SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(SPP_RFCOMM_SCN1),                // 17 bytes
            SDP_ATTR_BROWSE_LIST,                                               // 8
            SDP_ATTR_PROFILE_DESC_LIST(UUID_SERVCLASS_SERIAL_PORT, 0x0102),     // 13 byte
            SDP_ATTR_SERVICE_NAME(11),                                          // 16
            'S', 'P', 'P', ' ', 'S', 'E', 'R', 'V', 'E', 'R','1',

	SDP_ATTR_SEQUENCE_1(70),                                                // 2 bytes
			SDP_ATTR_RECORD_HANDLE(0x10002),                                    // 8 bytes
			SDP_ATTR_CLASS_ID(UUID_SERVCLASS_SERIAL_PORT),                      // 8
			SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(SPP_RFCOMM_SCN2),                // 17 bytes
			SDP_ATTR_BROWSE_LIST,                                               // 8
			SDP_ATTR_PROFILE_DESC_LIST(UUID_SERVCLASS_SERIAL_PORT, 0x0102),     // 13 byte
			SDP_ATTR_SERVICE_NAME(11),                                          // 16
			'S', 'P', 'P', ' ', 'S', 'E', 'R', 'V', 'E', 'R','2',

    // Device ID service
    SDP_ATTR_SEQUENCE_1(69),                                                // 2 bytes, length of the record
        SDP_ATTR_RECORD_HANDLE(0x10003),                                    // 8 byte
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_PNP_INFORMATION),                  // 8
        SDP_ATTR_PROTOCOL_DESC_LIST(1),                                     // 18
        SDP_ATTR_UINT2(ATTR_ID_SPECIFICATION_ID, 0x103),                    // 6
        SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID, 0x0f),                            // 6
        SDP_ATTR_UINT2(ATTR_ID_PRODUCT_ID, 0x0401),                         // 6
        SDP_ATTR_UINT2(ATTR_ID_PRODUCT_VERSION, 0x0001),                    // 6
        SDP_ATTR_BOOLEAN(ATTR_ID_PRIMARY_RECORD, 0x01),                     // 5
        SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID_SOURCE, DI_VENDOR_ID_SOURCE_BTSIG) // 6
};

// Length of the SDP database
const uint16_t spp_multi_port_sdp_db_len = sizeof(spp_multi_port_sdp_db);

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
uint8_t                              pincode[4]  = {0x30,0x30,0x30,0x30};
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS];

static uint32_t               spp_multi_port_proc_rx_cmd(uint8_t *p_data, uint32_t length);

const wiced_transport_cfg_t  transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate =  HCI_UART_DEFAULT_BAUD
        },
    },
    .rx_buff_pool_cfg =
    {
        .buffer_size  = TRANS_UART_BUFFER_SIZE,
        .buffer_count = 1
    },
    .p_status_handler    = NULL,
    .p_data_handler      = spp_multi_port_proc_rx_cmd,
    .p_tx_complete_cback = NULL
};


/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static wiced_bt_dev_status_t spp_multi_port_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static void                  spp_multi_port_write_eir(void);
static void                  spp_multi_port_handle_reset_cmd(void);
static void                  spp_multi_port_handle_trace_enable(uint8_t *p_data);
static void                  spp_multi_port_handle_set_local_bda(uint8_t *p_bda);
static void                  spp_multi_port_handle_set_visibility(uint8_t discoverability, uint8_t connectability);
static void                  spp_multi_port_send_device_started_evt(void);
static void                  spp_multi_port_send_command_status_evt(uint16_t code, uint8_t status);
static void                  spp_multi_port_send_pairing_completed_evt(uint8_t status, uint8_t *p_bda);
static void                  spp_multi_port_send_encryption_changed_evt(uint8_t encrypted, uint8_t *p_bda);
static int                   spp_multi_port_write_nvram(int nvram_id, int data_len, void *p_data, BOOLEAN from_host);
static void                  spp_multi_port_delete_nvram(int nvram_id);
static void                  hci_control_misc_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
static void                  hci_control_misc_handle_get_version( void );
static void                  hci_control_inquiry( uint8_t enable );
static int                   spp_multi_port_find_nvram_id(uint8_t *p_data, int len);
static int                   spp_multi_port_alloc_nvram_id();
static int                   spp_multi_port_read_nvram(int nvram_id, void *p_data, int data_len);

#ifdef HCI_TRACE_OVER_TRANSPORT
static void                  spp_multi_port_hci_trace_cback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);
#endif

#if defined (CYW20706A2)
extern BOOL32 wiced_hal_puart_select_uart_pads(UINT8 rxdPin, UINT8 txdPin, UINT8 ctsPin, UINT8 rtsPin);
extern wiced_result_t wiced_bt_app_init( void );
#endif

/*******************************************************************
 * Function Definitions
 ******************************************************************/

/*
 * Entry point to the application. Set device configuration and start BT
 * stack initialization.  The actual application initialization will happen
 * when stack reports that BT device is ready
 */
APPLICATION_START()
{
    wiced_transport_init(&transport_cfg);

    // create special pool for sending data to the MCU
    host_trans_pool = wiced_transport_create_buffer_pool(TRANS_UART_BUFFER_SIZE, TRANS_MAX_BUFFERS);

#ifdef WICED_BT_TRACE_ENABLE
    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

#ifdef NO_PUART_SUPPORT
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_WICED_UART );
#else
    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#if defined (CYW20706A2)
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif
#endif

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must
    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    WICED_BT_TRACE("SPP Multi Port START\n");

    /* Initialize Stack and Register Management Callback */
    // Register call back and configuration with stack
    wiced_bt_stack_init(spp_multi_port_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
}

/*
 * application initialization is executed after BT stack initialization is completed.
 */
void application_init(void)
{
    wiced_result_t         result;

#if defined (CYW20706A2)
    /* Initialize wiced app */
    wiced_bt_app_init();
#endif

    // configure eir data
    spp_multi_port_write_eir();

#if defined (CYW20706A2)
    // Initialize RFCOMM.  We will not be using application buffer pool and will rely on the
    // stack pools configured in the wiced_bt_cfg.c
    wiced_bt_rfcomm_init(MAX_TX_BUFFER, 1);
#endif

    // Initialize SPP unit
    hci_spp_init();

#ifdef HCI_TRACE_OVER_TRANSPORT
    // There is a virtual HCI interface between upper layers of the stack and
    // the controller portion of the chip with lower layers of the BT stack.
    // Register with the stack to receive all HCI commands, events and data.
    wiced_bt_dev_register_hci_trace(spp_multi_port_hci_trace_cback);
#endif
    /* create SDP records */
    wiced_bt_sdp_db_init((uint8_t *)spp_multi_port_sdp_db, sizeof(spp_multi_port_sdp_db));

    /* Creating a buffer pool for holding the peer devices's key info */
    p_key_info_pool = (wiced_bt_buffer_pool_t*) wiced_bt_create_pool(KEY_INFO_POOL_BUFFER_SIZE, KEY_INFO_POOL_BUFFER_COUNT);
    WICED_BT_TRACE("wiced_bt_create_pool %x\n", p_key_info_pool);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    spp_multi_port_send_device_started_evt();
}

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t spp_multi_port_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                     result = WICED_BT_SUCCESS;
    wiced_bt_dev_status_t              dev_status;
    wiced_bt_dev_pairing_info_t*       p_pairing_info;
    wiced_bt_dev_encryption_status_t  *p_encryption_status;
    int                                bytes_written, bytes_read;
    int                                nvram_id;
    wiced_bt_power_mgmt_notification_t *p_power_mgmt_notification;

    WICED_BT_TRACE("spp_multi_port_management_callback 0x%02x\n", event);

    switch(event)
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            application_init();
            WICED_BT_TRACE("Free mem:%d", wiced_memory_get_free_bytes());
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_PIN_REQUEST_EVT:
            WICED_BT_TRACE("remote address= %B\n", p_event_data->pin_request.bd_addr);
            wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr,result/*WICED_BT_SUCCESS*/,4, &pincode[0]);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            /* This application always confirms peer's attempt to pair */
            wiced_bt_dev_confirm_req_reply (WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            /* This application supports only Just Works pairing */
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_REQUEST_EVT bda %B\n", p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap   = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req       = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_pairing_info = &p_event_data->pairing_complete.pairing_complete_info;
            WICED_BT_TRACE("Pairing Complete: %d\n", p_pairing_info->br_edr.status);
            result = WICED_BT_USE_DEFAULT_SECURITY;

            spp_multi_port_send_pairing_completed_evt(p_pairing_info->br_edr.status, p_event_data->pairing_complete.bd_addr);
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_encryption_status = &p_event_data->encryption_status;
            WICED_BT_TRACE("Encryption Status Event: bd (%B) res %d\n", p_encryption_status->bd_addr, p_encryption_status->result);

            spp_multi_port_send_encryption_changed_evt (p_encryption_status->result, p_encryption_status->bd_addr);
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* Check if we already have information saved for this bd_addr */
            if ((nvram_id = spp_multi_port_find_nvram_id(p_event_data->paired_device_link_keys_update.bd_addr, BD_ADDR_LEN)) == 0)
            {
                /* This is the first time, allocate id for the new memory chunk */
                nvram_id = spp_multi_port_alloc_nvram_id();
                WICED_BT_TRACE("Allocated NVRAM ID:%d\n", nvram_id);
            }
            bytes_written = spp_multi_port_write_nvram(nvram_id, sizeof(wiced_bt_device_link_keys_t), &p_event_data->paired_device_link_keys_update, FALSE);

            WICED_BT_TRACE("NVRAM write:id:%d bytes:%d\n", nvram_id, bytes_written);
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* read existing key from the NVRAM  */
            if ((nvram_id = spp_multi_port_find_nvram_id(p_event_data->paired_device_link_keys_request.bd_addr, BD_ADDR_LEN)) != 0)
            {
                 bytes_read = spp_multi_port_read_nvram(nvram_id, &p_event_data->paired_device_link_keys_request, sizeof(wiced_bt_device_link_keys_t));

                 result = WICED_BT_SUCCESS;
                 WICED_BT_TRACE("Read:nvram_id:%d bytes:%d\n", nvram_id, bytes_read);
            }
            else
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE("Key retrieval failure\n");
            }
            break;

        default:
            result = WICED_BT_USE_DEFAULT_SECURITY;
            break;
    }
    return result;
}

/*
 *  Prepare extended inquiry response data.  Current version publishes device name,
 *  and 16 byte IAP2 service.
 */
void spp_multi_port_write_eir(void)
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t length;
    uint16_t eir_length;

    pBuf = (uint8_t *)wiced_bt_get_buffer(WICED_EIR_BUF_MAX_SIZE);
    WICED_BT_TRACE("hci_control_write_eir %x\n", pBuf);

    if (!pBuf)
    {
        WICED_BT_TRACE("app_write_eir %x\n", pBuf);
        return;
    }

    p = pBuf;

    length = strlen((char *)wiced_bt_cfg_settings.device_name);

    *p++ = length + 1;
    *p++ = BT_EIR_COMPLETE_LOCAL_NAME_TYPE;        // EIR type full name
    memcpy(p, wiced_bt_cfg_settings.device_name, length);
    p += length;

    *p++ = 2 + 1;                                   // Length of 16 bit services
    *p++ = BT_EIR_COMPLETE_16BITS_UUID_TYPE;        // 0x03 EIR type full list of 16 bit service UUIDs
    *p++ = UUID_SERVCLASS_SERIAL_PORT & 0xff;
    *p++ = (UUID_SERVCLASS_SERIAL_PORT >> 8) & 0xff;

    *p++ = 0;                                       // end of EIR Data is 0

    eir_length = (uint16_t) (p - pBuf);

    // print EIR data
    wiced_bt_trace_array("EIR :", pBuf, MIN(p-pBuf, 100));
    wiced_bt_dev_write_eir(pBuf, eir_length);
}

/*
 *  Pass protocol traces up through the UART
 */
void spp_multi_port_hci_trace_cback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    //send the trace
    wiced_transport_send_hci_trace(NULL, type, length, p_data );
}

wiced_timer_t spp_multi_port_app_timer;

/*
* The function invoked on timeout of app seconds timer.
*/
void spp_multi_port_timeout(uint32_t count)
{
    static uint32_t counter = 0;
//    WICED_BT_TRACE("spp_multi_port_timeout %d\n", counter++);
}


void spp_multi_port_init_timer(void)
{
    if (wiced_init_timer(&spp_multi_port_app_timer, &spp_multi_port_timeout, 0, WICED_SECONDS_PERIODIC_TIMER) == WICED_SUCCESS)
    {
        if (wiced_start_timer(&spp_multi_port_app_timer, 1) !=  WICED_SUCCESS)
        {
        }
    }
}



void spp_multi_port_device_handle_command(uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len)
{
    uint8_t                     bytes_written;
    int                         nvram_id;
    int                         read_bytes;
    wiced_bt_device_link_keys_t link_keys;
    int                         i;
    BD_ADDR                     bda;
    wiced_result_t              result;

    switch (cmd_opcode)
    {
    case HCI_CONTROL_COMMAND_RESET:
        spp_multi_port_handle_reset_cmd();
        break;

    case HCI_CONTROL_COMMAND_TRACE_ENABLE:
        spp_multi_port_handle_trace_enable(p_data);
        break;

    case HCI_CONTROL_COMMAND_SET_LOCAL_BDA:
        spp_multi_port_handle_set_local_bda(p_data);
        break;

    case HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA:
        bytes_written = spp_multi_port_write_nvram(p_data[0] | (p_data[1] << 8), data_len - 2, &p_data[2], TRUE);
        WICED_BT_TRACE("NVRAM write: %d\n", bytes_written);
        break;

    case HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA:
        nvram_id = p_data[0] | (p_data[1] << 8);
        WICED_BT_TRACE("NVRAM delete: %d\n", nvram_id);

        if( (read_bytes=spp_multi_port_read_nvram(nvram_id, &link_keys, sizeof(wiced_bt_device_link_keys_t))) )
        {
            WICED_BT_TRACE("Read:nvram_id:%d bytes:%d\n", nvram_id, read_bytes);
            hci_spp_disconnect(link_keys.bd_addr);
            spp_multi_port_delete_nvram(nvram_id);
            if(wiced_bt_dev_delete_bonded_device(link_keys.bd_addr))
            {
                WICED_BT_TRACE("Error removing link keys from internal db\n");
            }
        }
        break;

    case HCI_CONTROL_COMMAND_SET_VISIBILITY:
        spp_multi_port_handle_set_visibility(p_data[0], p_data[1]);
        break;

    case HCI_CONTROL_COMMAND_INQUIRY:
        hci_control_inquiry( *p_data );
        break;

    case HCI_CONTROL_COMMAND_UNBOND:
        for (i = 0; i < BD_ADDR_LEN; i++)
            bda[5 - i] = p_data[i];

        if ((nvram_id = spp_multi_port_find_nvram_id(bda, BD_ADDR_LEN)) != 0)
        {
            spp_multi_port_delete_nvram(nvram_id);
            if(wiced_bt_dev_delete_bonded_device(bda))
            {
                WICED_BT_TRACE("Error removing link keys from internal db\n");
            }
        }
        hci_spp_disconnect(bda);
        break;

    default:
        WICED_BT_TRACE("spp_multi_port_device_handle_command:unknown opcode code  %d\n", cmd_opcode);
        break;
    }

}

/*
 * Handle received command over UART. Please refer to the WICED Smart Ready
 * Software User Manual (WICED-Smart-Ready-SWUM100-R) for details on the
 * HCI UART control protocol.
*/
uint32_t spp_multi_port_proc_rx_cmd(uint8_t *p_data, uint32_t length)
{
    uint16_t                    opcode;
    uint16_t                    payload_len;
    uint8_t                     status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t*                    p_rx_buf = p_data;

    if (!p_rx_buf)
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    //Expected minimum 4 byte as the wiced header
    if(length < 4)
    {
        WICED_BT_TRACE("invalid params\n");
        wiced_transport_free_buffer(p_rx_buf);
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }


    opcode      = p_data[0] + (p_data[1] << 8);
    payload_len = p_data[2] + (p_data[3] << 8);

    p_data += 4;
    length -= 4;

    switch((opcode >> 8) & 0xff)
    {
	case HCI_CONTROL_GROUP_DEVICE:
		spp_multi_port_device_handle_command(opcode, p_data, payload_len);
		break;
	case HCI_CONTROL_GROUP_SPP:
		hci_spp_handle_command(opcode, p_data, payload_len);
		break;
	case HCI_CONTROL_GROUP_MISC:
		hci_control_misc_handle_command(opcode, p_data, payload_len);
		break;
	default:
		WICED_BT_TRACE("spp_multi_port_proc_rx_cmd:unknown class code len %d\n", length);
		break;
    }

    //Freeing the buffer in which data is received
    wiced_transport_free_buffer(p_rx_buf);
    return status;
}


/*
 * handle reset command from UART
 */
void spp_multi_port_handle_reset_cmd(void)
{
    // trip watch dog now.
	wiced_hal_wdog_reset_system();
}

/*
 * handle command from UART to configure traces
 */
void spp_multi_port_handle_trace_enable(uint8_t *p_data)
{
    uint8_t hci_trace_enable = *p_data++;
    wiced_debug_uart_types_t route_debug = *p_data;
    if (hci_trace_enable)
    {
        /* Register callback for receiving hci traces */
        // Disable while streaming audio over the uart.
        wiced_bt_dev_register_hci_trace(spp_multi_port_hci_trace_cback);
    }
    else
    {
        wiced_bt_dev_register_hci_trace(NULL);
    }
    wiced_set_debug_uart(route_debug);
}

/*
 * handle command to set local Bluetooth device address
 */
void spp_multi_port_handle_set_local_bda(uint8_t *p_bda)
{
    BD_ADDR bd_addr;
    STREAM_TO_BDADDR(bd_addr,p_bda);
    wiced_bt_set_local_bdaddr(bd_addr, BLE_ADDR_PUBLIC);

    spp_multi_port_send_command_status_evt(HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
}

/*
 *  Handle Set Visibility command received over UART
 */
void spp_multi_port_handle_set_visibility(uint8_t discoverability, uint8_t connectability)
{
    wiced_bt_dev_status_t dev_status;
    uint8_t               visibility;
    uint8_t               status = HCI_CONTROL_STATUS_SUCCESS;

    // we cannot be discoverable and not connectable
    if (((discoverability != 0) && (connectability == 0)) ||
           (discoverability > 1) ||
           (connectability > 1))
    {
        spp_multi_port_send_command_status_evt(HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS);
    }
    else
    {
        wiced_bt_dev_set_discoverability(((discoverability != 0) ? BTM_GENERAL_DISCOVERABLE : BTM_NON_DISCOVERABLE), 0x0012, 0x0800);
        wiced_bt_dev_set_connectability(BTM_CONNECTABLE, 0x0012, 0x0800);
        spp_multi_port_send_command_status_evt(HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
    }
}

/*
 *  Send Device Started event through UART
 */
void spp_multi_port_send_device_started_evt(void)
{
    wiced_transport_send_data(HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0);

//    WICED_BT_TRACE("maxLinks:%d maxChannels:%d maxClients:%d maxBdConn:%d\n", mpaf_stack_config_data.l2c.maxLinks,
//        mpaf_stack_config_data.l2c.maxChannels, mpaf_stack_config_data.l2c.maxClients, mpaf_stack_config_data.rfc.maxConn);
}

/*
* transfer command status event to UART
*/
void spp_multi_port_send_command_status_evt(uint16_t code, uint8_t status)
{
    wiced_transport_send_data(code, &status, 1);
}

/*
 *  Send Pairing Completed event through UART
 */
void spp_multi_port_send_pairing_completed_evt(uint8_t status, uint8_t *p_bda)
{
    uint8_t event_data[12];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = status;

    {
        int i;
        for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[cmd_bytes++] = p_bda[BD_ADDR_LEN - 1 - i];
    }

    wiced_transport_send_data(HCI_CONTROL_EVENT_PAIRING_COMPLETE, event_data, cmd_bytes );
}

/*
 *  Send Encryption Changed event through UART
 */
void spp_multi_port_send_encryption_changed_evt(uint8_t encrypted, uint8_t *p_bda)
{
    uint8_t tx_buf[12];
    uint8_t *p = tx_buf;
    int i;

    *p++ = encrypted;

    for (i = 0; i < 6; i++)
        *p++ = p_bda[5 - i];

    wiced_transport_send_data(HCI_CONTROL_EVENT_ENCRYPTION_CHANGED, tx_buf, (int)(p - tx_buf));
}

#ifdef HCI_TRACE_OVER_TRANSPORT
/*
 *  Pass protocol traces up over the transport
 */
void app_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    wiced_transport_send_hci_trace(NULL, type, length, p_data);
}
#endif

/*
 * Write NVRAM function is called to store information in the RAM.  This can be called when
 * stack requires persistent storage, for example to save link keys.  In this case
 * data is also formatted and send to the host for real NVRAM storage.  The same function is
 * called when host pushes NVRAM chunks during the startup.  Parameter from_host in this
 * case is set to FALSE indicating that data does not need to be forwarded.
 */
int spp_multi_port_write_nvram(int nvram_id, int data_len, void *p_data, BOOLEAN from_host)
{
    uint8_t                    tx_buf[257];
    uint8_t                    *p = tx_buf;
    spp_multi_port_nvram_chunk_t *p1;
    wiced_result_t             result;
    int                        bytes_written;

    /* first check if this ID is being reused and release the memory chunk */
    spp_multi_port_delete_nvram(nvram_id);

    if ((p1 = (spp_multi_port_nvram_chunk_t *)wiced_bt_get_buffer_from_pool(p_key_info_pool)) == NULL)
    {
        WICED_BT_TRACE("Failed to alloc:%d\n", data_len);
        return (0);
    }
    if (wiced_bt_get_buffer_size(p1) < (sizeof(spp_multi_port_nvram_chunk_t) + data_len - 1))
    {
        WICED_BT_TRACE("Insufficient buffer size, Buff Size %d, Len %d  \n",
                        wiced_bt_get_buffer_size(p1), (sizeof(spp_multi_port_nvram_chunk_t) + data_len - 1));
        wiced_bt_free_buffer(p1);
        return (0);
    }
    p1->p_next    = p_nvram_first;
    p1->nvram_id  = nvram_id;
    p1->chunk_len = data_len;
    memcpy(p1->data, p_data, data_len);

    p_nvram_first = p1;

    /* If NVRAM chunk arrived from host, no need to send it back, otherwise send over transport */
    if (!from_host)
    {
        *p++ = nvram_id & 0xff;
        *p++ = (nvram_id >> 8) & 0xff;
        memcpy(p, p_data, data_len);

        wiced_transport_send_data(HCI_CONTROL_EVENT_NVRAM_DATA, tx_buf, (int)(data_len + 2));
    }
    return (data_len);
}

/*
 * Find nvram_id of the NVRAM chunk with first bytes matching specified byte array.
 */
static int spp_multi_port_find_nvram_id(uint8_t *p_data, int len)
{
    spp_multi_port_nvram_chunk_t *p1;

    /* Go through the linked list of chunks */
    for (p1 = p_nvram_first; p1 != NULL; p1 = (spp_multi_port_nvram_chunk_t *)p1->p_next)
    {
        WICED_BT_TRACE("find %B %B len:%d", p1->data, p_data, len);
        if (memcmp(p1->data, p_data, len) == 0)
        {
            return (p1->nvram_id);
        }
    }
    return SPP_MULTI_PORT_INVALID_NVRAM_ID;
}

/*
 * Delete NVRAM function is called when host deletes NVRAM chunk.
 */
void spp_multi_port_delete_nvram(int nvram_id)
{
    spp_multi_port_nvram_chunk_t *p1, *p2;

    if (p_nvram_first == NULL)
        return;

    /* Special case when need to remove the first chunk */
    if ((p_nvram_first != NULL) && (p_nvram_first->nvram_id == nvram_id))
    {
        p1 = p_nvram_first;
        p_nvram_first = (spp_multi_port_nvram_chunk_t *)p_nvram_first->p_next;
        wiced_bt_free_buffer(p1);
        return;
    }

    /* Go through the linked list of chunks */
    for (p1 = p_nvram_first; p1 != NULL; p1 = (spp_multi_port_nvram_chunk_t *)p1->p_next)
    {
        p2 = (spp_multi_port_nvram_chunk_t *)p1->p_next;
        if ((p2 != NULL) && (p2->nvram_id == nvram_id))
        {
            p1->p_next = p2->p_next;
            wiced_bt_free_buffer(p2);
            return;
        }
    }
}

/*
 * Read NVRAM actually finds the memory chunk in the RAM
 */
static int spp_multi_port_read_nvram(int nvram_id, void *p_data, int data_len)
{
    spp_multi_port_nvram_chunk_t *p1;
    int                        data_read = 0;

    /* Go through the linked list of chunks */
    for (p1 = p_nvram_first; p1 != NULL; p1 = p1->p_next)
    {
        if (p1->nvram_id == nvram_id)
        {
            data_read = (data_len < p1->chunk_len) ? data_len : p1->chunk_len;
            memcpy(p_data, p1->data, data_read);
            break;
        }
    }
    return (data_read);
}

/*
 * Allocate nvram_id to save new NVRAM chunk
 */
static int spp_multi_port_alloc_nvram_id()
{
    spp_multi_port_nvram_chunk_t *p1 = p_nvram_first;
    int                    nvram_id;

    /* Go through the linked list of chunks */
    WICED_BT_TRACE ("spp_multi_port_alloc_nvram_id\n");
    for (nvram_id = SPP_MULTI_PORT_FIRST_VALID_NVRAM_ID; p1 != NULL; nvram_id++)
    {
        for (p1 = p_nvram_first; p1 != NULL; p1 = (spp_multi_port_nvram_chunk_t *)p1->p_next)
        {
            if (p1->nvram_id == nvram_id)
            {
                /* this nvram_id is already used */
                break;
            }
        }
        if (p1 == NULL)
        {
            break;
        }
    }
    WICED_BT_TRACE ("spp_multi_port_alloc_nvram_id:%d\n", nvram_id);
    return (nvram_id);
}

/* Handle misc command group */
void hci_control_misc_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    switch( cmd_opcode )
    {
    case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
        hci_control_misc_handle_get_version();
        break;
    }
}


/* Handle get version command */
void hci_control_misc_handle_get_version( void )
{
    uint8_t   tx_buf[15];
    uint8_t   cmd = 0;

// If this is 20819 or 20820, we do detect the device from hardware
#define RADIO_ID    0x006007c0
#define RADIO_20820 0x80
#define CHIP_20820  20820
#define CHIP_20819  20819
#if (CHIP==CHIP_20819) || (CHIP==CHIP_20820)
	uint32_t chip = CHIP_20819;
	if (*(UINT32*) RADIO_ID & RADIO_20820)
	{
		chip = CHIP_20820;
	}
#else
	uint32_t  chip = CHIP;
#endif

    tx_buf[cmd++] = WICED_SDK_MAJOR_VER;
    tx_buf[cmd++] = WICED_SDK_MINOR_VER;
    tx_buf[cmd++] = WICED_SDK_REV_NUMBER;
    tx_buf[cmd++] = WICED_SDK_BUILD_NUMBER & 0xFF;
    tx_buf[cmd++] = (WICED_SDK_BUILD_NUMBER>>8) & 0xFF;
    tx_buf[cmd++] = chip & 0xFF;
    tx_buf[cmd++] = (chip>>8) & 0xFF;
    tx_buf[cmd++] = (chip>>24) & 0xFF;
    tx_buf[cmd++] = 0; // not used

    /* Send MCU app the supported features */
    tx_buf[cmd++] = HCI_CONTROL_GROUP_SPP;

    wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd );
}

/*
 *  Handle Inquiry result callback from teh stack, format and send event over UART
 */
void hci_control_inquiry_result_cback( wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data )
{
    int       i;
    uint8_t   len;
    uint8_t   tx_buf[300];
    uint16_t  code;
    uint8_t   *p = tx_buf;

    if ( p_inquiry_result == NULL )
    {
        code = HCI_CONTROL_EVENT_INQUIRY_COMPLETE;
    }
    else
    {
        code = HCI_CONTROL_EVENT_INQUIRY_RESULT;
        WICED_BT_TRACE( "inquiry result %B\n", p_inquiry_result->remote_bd_addr );
        for ( i = 0; i < 6; i++ )
            *p++ = p_inquiry_result->remote_bd_addr[5 - i];
        for ( i = 0; i < 3; i++ )
            *p++ = p_inquiry_result->dev_class[2 - i];
        *p++ = p_inquiry_result->rssi;

        // currently callback does not pass the data of the adv data, need to go through the data
        // zero len in the LTV means that there is no more data
        while ( ( len = *p_eir_data ) != 0 )
        {
            // In the HCI event all parameters should fit into 255 bytes
            if ( p + len + 1 > tx_buf + 255 )
            {
                WICED_BT_TRACE( "Bad data\n" );
                break;
            }
            for ( i = 0; i < len + 1; i++ )
                *p++ = *p_eir_data++;
        }
    }
    wiced_transport_send_data( code, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  Handle Inquiry command received over UART
 */
void hci_control_inquiry( uint8_t enable )
{
    wiced_result_t           result;
    wiced_bt_dev_inq_parms_t params;

    if ( enable )
    {

        memset( &params, 0, sizeof( params ) );

        params.mode             = BTM_GENERAL_INQUIRY;
        params.duration         = 5;
        params.filter_cond_type = BTM_CLR_INQUIRY_FILTER;

        result = wiced_bt_start_inquiry( &params, &hci_control_inquiry_result_cback );
        WICED_BT_TRACE( "inquiry started:%d\n", result );
    }
    else
    {
        result = wiced_bt_cancel_inquiry( );
        WICED_BT_TRACE( "cancel inquiry:%d\n", result );
    }

    spp_multi_port_send_command_status_evt(HCI_CONTROL_EVENT_COMMAND_STATUS , HCI_CONTROL_STATUS_SUCCESS );
}
