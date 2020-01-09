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
 * OPP server Device Sample Application for 20xxx devices.
 *
 * This file implements 20xxx embedded application controlled over UART.
 * Current version of the application exposes OPP server
 * MCU connected over UART can send commands to control the application.
 *
 * To demonstrate the app, work through the following steps.
 * 1. Plug the WICED Bluetooth ( 20xxx ) evaluation board into your computer
 * 2. Build and download the application ( to the 20xxx board )
 * 3. Use ClientControl application to receive object files.
 *
 * The sample app performs as a Bluetooth OPP server
 *
 * The sample Windows ClientControl application is provided to show sample
 * MCU implementation.
 *
 * Features demonstrated
 *  - WICED BT OPP server APIs
 *  - Handling of the UART WICED protocol
 *  - SDP configuration
 *  - Setting of the Local Bluetooth Device address from the host MCU
 *
 * On startup this demo:
 *  - Initializes the Bluetooth sub system
 *  - Receive NVRAM information from the host
 *
 * Application Instructions
 *  - Connect a PC terminal to WICED Eval board.
 *  - Build and download the application to the board.
 *  - Run the ClientControl application and open the WICED HCI port
 *
 * OPP server Connection
 * The Object Push Profile is used to receive object files(e.g vCard,
 * Image, Text, ...). Send object files form OPP client(mobile phone or PC).
 * Received files are saved Clientcontrol running folder.
 *
 */

#include "sparcommon.h"
#include "wiced_bt_types.h"
#include "wiced_bt_dev.h"
#include "wiced_transport.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_sdp.h"
#include "wiced_memory.h"
#include "wiced_platform.h"
#include "hci_control_api.h"
#include "wiced_bt_ops_api.h"
#include "ops.h"
#include "string.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "wiced_hal_nvram.h"

/*******************************************************************************
 Definitions
 *******************************************************************************/

#define TRANS_UART_BUFFER_SIZE            1024
#define TRANS_UART_BUFFER_COUNT           10
#define KEY_INFO_POOL_BUFFER_COUNT        10
#define TRANS_DATA_SIZE                   TRANS_UART_BUFFER_SIZE

#define OPP_SUPPORTED_FORMAT              WICED_BT_OP_VCARD21_MASK | WICED_BT_OP_VCARD30_MASK | \
                                          WICED_BT_OP_VCAL_MASK | WICED_BT_OP_ICAL_MASK | \
                                          WICED_BT_OP_VNOTE_MASK | WICED_BT_OP_VMSG_MASK | \
                                          WICED_BT_OP_ANY_MASK

/*******************************************************************************
 Externs
 *******************************************************************************/
extern void hci_control_hci_trace_cback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);

/*******************************************************************************
 Local functions
 *******************************************************************************/
void wiced_bt_ops_enable_hdlr();

/*******************************************************************************
 Globals
 *******************************************************************************/

wiced_transport_buffer_pool_t* transport_pool; // Trans pool for sending the RFCOMM data to host
wiced_bt_buffer_pool_t* p_key_info_pool;  //Pool for storing the  key info

const wiced_transport_cfg_t transport_cfg =
{   WICED_TRANSPORT_UART,
    {{ WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_DEFAULT_BAUD }},
    { TRANS_UART_BUFFER_SIZE, 1 },
    NULL, hci_control_proc_rx_cmd,
    NULL
};

uint8_t pairing_allowed = 0;

/*******************************************************************************
 Functions
 *******************************************************************************/

void opp_server_write_eir()
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t length;

    pBuf = (uint8_t*) wiced_bt_get_buffer( 100);
    WICED_BT_TRACE( "hci_control_write_eir %x\n", pBuf );

    if (!pBuf)
    {
        return;
    }
    p = pBuf;

    length = strlen((char *) opp_server_cfg_settings.device_name);

    *p++ = length + 1;
    *p++ = BT_EIR_COMPLETE_LOCAL_NAME_TYPE;
    memcpy(p, opp_server_cfg_settings.device_name, length);
    p += length;
    *p++ = ( 1 * 2 ) + 1;     // length of services + 1
    *p++ = BT_EIR_COMPLETE_16BITS_UUID_TYPE;
    *p++ =   UUID_SERVCLASS_OBEX_OBJECT_PUSH        & 0xff;
    *p++ = ( UUID_SERVCLASS_OBEX_OBJECT_PUSH >> 8 ) & 0xff;

    *p++ = 0;

    // print EIR data
    wiced_bt_trace_array("EIR :", (uint8_t*) (pBuf + 1), MIN(p - (uint8_t* )pBuf, 100));
    wiced_bt_dev_write_eir(pBuf, (uint16_t) (p - pBuf));

    return;
}

void opp_server_post_bt_init(wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_ERROR;
    if (p_event_data->enabled.status == WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("Bluetooth stack initialized\n");

        /* Set-up EIR data */
        opp_server_write_eir();
        /* Set-up SDP database */
        wiced_bt_sdp_db_init((uint8_t *) opp_server_sdp_db, wiced_app_cfg_sdp_record_get_size());

        // Initialize RFCOMM.  We will not be using application buffer pool and will rely on the
        // stack pools configured in the ops_bt_cfg.c
        wiced_bt_rfcomm_init(1024, 1);

        wiced_bt_ops_enable_hdlr();

    } else
    {
        WICED_BT_TRACE("Bluetooth stack initialization failure!!\n");
        return;
    }
}

/*
 * Write NVRAM function is called to store information in the NVRAM.
 */
int opp_server_write_nvram(int nvram_id, int data_len, void *p_data)
{
    wiced_result_t result;
    int bytes_written = wiced_hal_write_nvram(nvram_id, data_len, (uint8_t*) p_data, &result);

    WICED_BT_TRACE("NVRAM ID:%d written :%d bytes result:%d\n", nvram_id, bytes_written, result);
    return (bytes_written);
}

/*
 * Read data from the NVRAM and return in the passed buffer
 */
int opp_server_read_nvram(int nvram_id, void *p_data, int data_len)
{
    uint16_t read_bytes = 0;
    wiced_result_t result;

    if (data_len >= sizeof(wiced_bt_device_link_keys_t))
    {
        read_bytes = wiced_hal_read_nvram(nvram_id, sizeof(wiced_bt_device_link_keys_t), p_data, &result);
        WICED_BT_TRACE("NVRAM ID:%d read out of %d bytes:%d result:%d\n", nvram_id, sizeof(wiced_bt_device_link_keys_t), read_bytes, result );
    }
    return (read_bytes);
}

/*******************************************************************************

 Function:       wiced_bt_ops_init

 Description:    Initializes the OPS control block

 Arguments:      None

 *******************************************************************************/
void wiced_bt_ops_init(void)
{
    WICED_BT_TRACE("wiced_bt_ops_init\n");
}

void wiced_bt_ops_data_callback(const uint8_t *p_buf, uint16_t nbytes)
{
    uint8_t* p_trans_buffer;
    uint8_t *p = (uint8_t*)p_buf;

    while(nbytes)
    {
        if ((p_trans_buffer = (uint8_t *) wiced_transport_allocate_buffer(transport_pool)) == NULL)
        {
            WICED_BT_TRACE("Error no transport buffer\n");
            return;
        }

        if( nbytes > TRANS_DATA_SIZE )
        {
            memcpy(p_trans_buffer, p, TRANS_DATA_SIZE);
            if (wiced_transport_send_buffer(HCI_CONTROL_OPS_EVENT_PUSH_DATA, p_trans_buffer, TRANS_DATA_SIZE) != WICED_SUCCESS)
            {
                WICED_BT_TRACE("Err: failed to transport buffer\n");
            }

            p += TRANS_DATA_SIZE;
            nbytes -= TRANS_DATA_SIZE;
        }
        else
        {
            memcpy(p_trans_buffer, p, nbytes);
            if (wiced_transport_send_buffer(HCI_CONTROL_OPS_EVENT_PUSH_DATA, p_trans_buffer, nbytes) != WICED_SUCCESS)
            {
                WICED_BT_TRACE("Err: failed to transport buffer\n");
            }
            break;
        }
    }
}

void wiced_bt_ops_event_callback(wiced_bt_ops_evt_t event, wiced_bt_ops_t *p_data)
{
    uint8_t event_data[500];
    memset(event_data, 0, 500);
    uint8_t *p = event_data;


    switch(event)
    {
    case WICED_BT_OPS_ENABLE_EVT:
        break;

    case WICED_BT_OPS_OPEN_EVT:
        BDADDR_TO_STREAM(p, p_data->bd_addr);
        UINT8_TO_STREAM(p, WICED_BT_SUCCESS);

        wiced_transport_send_data(HCI_CONTROL_OPS_EVENT_CONNECTED, event_data, (uint16_t)(p - event_data));
        break;

    case WICED_BT_OPS_PROGRESS_EVT:
        UINT32_TO_STREAM(p, p_data->prog.obj_size);
        UINT16_TO_STREAM(p, p_data->prog.bytes);
        UINT8_TO_STREAM(p, p_data->prog.operation);

        wiced_transport_send_data(HCI_CONTROL_OPS_EVENT_PROGRESS, event_data, (uint16_t)(p - event_data));
        break;

    case WICED_BT_OPS_OBJECT_EVT:
        UINT8_TO_STREAM(p, p_data->object.format);
        UINT8_TO_STREAM(p, strlen(p_data->object.p_name));
        memcpy(p, p_data->object.p_name, strlen(p_data->object.p_name));
        p += strlen(p_data->object.p_name);

        wiced_transport_send_data(HCI_CONTROL_OPS_EVENT_OBJECT, event_data, (uint16_t)(p - event_data));
        break;

    case WICED_BT_OPS_CLOSE_EVT:
        WICED_BT_TRACE("WICED_BT_OPS_CLOSE_EVT\n");
        BDADDR_TO_STREAM(p, p_data->bd_addr);
        wiced_transport_send_data(HCI_CONTROL_OPS_EVENT_CLOSE, event_data, (uint16_t)(p - event_data));
        break;

    case WICED_BT_OPS_ACCESS_EVT:
        BDADDR_TO_STREAM(p, p_data->access.bd_addr);
        UINT8_TO_STREAM(p, p_data->access.oper);
        UINT8_TO_STREAM(p, p_data->access.format);
        UINT32_TO_STREAM(p, p_data->access.size);
        UINT8_TO_STREAM(p, strlen(p_data->access.p_name));

        memcpy(p, p_data->access.p_name, strlen(p_data->access.p_name));
        p += strlen(p_data->access.p_name);

        if (p_data->access.p_type)
        {
            UINT8_TO_STREAM(p, strlen(p_data->access.p_type));
            memcpy(p, p_data->access.p_type, strlen(p_data->access.p_type));
            p += strlen(p_data->access.p_type);
        }

        WICED_BT_TRACE("ACCESS [%s]:%d [%s]%d\n",p_data->access.p_name, strlen(p_data->access.p_name),
                       p_data->access.p_type ? p_data->access.p_type : "NULL",
                       p_data->access.p_type ? strlen(p_data->access.p_type) : 0 );

        wiced_transport_send_data(HCI_CONTROL_OPS_EVENT_ACCESS, event_data, (uint16_t)(p - event_data));
        break;

    case WICED_BT_OPS_DISABLE_EVT:
        break;

    default:
        break;
    }
}
/*******************************************************************************
 **
 ** Function         wiced_bt_ops_enable_hdlr
 **
 ** Description      Handler function called when a msg is received from a client
 **
 ** Returns          void
 **
 *******************************************************************************/
void wiced_bt_ops_enable_hdlr()
{
    wiced_bt_ops_op_enable(OPP_SERVER_SCN, OPP_SUPPORTED_FORMAT, wiced_bt_ops_event_callback,
                           wiced_bt_ops_data_callback);
}

void wiced_bt_ops_close_hdlr()
{
    wiced_bt_ops_op_close();
}

void wiced_bt_ops_access_rsp_hdlr(wiced_bt_op_oper_t oper, wiced_bt_op_access_t access)
{
    WICED_BT_TRACE("wiced_bt_ops_access_rsp_hdlr oper = %d, access = %d\n", oper, access);

    wiced_bt_ops_op_access_rsp(oper, access);
}

wiced_result_t opp_server_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    int nvram_id;
    int bytes_written, bytes_read;
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_dev_pairing_cplt_t *p_pairing_cmpl;
    uint8_t pairing_result;
    wiced_bt_dev_encryption_status_t *p_encryption_status;

    WICED_BT_TRACE( "Bluetooth management callback event: 0x%02x\n", event );

    switch (event)
    {
    case BTM_ENABLED_EVT:
        //disable pairing
        wiced_bt_set_pairable_mode(0, 0);

        opp_server_post_bt_init(p_event_data);

        //Creating a buffer pool for holding the peer devices's key info
        p_key_info_pool = wiced_bt_create_pool( KEY_INFO_POOL_BUFFER_SIZE,
                                                KEY_INFO_POOL_BUFFER_COUNT);

        WICED_BT_TRACE( "wiced_bt_create_pool %x\n", p_key_info_pool );

        wiced_bt_dev_register_hci_trace(hci_control_hci_trace_cback);

        hci_control_send_device_started_evt();

        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_SECURITY_REQUEST_EVT:
        if (pairing_allowed)
        {
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        } else
        {
            // Pairing not allowed, return error
            result = WICED_BT_ERROR;
        }
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        p_pairing_cmpl = &p_event_data->pairing_complete;

        if (p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
        {
            pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
        } else
        {
            pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
        }
        hci_control_send_pairing_completed_evt(pairing_result, p_event_data->pairing_complete.bd_addr);
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        WICED_BT_TRACE( "BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT \n" );

        break;

    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        WICED_BT_TRACE( "BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT \n" );

        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        WICED_BT_TRACE( "BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT \n" );
        if ((nvram_id = hci_control_find_nvram_id(p_event_data->paired_device_link_keys_update.bd_addr, BD_ADDR_LEN)) == 0)
        {
            // This is the first time, allocate id for the new memory chunk
            nvram_id = hci_control_alloc_nvram_id();
            WICED_BT_TRACE( "Allocated NVRAM ID:%d\n", nvram_id );
        }
        bytes_written = hci_control_write_nvram(nvram_id, sizeof(wiced_bt_device_link_keys_t),
                                                &p_event_data->paired_device_link_keys_update, WICED_FALSE);

        WICED_BT_TRACE("NVRAM write:id:%d bytes:%d dev: [%B]\n", nvram_id, bytes_written, p_event_data->paired_device_link_keys_update.bd_addr);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        WICED_BT_TRACE( "BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT \n" );

        /* read existing key from the NVRAM  */

        WICED_BT_TRACE("\t\tfind device %B\n", p_event_data->paired_device_link_keys_request.bd_addr);

        if ((nvram_id = hci_control_find_nvram_id(p_event_data->paired_device_link_keys_request.bd_addr,
                        BD_ADDR_LEN)) != 0)
        {
            bytes_read = hci_control_read_nvram(nvram_id, &p_event_data->paired_device_link_keys_request,
                                                sizeof(wiced_bt_device_link_keys_t));

            result = WICED_BT_SUCCESS;
            WICED_BT_TRACE("Read:nvram_id:%d bytes:%d\n", nvram_id, bytes_read);
        } else
        {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE("Key retrieval failure\n");
        }
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        /* Use the default security for BLE */
        WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT bda %B\n",
                       p_event_data->pairing_io_capabilities_ble_request.bd_addr);
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_MITM_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size = 16;
        p_event_data->pairing_io_capabilities_ble_request.init_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
        /* Use the default security for BR/EDR*/
        WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT bda %B\n", p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
        p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_br_edr_request.auth_req = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
        p_event_data->pairing_io_capabilities_br_edr_request.oob_data = BTM_OOB_NONE;
        break;

    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        WICED_BT_TRACE( "BTM_USER_CONFIRMATION_REQUEST_EVT \n" );

        wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        WICED_BT_TRACE( "BTM_ENCRYPTION_STATUS_EVT \n" );

        p_encryption_status = &p_event_data->encryption_status;
        WICED_BT_TRACE( "Encryption Status:(%B) res:%d\n", p_encryption_status->bd_addr, p_encryption_status->result );
    default:
        break;
    }
    return result;
}

/*
 *  Application Start, ie, entry point to the application.
 */
APPLICATION_START()
{
#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT
    wiced_transport_init( &transport_cfg );

    //Set the debug uart to WICED_ROUTE_DEBUG_TO_DBG_UART enable the debug traces over debug UART
    //wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_WICED_UART);

    //Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#ifdef CYW20706A2
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif

    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    //wiced_set_debug_uart( WICED_ROUTE_DEBUG_NONE );

    // WICED_ROUTE_DEBUG_TO_WICED_UART to send debug strings over the WICED debug interface */
    //wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    wiced_transport_init(&transport_cfg);

    WICED_BT_TRACE( "Starting OPP Server Application...\n" );

    /* Initialize Bluetooth stack */
    wiced_bt_stack_init(opp_server_management_callback, &opp_server_cfg_settings, opp_server_cfg_buf_pools);

    transport_pool = wiced_transport_create_buffer_pool( TRANS_UART_BUFFER_SIZE,
                     TRANS_UART_BUFFER_COUNT);
}
