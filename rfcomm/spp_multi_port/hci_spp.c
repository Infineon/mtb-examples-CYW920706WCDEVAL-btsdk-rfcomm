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
 * This file implement serial port profile application controlled over UART
 *
 */
#include "wiced.h"
#include "wiced_gki.h"
#include "wiced_bt_dev.h"
#include "hci_control_api.h"
#include "wiced_bt_trace.h"
#include "spp_multi_port.h"
#include "wiced_bt_spp.h"
#include "wiced_timer.h"
#include "wiced_transport.h"
#include "string.h"

static void         spp_connection_up_callback(uint16_t handle, uint8_t* bda);
static void         spp_connection_down_callback(uint16_t handle);
static void         spp_connection_failed_callback(void);
static void         spp_service_not_found_callback(void);
static wiced_bool_t spp_rx_data_callback(uint16_t handle, uint8_t* p_data, uint32_t data_len);
static void         spp_tx_complete_callback(uint16_t handle, wiced_result_t result);
static void         spp_transport_flow_control_timeout(uint32_t param);
static void         spp_ota_flow_control_timeout(uint32_t param);
static spp_clients_t* spp_get_client_pointer( spp_client_index_type_t index, uint8_t* bda, uint16_t handle );

#define MAX_NUM_OF_SPP_CLIENTS      2

/* SPP SEVER1 */
wiced_bt_spp_reg_t spp_reg1 =
{
    SPP_RFCOMM_SCN1,                    /* RFCOMM service channel number for SPP connection */
    MAX_TX_BUFFER,                      /* RFCOMM MTU for SPP connection */
    spp_connection_up_callback,         /* SPP connection established */
    spp_connection_failed_callback,     /* SPP connection establishment failed */
    spp_service_not_found_callback,     /* SPP service not found */
    spp_connection_down_callback,       /* SPP connection disconnected */
    spp_rx_data_callback,               /* Data packet received */
};

/* SPP SEVER2 - can use same callbacks or define different ones depending on your application */
wiced_bt_spp_reg_t spp_reg2 =
{
    SPP_RFCOMM_SCN2,                    /* RFCOMM service channel number for SPP connection */
    MAX_TX_BUFFER,                      /* RFCOMM MTU for SPP connection */
    spp_connection_up_callback,         /* SPP connection established */
    spp_connection_failed_callback,     /* SPP connection establishment failed */
    spp_service_not_found_callback,     /* SPP service not found */
    spp_connection_down_callback,       /* SPP connection disconnected */
    spp_rx_data_callback,               /* Data packet received */
};

spp_clients_t   spp_clients[MAX_NUM_OF_SPP_CLIENTS];

wiced_timer_t   spp_transport_flow_control_timer;
wiced_timer_t   spp_ota_flow_control_timer;
wiced_bool_t    spp_ota_flow_controlled;  // TRUE if we were not able to send data OTA
uint16_t        spp_pending_tx_handle;
uint16_t        spp_pending_rx_handle;

uint32_t spp_tx_retry_count = 0;

/*
 * SPP initialization
 */
void hci_spp_init()
{
    // Initialize 1st SPP Server
    wiced_bt_spp_startup(&spp_reg1);

    // Initialize 2nd SPP Server
    wiced_bt_spp_startup(&spp_reg2);

    wiced_init_timer(&spp_transport_flow_control_timer, spp_transport_flow_control_timeout, 0, WICED_MILLI_SECONDS_TIMER);
    wiced_init_timer(&spp_ota_flow_control_timer, spp_ota_flow_control_timeout, 0, WICED_MILLI_SECONDS_TIMER);
}

/*
 * Handle various SPP commands received over UART
 */
void hci_spp_handle_command(uint16_t cmd_opcode, uint8_t* p, uint32_t data_len)
{
    uint16_t handle;
    wiced_result_t result = WICED_BT_ERROR;

    switch (cmd_opcode)
    {
    case HCI_CONTROL_SPP_COMMAND_CONNECT:
        wiced_bt_spp_connect(p);
        break;

    case HCI_CONTROL_SPP_COMMAND_DISCONNECT:
        handle = p[0] | (p[1] << 8);
        wiced_bt_spp_disconnect(handle);
        break;

    case HCI_CONTROL_SPP_COMMAND_DATA:
        // WICED_BT_TRACE("COMMAND_DATA handle:%d length:%d\n", handle, data_len - 2);

        if (data_len > SPP_MAX_PACKET_SIZE)
        {
            WICED_BT_TRACE("too many bytes\n");
            return;
        }

//	    wiced_bt_get_buffer_usage (buffer_stats, sizeof(buffer_stats));

//	    WICED_BT_TRACE("0:%d/%d 1:%d/%d 2:%d/%d 3:%d/%d\n", buffer_stats[0].current_allocated_count, buffer_stats[0].max_allocated_count,
//	                   buffer_stats[1].current_allocated_count, buffer_stats[1].max_allocated_count,
//	                   buffer_stats[2].current_allocated_count, buffer_stats[2].max_allocated_count,
//	                   buffer_stats[3].current_allocated_count, buffer_stats[3].max_allocated_count);

            // first 2 bytes should be set by the application with the sessio
        handle = p[0] | (p[1] << 8);

        if (!wiced_bt_spp_send_session_data(handle, p + 2, data_len - 2))
        {
            WICED_BT_TRACE("Cannot accept tx packet - NAK\n");
            spp_tx_complete_callback(handle, 1);
            break;
        }

        // check if there is place for more data, if true tell the MCU that the buffer is done
        // otherwise start flow control timer, to wait until buffer becomes available
        if (wiced_bt_spp_can_send_more_data(handle))
            spp_tx_complete_callback(handle, 0);
        else
        {
            spp_ota_flow_controlled = WICED_TRUE;
            spp_pending_tx_handle = handle;
            result  = wiced_start_timer(&spp_ota_flow_control_timer, 50);
            WICED_BT_TRACE("hci_spp_handle_command wiced_start_timer result - %d \n", result);
            spp_tx_retry_count = 0;
        }
        break;

    default:
        WICED_BT_TRACE("hci_spp_handle_command:unknown opcode %x\n", cmd_opcode);
        break;
    }
}

/*
 * SPP connection up callback
 */
void spp_connection_up_callback(uint16_t handle, uint8_t* bda)
{
	spp_clients_t* p_spp_client;
    uint8_t   tx_buf[10];
    uint8_t  *p = tx_buf;
    int       i;

    WICED_BT_TRACE("%s handle:%d address:%B\n", __FUNCTION__, handle, bda);

    p_spp_client = spp_get_client_pointer(INDEX_TYPE_STATE_IDLE, NULL, 0);

    for (i = 0; i < BD_ADDR_LEN; i++)
        *p++ = bda[BD_ADDR_LEN - 1 - i];

    WICED_BT_TRACE("%s handle:%d address:%B\n", __FUNCTION__, handle, bda);

    if(p_spp_client)
    {
	p_spp_client->spp_handle = handle;
	p_spp_client->spp_is_connected = WICED_TRUE;
	p_spp_client->spp_total_bytes_received = 0;
	memcpy(p_spp_client->spp_remote_bda, bda, BD_ADDR_LEN);
    }

    spp_ota_flow_controlled   = WICED_FALSE;

    *p++ = (handle & 0xff);
    *p++ = ((handle >> 8) & 0xff);

    wiced_transport_send_data(HCI_CONTROL_SPP_EVENT_CONNECTED, tx_buf, (int)(p - tx_buf));
}

/*
 * Connection failed indication from the library
 */
void spp_connection_failed_callback(void)
{
    uint8_t   tx_buf[10];
    uint8_t  *p = tx_buf;

    WICED_BT_TRACE("%s\n", __FUNCTION__);

    *p++ = 0;
    *p++ = 0;

    wiced_transport_send_data(HCI_CONTROL_SPP_EVENT_CONNECTION_FAILED, tx_buf, (int)(p - tx_buf));
}

/*
 * Service not found failed indication from the library
 */
void spp_service_not_found_callback(void)
{
    uint8_t   tx_buf[10];
    uint8_t  *p = tx_buf;

    WICED_BT_TRACE("%s\n", __FUNCTION__);

    *p++ = 0;
    *p++ = 0;

    wiced_transport_send_data(HCI_CONTROL_SPP_EVENT_SERVICE_NOT_FOUND, tx_buf, (int)(p - tx_buf));
}

/*
 * Send a tx complete indication to the server
 */
void spp_tx_complete_callback(uint16_t handle, wiced_result_t result)
{
    uint8_t  tx_buf[6];
    uint8_t *p = tx_buf;

    *p++ = (handle & 0xff);
    *p++ = ((handle >> 8) & 0xff);
    *p++ = result;

    WICED_BT_TRACE("spp_send_tx_complete - %d handle %d result\n", handle, result);
    wiced_transport_send_data(HCI_CONTROL_SPP_EVENT_TX_COMPLETE, tx_buf, (int)(p - tx_buf));
}


/*
 * SPP connection down callback
 */
void spp_connection_down_callback(uint16_t handle)
{
	spp_clients_t* p_spp_client;
    uint8_t   tx_buf[10];
    uint8_t  *p = tx_buf;

    WICED_BT_TRACE("%s handle:%d\n", __FUNCTION__, handle);

    p_spp_client = spp_get_client_pointer(INDEX_TYPE_PORT_HANDLE, NULL, handle);

    wiced_stop_timer(&spp_transport_flow_control_timer);

    // TBD uncomment when shim is fixed
    //wiced_deinit_timer(&spp_transport_flow_control_timer);

    wiced_stop_timer(&spp_ota_flow_control_timer);
    // TBD uncomment when shim is fixed
    //wiced_deinit_timer(&spp_ota_flow_control_timer);

    if(p_spp_client)
    {
		p_spp_client->spp_handle = 0;
		p_spp_client->spp_is_connected = WICED_FALSE;
		p_spp_client->spp_total_bytes_received = 0;
    }

    *p++ = (handle & 0xff);
    *p++ = ((handle >> 8) & 0xff);

    wiced_transport_send_data(HCI_CONTROL_SPP_EVENT_DISCONNECTED, tx_buf, (int)(p - tx_buf));
}

/*
 * Process data received over EA session.  Return TRUE if we were able to allocate buffer to
 * deliver to the host.
 */
wiced_bool_t spp_rx_data_callback(uint16_t handle, uint8_t* p_data, uint32_t data_len)
{
	spp_clients_t* p_spp_client;
    uint8_t *p_trans_buf;
    uint16_t buffer_size = wiced_transport_get_buffer_size(host_trans_pool);
    int      num_buffers_left;

//    wiced_bt_buffer_statistics_t buffer_stats[4];

//    wiced_bt_get_buffer_usage (buffer_stats, sizeof(buffer_stats));

//    WICED_BT_TRACE("0:%d/%d 1:%d/%d 2:%d/%d 3:%d/%d\n", buffer_stats[0].current_allocated_count, buffer_stats[0].max_allocated_count,
//                   buffer_stats[1].current_allocated_count, buffer_stats[1].max_allocated_count,
//                   buffer_stats[2].current_allocated_count, buffer_stats[2].max_allocated_count,
//                   buffer_stats[3].current_allocated_count, buffer_stats[3].max_allocated_count);

//    wiced_result_t wiced_bt_get_buffer_usage (&buffer_stats, sizeof(buffer_stats));
    p_spp_client = spp_get_client_pointer(INDEX_TYPE_PORT_HANDLE, NULL, handle);

    p_spp_client->spp_total_bytes_received += data_len;

    WICED_BT_TRACE("%s handle:%d len:%d total:%d %02x-%02x\n", __FUNCTION__, handle, data_len,
		                      p_spp_client->spp_total_bytes_received, p_data[0], p_data[data_len - 1]);

    if (data_len <= buffer_size - 2)
    {
        if ((p_trans_buf = (uint8_t*) wiced_transport_allocate_buffer(host_trans_pool)) != NULL)
        {
            p_trans_buf[0] = handle & 0x0ff;
            p_trans_buf[1] = (handle >> 8) & 0x0ff;

            memcpy(&p_trans_buf[2], p_data, data_len);
            if(wiced_transport_send_buffer(HCI_CONTROL_SPP_EVENT_RX_DATA, p_trans_buf, data_len + 2) != WICED_SUCCESS)
                WICED_BT_TRACE("wiced_transport_send_buffer failed length %d\n", data_len + 2);

            if ((num_buffers_left = wiced_transport_get_buffer_count(host_trans_pool)) < (TRANS_MAX_BUFFERS / 2))
            {
                // time to tell library that we do not want anymore data
                wiced_bt_spp_rx_flow_enable (handle, WICED_FALSE);

                spp_pending_rx_handle = handle;
                wiced_start_timer(&spp_transport_flow_control_timer, 50);
                WICED_BT_TRACE("SPP flow disabled buffers left:%d\n", num_buffers_left);
            }
            return WICED_TRUE;
        }
        else
        {
            WICED_BT_TRACE("spp_rx_data_callback: Buffer Not Available\n");
            return WICED_FALSE;
        }
    }
    else
    {
        WICED_BT_TRACE("spp_rx_data_callback: Buffer %u longer than transport pool buffers %u!\n", data_len, buffer_size);
        return WICED_TRUE;
    }
}

/*
 * disconnect device with specified BDADDR if connected
 */
void hci_spp_disconnect(BD_ADDR bda)
{
	spp_clients_t* p_spp_client;

    p_spp_client = spp_get_client_pointer(INDEX_TYPE_BDADDR, bda, 0);
    if(p_spp_client)
    {
	WICED_BT_TRACE("Disconnecting from %B handle %d\n", p_spp_client->spp_remote_bda, p_spp_client->spp_handle);
	wiced_bt_spp_disconnect(p_spp_client->spp_handle);
    }
}

/*
 * The timeout function is periodically called if we are not able to send data over UART
 */
void spp_transport_flow_control_timeout(uint32_t param)
{
    int num_buffers_left;

    //Disable the flow control if 3/4 buffers available
    if ((num_buffers_left = wiced_transport_get_buffer_count(host_trans_pool)) > ((TRANS_MAX_BUFFERS * 3) / 4))
    {
        // time to tell library that we can accept data again
        wiced_bt_spp_rx_flow_enable (spp_pending_rx_handle, WICED_TRUE);
        WICED_BT_TRACE("SPP flow enabled buffers left:%d\n", num_buffers_left);
    }
    else
    {
        WICED_BT_TRACE("SPP flow still disabled left:%d\n", num_buffers_left);
        wiced_start_timer(&spp_transport_flow_control_timer, 50);
    }
}

/*
* The timeout function is periodically called if we are not able to send data OTA
*/
void spp_ota_flow_control_timeout(uint32_t param)
{
    WICED_BT_TRACE("%s spp_pending_tx_handle:%d spp_ota_flow_controlled:%d\n", __FUNCTION__, spp_pending_tx_handle, spp_ota_flow_controlled);

    if (spp_ota_flow_controlled)
    {
        // check if there is place for more data, if true tell the MCU that the buffer is done
        // otherwise start flow control timer, to wait until buffer becomes available
        if (wiced_bt_spp_can_send_more_data(spp_pending_tx_handle))
        {
            WICED_BT_TRACE("spp_ota_flow_control_timeout - wiced_bt_spp_can_send_more_data FALSE");
            spp_ota_flow_controlled = WICED_FALSE;
            spp_tx_complete_callback(spp_pending_tx_handle, 0);
        }
        else
        {
            WICED_BT_TRACE("wiced_start_timer");
            wiced_start_timer(&spp_ota_flow_control_timer, 50);
            spp_tx_retry_count++;

            if(spp_tx_retry_count >= MAX_TX_RETRY)
            {
		WICED_BT_TRACE("Purging port handle %d\n", spp_pending_tx_handle);
		wiced_bt_spp_port_purge(spp_pending_tx_handle, PORT_PURGE_TXCLEAR);
            }
        }
    }
}

spp_clients_t* spp_get_client_pointer( spp_client_index_type_t index, uint8_t* bda, uint16_t handle )
{
	uint8_t i;

	for( i=0; i<MAX_NUM_OF_SPP_CLIENTS; i++)
	{
		if( ((index == INDEX_TYPE_STATE_IDLE) && (!spp_clients[i].spp_is_connected     )) ||
            ((index == INDEX_TYPE_PORT_HANDLE) && (handle == spp_clients[i].spp_handle )) ||
            ((index == INDEX_TYPE_BDADDR) && (memcmp(bda,spp_clients[i].spp_remote_bda,BD_ADDR_LEN))==0) )
			return &spp_clients[i];
	}
	return NULL;
}
