/*
 * Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
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
#ifndef _SPP_MULTI_PORT_H
#define _SPP_MULTI_PORT_H

#include "wiced_transport.h"

/* Service channel numbers for SPP Servers */
#define SPP_RFCOMM_SCN1                 2
#define SPP_RFCOMM_SCN2                 4

/* Max TX packet to be sent over SPP */
#define MAX_TX_BUFFER               1017
#define MAX_TX_RETRY                60

/* Transport (UART) connection */
#define TRANS_MAX_BUFFERS                   10
#define TRANS_UART_BUFFER_SIZE              1024

typedef enum
{
	INDEX_TYPE_STATE_IDLE,
	INDEX_TYPE_BDADDR,
    INDEX_TYPE_PORT_HANDLE,
} spp_client_index_type_t;

typedef struct
{
	wiced_bt_device_address_t spp_remote_bda;
	uint16_t                  spp_handle;
	wiced_bool_t              spp_is_connected;
	uint32_t                  spp_total_bytes_received; // number of bytes received during the connection
} spp_clients_t;

extern wiced_transport_buffer_pool_t* host_trans_pool;;

void hci_spp_init();
void hci_spp_handle_command(uint16_t cmd_opcode, uint8_t* p, uint32_t data_len);
void hci_spp_disconnect(BD_ADDR bda);

extern void wiced_bt_trace_array( const char *string, const uint8_t* array, const uint16_t len );
extern void wiced_bt_spp_rx_flow_enable(uint16_t handle, wiced_bool_t enable);

#endif
