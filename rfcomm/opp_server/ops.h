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

#pragma once

#include "wiced_bt_cfg.h"
#include "hci_control_api.h"
#include "wiced_bt_ops_int.h"

#define HDLR_OPP_SERVER_UNIT                   0x10001
#define OPP_SERVER_SCN                         0x03

#define KEY_INFO_POOL_BUFFER_SIZE               145 //Size of the buffer used for holding the peer device key info
#define KEY_INFO_POOL_BUFFER_COUNT              10  //Corresponds to the number of peer devices

extern wiced_bt_cfg_settings_t opp_server_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t opp_server_cfg_buf_pools[];

extern uint32_t  hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length );
extern void      wiced_bt_trace_array( const char *string, const uint8_t* array, const uint16_t len );
extern void      GKI_freebuf (void *memPtr);
extern uint16_t  wiced_app_cfg_sdp_record_get_size(void);
extern int       hci_control_find_nvram_id(uint8_t *p_data, int len);
extern int       hci_control_alloc_nvram_id( );
extern int       hci_control_write_nvram( int nvram_id, int data_len, void *p_data, BOOLEAN from_host );
extern int       hci_control_read_nvram( int nvram_id, void *p_data, int data_len );
extern void      hci_control_delete_nvram( int nvram_id ,wiced_bool_t from_host);
extern void      hci_control_send_device_started_evt( void );
extern void      hci_control_send_pairing_completed_evt( uint8_t status , wiced_bt_device_address_t bdaddr );
extern void      wiced_bt_ops_close_hdlr();
extern void      wiced_bt_ops_access_rsp_hdlr(wiced_bt_op_oper_t oper, wiced_bt_op_access_t access);
extern const uint8_t opp_server_sdp_db[];

extern uint8_t pairing_allowed;
