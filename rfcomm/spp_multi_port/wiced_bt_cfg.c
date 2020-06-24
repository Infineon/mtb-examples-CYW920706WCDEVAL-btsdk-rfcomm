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

/** @file
 *
 * Runtime Bluetooth stack configuration parameters
 *
 */

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"

/* Null-Terminated Local Device Name */
uint8_t BT_LOCAL_NAME[] = { 's','p','p','_','m','u','l','t','i','_','p','o','r','t','\0' };
const uint16_t BT_LOCAL_NAME_CAPACITY = sizeof(BT_LOCAL_NAME);


/*******************************************************************
 * wiced_bt core stack configuration
 ******************************************************************/
const wiced_bt_cfg_settings_t wiced_bt_cfg_settings =
{
    (uint8_t*)BT_LOCAL_NAME,                                                /**< Local device name (NULL terminated) */
    {0x00, 0x00, 0x00},                                                     /**< Local device class */
    BTM_SEC_IN_AUTHENTICATE | BTM_SEC_OUT_AUTHENTICATE | BTM_SEC_ENCRYPT,   /**< Security requirements mask (BTM_SEC_NONE, or combination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT */
    3,
    /* BR/EDR Scan Configuration */
    {
        BTM_SCAN_TYPE_STANDARD,                                             /**< Inquiry Scan Type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,                         /**< Inquiry Scan Interval (0 to use default) */
        WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW,                           /**< Inquiry Scan Window (0 to use default) */

        BTM_SCAN_TYPE_STANDARD,                                             /**< Page Scan Type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,                            /**< Page Scan Interval (0 to use default) */
        WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW,                              /**< Page Scan Window (0 to use default) */
    },

    /* BLE Scan Settings */
    {
        BTM_BLE_SCAN_MODE_NONE,                                             /**< BLE Scan Mode (BTM_BLE_SCAN_MODE_PASSIVE or BTM_BLE_SCAN_MODE_ACTIVE) */

        /* Advertisement Scan Configuration */
        0x0000,                                                             /**< High Duty Scan Interval */
        0x0000,                                                             /**< High Duty Scan Window */
        0,                                                                  /**< High Duty Scan Duration in seconds (0 for infinite) */

        0x0000,                                                             /**< Low Duty Scan Interval */
        0x0000,                                                             /**< Low Duty Scan Window */
        0,                                                                  /**< Low Duty Scan Duration in seconds (0 for infinite) */

        /* Connection Scan Configuration */
        0x0000,                                                             /**< High Duty Connection Cycle Connection Scan Interval */
        0x0000,                                                             /**< High Duty Connection Cycle Connection Scan Window */
        0,                                                                  /**< High Duty Connection Cycle Connection Duration in seconds (0 for infinite) */

        0x0000,                                                             /**< Low Duty Connection Cycle Connection Scan Interval */
        0x0000,                                                             /**< Low Duty Connection Cycle Connection Scan Window */
        0,                                                                  /**< Low Duty Connection Cycle Connection Duration in seconds (0 for infinite) */

        /* Connection Configuration */
        WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,                             /**< Minimum Connection Interval */
        WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,                             /**< Maximum Connection Interval */
        WICED_BT_CFG_DEFAULT_CONN_LATENCY,                                  /**< Connection Latency */
        WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT,                      /**< Connection Link Supervision Timeout */
    },

    /* BLE Advertisement Settings */
    {
        0,                                                                  /**< Advertising Channel Map (mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39) */

        0,                                                                  /**< High Duty Undirected Connectable Minimum Advertising Interval */
        0,                                                                  /**< High Duty Undirected Connectable Maximum Advertising Interval */
        0,                                                                  /**< High Duty Undirected Connectable Advertising Duration in seconds (0 for infinite) */

        0,                                                                  /**< Low Duty Undirected Connectable Minimum Advertising Interval */
        0,                                                                  /**< Low Duty Undirected Connectable Maximum Advertising Interval */
        0,                                                                  /**< Low Duty Undirected Connectable Advertising Duration in seconds (0 for infinite) */

        0,                                                                  /**< High Duty Directed Minimum Advertising Interval */
        0,                                                                  /**< High Duty Directed Maximum Advertising Interval */

        0,                                                                  /**< Low Duty Directed Minimum Advertising Interval */
        0,                                                                  /**< Low Duty Directed Maximum Advertising Interval */
        0,                                                                  /**< Low Duty Directed Advertising Duration in seconds (0 for infinite) */

        0,                                                                  /**< High Duty Non-Connectable Minimum Advertising Interval */
        0,                                                                  /**< High Duty Non-Connectable Maximum Advertising Interval */
        0,                                                                  /**< High Duty Non-Connectable Advertising Duration in seconds (0 for infinite) */

        0,                                                                  /**< Low Duty Non-Connectable Minimum Advertising Interval */
        0,                                                                  /**< Low Duty Non-Connectable Maximum Advertising Interval */
        0,                                                                  /**< Low Duty Non-Connectable Advertising Duration in seconds (0 for infinite) */
    },

    /* GATT Configuration */
    {
        APPEARANCE_GENERIC_COMPUTER,                                        /**< GATT Appearance */
        0,                                                                  /**< Client Config: Maximum number of servers that local client can connect to */
        0,                                                                  /**< Server Config: Maximum number of remote client connections allowed by local server */
        GATT_DEF_BLE_MTU_SIZE,                                              /**< Client Config: Maximum number of bytes that local client can receive over LE link */
#if !defined(CYW20706A2)
        (GATT_DEF_BLE_MTU_SIZE+5)                                           /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
#endif
    },

    /* RFCOMM Configuration */
    {
        2,                                                                  /**< Maximum Number of simultaneous RFCOMM ports */
        2,                                                                  /**< Maximum Number of simultaneous RFCOMM connections */
    },

    /* Application managed l2cap protocol configuration */
    {
        0,                                                          /**< Maximum number of application-managed l2cap links (BR/EDR and LE) */
        /* BR EDR l2cap configuration */
        0,                                                          /**< Maximum number of application-managed BR/EDR PSMs */
        0,                                                          /**< Maximum number of application-managed BR/EDR channels  */
        /* LE L2cap connection-oriented channels configuration */
        0,                                                          /**< Maximum number of application-managed LE PSMs */
        0,                                                          /**< Maximum number of application-managed LE channels */
#if !defined(CYW20706A2)
        0                                                           /**< Maximum number of application managed fixed channels supported (in addition to mandatory channels 4, 5 and 6). > */
#endif
    },

    /* Audio/Video Distribution configuration */
    {
        0,                                                          /**< Maximum simultaneous audio/video links */
#if !defined(CYW20706A2)
        0,                                                          /**< Maximum number of stream end points */
#endif
    },

    /* Audio/Video Remote Control configuration */
    {
        0,                                                          /**< Mask of local roles supported (AVRC_CONN_INITIATOR|AVRC_CONN_ACCEPTOR) */
        0,                                                          /**< Maximum simultaneous remote control links */
    },
    0,                                                              /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/
#ifdef CYW20706A2
    517,                                                            /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
    12                                                              /**< Max. power level of the device */
#else
    6,                                                              /**< Maximum number of buffer pools in p_btm_cfg_buf_pools and by wiced_create_pool */
    WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,               /**< Interval of  random address refreshing - secs */
    0,                                                              /**< Maximum number of white list devices allowed. Cannot be more than 128 */
#endif
#if defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20819A1) || defined (CYW20820A1)
    0                                                               /**< Default LE power level, Refer lm_TxPwrTable table for the power range */
#endif
};

/*******************************************************************
 * wiced_bt_stack buffer pool configuration
 *
 * Configure buffer pools used by the stack
 *
 * Pools must be ordered in increasing buf_size.
 * If a pools runs out of buffers, the next pool will be used.
 ******************************************************************/

const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS] =
{
/*  { buf_size, buf_count, }, */
    { 64,       12,        }, /* Small Buffer Pool */
    { 272,      6,         }, /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
    { 1056,     6,         }, /* Large Buffer Pool  (used for HCI ACL messages) */
    { 1056,     4,         }, /* Extra Large Buffer Pool (used for AVDT media packets and miscellaneous; if not needed, set buf_count to 0) */
};
