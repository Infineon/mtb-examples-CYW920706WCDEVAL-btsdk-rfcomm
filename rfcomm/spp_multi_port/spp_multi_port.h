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
