/*******************************************************************************
* Title                 :   NRF24L01 driver header file
* Filename              :   nrf24l01.h
* Author                :   Nhan
* Origin Date           :   Oct 17th 2021
* Notes                 :   None
*******************************************************************************/
#ifndef NRF24L01_H_
#define NRF24L01_H_

/******************************************************************************
* INCLUDES
*******************************************************************************/
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "log_debug.h"
#include "cmsis_os2.h"

/******************************************************************************
 * CONFIGURATION CONSTANTS
 *******************************************************************************/
#define NRF24L01_RX_MODE                        (0)
#define NRF24L01_TX_MODE                        (1)
#define NRF24L01_EXEC_MODE                      (NRF24L01_TX_MODE)
#define NRF24L01_ADDR_WIDTH                     (3)
#define NRF24L01_STATIC_PAYLOAD_LEN             (3)
#define NRF24L01_TOTAL_PIPES                    (6)
#define NRF24L01_PAYLOAD_HEADER_1               (0xAB)
#define NRF24L01_PAYLOAD_HEADER_2               (0xBA)
#define NRF24L01_MAX_NUM_PACKET                 (32)
#define NRF24L01_PACKET_MAX_SIZE                (32)
#define NRF24L01_RX_ACK_PAYLOAD                 "AcKPaYlOaD"

// DEFINITION OF CONFIGURATION FOR NRF24L01
// Exit power down mode
#define NEF24L01_EXIT_POWER_DONW_MASK           (~(0x02))
#define NEF24L01_EXIT_POWER_DONW_ENABLE         (0x00)
#define NEF24L01_EXIT_POWER_DONW_DISABLE        (0x02)

// Exit power down mode
#define NEF24L01_TXRX_CTRL_MASK                 (~(0x01))
#define NEF24L01_TXRX_CTRL_RX_ENABLE            (0x01)
#define NEF24L01_TXRX_CTRL_TX_ENABLE            (0x00)

// On air data rate
#define NEF24L01_AIR_DR_LOW_MASK                (~(0x20))
#define NEF24L01_AIR_DR_HIGH_MASK               (~(0x08))
#define NEF24L01_AIR_DR_MASK                    (NEF24L01_AIR_DR_LOW_MASK & NEF24L01_AIR_DR_HIGH_MASK)
#define NRF24L01_AIR_DR_1MBPS                   (0x00)
#define NRF24L01_AIR_DR_2MBPS                   (0x08)
#define NRF24L01_AIR_DR_250kBPS                 (0x10)

// Set RF output power in TX mode
#define NEF24L01_RF_TX_PWR_MASK                 (~(0x06))
#define NRF24L01_TX_PWR_NEG_18dBm               (0x00)
#define NRF24L01_TX_PWR_NEG_12dBm               (0x02)
#define NRF24L01_TX_PWR_NEG_6dBm                (0x04)
#define NRF24L01_TX_PWR_0dBm                    (0x06)

// Set the RF channel frequency
#define NRF24L01_CHANNEL_FREQ_MASK              (~(0x80))   

// Set Adrress width for RX and TX device
#define NRF24L01_ADDRESS_WIDTH_MASK             (~(0x03))   

// Set the number of bytes in RX payload in data pipe 0
// This configuration is used in Static payload length
#define NRF24L01_RX_NUM_PAYLOAD_MASK            (~(0x3F))     

// Enable Dynamic payload length 
#define NRF24L01_DYNAMIC_PAYLOAD_MASK           (~(0x04))     
#define NRF24L01_DYNAMIC_PAYLOAD_ENABLE         (0x04)     
#define NRF24L01_DYNAMIC_PAYLOAD_DISABLE        (0x00)     
#define NRF24L01_DYNAMIC_PAYLOAD_P0_MASK        (~(0x01))  

// Enable interrupt mask
#define NRF24L01_INT_RX_DR_MASK                 (~(0x40))   
#define NRF24L01_INT_RX_DR_ENABLE               (0x00)   
#define NRF24L01_INT_RX_DR_DISABLE               (0x40)   

#define NRF24L01_INT_TX_DS_MASK                 (~(0x20))     
#define NRF24L01_INT_TX_DS_ENABLE               (0x00)   
#define NRF24L01_INT_TX_DS_DISABLE              (0x20)   

#define NRF24L01_INT_MAX_RT_MASK                (~(0x10))     
#define NRF24L01_INT_MAX_RT_ENABLE              (0x00)  
#define NRF24L01_INT_MAX_RT_DISABLE             (0x10)   

// Available Pipes fro reding mask
#define NRF24L01_AVAILABLE_PIPES_MASK           (0xE0)  

/******************************************************************************
 * MACROS
 *******************************************************************************/


/******************************************************************************
 * TYPEDEFS
 *******************************************************************************/
typedef enum {
    NRF24L01_DATA_PIPE_0 = 0x00,
    NRF24L01_DATA_PIPE_1,
    NRF24L01_DATA_PIPE_2,
    NRF24L01_DATA_PIPE_3,
    NRF24L01_DATA_PIPE_4,
    NRF24L01_DATA_PIPE_5,
    NRF24L01_MAX_DATA_PIPE
} rx_data_pipe_t;

typedef enum {
    NRF24L01_DR_1MBPS = 0x00,
    NRF24L01_DR_2MBPS,
    NRF24L01_DR_250kBPS,
    NRF24L01_DR_INVALID
} nrf24l01_dr_t;

typedef enum {
    NRF24L01_ADDR_WIDTH_INVALID = 0x00,
    NRF24L01_ADDR_WIDTH_3BYTES,
    NRF24L01_ADDR_WIDTH_4BYTES,
    NRF24L01_ADDR_WIDTH_5BYTES
} nrf24l01_aw_t;

typedef struct {
    uint32_t payload_size;
    bool is_dynamic_payload_enable;
} nrf24l01_instance_t;

/******************************************************************************
 * VARIABLES DEFINITIONS
 *******************************************************************************/


/******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************/
void nrf24l01_init(void);
void nrf24l01_deinit(void);
bool nrf24l01_read_data_fifo(uint8_t * pdata, uint8_t * len);
void nrf24l01_send_data_fifo(uint8_t* pdata, uint8_t len);
void nrf24l01_flush_tx_fifo(void);
void nrf24l01_flush_tx_fifo(void);
void nrf24l01_flush_rx_fifo(void);
void nrf24l01_enter_power_down_mode(bool en);
void nrf24l01_enter_stanby_I_mode(void);
void nrf24l01_enter_rx_mode(void);
void nrf24l01_exit_rx_mode(void);
void nrf24l01_pre_condition_enter_tx_mode(void);
void nrf24l01_txrx_control(uint8_t mode);
void nrf24l01_set_data_rate(nrf24l01_dr_t data_rate);
void nrf24l01_set_transmit_power(uint8_t tx_pwr);
void nrf24l01_set_rf_channel_freq(uint8_t value);
void nrf24l01_enable_rx_address(rx_data_pipe_t data_pipe);
void nrf24l01_set_address_width(nrf24l01_aw_t aw);
void nrf24l01_set_rx_address(rx_data_pipe_t data_pipe, uint8_t * paddr, uint8_t len);
void nrf24l01_set_tx_address(uint8_t * paddr, uint8_t len);
void nrf24l01_set_static_payload_length(rx_data_pipe_t data_pipe, uint8_t len);
uint8_t nrf24l01_num_available_data_fifo(void); 
void nrf24l01_enable_interrupt(uint8_t int_type);
void nrf24l01_data_ready_callback(void);
void nrf24l01_data_wait_new_data(void);
void nrf24l01_set_payload_size(rx_data_pipe_t data_pipe, uint32_t size);
void nrf24l01_enable_dynamic_payloads(void);
void nrf24l01_enable_ack_payload(void);
void nrf24l01_print_all_configurations(void);
uint8_t nrf24l01_read_fifo_status(void);
void nrf24l01_clear_all_flags(void);
void nrf24l01_write_back_ack_payload(rx_data_pipe_t pipe, uint8_t* pdata, uint8_t len);

#endif /* NRF24L01_H_ */
/*** END OF FILE **************************************************************/
