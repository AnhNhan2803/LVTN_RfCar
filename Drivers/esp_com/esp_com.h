/*******************************************************************************
* Title                 :   esp communication header file
* Filename              :   esp_com.h
* Author                :   Nhan
* Origin Date           :   09/10/2021
* Notes                 :   None
*******************************************************************************/

#ifndef ESP_COM_H_
#define ESP_COM_H_

/******************************************************************************
* INCLUDES
*******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "cmsis_os2.h"

/******************************************************************************
* CONFIGURATION CONSTANTS
*******************************************************************************/
#define ESP_COM_HEADER1        (0xAB)
#define ESP_COM_HEADER2        (0xBA)
#define ESP_COM_HEADER_SIZE    (2)
#define ESP_COM_RX_MAX_PACKET_SIZE (30)
#define ESP_COM_RX_MAX_NUM_SSID (30)
#define ESP_COM_TRANSFER_TIMEOUT (500)

#define FIFO_BUF_DEF(pbuff ,size , element)  \
    uint8_t pbuff##data[size * element];     \
    fifo_buf_t pbuff = {                     \
        .buffer = pbuff##data,               \
        .head = 0,                           \
        .tail = 0,                           \
        .maxlen = size * element,            \
        .elem = element,                     \
        .cnt = 0                             \
    }
    
/******************************************************************************
* MACROS
*******************************************************************************/
typedef struct {
    uint8_t is_rx_enter_frame;
    uint8_t rx_char;
    uint8_t rx_idx;
    uint8_t rx_len;
    uint8_t payload_len;
    uint8_t rx_buff[64];
} esp_com_rx_params_t;

typedef struct {
    uint8_t ssid_len;
    // Plus one byte of true/false to indicate that
    // there are available ssid need to transmit
    uint8_t ssid[ESP_COM_RX_MAX_PACKET_SIZE];
} esp_com_ssid_t;

/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void esp_com_init(void);
void esp_com_deinit(void);
uint8_t esp_com_get_current_data(uint8_t * pdata);
bool esp_com_transmit_data(uint8_t * data, uint8_t len);
bool esp_com_put_data_into_queue(uint8_t * pdata);
bool esp_com_get_data_from_queue(uint8_t * pdata);
void esp_com_wait_sem(void);
void esp_com_release_sem(void);

#endif // ESP_COM_H_
/*** END OF FILE **************************************************************/
