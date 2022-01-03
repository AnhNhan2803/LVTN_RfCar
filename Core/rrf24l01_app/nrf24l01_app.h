/*******************************************************************************
* Title                 :   NRF24L01 application header file
* Filename              :   nrf24l01.h
* Author                :   Nhan
* Origin Date           :   Oct 17th 2021
* Notes                 :   None
*******************************************************************************/

#ifndef NRF24L01_APP_H_
#define NRF24L01_APP_H_

/******************************************************************************
* INCLUDES
*******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "nrf24l01.h"
#include "log_debug.h"
#include "esp_com.h"
#include "esp_app.h"
#include "usbd_cdc_if.h"

/******************************************************************************
* CONFIGURATION CONSTANTS
*******************************************************************************/
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
#define NRF24L01_MSG_SIZE            (NRF24L01_PACKET_MAX_SIZE)
#else
#define NRF24L01_MSG_SIZE            (30)
#endif
#define NRF24L01_MAX_NUM_MSG         (30)
#define NRF24L01_QUEUE_MSG_SIZE      (NRF24L01_MSG_SIZE * NRF24L01_MAX_NUM_MSG)

// Send 2 times to ensure that the ACK payload 
// packet is received successfully
#define NRF24L01_MAX_NUM_SEND_ORDER  (2)

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define RF_CMD_CAR_CTRL_MOVE_FORWARD  (0x00)
#define RF_CMD_CAR_CTRL_MOVE_BACKWARD (0x01)
#define RF_CMD_CAR_CTRL_TURN_LEFT     (0x02)
#define RF_CMD_CAR_CTRL_TURN_RIGHT    (0x03)
#define RF_CMD_CAR_CTRL_BACK_LEFT     (0x04)
#define RF_CMD_CAR_CTRL_BACK_RIGHT    (0x05)
#define RF_CMD_CAR_CTRL_ROTATE_LEFT   (0x06)
#define RF_CMD_CAR_CTRL_ROTATE_RIGHT  (0x07)
#define RF_CMD_CAR_CTRL_STOP          (0x08)

#define RF_CMD_GET_WIFI_APS           (0xA0)

#define RF_CMD_CONNECT                (0xA0)
#define RF_CMD_DISCONNECT             (0xA1)
#define RF_CMD_GET_AVAILABLE_SSID     (0xA2)
#define RF_CMD_SET_SSID               (0xA3)
#define RF_CMD_GET_IP                 (0xA4)
#define RF_CMD_GET_RSSI               (0xA5)

#define RF_CMD_RESPONSE_ACK           (0xB0)

#define RF_CMD_HEADER1                (0xAB)
#define RF_CMD_HEADER2                (0xBA)
#define RF_CMD_HEADER_SIZE            (2)
#define RF_CMD_CMD_SIZE               (1)
#define RF_CMD_PAYLOAD_LEN_SIZE       (1)

#define RF_CMD_RX_MAX_PACKET_SIZE     (30)
#define RF_CMD_RX_MAX_NUM_SSID        (30)
#define RF_CMD_TRANSFER_TIMEOUT       (500)
#define RF_TRANSFER_READY_TIMEOUT_MS  (50)

/******************************************************************************
* MACROS
*******************************************************************************/


/******************************************************************************
* TYPEDEFS
*******************************************************************************/
typedef struct {
    uint8_t data[NRF24L01_MAX_NUM_PACKET];
} nrf24l01_data_t;

/******************************************************************************
* VARIABLES
*******************************************************************************/


/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void nrf24l01_thread_app_init(void);
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
osStatus_t nrf24l01_get_chunk_data(nrf24l01_data_t * pdata);
#else
bool nrf24l01_put_data_into_tx_queue(uint8_t * data);
bool nrf24l01_get_data_from_tx_queue(uint8_t * data);
#endif

#endif // NRF24L01_APP_H_
/*** END OF FILE **************************************************************/
