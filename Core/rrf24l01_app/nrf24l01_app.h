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

/******************************************************************************
* CONFIGURATION CONSTANTS
*******************************************************************************/
#define NRF24L01_MSG_SIZE            (NRF24L01_PACKET_MAX_SIZE)
#define NRF24L01_MAX_NUM_MSG         (20)
#define NRF24L01_QUEUE_MSG_SIZE      (NRF24L01_MSG_SIZE * NRF24L01_MAX_NUM_MSG)

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
osStatus_t nrf24l01_get_chunk_data(nrf24l01_data_t * pdata);

#endif // NRF24L01_APP_H_
/*** END OF FILE **************************************************************/
