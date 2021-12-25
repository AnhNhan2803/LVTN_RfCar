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

/******************************************************************************
* CONFIGURATION CONSTANTS
*******************************************************************************/
#define NRF24L01_MSG_SIZE            (NRF24L01_STATIC_PAYLOAD_LEN)
#define NRF24L01_MAX_NUM_MSG         (20)
#define NRF24L01_QUEUE_MSG_SIZE      (NRF24L01_MSG_SIZE * NRF24L01_MAX_NUM_MSG)

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/


/******************************************************************************
* MACROS
*******************************************************************************/


/******************************************************************************
* TYPEDEFS
*******************************************************************************/
typedef struct {
    uint8_t data[NRF24L01_STATIC_PAYLOAD_LEN];
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
