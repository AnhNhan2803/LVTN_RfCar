/*******************************************************************************
* Title                 :   USB CDC app header file
* Filename              :   usb_cmd_app.h
* Author                :   Nhan
* Origin Date           :   Oct 19th 2021
* Notes                 :   None
*******************************************************************************/

#ifndef USB_CMD_APP_H_
#define USB_CMD_APP_H_

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
#include "usbd_cdc_if.h"
#include "nrf24l01_app.h"
#include "log_debug.h"

/******************************************************************************
* CONFIGURATION CONSTANTS
*******************************************************************************/
#define USB_CMD_RX_MSG_SIZE            (30)
#define USB_CMD_RX_MAX_NUM_MSG         (20)
#define USB_CMD_RX_QUEUE_MSG_SIZE      (USB_CMD_RX_MSG_SIZE * USB_CMD_RX_MAX_NUM_MSG)

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define USB_CMD_CONNECT                 (0xA0)
#define USB_CMD_DISCONNECT              (0xA1)
#define USB_CMD_GET_AVAILABLE_SSID      (0xA2)
#define USB_CMD_SET_SSID                (0xA3)
#define USB_CMD_GET_IP                  (0xA4)
#define USB_CMD_GET_RSSI                (0xA5)

#define USB_CMD_RESPONSE_ACK            (0xB0)

#define USB_CMD_HEADER1        (0xAB)
#define USB_CMD_HEADER2        (0xBA)
#define USB_CMD_HEADER_SIZE    (2)
#define USB_CMD_CMD_SIZE                (1)
#define USB_CMD_PAYLOAD_LEN_SIZE        (1)

#define USB_CMD_RX_MAX_PACKET_SIZE (30)
#define USB_CMD_RX_MAX_NUM_SSID (30)
#define USB_CMD_TRANSFER_TIMEOUT (500)

/******************************************************************************
* MACROS
*******************************************************************************/


/******************************************************************************
* TYPEDEFS
*******************************************************************************/


/******************************************************************************
* VARIABLES
*******************************************************************************/


/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void usb_app_thread_app_init(void);
void usb_cmd_put_data_to_rx_queue(uint8_t * data);
bool usb_cmd_get_data_from_rx_queue(uint8_t * data);
// uint8_t esp_com_get_rx_data(uint8_t * pdata, uint8_t * len);

#endif // USB_CMD_APP_H_
/*** END OF FILE **************************************************************/
