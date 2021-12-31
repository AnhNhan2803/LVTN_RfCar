/*******************************************************************************
* Title                 :   USBCMD app source file
* Filename              :   usb_cmd_app.c
* Author                :   Nhan
* Origin Date           :   Oct 19th 2021
* Notes                 :   None
*******************************************************************************/

/*******************************************************************************
* INCLUDE
*******************************************************************************/
#include "usb_cmd_app.h"

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define USB_CMD_RX_THREAD_NAME             ("usb_cmd_rx_com_thread")
#define USB_CMD_RX_THREAD_STACK_SIZE       (512) // Bytes
#define USB_CMD_RX_THREAD_PRIORITY         ((osPriority_t) osPriorityHigh)
#define USB_CMD_RX_QUEUE_SUB_NAME          ("esp_com_tx_queue")


#define USB_CMD_TX_THREAD_NAME             ("usb_cmd_tx_com_thread")
#define USB_CMD_TX_THREAD_STACK_SIZE       (512) // Bytes
#define USB_CMD_TX_THREAD_PRIORITY         ((osPriority_t) osPriorityHigh)

/******************************************************************************
* PREPROCESSOR MACROS
*******************************************************************************/
#define USB_CMD_CS_INIT()                 usb_cmd_app_mutex = osMutexNew(NULL);                                  
#define USB_CMD_CS_ENTER(timeout_ms)      osMutexAcquire(usb_cmd_app_mutex, timeout_ms)     // critical section enter
#define USB_CMD_CS_EXIT()                 osMutexRelease(usb_cmd_app_mutex)                 // critical section end

/******************************************************************************
* TYPEDEFS
*******************************************************************************/
typedef osMessageQueueId_t usb_cmd_rx_id_t;

/******************************************************************************
* VARIABLE DEFINITIONS
*******************************************************************************/
// Setup all params for creating a thread
static const osThreadAttr_t usb_cmd_rx_thread_attr =
{
    .name       = USB_CMD_RX_THREAD_NAME,
    .priority   = USB_CMD_RX_THREAD_PRIORITY,
    .stack_size = USB_CMD_RX_THREAD_STACK_SIZE,
};
static __attribute__((unused)) osThreadId_t usb_cmd_rx_thread_handle;
// Setup all params for creating a USB command RX queue
static uint8_t usb_cmd_rx_msg_data[USB_CMD_RX_QUEUE_MSG_SIZE] = {0};
static osMessageQueueAttr_t usb_cmd_rx_msg_queue_attr;
static StaticQueue_t        usb_cmd_rx_xQueueBuffer; // xQueueBuffer will hold the queue structure.
static usb_cmd_rx_id_t  usb_cmd_rx_msg_queue_id;


static const osThreadAttr_t usb_cmd_tx_thread_attr =
{
    .name       = USB_CMD_TX_THREAD_NAME,
    .priority   = USB_CMD_TX_THREAD_PRIORITY,
    .stack_size = USB_CMD_TX_THREAD_STACK_SIZE,
};
static __attribute__((unused)) osThreadId_t usb_cmd_tx_thread_handle;

static osMutexId_t usb_cmd_app_mutex = NULL;
// bool is_no_available_ssid = false;
// bool is_received_ssid_ip = false;


/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
static void usb_cmd_rx_thread_entry (void *argument);
static void usb_cmd_tx_thread_entry (void *argument);

/******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/
/******************************************************************************
* Function : uint8_t esp_com_get_rx_data(uint8_t * data, uint8_t len)
* Brief    : Get available SSID.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
// uint8_t esp_com_get_rx_data(uint8_t * pdata, uint8_t * len)
// {
//     uint8_t ret = 0;
//     uint8_t data[ESP_COM_RX_MAX_PACKET_SIZE];
//     //|-- Header 1 --|-- Header 2 --|-- payload length --|--- CMD ---|---- data ----|
//     //|--- 1 byte ---|--- 1 byte ---|------ 1 byte ------|-- 1 byte--|-- n bytes ---|
//     if(!esp_com_get_data_from_queue(data))
//     {
//         ret = 1;
//     }
//     else
//     {
//         // Check 2 headers and a byte of payload length
//         if((data[0] == ESP_COM_HEADER1) && (data[1] == ESP_COM_HEADER2) && (data[2] > 0))
//         {
//             // Assign data length to payload length
//             *len = data[2] - 1;
//             memcpy(pdata, &data[4], *len);
//         }
//         else
//         {
//             ret = 2;
//         }
//     }

//     return ret;
// }

/******************************************************************************
* Function : void usb_cmd_put_data_to_rx_queue(uint8_t * data)(uint8_t * data)
* Brief    : Put data into Tx uart Queue.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void usb_cmd_put_data_to_rx_queue(uint8_t * data)
{
    USB_CMD_CS_ENTER(osWaitForever);
    //  TODO: Need a retry mechanism here if Failed to put data into Queue
    osStatus_t status;    
    status = osMessageQueuePut(usb_cmd_rx_msg_queue_id, data, 0xff, 0);
    if(osOK != status)
    {
        // Reset the Queue in case encountered FULL FIFO scenario
        osMessageQueueReset(usb_cmd_rx_msg_queue_id);
        PRINT_ERROR_LOG("Fail to put data into USB CMD Rx Queue\r\n");
    }
    USB_CMD_CS_EXIT();
}

/******************************************************************************
* Function : bool usb_cmd_get_data_from_rx_queue(uint8_t * data)
* Brief    : Put data into Tx uart Queue.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
bool usb_cmd_get_data_from_rx_queue(uint8_t * data)
{
	bool ret = true;

    if(osMessageQueueGet(usb_cmd_rx_msg_queue_id, data, NULL, osWaitForever) != osOK)
    {
        ret = false
    }

    return ret;
}	

/******************************************************************************
* Function : void car_control_thread_app_init(void)
* Brief    : Create new thread for Car control module.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void esp_com_thread_app_init(void)
{
    // Creating esp communication app thread
    usb_cmd_rx_thread_handle = osThreadNew(usb_cmd_rx_thread_entry, NULL, &usb_cmd_rx_thread_attr);
    usb_cmd_tx_thread_handle = osThreadNew(usb_cmd_tx_thread_entry, NULL, &usb_cmd_tx_thread_attr);
    USB_CMD_CS_INIT(); 
    if(usb_cmd_rx_thread_handle != NULL)
    {
        PRINT_INFO_LOG("Successfully create the USB CMD thread!\r\n");
    }
}

/******************************************************************************
* STATIC FUNCTIONS
*******************************************************************************/
/******************************************************************************
* Function : static void usb_cmd_rx_thread_entry (void *argument)
* Brief    : Thread entry handles all operations of usb command RX.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void usb_cmd_rx_thread_entry (void *argument)
{
    uint8_t data[USB_CMD_RX_MAX_PACKET_SIZE] = {0};
    // uint8_t tx_packet[ESP_COM_RX_MAX_PACKET_SIZE];
    // uint8_t data_len = 0;
    // tx_packet[0] = ESP_COM_HEADER1;
    // tx_packet[1] = ESP_COM_HEADER2;

    usb_cmd_rx_msg_queue_attr.name      = USB_CMD_RX_QUEUE_SUB_NAME;
    usb_cmd_rx_msg_queue_attr.cb_mem    = &usb_cmd_rx_xQueueBuffer;
    usb_cmd_rx_msg_queue_attr.cb_size   = sizeof(usb_cmd_rx_xQueueBuffer);
    usb_cmd_rx_msg_queue_attr.mq_mem    = usb_cmd_rx_msg_data;
    usb_cmd_rx_msg_queue_attr.mq_size   = USB_CMD_RX_QUEUE_MSG_SIZE;
    usb_cmd_rx_msg_queue_id = osMessageQueueNew(USB_CMD_RX_MAX_NUM_MSG, 
                                        USB_CMD_RX_MSG_SIZE, &usb_cmd_rx_msg_queue_attr);  
    //|-- Header 1 --|-- Header 2 --|-- payload length --|--- CMD ---|---- data ----|
    //|--- 1 byte ---|--- 1 byte ---|------ 1 byte ------|-- 1 byte--|-- n bytes ---|



    while(1)
    {

        // Wait for data from PC via USB Virtual COM Port
        if(usb_cmd_get_data_from_rx_queue(data))
        {
            memset(data, 0, sizeof(data));
            PRINT_INFO_LOG("Receive data from PC\r\n");

        }
        else
        {

        }

        // Parse the received data and transmit back ACK for ESP32
        memset(data, 0, sizeof(data));
        data_len = esp_com_get_current_data(data);
        PRINT_INFO_LOG("SSID len: %d\r\n", data_len);
        if((data[0] == ESP_COM_HEADER1) && (data[1] == ESP_COM_HEADER2))
        {
            switch(data[3])
            {
                case ESP_CMD_CONNECT:
                case ESP_CMD_DISCONNECT:
                case ESP_CMD_SET_SSID:
                case ESP_CMD_GET_RSSI:
                    break;

                case ESP_CMD_GET_AVAILABLE_SSID:
                    // There is no available SSID
                    if((data[2] == 0x02) && (data[4] == 0x00))
                    {
                        is_no_available_ssid = true;
                    }
                    else
                    {
                        PRINT_INFO_LOG("WIFI - idx:%d - SSID:%s\r\n", data[5], &data[6]);
                        // Append to the a ring buffer to send to Raspberry via RF 
                        esp_com_put_data_into_queue(data);
                    }
                    break;

                case ESP_CMD_GET_IP:
                    esp_com_put_data_into_queue(data);
                    is_received_ssid_ip = true;
                    PRINT_INFO_LOG("WIFI Local IP: %s\r\n", &data[5]);
                    break;

                default:
                    break;
            }

            // Append the TX packet to Tx queue
            // Transmit ACK back to ESP32
            tx_packet[2] = 0x01;
            tx_packet[3] = ESP_CMD_RESPONSE_ACK;
            esp_com_put_data_tx_to_queue(tx_packet);
        }   
    }
}

/******************************************************************************
* Function : static void usb_cmd_tx_thread_entry (void *argument)
* Brief    : Thread entry handles all operations of usb command Tx module.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void usb_cmd_tx_thread_entry (void *argument)
{
    //|-- Header 1 --|-- Header 2 --|-- payload length --|--- CMD ---|---- data ----|
    //|--- 1 byte ---|--- 1 byte ---|------ 1 byte ------|-- 1 byte--|-- n bytes ---|
    osStatus_t status;
    uint8_t data[ESP_COM_RX_MAX_PACKET_SIZE];
    // Init the Queue for storing all data
    // received from NRF24L01
	
		// uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)

    while(1)
    {
        // Wait for tx data
        status = osMessageQueueGet(esp_com_tx_msg_queue_id, data, NULL, osWaitForever);
        
        if((status == osOK) && (data[0] == ESP_COM_HEADER1) && (data[1] == ESP_COM_HEADER2))
        {
            if(!esp_com_transmit_data(data, ESP_COM_HEADER_SIZE + ESP_COM_PAYLOAD_LEN_SIZE + data[2]))
            {
                PRINT_ERROR_LOG("Fail to transmit data from MCU to ESP32!\r\n");
            }
            osDelay(10);
        }
    }
}

/*************** END OF FILES *************************************************/
