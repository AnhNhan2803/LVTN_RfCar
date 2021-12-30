/*******************************************************************************
* Title                 :   esp communication app source file
* Filename              :   esp_app.c
* Author                :   Nhan
* Origin Date           :   Oct 19th 2021
* Notes                 :   None
*******************************************************************************/

/*******************************************************************************
* INCLUDE
*******************************************************************************/
#include "esp_app.h"

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define ESP_COM_THREAD_NAME             ("esp_com_thread")
#define ESP_COM_THREAD_STACK_SIZE       (500) // Bytes
#define ESP_COM_THREAD_PRIORITY         ((osPriority_t) osPriorityHigh)
#define ESP_COM_TX_ACK                  ("AcK")

#define ESP_COM_TX_THREAD_NAME             ("esp_tx_com_thread")
#define ESP_COM_TX_THREAD_STACK_SIZE       (1024) // Bytes
#define ESP_COM_TX_THREAD_PRIORITY         ((osPriority_t) osPriorityHigh)
#define ESP_COM_TX_QUEUE_SUB_NAME          ("esp_com_tx_queue")

/******************************************************************************
* PREPROCESSOR MACROS
*******************************************************************************/
#define ESP_APP_CS_INIT()                 esp_app_mutex = osMutexNew(NULL);                                  
#define ESP_APP_CS_ENTER(timeout_ms)      osMutexAcquire(esp_app_mutex, timeout_ms)     // critical section enter
#define ESP_APP_CS_EXIT()                 osMutexRelease(esp_app_mutex)                 // critical section end

/******************************************************************************
* TYPEDEFS
*******************************************************************************/
typedef osMessageQueueId_t esp_com_tx_id_t;

/******************************************************************************
* VARIABLE DEFINITIONS
*******************************************************************************/
// Setup all params for creating a thread
static const osThreadAttr_t esp_com_thread_attr =
{
    .name       = ESP_COM_THREAD_NAME,
    .priority   = ESP_COM_THREAD_PRIORITY,
    .stack_size = ESP_COM_THREAD_STACK_SIZE,
};
static __attribute__((unused)) osThreadId_t esp_com_thread_handle;

static const osThreadAttr_t esp_com_tx_thread_attr =
{
    .name       = ESP_COM_TX_THREAD_NAME,
    .priority   = ESP_COM_TX_THREAD_PRIORITY,
    .stack_size = ESP_COM_TX_THREAD_STACK_SIZE,
};
static __attribute__((unused)) osThreadId_t esp_com_tx_thread_handle;

// Setup all params for creating a Queue
static uint8_t esp_com_tx_msg_data[ESP_COM_TX_QUEUE_MSG_SIZE] = {0};
static osMessageQueueAttr_t esp_com_tx_msg_queue_attr;
static StaticQueue_t        xQueueBuffer; // xQueueBuffer will hold the queue structure.
static esp_com_tx_id_t  esp_com_tx_msg_queue_id;
static osMutexId_t esp_app_mutex = NULL;
bool is_no_available_ssid = false;
bool is_received_ssid_ip = false;


/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
static void esp_com_thread_entry (void *argument);
static void esp_com_tx_thread_entry (void *argument);

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
uint8_t esp_com_get_rx_data(uint8_t * pdata, uint8_t * len)
{
    uint8_t ret = 0;
    uint8_t data[ESP_COM_RX_MAX_PACKET_SIZE];
    //|-- Header 1 --|-- Header 2 --|-- payload length --|--- CMD ---|---- data ----|
    //|--- 1 byte ---|--- 1 byte ---|------ 1 byte ------|-- 1 byte--|-- n bytes ---|
    if(!esp_com_get_data_from_queue(data))
    {
        ret = 1;
    }
    else
    {
        // Check 2 headers and a byte of payload length
        if((data[0] == ESP_COM_HEADER1) && (data[1] == ESP_COM_HEADER2) && (data[2] > 0))
        {
            // Assign data length to payload length
            *len = data[2] - 1;
            memcpy(pdata, &data[4], *len);
        }
        else
        {
            ret = 2;
        }
    }

    return ret;
}

/******************************************************************************
* Function : static void esp_com_put_data_tx_to_queue(uint8_t * data, uint8_t len)
* Brief    : Put data into Tx uart Queue.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void esp_com_put_data_tx_to_queue(uint8_t * data)
{
    ESP_APP_CS_ENTER(osWaitForever);
    //  TODO: Need a retry mechanism here if Failed to put data into Queue
    osStatus_t status;    
    status = osMessageQueuePut(esp_com_tx_msg_queue_id, data, 0xff, 0);
    if(osOK != status)
    {
        // Reset the Queue in case encountered FULL FIFO scenario
        osMessageQueueReset(esp_com_tx_msg_queue_id);
        PRINT_ERROR_LOG("Fail to put data into ESP communication queue\r\n");
    }
    ESP_APP_CS_EXIT();
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
    esp_com_thread_handle = osThreadNew(esp_com_thread_entry, NULL, &esp_com_thread_attr);
    esp_com_tx_thread_handle = osThreadNew(esp_com_tx_thread_entry, NULL, &esp_com_tx_thread_attr);
    ESP_APP_CS_INIT();
		esp_com_init();
    if(esp_com_thread_handle != NULL)
    {
        PRINT_INFO_LOG("Successfully create the ESP communication thread!\r\n");
    }
}

/******************************************************************************
* STATIC FUNCTIONS
*******************************************************************************/
/******************************************************************************
* Function : static void esp_com_thread_entry (void *argument)
* Brief    : Thread entry handles all operations of esp communication module.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void esp_com_thread_entry (void *argument)
{
    uint8_t data[ESP_COM_RX_MAX_PACKET_SIZE];
    uint8_t tx_packet[ESP_COM_RX_MAX_PACKET_SIZE];
    uint8_t data_len = 0;
    tx_packet[0] = ESP_COM_HEADER1;
    tx_packet[1] = ESP_COM_HEADER2;
    //|-- Header 1 --|-- Header 2 --|-- payload length --|--- CMD ---|---- data ----|
    //|--- 1 byte ---|--- 1 byte ---|------ 1 byte ------|-- 1 byte--|-- n bytes ---|

    while(1)
    {
        esp_com_wait_sem();

        // Parse the received data and transmit back ACK for ESP32
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
                        // Append to the a ring buffer to send to Raspberry via RF 
                        esp_com_put_data_into_queue(data);
                    }
                    break;

                case ESP_CMD_GET_IP:
                    esp_com_put_data_into_queue(data);
                    is_received_ssid_ip = true;
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
* Function : static void esp_com_tx_thread_entry (void *argument)
* Brief    : Thread entry handles all operations of esp communication Tx module.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void esp_com_tx_thread_entry (void *argument)
{
    //|-- Header 1 --|-- Header 2 --|-- payload length --|--- CMD ---|---- data ----|
    //|--- 1 byte ---|--- 1 byte ---|------ 1 byte ------|-- 1 byte--|-- n bytes ---|
    osStatus_t status;
    uint8_t data[ESP_COM_RX_MAX_PACKET_SIZE];
    // Init the Queue for storing all data
    // received from NRF24L01
    esp_com_tx_msg_queue_attr.name      = ESP_COM_TX_QUEUE_SUB_NAME;
    esp_com_tx_msg_queue_attr.cb_mem    = &xQueueBuffer;
    esp_com_tx_msg_queue_attr.cb_size   = sizeof(xQueueBuffer);
    esp_com_tx_msg_queue_attr.mq_mem    = esp_com_tx_msg_data;
    esp_com_tx_msg_queue_attr.mq_size   = ESP_COM_TX_QUEUE_MSG_SIZE;
    esp_com_tx_msg_queue_id = osMessageQueueNew(ESP_COM_TX_MAX_NUM_MSG, 
                                        ESP_COM_TX_MSG_SIZE, &esp_com_tx_msg_queue_attr);   

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
