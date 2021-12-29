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

/******************************************************************************
* PREPROCESSOR MACROS
*******************************************************************************/


/******************************************************************************
* TYPEDEFS
*******************************************************************************/


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

/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
static void esp_com_thread_entry (void *argument);

/******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/
/******************************************************************************
* Function : uint8_t esp_com_get_ssid(uint8_t * ssid, uint8_t len)
* Brief    : Get available SSID.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
uint8_t esp_com_get_ssid(uint8_t * ssid, uint8_t * len)
{
    uint8_t ret = 0;
    uint8_t data[ESP_COM_RX_MAX_PACKET_SIZE];
    
    if(!esp_com_get_data_from_queue(data))
    {
        ret = 1;
    }
    else
    {
        // Check 2 headers and a byte of payload length
        if((data[0] == ESP_COM_HEADER1) && (data[0] == ESP_COM_HEADER2) && (data[2] > 0))
        {
            // Assign data length to payload length
            *len = data[2];
            memcpy(ssid, &data[3], *len);
        }
        else
        {
            ret = 2;
        }
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
    esp_com_thread_handle = osThreadNew(esp_com_thread_entry, NULL, &esp_com_thread_attr);
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
    uint8_t data_len = 0;
    //|-- Header 1 --|-- Header 2 --|-- payload length --|---- data ----|
    //|--- 1 byte ---|--- 1 byte ---|------ 1 byte ------|-- n bytes ---|
    esp_com_init();

    while(1)
    {
        esp_com_wait_sem();

        // Parse the received data and transmit back ACK for ESP32
        data_len = esp_com_get_current_data(data);
        PRINT_INFO_LOG("SSID len: %d\r\n", data_len);
        if((data[0] == ESP_COM_HEADER1) && (data[0] == ESP_COM_HEADER2))
        {
            // Append to the SSID array
            esp_com_put_data_into_queue(data);

            // Transmit ACK back to ESP32
            esp_com_transmit_data((uint8_t *)ESP_COM_TX_ACK, strlen(ESP_COM_TX_ACK));
        }   
    }
}
/*************** END OF FILES *************************************************/
