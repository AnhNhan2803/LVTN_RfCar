/*******************************************************************************
* Title                 :   NRF24L01 application source file
* Filename              :   nrf24l01_app.c
* Author                :   Nhan
* Origin Date           :   Oct 17th 2021
* Notes                 :   None
*******************************************************************************/

/*******************************************************************************
* INCLUDE
*******************************************************************************/
#include "nrf24l01_app.h"

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define NRF24L01_THREAD_NAME             ("nrf24l01_thread")
#define NRF24L01_THREAD_STACK_SIZE       (1024) // Bytes
#define NRF24L01_THREAD_PRIORITY         ((osPriority_t) osPriorityHigh1)
#define NRF24L01_QUEUE_SUB_NAME          ("nrf24l01_queue")
#define NEF24L01_QUEUE_TIMEOUT_MS        (1000)

/******************************************************************************
* PREPROCESSOR MACROS
*******************************************************************************/


/******************************************************************************
* TYPEDEFS
*******************************************************************************/
typedef osMessageQueueId_t nrf24l01_queue_id_t;

/******************************************************************************
* VARIABLE DEFINITIONS
*******************************************************************************/
// Setup all params for creating a thread
static const osThreadAttr_t nrf24l01_thread_attr =
{
    .name       = NRF24L01_THREAD_NAME,
    .priority   = NRF24L01_THREAD_PRIORITY,
    .stack_size = NRF24L01_THREAD_STACK_SIZE,
};
static __attribute__((unused)) osThreadId_t nrf24l01_thread_handle;

// Setup all params for creating a Queue
static uint8_t nrf24l01_msg_data[NRF24L01_QUEUE_MSG_SIZE] = {0};
static osMessageQueueAttr_t nrf24l01_msg_queue_attr;
static StaticQueue_t        xQueueBuffer; // xQueueBuffer will hold the queue structure.
static nrf24l01_queue_id_t  nrf24l01_msg_queue_id;
extern bool is_no_available_ssid;
extern bool is_received_ssid_ip;

/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
static void nrf24l01_thread_entry (void *argument);

/******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/
/******************************************************************************
* Function : void nrf24l01_thread_init(void)
* Brief    : Create new thread for NRF24L01
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_thread_app_init(void)
{
  // Creating the nrf24l01 app thread
  nrf24l01_thread_handle = osThreadNew(nrf24l01_thread_entry, NULL, &nrf24l01_thread_attr);
  if(nrf24l01_thread_handle != NULL)
  {
    PRINT_INFO_LOG("Successfully create the NRF24L01 thread!\r\n");
  }
}

/******************************************************************************
* Function : nrf24l01_get_chunk_data
* Brief    : Wait and get chunk of available NRF24L01 data.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
osStatus_t nrf24l01_get_chunk_data(nrf24l01_data_t * pdata)
{
  return osMessageQueueGet(nrf24l01_msg_queue_id, pdata, NULL, osWaitForever);
}

/******************************************************************************
* STATIC FUNCTIONS
*******************************************************************************/
/******************************************************************************
* Function : static void nrf24l01_thread_entry (void *argument)
* Brief    : Thread entry handles all operations of NRF24L01.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void nrf24l01_thread_entry (void *argument)
{
	osStatus_t status;
  nrf24l01_data_t nrf24l01_data;
  uint8_t data_len;
  uint8_t nrf_status;
  rx_data_t rx_data;
  uint8_t esp_status;
  bool is_start_get_available_ssid = false;
  uint8_t uart_tx_data[ESP_COM_RX_MAX_PACKET_SIZE];
  uint8_t tx_packet[ESP_COM_RX_MAX_PACKET_SIZE + 1];
  tx_packet[0] = NRF24L01_PAYLOAD_HEADER_1; 
  tx_packet[1] = NRF24L01_PAYLOAD_HEADER_2; 

  // Init the Queue for storing all data
  // received from NRF24L01
  nrf24l01_msg_queue_attr.name      = NRF24L01_QUEUE_SUB_NAME;
  nrf24l01_msg_queue_attr.cb_mem    = &xQueueBuffer;
  nrf24l01_msg_queue_attr.cb_size   = sizeof(xQueueBuffer);
  nrf24l01_msg_queue_attr.mq_mem    = nrf24l01_msg_data;
  nrf24l01_msg_queue_attr.mq_size   = NRF24L01_QUEUE_MSG_SIZE;
  nrf24l01_msg_queue_id = osMessageQueueNew(NRF24L01_MAX_NUM_MSG, 
                                      NRF24L01_PACKET_MAX_SIZE, &nrf24l01_msg_queue_attr);    

  // Init the NRF24L01 driver as Receiver role   
  nrf24l01_init();
  // Temporaily define the NRF24L01 data frame as below
  //|-- Header 1 --|-- Header 2 --|-- payload length --|-- NRF24L01Command --|---- data ----|
  //|--- 1 byte ---|--- 1 byte ---|------ 1 byte ------|------ 1 byte -------|-- n bytes ---|

  while(1)
  {
    nrf24l01_data_wait_new_data();
		PRINT_INFO_LOG("Receive message from Tx device: %d\r\n", idx++); 

    // Read data from RX FIFO, need to read the number of available
    // data in RX FIFO firstly if using Dynamic payload length
    // ESP_COM_CS_ENTER();
    nrf_status = nrf24l01_read_fifo_status();
    if (nrf_status != 0x01)
    {
      if(nrf24l01_read_data_fifo((uint8_t *)&nrf24l01_data, &data_len))
      {
        PRINT_INFO_LOG("Data length: %d\r\nData", data_len);
        for(uint8_t i=0; i<data_len; i++)
        {
          PRINT_INFO_LOG(":0x%x", nrf24l01_data.data[i]);
        }
        PRINT_INFO_LOG("\r\n");

        // Put the data received from NRF24L01 into Queue
        switch(nrf24l01_data.data[3])
        {
          case ESP_CMD_CONNECT:
            memset(uart_tx_data, 0, sizeof(uart_tx_data));
            // Connect to the SSID+PASSWORD received from raspberry
            memcpy(uart_tx_data, nrf24l01_data.data, ESP_COM_HEADER_SIZE + ESP_COM_PAYLOAD_LEN_SIZE + nrf24l01_data.data[2]);
            esp_com_put_data_tx_to_queue(uart_tx_data);
            nrf24l01_write_back_ack_payload(NRF24L01_DATA_PIPE_1, (uint8_t *)NRF24L01_RX_ACK_PAYLOAD, 10);
            break;


          case ESP_CMD_DISCONNECT:
            memset(uart_tx_data, 0, sizeof(uart_tx_data));
            // disconnect from the current wifi
            memcpy(uart_tx_data, nrf24l01_data.data, ESP_COM_HEADER_SIZE + ESP_COM_PAYLOAD_LEN_SIZE + nrf24l01_data.data[2]);
            esp_com_put_data_tx_to_queue(uart_tx_data);
            nrf24l01_write_back_ack_payload(NRF24L01_DATA_PIPE_1, (uint8_t *)NRF24L01_RX_ACK_PAYLOAD, 10);
            break;


          case ESP_CMD_GET_AVAILABLE_SSID:
            // Need to send a start get available wifi
            // for the first time
            if(!is_start_get_available_ssid)
            {
              memset(uart_tx_data, 0, sizeof(uart_tx_data));
              is_start_get_available_ssid = true;
              memcpy(uart_tx_data, nrf24l01_data.data, ESP_COM_HEADER_SIZE + ESP_COM_PAYLOAD_LEN_SIZE + ESP_COM_CMD_SIZE);
              esp_com_put_data_tx_to_queue(uart_tx_data);
            }

            // Wait for the scanning action
            while(!is_no_available_ssid){
              osDelay(10);
            }

            // Detect get WIFI list then send back ACK payload as
            // a ssid each time transmit a packet to RF
            esp_status = esp_com_get_rx_data(rx_data.data, &rx_data.len);
            if(esp_status == 0)
            {
              // Payload length = SSID length + command length + Is_available_ssid length
              tx_packet[2] = rx_data.len + 1 + 1; 
              tx_packet[3] = ESP_CMD_GET_AVAILABLE_SSID;
              // A byte to notify that there is available ssid or not
              tx_packet[4] = 0x01;
              memcpy(&tx_packet[5], rx_data.data, rx_data.len);
              nrf24l01_write_back_ack_payload(NRF24L01_DATA_PIPE_1, tx_packet, 5 + rx_data.len);
            }
            else
            {
              // Re-enable these flags for the next check
              is_no_available_ssid = false;
              is_start_get_available_ssid = true;
              PRINT_INFO_LOG("Can not get data from SSID buffer: eer_code %d\r\n", esp_status);
              // Payload length = command length + Is_available_ssid length
              tx_packet[2] = 1 + 1; 
              tx_packet[3] = ESP_CMD_GET_AVAILABLE_SSID;
              tx_packet[4] = 0x00;
              nrf24l01_write_back_ack_payload(NRF24L01_DATA_PIPE_1, tx_packet, 5);
            }
            break;

          case ESP_CMD_SET_SSID:
            memset(uart_tx_data, 0, sizeof(uart_tx_data));
            // Connect to the SSID+PASSWORD received from raspberry
            memcpy(uart_tx_data, nrf24l01_data.data, ESP_COM_HEADER_SIZE + ESP_COM_PAYLOAD_LEN_SIZE + nrf24l01_data.data[2]);
            esp_com_put_data_tx_to_queue(uart_tx_data);

            // Delay to ensure that the wifi connection is established
            // osDelay(1000);
            nrf24l01_write_back_ack_payload(NRF24L01_DATA_PIPE_1, (uint8_t *)NRF24L01_RX_ACK_PAYLOAD, 10);
            break;


          case ESP_CMD_GET_IP:
            memset(uart_tx_data, 0, sizeof(uart_tx_data));
            // Get local IP of the connected network
            memcpy(uart_tx_data, nrf24l01_data.data, ESP_COM_HEADER_SIZE + ESP_COM_PAYLOAD_LEN_SIZE + nrf24l01_data.data[2]);
            esp_com_put_data_tx_to_queue(uart_tx_data);
            while(!is_received_ssid_ip)
            {
              osDelay(10);
            }
            is_received_ssid_ip = false;
            esp_status = esp_com_get_rx_data(rx_data.data, &rx_data.len);

            if(esp_status == 0)
            {
              // Payload length = SSID length + command length + Is_available_ssid length
              tx_packet[2] = rx_data.len + 1 + 1; 
              tx_packet[3] = ESP_CMD_GET_IP;
              // A byte to notify that there is available ssid or not
              tx_packet[4] = 0x01;
              memcpy(&tx_packet[5], rx_data.data, rx_data.len);
              nrf24l01_write_back_ack_payload(NRF24L01_DATA_PIPE_1, tx_packet, 5 + rx_data.len);
            }
            else
            {
              PRINT_INFO_LOG("Can not get Local IP: eer_code %d\r\n", esp_status);
              // Payload length = command length + Is_available_ssid length
              tx_packet[2] = 1 + 1; 
              tx_packet[3] = ESP_CMD_GET_IP;
              tx_packet[4] = 0x00;
              nrf24l01_write_back_ack_payload(NRF24L01_DATA_PIPE_1, tx_packet, 5);
            }
            break;


          case ESP_CMD_GET_RSSI:
            break;


          default:
            // Detect car control command
            status = osMessageQueuePut(nrf24l01_msg_queue_id, &nrf24l01_data, 0xff, 0);
            if(osOK != status)
            {
              // Reset the Queue in case encountered FULL FIFO scenario
              osMessageQueueReset(nrf24l01_msg_queue_id);
              PRINT_ERROR_LOG("Fail to put data into queue\r\n");
            }
            // Send ACK payload back to the transmitter 
            nrf24l01_write_back_ack_payload(NRF24L01_DATA_PIPE_1, (uint8_t *)NRF24L01_RX_ACK_PAYLOAD, 10);
        }
      }
      nrf24l01_clear_all_flags();
      // memset(data, 0, sizeof(data));
    }
    // ESP_COM_CS_EXIT();
  }
}
/*************** END OF FILES *************************************************/
