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
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
#define NRF24L01_THREAD_NAME ("nrf24l01_thread")
#define NRF24L01_THREAD_STACK_SIZE (2048) // Bytes
#define NRF24L01_THREAD_PRIORITY ((osPriority_t)osPriorityHigh1)
#define NRF24L01_QUEUE_SUB_NAME ("nrf24l01_queue")
#define NEF24L01_QUEUE_TIMEOUT_MS (1000)
#else
#define NRF24L01_TX_THREAD_NAME ("nrf24l01_tx_thread")
#define NRF24L01_TX_THREAD_STACK_SIZE (512) // Bytes
#define NRF24L01_TX_THREAD_PRIORITY ((osPriority_t)osPriorityHigh1)
#define NRF24L01_TX_QUEUE_SUB_NAME ("nrf24l01_tx_queue")
#define NEF24L01_TX_QUEUE_TIMEOUT_MS (1000)
#endif

/******************************************************************************
* PREPROCESSOR MACROS
*******************************************************************************/

/******************************************************************************
* TYPEDEFS
*******************************************************************************/
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
typedef osMessageQueueId_t nrf24l01_queue_id_t;
#else
typedef osMessageQueueId_t nrf24l01_tx_queue_id_t;
#endif

/******************************************************************************
* VARIABLE DEFINITIONS
*******************************************************************************/
// Setup all params for creating a thread
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
static const osThreadAttr_t nrf24l01_thread_attr =
    {
        .name = NRF24L01_THREAD_NAME,
        .priority = NRF24L01_THREAD_PRIORITY,
        .stack_size = NRF24L01_THREAD_STACK_SIZE,
};
static __attribute__((unused)) osThreadId_t nrf24l01_thread_handle;

// Setup all params for creating a Queue
static uint8_t nrf24l01_msg_data[NRF24L01_QUEUE_MSG_SIZE] = {0};
static osMessageQueueAttr_t nrf24l01_msg_queue_attr;
static StaticQueue_t xQueueBuffer; // xQueueBuffer will hold the queue structure.
static nrf24l01_queue_id_t nrf24l01_msg_queue_id;
extern bool is_no_available_ssid;
extern bool is_received_ssid_ip;
#else
// Create threading parameters for the Transmitter
static const osThreadAttr_t nrf24l01_tx_thread_attr =
    {
        .name = NRF24L01_TX_THREAD_NAME,
        .priority = NRF24L01_TX_THREAD_PRIORITY,
        .stack_size = NRF24L01_TX_THREAD_STACK_SIZE,
};
static __attribute__((unused)) osThreadId_t nrf24l01_tx_thread_handle;

// Setup all params for creating a Queue
static uint8_t nrf24l01_tx_msg_data[NRF24L01_QUEUE_MSG_SIZE] = {0};
static osMessageQueueAttr_t nrf24l01_tx_msg_queue_attr;
static StaticQueue_t tx_xQueueBuffer; // xQueueBuffer will hold the queue structure.
static nrf24l01_tx_queue_id_t nrf24l01_tx_msg_queue_id;
#endif

/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
static void nrf24l01_thread_entry(void *argument);
#else
static void nrf24l01_tx_thread_entry(void *argument);
static void nrf24l01_tx_send_in_order_and_get_response(uint8_t * tx_data, uint8_t tx_len, uint8_t * rx_data, uint8_t * rx_len);
#endif

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
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
  // Creating the nrf24l01 app thread
  nrf24l01_thread_handle = osThreadNew(nrf24l01_thread_entry, NULL, &nrf24l01_thread_attr);
  if (nrf24l01_thread_handle != NULL)
  {
    PRINT_INFO_LOG("Successfully create the NRF24L01 thread!\r\n");
  }
#else
  // Init the NRF24L01 driver as Receiver role
  nrf24l01_init();

  // Creating the nrf24l01 app thread
  nrf24l01_tx_thread_handle = osThreadNew(nrf24l01_tx_thread_entry, NULL, &nrf24l01_tx_thread_attr);
  if (nrf24l01_tx_thread_handle != NULL)
  {
    PRINT_INFO_LOG("Successfully create the NRF24L01 TX thread!\r\n");
  }
#endif
}

/******************************************************************************
* Function : nrf24l01_get_chunk_data
* Brief    : Wait and get chunk of available NRF24L01 data.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
osStatus_t nrf24l01_get_chunk_data(nrf24l01_data_t *pdata)
{
  return osMessageQueueGet(nrf24l01_msg_queue_id, pdata, NULL, osWaitForever);
}
#endif

/******************************************************************************
* Function : bool nrf24l01_put_data_into_tx_queue(uint8_t * data)
* Brief    : Put data to TX fifo for transmitting.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
#if (DEVICE_ROLE == DEVICE_ROLE_TX)
bool nrf24l01_put_data_into_tx_queue(uint8_t * data)
{
  bool ret = true;
  osStatus_t status;    
  status = osMessageQueuePut(nrf24l01_tx_msg_queue_id, data, 0xff, 0);
  if(osOK != status)
  {
    ret = false;
    // Reset the Queue in case encountered FULL FIFO scenario
    osMessageQueueReset(nrf24l01_tx_msg_queue_id);
    PRINT_ERROR_LOG("Fail to put data into TX FIFO of NRF24L01\r\n");
  }

  return ret;
}
#endif

/******************************************************************************
* Function : bool nrf24l01_get_data_from_tx_queue(uint8_t * data)
* Brief    : Get data from Tx uart Queue.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
#if (DEVICE_ROLE == DEVICE_ROLE_TX)
bool nrf24l01_get_data_from_tx_queue(uint8_t * data)
{
	bool ret = true;

    if(osMessageQueueGet(nrf24l01_tx_msg_queue_id, data, NULL, osWaitForever) != osOK)
    {
        ret = false;
    }

    return ret;
}	
#endif


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
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
static void nrf24l01_thread_entry(void *argument)
{
  osStatus_t status;
  nrf24l01_data_t nrf24l01_data;
  uint8_t data_len;
  uint8_t nrf_status;
  rx_data_t rx_data;
  uint8_t esp_status;
  uint16_t idx = 0;
  bool is_start_get_available_ssid = false;
  bool is_start_get_ip = false;


  uint8_t uart_tx_data[ESP_COM_RX_MAX_PACKET_SIZE];
  uint8_t tx_packet[ESP_COM_RX_MAX_PACKET_SIZE + 1];
  tx_packet[0] = NRF24L01_PAYLOAD_HEADER_1;
  tx_packet[1] = NRF24L01_PAYLOAD_HEADER_2;

  uint8_t tx_check_order = 0;

  // Init the Queue for storing all data
  // received from NRF24L01
  nrf24l01_msg_queue_attr.name = NRF24L01_QUEUE_SUB_NAME;
  nrf24l01_msg_queue_attr.cb_mem = &xQueueBuffer;
  nrf24l01_msg_queue_attr.cb_size = sizeof(xQueueBuffer);
  nrf24l01_msg_queue_attr.mq_mem = nrf24l01_msg_data;
  nrf24l01_msg_queue_attr.mq_size = NRF24L01_QUEUE_MSG_SIZE;
  nrf24l01_msg_queue_id = osMessageQueueNew(NRF24L01_MAX_NUM_MSG,
                                            NRF24L01_PACKET_MAX_SIZE, &nrf24l01_msg_queue_attr);

  // Init the NRF24L01 driver as Receiver role
  nrf24l01_init();
  // Temporaily define the NRF24L01 data frame as below
  //|-- Header 1 --|-- Header 2 --|-- payload length --|-- NRF24L01Command --|---- data ----|
  //|--- 1 byte ---|--- 1 byte ---|------ 1 byte ------|------ 1 byte -------|-- n bytes ---|

  while (1)
  {
    nrf24l01_data_wait_new_data();
    PRINT_INFO_LOG("Receive message from Tx device: %d\r\n", idx++);

    // Read data from RX FIFO, need to read the number of available
    // data in RX FIFO firstly if using Dynamic payload length
    // ESP_COM_CS_ENTER();
    nrf_status = nrf24l01_read_fifo_status();
    if (nrf_status != 0x01)
    {
      if (nrf24l01_read_data_fifo((uint8_t *)&nrf24l01_data, &data_len))
      {
        PRINT_INFO_LOG("Data length: %d\r\nData", data_len);
        for (uint8_t i = 0; i < data_len; i++)
        {
          PRINT_INFO_LOG(":0x%x", nrf24l01_data.data[i]);
        }
        PRINT_INFO_LOG("\r\n");

        // Put the data received from NRF24L01 into Queue
        switch (nrf24l01_data.data[3])
        {
          case ESP_CMD_CONNECT:
            tx_check_order++;

            if (tx_check_order == 1)
            {
              memset(uart_tx_data, 0, sizeof(uart_tx_data));
              // Connect to the SSID+PASSWORD received from raspberry
              memcpy(uart_tx_data, nrf24l01_data.data, ESP_COM_HEADER_SIZE + ESP_COM_PAYLOAD_LEN_SIZE + nrf24l01_data.data[2]);
              esp_com_put_data_tx_to_queue(uart_tx_data);
              PRINT_INFO_LOG("Connect to the WIFI network!\r\n");
              nrf24l01_write_back_ack_payload(NRF24L01_DATA_PIPE_1, (uint8_t *)NRF24L01_RX_ACK_PAYLOAD, 10);
            }
            else
            {
              tx_check_order = 0;
            }
            break;

          case ESP_CMD_DISCONNECT:
            tx_check_order++;

            if (tx_check_order == 1)
            {
              memset(uart_tx_data, 0, sizeof(uart_tx_data));
              // disconnect from the current wifi
              memcpy(uart_tx_data, nrf24l01_data.data, ESP_COM_HEADER_SIZE + ESP_COM_PAYLOAD_LEN_SIZE + nrf24l01_data.data[2]);
              esp_com_put_data_tx_to_queue(uart_tx_data);
              nrf24l01_write_back_ack_payload(NRF24L01_DATA_PIPE_1, (uint8_t *)NRF24L01_RX_ACK_PAYLOAD, 10);
            }
            else
            {
              tx_check_order = 0;
            }
            break;

          case ESP_CMD_GET_AVAILABLE_SSID:
            // Need to send a start get available wifi
            // for the first time

            tx_check_order++;

            if (tx_check_order == 1)
            {
              if (!is_start_get_available_ssid)
              {
                memset(uart_tx_data, 0, sizeof(uart_tx_data));
                is_start_get_available_ssid = true;
                is_no_available_ssid = false;

                memcpy(uart_tx_data, nrf24l01_data.data, ESP_COM_HEADER_SIZE + ESP_COM_PAYLOAD_LEN_SIZE + ESP_COM_CMD_SIZE);
                esp_com_put_data_tx_to_queue(uart_tx_data);
              }

              if (is_no_available_ssid)
              {
                // Detect get WIFI list then send back ACK payload as
                // a ssid each time transmit a packet to RF
                esp_status = esp_com_get_rx_data(rx_data.data, &rx_data.len);
                if (esp_status == 0)
                {
                  // Payload length = SSID length + command length + Is_available_ssid length
                  tx_packet[2] = rx_data.len + 1 + 1;
                  tx_packet[3] = ESP_CMD_GET_AVAILABLE_SSID;
                  // A byte to notify that there is available ssid or not
                  tx_packet[4] = 0x01;
                  PRINT_INFO_LOG("Available SSID detected: %s\r\n", &rx_data.data[2]);
                  memcpy(&tx_packet[5], rx_data.data, rx_data.len);
                  nrf24l01_write_back_ack_payload(NRF24L01_DATA_PIPE_1, tx_packet, 5 + rx_data.len);
                }
                else
                {
                  // Re-enable these flags for the next check
                  is_start_get_available_ssid = false;
                  PRINT_INFO_LOG("Can not get data from SSID buffer: eer_code %d\r\n", esp_status);
                  // Payload length = command length + Is_available_ssid length
                  tx_packet[2] = 1 + 1;
                  tx_packet[3] = ESP_CMD_GET_AVAILABLE_SSID;
                  tx_packet[4] = 0x00;
                  nrf24l01_write_back_ack_payload(NRF24L01_DATA_PIPE_1, tx_packet, 5);
                }
              }
              else
              {
                PRINT_INFO_LOG("SSID not ready yet\r\n");
                // Payload length = command length + Is_available_ssid length
                tx_packet[2] = 1 + 1;
                tx_packet[3] = ESP_CMD_GET_AVAILABLE_SSID;
                tx_packet[4] = 0x01;
                nrf24l01_write_back_ack_payload(NRF24L01_DATA_PIPE_1, tx_packet, 5);
              }
            }
            else
            {
              tx_check_order = 0;
            }
            break;

          case ESP_CMD_SET_SSID:
            tx_check_order++;

            if (tx_check_order == 1)
            {
              memset(uart_tx_data, 0, sizeof(uart_tx_data));
              // Connect to the SSID+PASSWORD received from raspberry
              memcpy(uart_tx_data, nrf24l01_data.data, ESP_COM_HEADER_SIZE + ESP_COM_PAYLOAD_LEN_SIZE + nrf24l01_data.data[2]);
              esp_com_put_data_tx_to_queue(uart_tx_data);
              PRINT_INFO_LOG("Successfully set SSID password: %s\r\n", &uart_tx_data[4]);
              nrf24l01_write_back_ack_payload(NRF24L01_DATA_PIPE_1, (uint8_t *)NRF24L01_RX_ACK_PAYLOAD, 10);
            }
            else
            {
              tx_check_order = 0;
            }
            break;

          case ESP_CMD_GET_IP:
            tx_check_order++;

            if (tx_check_order == 1)
            {
              if(!is_start_get_ip)
              {
                memset(uart_tx_data, 0, sizeof(uart_tx_data));
                // Get local IP of the connected network
                memcpy(uart_tx_data, nrf24l01_data.data, ESP_COM_HEADER_SIZE + ESP_COM_PAYLOAD_LEN_SIZE + nrf24l01_data.data[2]);
                esp_com_put_data_tx_to_queue(uart_tx_data);
              }

              if(is_received_ssid_ip)
              {
                is_received_ssid_ip = false;
                esp_status = esp_com_get_rx_data(rx_data.data, &rx_data.len);

                if (esp_status == 0)
                {
                  // Payload length = SSID length + command length + Is_available_ssid length
                  tx_packet[2] = rx_data.len + 1 + 1;
                  tx_packet[3] = ESP_CMD_GET_IP;
                  PRINT_INFO_LOG("Successfully get Local IP: %s\r\n", &rx_data.data[1]);
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
              }
              else
              {
                PRINT_INFO_LOG("Local IP not ready yet\r\n");
                // Payload length = command length + Is_available_ssid length
                tx_packet[2] = 1 + 1;
                tx_packet[3] = ESP_CMD_GET_IP;
                tx_packet[4] = 0x01;
                nrf24l01_write_back_ack_payload(NRF24L01_DATA_PIPE_1, tx_packet, 5);
              }
            }
            else
            {
              tx_check_order = 0;
            }
            break;

          case ESP_CMD_GET_RSSI:
            break;

          default:
            tx_check_order++;

          if (tx_check_order == 1)
          {
            // Send ACK payload back to the transmitter
            nrf24l01_write_back_ack_payload(NRF24L01_DATA_PIPE_1, (uint8_t *)NRF24L01_RX_ACK_PAYLOAD, 10);
            PRINT_INFO_LOG("Clock ACK payload for the second\r\n");
            // Detect car control command
            status = osMessageQueuePut(nrf24l01_msg_queue_id, &nrf24l01_data, 0xff, 0);
            if (osOK != status)
            {
              // Reset the Queue in case encountered FULL FIFO scenario
              osMessageQueueReset(nrf24l01_msg_queue_id);
              PRINT_ERROR_LOG("Fail to put data into queue\r\n");
            }
          }
          else
          {
            tx_check_order = 0;
          }
        }
      }
      nrf24l01_clear_all_flags();
      memset(&rx_data, 0, sizeof(rx_data));
    }
    // ESP_COM_CS_EXIT();
  }
}
#endif

/******************************************************************************
* Function : static void nrf24l01_tx_thread_entry (void *argument)
* Brief    : Thread entry handles all TX operations of NRF24L01.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
#if (DEVICE_ROLE == DEVICE_ROLE_TX)
static void nrf24l01_tx_thread_entry(void *argument)
{
    uint8_t tx_data[NRF24L01_MSG_SIZE];
    uint8_t tx_len;
    uint8_t rx_data[NRF24L01_MSG_SIZE];
    uint8_t rx_len;
    uint8_t status;

    nrf24l01_tx_msg_queue_attr.name      = NRF24L01_TX_QUEUE_SUB_NAME;
    nrf24l01_tx_msg_queue_attr.cb_mem    = &tx_xQueueBuffer;
    nrf24l01_tx_msg_queue_attr.cb_size   = sizeof(tx_xQueueBuffer);
    nrf24l01_tx_msg_queue_attr.mq_mem    = nrf24l01_tx_msg_data;
    nrf24l01_tx_msg_queue_attr.mq_size   = NRF24L01_QUEUE_MSG_SIZE;
    nrf24l01_tx_msg_queue_id = osMessageQueueNew(NRF24L01_MAX_NUM_MSG, 
                                        NRF24L01_MSG_SIZE, &nrf24l01_tx_msg_queue_attr);  
    //|-- Header 1 --|-- Header 2 --|-- payload length --|--- CMD ---|---- data ----|
    //|--- 1 byte ---|--- 1 byte ---|------ 1 byte ------|-- 1 byte--|-- n bytes ---|


    while(1)
    {
      if(nrf24l01_get_data_from_tx_queue(tx_data))
      {
        if((tx_data[0] == RF_CMD_HEADER1) && (tx_data[1] == RF_CMD_HEADER2))
        {
          nrf24l01_tx_send_in_order_and_get_response(tx_data, RF_CMD_HEADER_SIZE + RF_CMD_PAYLOAD_LEN_SIZE + tx_data[2], rx_data, &rx_len);
          // Transmit to PC for data display
          status = CDC_Transmit_FS(rx_data, rx_len);
          PRINT_INFO_LOG("USB Transmit status - %d\r\n", status):
        }
        
      }
    }
}


/******************************************************************************
* Function : nrf24l01_tx_send_packet_in_order
* Brief    : Send tx packet in order to ensure received ACK payload.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void nrf24l01_tx_send_in_order_and_get_response(uint8_t * tx_data, uint8_t tx_len, uint8_t * rx_data, uint8_t * rx_len)
{
  bool is_received_response = false;
  uint8_t status;
  uint32_t start_tick;

  for(uint8_t idx=0; idx<NRF24L01_MAX_NUM_SEND_ORDER; idx++)
  {
    nrf24l01_send_data_fifo(tx_data, tx_len);

    // Check status register for the TX and MAX_RT ready
    start_tick = HAL_GetTick();

    while(1)
    {
      status = nrf24l01_get_status();
      if((status & ((~NRF24L01_INT_MAX_RT_MASK) | (~NRF24L01_INT_TX_DS_MASK))) || ((HAL_GetTick() - start_tick) > RF_TRANSFER_READY_TIMEOUT_MS))
      {
        break;
      }
      osDelay(2);
    }

    // Wait fro response from MCU
    if(!nrf24l01_data_wait_new_data())
    {
      // Nothing to do here
    }
    else
    {
      status = nrf24l01_read_fifo_status();
      if (status != 0x01)
      {
        // Cache the data received from MCU
        if (nrf24l01_read_data_fifo(rx_data, rx_len))
        {
          PRINT_INFO_LOG("Received packet from MCU!\r\n");
          break;
        }
      }
    }

    nrf24l01_clear_all_flags();
  }
}
#endif

/*************** END OF FILES *************************************************/
