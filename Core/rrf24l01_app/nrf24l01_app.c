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
#define NRF24L01_THREAD_STACK_SIZE       (500) // Bytes
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
  nrf24l01_data_t nrf24l01_data;
  osStatus_t status;
  static uint8_t data[128];
  uint8_t data_len;

  // Init the Queue for storing all data
  // received from NRF24L01
  nrf24l01_msg_queue_attr.name      = NRF24L01_QUEUE_SUB_NAME;
  nrf24l01_msg_queue_attr.cb_mem    = &xQueueBuffer;
  nrf24l01_msg_queue_attr.cb_size   = sizeof(xQueueBuffer);
  nrf24l01_msg_queue_attr.mq_mem    = nrf24l01_msg_data;
  nrf24l01_msg_queue_attr.mq_size   = NRF24L01_QUEUE_MSG_SIZE;
  nrf24l01_msg_queue_id = osMessageQueueNew(NRF24L01_MAX_NUM_MSG, 
                                      NRF24L01_MSG_SIZE, &nrf24l01_msg_queue_attr);    
  uint8_t value = 0x00;
  uint8_t test_value = 0x00;
  // Init the NRF24L01 driver as Receiver role   
  nrf24l01_init();
  // Temporaily define the NRF24L01 data frame as below
  //|-- Header 1 --|-- Header 2 --|-- NRF24L01Command --|
  //|--- 1 byte ---|--- 1 byte ---|------ 1 byte -------|
  uint16_t idx = 0;
  while(1)
  {
    nrf24l01_data_wait_new_data();

    value = nrf24l01_read_fifo_status();
    if (value != 0x01)
    {
      if(nrf24l01_read_data_fifo(data, &data_len))
      {
        PRINT_INFO_LOG("Data length: %d\r\n", data_len);
        for(uint8_t i=0; i<data_len; i++)
        {
          PRINT_INFO_LOG("Data: %d\r\n", data[i]);
        }
        // Send ACK payload back to the transmitter 
        nrf24l01_write_back_ack_payload(NRF24L01_DATA_PIPE_1, NRF24L01_RX_ACK_PAYLOAD, 10);
      }
      nrf24l01_clear_all_flags();
      memset(data, 0, sizeof(data));
    }

    // nrf24l01_test_read_register(0x07, &value, 1);
    // if(value != 0x0e)
    // {
    //   PRINT_INFO_LOG("Status register value: %d\r\n", value);
    //   value = 0x40;
    //   nrf24l01_test_write_register(0x07, &value, 1);
    // }

		PRINT_INFO_LOG("Receive message from Tx device: %d\r\n", idx++); 
    
    // Read data from RX FIFO, need to read the number of available
    // data in RX FIFO firstly if using Dynamic payload length
    // For Static payload length, just read the data without checking
    // the amount of data since it's already known
    // nrf24l01_read_data_fifo((uint8_t *)&nrf24l01_data, sizeof(nrf24l01_data));

    // // Put the data received from NRF24L01 into Queue
    // status = osMessageQueuePut(nrf24l01_msg_queue_id, &nrf24l01_data, 0xff, 0);
    // if(osOK != status)
    // {
    //   // Reset the Queue in case encountered FULL FIFO scenario
    //   osMessageQueueReset(nrf24l01_msg_queue_id);
    //   PRINT_ERROR_LOG("Fail to put data into queue\r\n");
    // }
  }
}
/*************** END OF FILES *************************************************/
