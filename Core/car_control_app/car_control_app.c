/*******************************************************************************
* Title                 :   Car control app source file
* Filename              :   car_control_app.c
* Author                :   Nhan
* Origin Date           :   Oct 19th 2021
* Notes                 :   None
*******************************************************************************/

/*******************************************************************************
* INCLUDE
*******************************************************************************/
#include "car_control_app.h"

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define CAR_CONTROL_THREAD_NAME             ("car_control_thread")
#define CAR_CONTROL_THREAD_STACK_SIZE       (500) // Bytes
#define CAR_CONTROL_THREAD_PRIORITY         ((osPriority_t) osPriorityHigh)

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
static const osThreadAttr_t car_control_thread_attr =
{
    .name       = CAR_CONTROL_THREAD_NAME,
    .priority   = CAR_CONTROL_THREAD_PRIORITY,
    .stack_size = CAR_CONTROL_THREAD_STACK_SIZE,
};
static __attribute__((unused)) osThreadId_t car_control_thread_handle;

/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
static void car_control_thread_entry (void *argument);

/******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/
/******************************************************************************
* Function : void car_control_thread_app_init(void)
* Brief    : Create new thread for Car control module.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void car_control_thread_app_init(void)
{
  // Creating car control app thread
  car_control_thread_handle = osThreadNew(car_control_thread_entry, NULL, &car_control_thread_attr);
}

/******************************************************************************
* STATIC FUNCTIONS
*******************************************************************************/
/******************************************************************************
* Function : static void car_control_thread_entry (void *argument)
* Brief    : Thread entry handles all operations of Car control module.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void car_control_thread_entry (void *argument)
{
  uint8_t data[NRF24L01_STATIC_PAYLOAD_LEN];
  osStatus_t status;   

  // Init the car control driver 
  car_control_init();
  car_control_pwm_start();

  while(1)
  {
    status = nrf24l01_get_chunk_data((nrf24l01_data_t *) data);

    if(status == osOK)
    {
        // Handle all operations of RF car here
        if(data[0] == NRF24L01_PAYLOAD_HEADER_1 
          && data[1] == NRF24L01_PAYLOAD_HEADER_2)
        {
          car_control_exec_cmd((car_ctrl_t)data[2]);
        }
        
    }
  }
}
/*************** END OF FILES *************************************************/