/*******************************************************************************
* Title                 :   Print log debug source file
* Filename              :   log_debug.c
* Author                :   Nhan
* Origin Date           :   09/10/2021
* Notes                 :   None
*******************************************************************************/

/*******************************************************************************
* INCLUDE
*******************************************************************************/
#include <string.h>
#include "main.h"
#include "log_debug.h"

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define LOG_DEBUG_UART_INSTANCE    (USART1)
#define LOG_DEBUG_UART_BAUDRATE    (115200)

#define LOG_DEBUG_UART_TX_PIN      (UART_TX_Pin)         
#define LOG_DEBUG_UART_TX_PORT     (UART_TX_GPIO_Port)
#define LOG_DEBUG_UART_RX_PIN      (UART_RX_Pin)         
#define LOG_DEBUG_UART_RX_PORT     (UART_RX_GPIO_Port)

#define LOG_DEBUG_UART_MAX_BUF     (128)

/******************************************************************************
* PREPROCESSOR MACROS
*******************************************************************************/


/******************************************************************************
* TYPEDEFS
*******************************************************************************/


/******************************************************************************
* VARIABLE DEFINITIONS
*******************************************************************************/
#if (LOG_DEBUG_EN)
UART_HandleTypeDef huart1;

#endif

static char tx_buff[LOG_DEBUG_UART_MAX_BUF];

/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void log_debug_gpio_init(void);
void log_debug_uart_init(void);
void log_debug_gpio_deinit(void);
void log_debug_uart_deinit(void);

/******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/
/******************************************************************************
* Function : void log_debug_init(void)
* Brief    : Initialize print log debug over uart
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void log_debug_init(void)
{
#if (LOG_DEBUG_EN)
    log_debug_gpio_init();
    log_debug_uart_init();
#endif
}

/******************************************************************************
* Function : void log_debug_deinit(void)
* Brief    : DeInitialize print log debug over uart
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void log_debug_deinit(void)
{
#if (LOG_DEBUG_EN)
    log_debug_gpio_deinit();
    log_debug_uart_deinit();
#endif
}

#if (LOG_DEBUG_EN)
  #if defined(__GNUC__)
  int _write(int fd, char * ptr, int len)
  {
    memset(tx_buff, 0, LOG_DEBUG_UART_MAX_BUF);

    if(len > LOG_DEBUG_UART_MAX_BUF)
    {
      memcpy(tx_buff, ptr, LOG_DEBUG_UART_MAX_BUF);
    }
    else
    {
      memcpy(tx_buff, ptr, len);
    }
    
    HAL_UART_Transmit(&huart1, (uint8_t *) tx_buff, strlen(tx_buff) + len, HAL_MAX_DELAY);

    return (strlen(tx_buff));
  }
  #elif defined (__ICCARM__)
  #include "LowLevelIOInterface.h"
  size_t __write(int handle, const unsigned char * buffer, size_t size)
  {
    HAL_UART_Transmit(&huart1, (uint8_t *) buffer, size, HAL_MAX_DELAY);
    return size;
  }
  #elif defined (__CC_ARM)
  int fputc(int ch, FILE *f)
  {
      HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
      return ch;
  }
  #endif
#endif

/******************************************************************************
* STATIC FUNCTIONS
*******************************************************************************/
/******************************************************************************
* Function : void log_debug_gpio_init(void)
* Brief    : Initialize GPIO for log debug module
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void log_debug_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = LOG_DEBUG_UART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LOG_DEBUG_UART_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LOG_DEBUG_UART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LOG_DEBUG_UART_RX_PORT, &GPIO_InitStruct);
}

/******************************************************************************
* Function : void log_debug_uart_init(void)
* Brief    : Initialize UART peripheral for log debug module
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void log_debug_uart_init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();

    huart1.Instance = LOG_DEBUG_UART_INSTANCE;
    huart1.Init.BaudRate = LOG_DEBUG_UART_BAUDRATE;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/******************************************************************************
* Function : void log_debug_gpio_deinit(void)
* Brief    : DeInitialize GPIO peripheral for log debug module
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void log_debug_gpio_deinit(void)
{
    HAL_GPIO_DeInit(LOG_DEBUG_UART_RX_PORT, LOG_DEBUG_UART_TX_PIN);
    HAL_GPIO_DeInit(LOG_DEBUG_UART_RX_PORT, LOG_DEBUG_UART_RX_PIN);
}

/******************************************************************************
* Function : void log_debug_uart_deinit(void)
* Brief    : DeInitialize UART peripheral for log debug module
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void log_debug_uart_deinit(void)
{
    __HAL_RCC_USART1_CLK_DISABLE();

    if (HAL_UART_DeInit(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}


/*************** END OF FILES *************************************************/
