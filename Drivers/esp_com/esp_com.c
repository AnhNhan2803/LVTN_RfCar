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
#include "esp_com.h"

/******************************************************************************
* PREPROCESSOR CONSTANTS
*******************************************************************************/
#define ESP_UART_INSTANCE    (USART3)
#define ESP_UART_BAUDRATE    (115200)

#define ESP_UART_TX_PIN      (ESP_UART_TX_Pin)         
#define ESP_UART_TX_PORT     (ESP_UART_TX_GPIO_Port)
#define ESP_UART_RX_PIN      (ESP_UART_RX_Pin)         
#define ESP_UART_RX_PORT     (ESP_UART_RX_GPIO_Port)

#if (DEVICE_ROLE == DEVICE_ROLE_RX)
#define ESP_COM_CS_INIT()                 esp_com_mutex = osMutexNew(NULL);                                  
#define ESP_COM_CS_ENTER(timeout_ms)      osMutexAcquire(esp_com_mutex, timeout_ms)     // critical section enter
#define ESP_COM_CS_EXIT()                 osMutexRelease(esp_com_mutex)                 // critical section end

/******************************************************************************
* PREPROCESSOR MACROS
*******************************************************************************/


/******************************************************************************
* TYPEDEFS
*******************************************************************************/


/******************************************************************************
* VARIABLE DEFINITIONS
*******************************************************************************/
UART_HandleTypeDef huart3;
esp_com_rx_params_t esp_com = {
    .rx_idx = 0,
    .rx_len = 0,
    .is_rx_enter_frame = false,
    .payload_len = 0,
};
static osSemaphoreId_t esp_data_sem;
FIFO_BUF_DEF(esp_com_data, ESP_COM_RX_MAX_NUM_SSID, ESP_COM_RX_MAX_PACKET_SIZE);
static osMutexId_t esp_com_mutex = NULL;

/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void esp_gpio_init(void);
void esp_uart_init(void);
void esp_gpio_deinit(void);
void esp_uart_deinit(void);
#endif

/******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/
/******************************************************************************
* Function : void esp_com_init(void)
* Brief    : Initialize esp communication via UART
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void esp_com_init(void)
{   
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
    esp_data_sem = osSemaphoreNew(100, 0, NULL);
    ESP_COM_CS_INIT();
    esp_gpio_init();
    esp_uart_init();
#endif
}

/******************************************************************************
* Function : void esp_com_deinit(void)
* Brief    : DeInitialize print log debug over uart
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void esp_com_deinit(void)
{
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
    esp_gpio_deinit();
    esp_uart_deinit();
#endif
}

/******************************************************************************
* Function : void esp_com_wait_sem(void)
* Brief    : Wait for semaphore from UART RX interrupt
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void esp_com_wait_sem(void)
{
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
    osSemaphoreAcquire(esp_data_sem, osWaitForever);
#endif
}

/******************************************************************************
* Function : bool esp_com_put_data_into_queue(uint8_t * pdata)
* Brief    : Put data into a ring buffer
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void esp_com_release_sem(void)
{
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
    osSemaphoreRelease(esp_data_sem);
#endif
}

/******************************************************************************
* Function : bool esp_com_transmit_data(uint8_t * data, uint8_t len)
* Brief    : Transmit data back via UART
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
bool esp_com_transmit_data(uint8_t * data, uint8_t len)
{
    bool ret = true;
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
    if(HAL_UART_Transmit(&huart3, data, len, 
                        ESP_COM_TRANSFER_TIMEOUT) != HAL_OK)
    {
        ret = false;
    }
#endif
    return ret;
}

/******************************************************************************
* Function : bool esp_com_put_data_into_queue(uint8_t * pdata)
* Brief    : Put data into a ring buffer
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
bool esp_com_put_data_into_queue(uint8_t * pdata)
{
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
    int next;
    // ESP_COM_CS_ENTER(osWaitForever);
    // next is where head will point to after this write. 
    next = esp_com_data.head + esp_com_data.elem;  
    if (next >= esp_com_data.maxlen)
    {
        next = 0;
    }

    // if the head + 1 == tail, circular buffer is full
    if (next == esp_com_data.tail)  
    {
        // Reset all parameters 
        // to make the FIFO
        // work in roll over mode
        esp_com_data.head = 0;
        esp_com_data.tail = 0;
        esp_com_data.cnt = 0;
        // ESP_COM_CS_EXIT();    
        return false;
    }

    memcpy(&(esp_com_data.buffer[esp_com_data.head]), pdata, esp_com_data.elem); // Load data and then move
    esp_com_data.head = next;                                  // head to next data offset.
    esp_com_data.cnt++;
    // ESP_COM_CS_EXIT();
#endif
    return true; 
}

/******************************************************************************
* Function : bool esp_com_get_data_from_queue(uint8_t * pdata)
* Brief    : Get data from a ring buffer
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
bool esp_com_get_data_from_queue(uint8_t * pdata)
{
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
    int next;
    // ESP_COM_CS_ENTER(osWaitForever);
    // if the head == tail, we don't have any data
    if (esp_com_data.head == esp_com_data.tail)  
    {
        // ESP_COM_CS_EXIT();    
        return false;
    }

    // next is where tail will point to after this read.
    next = esp_com_data.tail + esp_com_data.elem;  
    if(next >= esp_com_data.maxlen)
    {
        next = 0;    
    }

    // Read data and then move tail to next offset.
    memcpy(pdata, &(esp_com_data.buffer[esp_com_data.tail]), esp_com_data.elem); 

    esp_com_data.tail = next;      
    esp_com_data.cnt--;       
    // ESP_COM_CS_EXIT();    
#endif
    return true;  
}

/******************************************************************************
* Function : uint8_t esp_com_get_current_data(uint8_t * pdata)
* Brief    : Get available received data via UART
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
uint8_t esp_com_get_current_data(uint8_t * pdata)
{   
    uint8_t ret = 0;
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
    if (esp_com.rx_len > 0)
    {
        memcpy(pdata, esp_com.rx_buff, esp_com.rx_len);
        ret = esp_com.rx_len;
        esp_com.rx_len = 0;
    }    
#endif
    return ret;
}

/******************************************************************************
* STATIC FUNCTIONS
*******************************************************************************/
#if (DEVICE_ROLE == DEVICE_ROLE_RX)
/******************************************************************************
* Function : void esp_gpio_init(void)
* Brief    : Initialize GPIO for esp communication module
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void esp_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /**USART1 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = ESP_UART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ESP_UART_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ESP_UART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ESP_UART_RX_PORT, &GPIO_InitStruct);
}

/******************************************************************************
* Function : void esp_uart_init(void)
* Brief    : Initialize UART peripheral for esp communication module
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void esp_uart_init(void)
{
    __HAL_RCC_USART3_CLK_ENABLE();

    huart3.Instance = ESP_UART_INSTANCE;
    huart3.Init.BaudRate = ESP_UART_BAUDRATE;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    // huart3.RxCpltCallback = esp_rx_transfer_cplt;

    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    HAL_UART_Receive_IT(&huart3, &esp_com.rx_char, 1);  
}

/******************************************************************************
* Function : void log_debug_gpio_deinit(void)
* Brief    : DeInitialize GPIO peripheral for esp communication module
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void esp_gpio_deinit(void)
{
    HAL_GPIO_DeInit(ESP_UART_RX_PORT, ESP_UART_TX_PIN);
    HAL_GPIO_DeInit(ESP_UART_RX_PORT, ESP_UART_RX_PIN);
}

/******************************************************************************
* Function : void esp_uart_deinit(void)
* Brief    : DeInitialize UART peripheral for esp communication module
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void esp_uart_deinit(void)
{
    __HAL_RCC_USART3_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(USART3_IRQn);
    if (HAL_UART_DeInit(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}

/******************************************************************************
* Function : esp_rx_transfer_cplt
* Brief    : Callback function for esp communiction module
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    //|-- Header 1 --|-- Header 2 --|-- payload length --|-- Component ID --|-- Event ID --|------------ data ------------|
    //|--- 1 byte ---|--- 1 byte ---|------ 1 byte ------|----- 1 byte -----|--- 1 byte ---|-- n bytes = payload length --|
    //|------------------------------------------------ message size -----------------------------------------------------|
    if(huart->Instance == USART3)
    {
        esp_com.rx_buff[esp_com.rx_idx++] = esp_com.rx_char;
        if(esp_com.is_rx_enter_frame)
        {
            if(esp_com.payload_len == 0)
            {
                esp_com.payload_len = esp_com.rx_char;
                // To tal length contain of 2 header bytes and 1 payload len byte
                esp_com.rx_len = ESP_COM_HEADER_SIZE + esp_com.payload_len + 1;
            }
            else
            {
                // Reaching the end of RX message frame
                if(esp_com.rx_idx == esp_com.rx_len)
                {
                    esp_com.payload_len = 0;
                    esp_com.rx_idx = 0;
                    esp_com.is_rx_enter_frame = 0;
                    esp_com_release_sem();
                }
            }
        }

        if(esp_com.rx_idx == ESP_COM_HEADER_SIZE)
        {
            // Check the header from the RX message
            if((esp_com.rx_buff[0] == ESP_COM_HEADER1) && (esp_com.rx_buff[1] == ESP_COM_HEADER2)) 
            {
                esp_com.is_rx_enter_frame = true;
            }
            else
            {
                // Shift left to receive next header byte
                esp_com.rx_buff[0] = esp_com.rx_buff[1];
                esp_com.rx_idx--;
            }
        }
    }

    HAL_UART_Receive_IT(&huart3, &esp_com.rx_char, 1);   
}
#endif
/*************** END OF FILES *************************************************/
