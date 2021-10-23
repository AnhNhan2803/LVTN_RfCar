/*******************************************************************************
* Title                 :   NRF24L01 driver source file
* Filename              :   nrf24l01.c
* Author                :   Nhan
* Origin Date           :   Oct 17th 2021
* Notes                 :   None
*******************************************************************************/

/******************************************************************************
 * INCLUDES
 *******************************************************************************/
#include "nrf24l01.h"

/******************************************************************************
 * CONFIGURATION CONSTANTS
 *******************************************************************************/
#define NRF24L01_SPI_TIMEOUT   (1000)

// NRF24L01 COMMAND LIST
#define NRF24L01_CMD_R_REGISTER            0x00
#define NRF24L01_CMD_W_REGISTER            0x20
#define NRF24L01_CMD_R_RX_PAYLOAD          0x61
#define NRF24L01_CMD_W_TX_PAYLOAD          0xA0
#define NRF24L01_CMD_FLUSH_TX              0xE1 
#define NRF24L01_CMD_FLUSH_RX              0xE2
#define NRF24L01_CMD_REUSE_TX_PL           0xE3
#define NRF24L01_CMD_R_RX_PL_WID           0x60
#define NRF24L01_CMD_W_ACK_PAYLOAD         0xA8
#define NRF24L01_CMD_W_TX_PAYLOAD_NO_ACK   0xB0
#define NRF24L01_CMD_NOP                   0xFF

// NRF24L01 REGISTER LIST
#define NRF24L01_REG_CONFIG                0x00
#define NRF24L01_REG_EN_AA                 0x01
#define NRF24L01_REG_EN_RXADDR             0x02
#define NRF24L01_REG_SETUP_AW              0x03
#define NRF24L01_REG_SETUP_RETR            0x04
#define NRF24L01_REG_RF_CH                 0x05
#define NRF24L01_REG_RF_SETUP              0x06  
#define NRF24L01_REG_STATUS                0x07
#define NRF24L01_REG_OBSERVE_TX            0x08
#define NRF24L01_REG_RPD                   0x09
#define NRF24L01_REG_RX_ADDR_P0            0x0A
#define NRF24L01_REG_RX_ADDR_P1            0x0B
#define NRF24L01_REG_RX_ADDR_P2            0x0C
#define NRF24L01_REG_RX_ADDR_P3            0x0D
#define NRF24L01_REG_RX_ADDR_P4            0x0E
#define NRF24L01_REG_RX_ADDR_P5            0x0F
#define NRF24L01_REG_TX_ADDR               0x10
#define NRF24L01_REG_RX_PW_P0              0x11
#define NRF24L01_REG_RX_PW_P1              0x12
#define NRF24L01_REG_RX_PW_P2              0x13
#define NRF24L01_REG_RX_PW_P3              0x14
#define NRF24L01_REG_RX_PW_P4              0x15
#define NRF24L01_REG_RX_PW_P5              0x16
#define NRF24L01_REG_FIFO_STATUS           0x17
#define NRF24L01_REG_DYNPD                 0x1C
#define NRF24L01_REG_FEATURE               0x1D

/******************************************************************************
 * PREPROCESSOR MACROS
 *******************************************************************************/
#define NRF24L01_SPI_CS_LOW()  HAL_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_Port, NRF24L01_SPI_CS_Pin, GPIO_PIN_RESET)
#define NRF24L01_SPI_CS_HIGH() HAL_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_Port, NRF24L01_SPI_CS_Pin, GPIO_PIN_SET)
#define NRF24L01_SPI_CE_LOW()  HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_RESET)
#define NRF24L01_SPI_CE_HIGH() HAL_GPIO_WritePin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin, GPIO_PIN_SET)

/******************************************************************************
 * TYPEDEFS
 *******************************************************************************/


/******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************/
static void nrf24l01_gpio_init(void);
static void nrf24l01_gpio_deinit(void);
static void nrf24l01_spi_init(void);
static void nrf24l01_spi_deinit(void);
static void nrf24l01_set_data_regs(uint8_t reg, uint8_t mask, uint8_t value);
static void nrf24l01_read_regs(uint8_t reg, uint8_t * pdata, uint32_t length);
static void nrf24l01_write_regs(uint8_t reg, uint8_t * pdata, uint32_t length);

/******************************************************************************
 * VARIABLE DEFINITIONS
 *******************************************************************************/
SPI_HandleTypeDef nrf24l01_spi;
// Array stored 4 bytes address for TX/RX communication
static uint8_t nrf24l01_adrr[NRF24L01_ADDR_WIDTH] = {0x77, 0x35, 0xF0, 0xD3};
// Create semaphore for RX data ready synchorize
static osSemaphoreId_t nrf24l01_data_sem;

/******************************************************************************
 * PUBLIC FUNCTIONS
 *******************************************************************************/
/******************************************************************************
* Function : void nrf24l01_data_ready_callback(void)
* Brief    : Callback function to release semaphore from interrupt
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_data_ready_callback(void)
{
  osSemaphoreRelease(nrf24l01_data_sem);
}

/******************************************************************************
* Function : void nrf24l01_data_wait_new_data(void)
* Brief    : Wait for a semaphore from NRF24L01 interrupt.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_data_wait_new_data(void)
{
  osSemaphoreAcquire(nrf24l01_data_sem, osWaitForever);
}

/******************************************************************************
* Function : void nrf24l01_init(void)
* Brief    : Initialize NRF24L01 driver 
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_init(void)
{
  nrf24l01_gpio_init();
  nrf24l01_spi_init();
  osDelay(2);
  // Exit the power down mode
  nrf24l01_enter_power_down_mode(false);
  osDelay(2);
  /* Config all parameters here */
  // Enable data pipe0
  nrf24l01_enable_rx_address(NRF24L01_DATA_PIPE_0);
  // Set the address width for TX/RX
  nrf24l01_set_address_width(NRF24L01_ADDR_WIDTH_4BYTES);
  // Set TX and RX address
  nrf24l01_set_rx_address(NRF24L01_DATA_PIPE_0, nrf24l01_adrr, sizeof(nrf24l01_adrr));
  nrf24l01_set_tx_address(nrf24l01_adrr, sizeof(nrf24l01_adrr));
  // Enable Dynamic payload length for receiver device
  // nrf24l01_rx_enable_dynamic_payload_length(NRF24L01_DATA_PIPE_0);
  // Enable Static payload length for receiver device
  nrf24l01_set_static_payload_length(NRF24L01_DATA_PIPE_0, NRF24L01_STATIC_PAYLOAD_LEN);
  // Setup the on air data rate
  nrf24l01_set_data_rate(NRF24L01_DR_1MBPS);
  // Enable interrupt for RX data ready
  nrf24l01_enable_interrupt(NRF24L01_INT_RX_DR_ENABLE);

  // Enter Rx mode
  nrf24l01_enter_rx_mode();

  nrf24l01_data_sem = osSemaphoreNew(1, 0, NULL);
}

/******************************************************************************
* Function : void nrf24l01_deinit(void)
* Brief    : DeInitialize NRF24L01 driver 
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_deinit(void)
{
  nrf24l01_spi_deinit();
  nrf24l01_gpio_deinit();
}

/******************************************************************************
* Function : void nrf24l01_read_data_fifo(uint8_t * pdata)
* Brief    : Read data from Receiver FIFO in RX mode.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_read_data_fifo(uint8_t * pdata, uint8_t len)
{
  uint8_t read_cmd;
  uint8_t rx_buff[128]; 
  read_cmd = NRF24L01_CMD_R_RX_PAYLOAD;

  NRF24L01_SPI_CS_LOW();
  HAL_SPI_TransmitReceive(&nrf24l01_spi, &read_cmd, rx_buff, len + 1, NRF24L01_SPI_TIMEOUT);
  NRF24L01_SPI_CS_HIGH();
  // TODO: Add max length condition to avoid
  // buffer overflow
  memcpy(pdata, &rx_buff[1], len);
}

/******************************************************************************
* Function : void nrf24l01_send_data_fifo(uint8_t* pdata, uint8_t len)
* Brief    : Send data to Transmiting FIFO in TX mode.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_send_data_fifo(uint8_t* pdata, uint8_t len)
{
  uint8_t tx_buff[128]; 
  tx_buff[0] = NRF24L01_CMD_W_TX_PAYLOAD;
  // TODO: Add max length condition to avoid
  // buffer overflow
  memcpy(&tx_buff[1], pdata, len);

  NRF24L01_SPI_CS_LOW();
  HAL_SPI_Transmit(&nrf24l01_spi, tx_buff, len + 1, NRF24L01_SPI_TIMEOUT);
  NRF24L01_SPI_CS_HIGH();
}

/******************************************************************************
* Function : void nrf24l01_flush_tx_fifo(void)
* Brief    : Flush the RX TX FIFO.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_flush_tx_fifo(void)
{
  uint8_t cmd;
  cmd = NRF24L01_CMD_FLUSH_TX;
  NRF24L01_SPI_CS_LOW();
  HAL_SPI_Transmit(&nrf24l01_spi, &cmd, 1, NRF24L01_SPI_TIMEOUT);
  NRF24L01_SPI_CS_HIGH();
}

/******************************************************************************
* Function : void nrf24l01_flush_rx_fifo(void)
* Brief    : Flush the RX RX FIFO.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_flush_rx_fifo(void)
{
  uint8_t cmd;
  cmd = NRF24L01_CMD_FLUSH_RX;
  NRF24L01_SPI_CS_LOW();
  HAL_SPI_Transmit(&nrf24l01_spi, &cmd, 1, NRF24L01_SPI_TIMEOUT);
  NRF24L01_SPI_CS_HIGH();
}

/******************************************************************************
* Function : void nrf24l01_exit_power_down_mode(bool en)
* Brief    : Exit from Power down mode in initialization phase.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_enter_power_down_mode(bool en)
{
  if(en)
  {
    nrf24l01_set_data_regs(NRF24L01_REG_CONFIG, NEF24L01_EXIT_POWER_DONW_MASK, NEF24L01_EXIT_POWER_DONW_DISABLE);
  }
  else
  {
    nrf24l01_set_data_regs(NRF24L01_REG_CONFIG, NEF24L01_EXIT_POWER_DONW_MASK, NEF24L01_EXIT_POWER_DONW_ENABLE);
  }
}

/******************************************************************************
* Function : void nrf24l01_enter_stanby_I_mode(void)
* Brief    : Enter Standby I mode.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_enter_stanby_I_mode(void)
{
  // Set CE pin to LOW to enter Standby I mode
  NRF24L01_SPI_CE_LOW();
}

/******************************************************************************
* Function : void nrf24l01_enter_rx_mode(void)
* Brief    : Enter RX mode.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_enter_rx_mode(void)
{
  // Set CE pin to HIGH and PRIM_RX to 1
  NRF24L01_SPI_CE_HIGH();
  nrf24l01_txrx_control(NEF24L01_TXRX_CTRL_RX_ENABLE);
}

/******************************************************************************
* Function : void nrf24l01_exit_power_down_mode(void)
* Brief    : Exit from Power down mode in initialization phase.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_pre_condition_enter_tx_mode(void)
{
  // Set CE pin to HIGH and PRIM_RX to 0
  NRF24L01_SPI_CE_HIGH();
  nrf24l01_txrx_control(NEF24L01_TXRX_CTRL_TX_ENABLE);
}

/******************************************************************************
* Function : void nrf24l01_txrx_control(uint8_t mode)
* Brief    : Config the TX/RX control.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_txrx_control(uint8_t mode)
{
  nrf24l01_set_data_regs(NRF24L01_REG_CONFIG, NEF24L01_TXRX_CTRL_MASK, mode);
}

/******************************************************************************
* Function : void nrf24l01_set_data_rate(nrf24l01_dr_t data_rate)
* Brief    : Set on air data rate for TX/RX transaction.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_set_data_rate(nrf24l01_dr_t data_rate)
{
  switch (data_rate)
  {
    case NRF24L01_DR_1MBPS:
      nrf24l01_set_data_regs(NRF24L01_REG_RF_SETUP, NEF24L01_AIR_DR_MASK, NRF24L01_AIR_DR_1MBPS);
      break;

    case NRF24L01_DR_2MBPS:
      nrf24l01_set_data_regs(NRF24L01_REG_RF_SETUP, NEF24L01_AIR_DR_MASK, NRF24L01_AIR_DR_2MBPS);
      break;

    case NRF24L01_DR_250kBPS:
      nrf24l01_set_data_regs(NRF24L01_REG_RF_SETUP, NEF24L01_AIR_DR_MASK, NRF24L01_AIR_DR_250kBPS);
      break;

    default:
      break;
  }
}

/******************************************************************************
* Function : void nrf24l01_set_transmit_power(nrf24l01_tx_pwr_t tx_pwr)
* Brief    : Set tx output power in dBm.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_set_transmit_power(nrf24l01_tx_pwr_t tx_pwr)
{
  nrf24l01_set_data_regs(NRF24L01_REG_RF_SETUP, NEF24L01_RF_TX_PWR_MASK, (uint8_t)tx_pwr);
}

/******************************************************************************
* Function : void nrf24l01_set_rf_channel_freq(uint8_t value)
* Brief    : Set tx output power in dBm.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_set_rf_channel_freq(uint8_t value)
{
  nrf24l01_set_data_regs(NRF24L01_REG_RF_CH, (uint8_t)NRF24L01_CHANNEL_FREQ_MASK, value);
}

/******************************************************************************
* Function : void nrf24l01_enable_rx_adrress(rx_data_pipe_t data_pipe)
* Brief    : ENable RX addresses.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_enable_rx_address(rx_data_pipe_t data_pipe)
{
  nrf24l01_set_data_regs(NRF24L01_REG_EN_RXADDR, ~(1 << data_pipe), 1 << data_pipe);
}

/******************************************************************************
* Function : void nrf24l01_set_address_width(nrf24l01_aw_t aw)
* Brief    : Set address width for the transaction between TX and RX device.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_set_address_width(nrf24l01_aw_t aw)
{
  nrf24l01_set_data_regs(NRF24L01_REG_SETUP_AW, NRF24L01_ADDRESS_WIDTH_MASK, (uint8_t)aw);
}

/******************************************************************************
* Function : void nrf24l01_set_rx_address(rx_data_pipe_t data_pipe, uint8_t * paddr, uint8_t len)
* Brief    : Set RX address for NRF24L01.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_set_rx_address(rx_data_pipe_t data_pipe, uint8_t * paddr, uint8_t len)
{
  nrf24l01_write_regs(NRF24L01_REG_RX_ADDR_P0 + data_pipe, paddr, len);
}

/******************************************************************************
* Function : void nrf24l01_set_tx_address(uint8_t * paddr, uint8_t len)
* Brief    : Set TX address for NRF24L01.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_set_tx_address(uint8_t * paddr, uint8_t len)
{
  nrf24l01_write_regs(NRF24L01_REG_TX_ADDR, paddr, len);
}

/******************************************************************************
* Function : void nrf24l01_set_static_payload_length(rx_data_pipe_t data_pipe, uint8_t len)
* Brief    : Set payload length for the RF transaction in static mode.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_set_static_payload_length(rx_data_pipe_t data_pipe, uint8_t len)
{
  nrf24l01_set_data_regs(NRF24L01_REG_RX_PW_P0 + data_pipe, NRF24L01_RX_NUM_PAYLOAD_MASK, len);
}

/******************************************************************************
* Function : void nrf24l01_rx_enable_dynamic_payload_length(rx_data_pipe_t data_pipe)
* Brief    : enable RX dynamic payload length mode.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_rx_enable_dynamic_payload_length(rx_data_pipe_t data_pipe)
{
  // Firstly, Enable the EN_DPL bit in FEATURE register
  nrf24l01_set_data_regs(NRF24L01_REG_FEATURE, NRF24L01_DYNAMIC_PAYLOAD_MASK, NRF24L01_DYNAMIC_PAYLOAD_ENABLE);

  // Set the corresponding bii to DYNPD register
  nrf24l01_set_data_regs(NRF24L01_REG_DYNPD, ~(1 << data_pipe), 0x01);
}

/******************************************************************************
* Function : void nrf24l01_tx_enable_dynamic_payload_length(void)
* Brief    : enable TX dynamic payload length mode.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_tx_enable_dynamic_payload_length(void)
{
  // Firstly, Enable the EN_DPL bit in FEATURE register
  nrf24l01_set_data_regs(NRF24L01_REG_FEATURE, NRF24L01_DYNAMIC_PAYLOAD_MASK, NRF24L01_DYNAMIC_PAYLOAD_ENABLE);

  // Set the DPL_P0 bit to HIGH to enable the DPL feature in TX side
  nrf24l01_set_data_regs(NRF24L01_REG_DYNPD, NRF24L01_DYNAMIC_PAYLOAD_P0_MASK, 0x01);
}

/******************************************************************************
* Function : uint8_t nrf24l01_num_available_data_fifo(void)
* Brief    : Read number of available data in .
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
uint8_t nrf24l01_num_available_data_fifo(void)
{
  uint8_t ret;
  uint8_t read_cmd;
  uint8_t rx_buff[2]; 
  // Consider to edit this line based on the datasheet
  read_cmd = NRF24L01_CMD_R_RX_PL_WID;

  NRF24L01_SPI_CS_LOW();
  HAL_SPI_TransmitReceive(&nrf24l01_spi, &read_cmd, rx_buff, 2, NRF24L01_SPI_TIMEOUT);
  NRF24L01_SPI_CS_HIGH();
  ret = rx_buff[1];

  return ret;
}

/******************************************************************************
* Function : void nrf24l01_enable_interrupt(uint8_t int_mask)
* Brief    : Enable the interrupt according to interrupt mask.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_enable_interrupt(uint8_t int_type)
{
  switch(int_type)
  {
    case NRF24L01_INT_RX_DR_ENABLE:
      nrf24l01_set_data_regs(NRF24L01_REG_CONFIG, NRF24L01_INT_RX_DR_MASK, NRF24L01_INT_RX_DR_ENABLE);
      break;

    case NRF24L01_INT_TX_DS_ENABLE:
      nrf24l01_set_data_regs(NRF24L01_REG_CONFIG, NRF24L01_INT_TX_DS_MASK, NRF24L01_INT_TX_DS_ENABLE);
      break;

    case NRF24L01_INT_MAX_RT_ENABLE:
      nrf24l01_set_data_regs(NRF24L01_REG_CONFIG, NRF24L01_INT_MAX_RT_MASK, NRF24L01_INT_MAX_RT_ENABLE);
      break;

    default:
      break;
  }
}

/******************************************************************************
 * STATIC FUNCTIONS
 *******************************************************************************/
/******************************************************************************
* Function : void nrf24l01_gpio_init(void)
* Brief    : Initialize GPIO for NRF24L01
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void nrf24l01_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = NRF24L01_SPI_SCK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NRF24L01_SPI_SCK_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = NRF24L01_SPI_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NRF24L01_SPI_MOSI_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = NRF24L01_SPI_MISO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(NRF24L01_SPI_MISO_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = NRF24L01_SPI_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(NRF24L01_SPI_CS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = NRF24L01_CE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(NRF24L01_CE_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = NRF24L01_IRQ_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(NRF24L01_IRQ_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

/******************************************************************************
* Function : void nrf24l01_gpio_deinit(void)
* Brief    : DeInitialize GPIO for NRF24L01
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void nrf24l01_gpio_deinit(void)
{
    HAL_GPIO_DeInit(NRF24L01_SPI_SCK_GPIO_Port, NRF24L01_SPI_SCK_Pin);
    HAL_GPIO_DeInit(NRF24L01_SPI_MOSI_GPIO_Port, NRF24L01_SPI_MOSI_Pin);
    HAL_GPIO_DeInit(NRF24L01_SPI_MISO_GPIO_Port, NRF24L01_SPI_MISO_Pin);
    HAL_GPIO_DeInit(NRF24L01_SPI_CS_GPIO_Port, NRF24L01_SPI_CS_Pin);
    HAL_GPIO_DeInit(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin);
    HAL_GPIO_DeInit(NRF24L01_IRQ_GPIO_Port, NRF24L01_IRQ_Pin);
    HAL_NVIC_DisableIRQ(EXTI2_IRQn);
}

/******************************************************************************
* Function : void nrf24l01_spi_init(void)
* Brief    : Initialize SPI peripheral for NRF24L01
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void nrf24l01_spi_init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();

    nrf24l01_spi.Instance = SPI1;
    nrf24l01_spi.Init.Mode = SPI_MODE_MASTER;
    nrf24l01_spi.Init.Direction = SPI_DIRECTION_2LINES;
    nrf24l01_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    nrf24l01_spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    nrf24l01_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    nrf24l01_spi.Init.NSS = SPI_NSS_SOFT;
    nrf24l01_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    nrf24l01_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    nrf24l01_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    nrf24l01_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    nrf24l01_spi.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&nrf24l01_spi) != HAL_OK)
    {
        Error_Handler();
    }
}

/******************************************************************************
* Function : void nrf24l01_spi_deinit(void)
* Brief    : DeInitialize SPI peripheral for NRF24L01
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void nrf24l01_spi_deinit(void)
{
    if (HAL_SPI_DeInit(&nrf24l01_spi) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_RCC_SPI1_CLK_DISABLE();
}

/******************************************************************************
* Function : nrf24l01_set_data_regs
* Brief    : API to set specific bit in NRF24L01 registers
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void nrf24l01_set_data_regs(uint8_t reg, uint8_t mask, uint8_t value)
{
  uint8_t data;
  nrf24l01_read_regs(reg, &data, 1);
  data &= mask;
  data |= value;
  nrf24l01_write_regs(reg, &data, 1);
}

/******************************************************************************
* Function : nrf24l01_write_regs
* Brief    : API to write to registers of NRF24L01
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void nrf24l01_write_regs(uint8_t reg, uint8_t * pdata, uint32_t length)
{
  uint8_t write_cmd;
  uint8_t tx_buff[128 + 1]; // Add 1 byte for write command

  // Consider to edit this line based on the datasheet
  write_cmd = NRF24L01_CMD_W_REGISTER | reg;
  tx_buff[0] = write_cmd;
  // TODO: Firstly need to check the data length
  // to void encountered buffer overflow
  memcpy(&tx_buff[1], pdata, length);
	
  NRF24L01_SPI_CS_LOW();
  HAL_SPI_Transmit(&nrf24l01_spi, tx_buff, length + 1, NRF24L01_SPI_TIMEOUT);
  NRF24L01_SPI_CS_HIGH();
}

/******************************************************************************
* Function : nrf24l01_read_regs
* Brief    : API to read from registers of NRF24L01
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void nrf24l01_read_regs(uint8_t reg, uint8_t * pdata, uint32_t length)
{
  uint8_t read_cmd;
  uint8_t rx_buff[128 + 1]; // Add 1 byte for read command, no dummy byte received
  // Consider to edit this line based on the datasheet
  read_cmd = NRF24L01_CMD_R_REGISTER | reg;

  NRF24L01_SPI_CS_LOW();
  HAL_SPI_TransmitReceive(&nrf24l01_spi, &read_cmd, rx_buff, length + 1, NRF24L01_SPI_TIMEOUT);
  NRF24L01_SPI_CS_HIGH();
  memcpy(pdata, &rx_buff[1], length);
}
