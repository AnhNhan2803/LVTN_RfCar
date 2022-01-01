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
const uint8_t nrf24l01_tx_adrr[NRF24L01_ADDR_WIDTH] = {0xE7, 0xE7, 0xE7};
static uint8_t nrf24l01_rx_adrr[NRF24L01_ADDR_WIDTH] = {0xC3, 0xC3, 0xC3};
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
* Function : bool nrf24l01_data_wait_new_data(void)
* Brief    : Wait for a semaphore from NRF24L01 interrupt.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
bool nrf24l01_data_wait_new_data(void)
{
  bool ret = true;
#if (DEVICE_ROLE == DEVICE_ROLE_TX)
  if(osSemaphoreAcquire(nrf24l01_data_sem, 50) != osOK)
  {
    ret = false;
  }
#else
  osSemaphoreAcquire(nrf24l01_data_sem, osWaitForever);
#endif

  return ret;
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
  uint8_t value;
  nrf24l01_gpio_init();
  nrf24l01_spi_init();
  HAL_Delay(2);
  
  nrf24l01_print_all_configurations();

	/* Config all parameters here */ 
#if (DEVICE_ROLE == DEVICE_ROLE_TX)
  // Set 1550uS timeout
	value = 0x4F;
  nrf24l01_write_regs(NRF24L01_REG_SETUP_RETR, &value, sizeof(value));
  // Set frequency for the RF channel
  nrf24l01_set_rf_channel_freq(40);
  // Setup the on air data rate
  nrf24l01_set_data_rate(NRF24L01_DR_2MBPS);
  // Set transmit power for nrf24l01
  nrf24l01_set_transmit_power(NRF24L01_TX_PWR_0dBm);
  // Enable CRC- 2bytes
  nrf24l01_set_data_regs(NRF24L01_REG_CONFIG, ~(0x04), 0x04);
	
  // Set the address width for TX/RX
  nrf24l01_set_address_width(NRF24L01_ADDR_WIDTH_3BYTES);
	
  // Set TX and RX address for opening writing pipe on Data pipe 0
  nrf24l01_set_rx_address(NRF24L01_DATA_PIPE_0, nrf24l01_rx_adrr, sizeof(nrf24l01_rx_adrr));
  nrf24l01_set_tx_address(nrf24l01_rx_adrr, sizeof(nrf24l01_rx_adrr));
  // Set payload size for Writing pipe
  nrf24l01_set_payload_size(NRF24L01_DATA_PIPE_0, NRF24L01_PACKET_MAX_SIZE);
	
  nrf24l01_clear_all_flags();
  // Enable interrupt for RX data ready
  nrf24l01_enable_interrupt(NRF24L01_INT_RX_DR_ENABLE);
	
  // Enable Dynamic payload length
  nrf24l01_enable_dynamic_payloads();
  // Enable ACK payload
  nrf24l01_enable_ack_payload();

	// Set nrf24l01 operation mode as TX  
  nrf24l01_txrx_control(NEF24L01_TXRX_CTRL_TX_ENABLE);
	
  // Exit the power down mode
  nrf24l01_enter_power_down_mode(false);
	
#else
  // Set frequency for the RF channel
  nrf24l01_set_rf_channel_freq(40);
  // Setup the on air data rate
  nrf24l01_set_data_rate(NRF24L01_DR_2MBPS);
  // Set transmit power for nrf24l01
  nrf24l01_set_transmit_power(NRF24L01_TX_PWR_0dBm);

  // Enable CRC- 2bytes
  nrf24l01_set_data_regs(NRF24L01_REG_CONFIG, ~(0x04), 0x04);

  // Set the address width for TX/RX
  nrf24l01_set_address_width(NRF24L01_ADDR_WIDTH_3BYTES);

  // Open Reading pipe on Data pipe 1
  nrf24l01_set_rx_address(NRF24L01_DATA_PIPE_1, nrf24l01_rx_adrr, sizeof(nrf24l01_rx_adrr));
  // Set payload size for Reading pipe
  nrf24l01_set_payload_size(NRF24L01_DATA_PIPE_1, NRF24L01_PACKET_MAX_SIZE);
  nrf24l01_clear_all_flags();
  // Enable interrupt for RX data ready
  nrf24l01_enable_interrupt(NRF24L01_INT_RX_DR_ENABLE);
  // Flush RX and TX FIFO
  // nrf24l01_flush_tx_fifo();
  // nrf24l01_flush_rx_fifo();

  // Set nrf24l01 operation mode as RX  
  nrf24l01_txrx_control(NEF24L01_TXRX_CTRL_RX_ENABLE);

  // Exit the power down mode
  nrf24l01_enter_power_down_mode(false);

  // Enable Dynamic payload length
  nrf24l01_enable_dynamic_payloads();
  // Enable ACK payload
  nrf24l01_enable_ack_payload();
  
  nrf24l01_print_all_configurations();
  nrf24l01_enter_rx_mode();
#endif

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
bool nrf24l01_read_data_fifo(uint8_t * pdata, uint8_t * len)
{
  bool ret = true;
  uint8_t read_cmd;
  uint8_t value;
  uint8_t rx_buff[32]; 

  // Check if it's RX FIOF empty
  nrf24l01_read_regs(NRF24L01_REG_STATUS, &value, 1);
  if(((value & NRF24L01_AVAILABLE_PIPES_MASK) >> 1) < NRF24L01_TOTAL_PIPES)
  {
    // Get the number of available data in RX FIFO
    *len = nrf24l01_num_available_data_fifo();

    // Check if broken packet is received
    if(*len > NRF24L01_MAX_NUM_PACKET)
    {
      *len = 0;
      PRINT_INFO_LOG("RF packet broken");
      nrf24l01_flush_rx_fifo();
    }
    else if(* len == 0)
    {
      PRINT_INFO_LOG("LEN = 0");
      ret = false;
    }
    else
    {
      read_cmd = NRF24L01_CMD_R_RX_PAYLOAD;
      NRF24L01_SPI_CS_LOW();
      HAL_SPI_TransmitReceive(&nrf24l01_spi, &read_cmd, rx_buff, *len + 1, NRF24L01_SPI_TIMEOUT);
      NRF24L01_SPI_CS_HIGH();
      // TODO: Add max length condition to avoid
      // buffer overflow
      memcpy(pdata, &rx_buff[1], *len);
    }
  }
  else
  {
    ret = false;
  }

  return ret;
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
    nrf24l01_set_data_regs(NRF24L01_REG_CONFIG, NEF24L01_EXIT_POWER_DONW_MASK, NEF24L01_EXIT_POWER_DONW_ENABLE);
  }
  else
  {
    nrf24l01_set_data_regs(NRF24L01_REG_CONFIG, NEF24L01_EXIT_POWER_DONW_MASK, NEF24L01_EXIT_POWER_DONW_DISABLE);
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
}

/******************************************************************************
* Function : void nrf24l01_exit_rx_mode(void)
* Brief    : Exit RX mode.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_exit_rx_mode(void)
{
  NRF24L01_SPI_CE_LOW();
  nrf24l01_flush_tx_fifo();
  nrf24l01_flush_rx_fifo();
  osDelay(10);
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
void nrf24l01_set_transmit_power(uint8_t tx_pwr)
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
  nrf24l01_write_regs(NRF24L01_REG_RF_CH, &value, 1);
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
    case 0x00:
      nrf24l01_set_data_regs(NRF24L01_REG_CONFIG, NRF24L01_INT_RX_DR_MASK, NRF24L01_INT_RX_DR_ENABLE);
      nrf24l01_set_data_regs(NRF24L01_REG_CONFIG, NRF24L01_INT_TX_DS_MASK, NRF24L01_INT_TX_DS_DISABLE);
      nrf24l01_set_data_regs(NRF24L01_REG_CONFIG, NRF24L01_INT_MAX_RT_MASK, NRF24L01_INT_MAX_RT_DISABLE);
      break;

    case 0x01:
      nrf24l01_set_data_regs(NRF24L01_REG_CONFIG, NRF24L01_INT_TX_DS_MASK, NRF24L01_INT_TX_DS_ENABLE);
      break;

    case 0x02:
      nrf24l01_set_data_regs(NRF24L01_REG_CONFIG, NRF24L01_INT_MAX_RT_MASK, NRF24L01_INT_MAX_RT_ENABLE);
      break;

    default:
      break;
  }
}

/******************************************************************************
* Function : void nrf24l01_set_payload_size(uint32_t size)
* Brief    : Set payload size for NRF24L01.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_set_payload_size(rx_data_pipe_t data_pipe, uint32_t size)
{
  nrf24l01_set_data_regs(NRF24L01_REG_RX_PW_P0 + data_pipe, ~(0x3F), size);
}

/******************************************************************************
* Function : void nrf24l01_set_payload_size(uint32_t size)
* Brief    : Set payload size for NRF24L01.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_enable_dynamic_payloads(void)
{
  uint8_t value;
  // Firstly, Enable the EN_DPL bit in FEATURE register
  nrf24l01_set_data_regs(NRF24L01_REG_FEATURE, NRF24L01_DYNAMIC_PAYLOAD_MASK, NRF24L01_DYNAMIC_PAYLOAD_ENABLE);

  // Check the status of NRF24L01_REG_FEATURE register 
  // to ensure that the EN_DPL has been set
  nrf24l01_read_regs(NRF24L01_REG_FEATURE, &value, 1);
  if((value & 0x04) == 0)
  {
    // Failed to set the EN_DPL bit 
    PRINT_ERROR_LOG("Falied to set the EN_DPL bit in Enable Dynamic payload\r\n");
  }

  // Enable Dynamic payload length on all data pipes
  value = 0x3F;
  nrf24l01_write_regs(NRF24L01_REG_DYNPD, &value, 1);
}

/******************************************************************************
* Function : void nrf24l01_enable_ack_payload(void)
* Brief    : Enable ACK payload.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_enable_ack_payload(void)
{
  uint8_t value;

  value = 0x02;
  // Enable EN_ACK_PAY bit
  nrf24l01_set_data_regs(NRF24L01_REG_FEATURE, ~value, value);
}

/******************************************************************************
* Function : uint8_t nrf24l01_read_fifo_status(void)
* Brief    : Read FIFO status.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
uint8_t nrf24l01_read_fifo_status(void)
{
  uint8_t value;
  nrf24l01_read_regs(NRF24L01_REG_FIFO_STATUS, &value, 1);
  return value;
}

/******************************************************************************
* Function : void nrf24l01_clear_all_flags(void)
* Brief    : Clear all status flag of NRF24L01.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_clear_all_flags(void)
{
  uint8_t value;
  nrf24l01_read_regs(NRF24L01_REG_STATUS, &value, sizeof(value));
  value &= ~(0x70);
  value |= 0x70;
  // Clear all interrupt flag
  nrf24l01_write_regs(NRF24L01_REG_STATUS, &value, sizeof(value));
}

/******************************************************************************
* Function : nrf24l01_write_back_ack_payload
* Brief    : Write back payload as ACK.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_write_back_ack_payload(rx_data_pipe_t pipe, uint8_t* pdata, uint8_t len)
{
  uint8_t write_cmd;
  uint8_t tx_buff[128 + 1]; // Add 1 byte for write command

  // Consider to edit this line based on the datasheet
  write_cmd = NRF24L01_CMD_W_ACK_PAYLOAD | pipe;
  tx_buff[0] = write_cmd;
  // TODO: Firstly need to check the data length
  // to void encountered buffer overflow
  memcpy(&tx_buff[1], pdata, len);
	
  NRF24L01_SPI_CS_LOW();
  HAL_SPI_Transmit(&nrf24l01_spi, tx_buff, len + 1, NRF24L01_SPI_TIMEOUT);
  NRF24L01_SPI_CS_HIGH();
}

/******************************************************************************
* Function : uint8_t nrf24l01_get_status(void)
* Brief    : Get status of register 0x07.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
uint8_t nrf24l01_get_status(void)
{
  uint8_t value;
  nrf24l01_read_regs(NRF24L01_REG_STATUS, &value, 1);
  return value;
}

/******************************************************************************
* Function : void nrf24l01_print_all_configurations(void)
* Brief    : Print all configured parameters.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void nrf24l01_print_all_configurations(void)
{
  uint8_t read_value[NRF24L01_ADDR_WIDTH];

  nrf24l01_read_regs(NRF24L01_REG_STATUS, read_value, 1);
  PRINT_INFO_LOG("\r\nNRF24L01_REG_STATUS-0x%x: 0x%x\r\n", NRF24L01_REG_STATUS, read_value[0]);
  // Print address of Reading Pipe
  nrf24l01_read_regs(NRF24L01_REG_RX_ADDR_P1, read_value, 3);
  PRINT_INFO_LOG("Reading Pipe Address-0x%x: 0x%x:0x%x:0x%x\r\n", NRF24L01_REG_RX_ADDR_P1, 
                  read_value[0], read_value[1], read_value[2]);

  // Print address of Writing Pipt
  nrf24l01_read_regs(NRF24L01_REG_RX_ADDR_P0, read_value, 3);
  PRINT_INFO_LOG("Writing Pipe RX Address-0x%x: 0x%x:0x%x:0x%x\r\n", NRF24L01_REG_RX_ADDR_P0, 
                  read_value[0], read_value[1], read_value[2]);
  nrf24l01_read_regs(NRF24L01_REG_TX_ADDR, read_value, 3);
  PRINT_INFO_LOG("Writing Pipe TX Address-0x%x: 0x%x:0x%x:0x%x\r\n", NRF24L01_REG_TX_ADDR, 
                  read_value[0], read_value[1], read_value[2]);
  
  // Print status of NRF24L01_REG_CONFIG register
  nrf24l01_read_regs(NRF24L01_REG_CONFIG, read_value, 1);
  PRINT_INFO_LOG("NRF24L01_REG_CONFIG-0x%x: 0x%x\r\n", NRF24L01_REG_CONFIG, read_value[0]);

  // Print status of NRF24L01_REG_EN_AA register
  nrf24l01_read_regs(NRF24L01_REG_EN_AA, read_value, 1);
  PRINT_INFO_LOG("NRF24L01_REG_EN_AA-0x%x: 0x%x\r\n", NRF24L01_REG_EN_AA, read_value[0]);

  // Print Rx payload of Data Pipe 0
  nrf24l01_read_regs(NRF24L01_REG_RX_PW_P0, read_value, 1);
  PRINT_INFO_LOG("NRF24L01_REG_RX_PW_P0-0x%x: 0x%x\r\n", NRF24L01_REG_RX_PW_P0, read_value[0]);

  // Print Rx payload of Data Pipe 1
  nrf24l01_read_regs(NRF24L01_REG_RX_PW_P1, read_value, 1);
  PRINT_INFO_LOG("NRF24L01_REG_RX_PW_P1-0x%x: 0x%x\r\n", NRF24L01_REG_RX_PW_P1, read_value[0]);

  // Print status of EN_AA register
  nrf24l01_read_regs(NRF24L01_REG_EN_AA, read_value, 1);
  PRINT_INFO_LOG("NRF24L01_REG_EN_AA-0x%x: 0x%x\r\n", NRF24L01_REG_EN_AA, read_value[0]);

  // Print status of NRF24L01_REG_SETUP_AW register
  nrf24l01_read_regs(NRF24L01_REG_SETUP_AW, read_value, 1);
  PRINT_INFO_LOG("NRF24L01_REG_SETUP_AW-0x%x: 0x%x\r\n", NRF24L01_REG_SETUP_AW, read_value[0]);

  // Print status of EN_RXADDR register
  nrf24l01_read_regs(NRF24L01_REG_EN_RXADDR, read_value, 1);
  PRINT_INFO_LOG("NRF24L01_REG_EN_RXADDR-0x%x: 0x%x\r\n", NRF24L01_REG_EN_RXADDR, read_value[0]);

  // Print status of NRF24L01_REG_RF_CH register
  nrf24l01_read_regs(NRF24L01_REG_RF_CH, read_value, 1);
  PRINT_INFO_LOG("NRF24L01_REG_RF_CH-0x%x: 0x%x\r\n", NRF24L01_REG_RF_CH, read_value[0]);

  // Print status of NRF24L01_REG_RF_SETUP register
  nrf24l01_read_regs(NRF24L01_REG_RF_SETUP, read_value, 1);
  PRINT_INFO_LOG("NRF24L01_REG_RF_SETUP-0x%x: 0x%x\r\n", NRF24L01_REG_RF_SETUP, read_value[0]);

  // Print status of NRF24L01_REG_DYNPD register
  nrf24l01_read_regs(NRF24L01_REG_DYNPD, read_value, 1);
  PRINT_INFO_LOG("NRF24L01_REG_DYNPD-0x%x: 0x%x\r\n", NRF24L01_REG_DYNPD, read_value[0]);

  // Print status of NRF24L01_REG_FEATURE register
  nrf24l01_read_regs(NRF24L01_REG_FEATURE, read_value, 1);
  PRINT_INFO_LOG("NRF24L01_REG_FEATURE-0x%x: 0x%x\r\n", NRF24L01_REG_FEATURE, read_value[0]);

  // Print status of NRF24L01_REG_FIFO_STATUS register
  nrf24l01_read_regs(NRF24L01_REG_FIFO_STATUS, read_value, 1);
  PRINT_INFO_LOG("NRF24L01_REG_FIFO_STATUS-0x%x: 0x%x\r\n", NRF24L01_REG_FIFO_STATUS, read_value[0]);
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
    HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
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
