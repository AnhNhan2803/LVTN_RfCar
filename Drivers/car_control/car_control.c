/*******************************************************************************
* Title                 :   Car control driver source file
* Filename              :   car_control.h
* Author                :   Nhan
* Origin Date           :   Oct 19th 2021
* Notes                 :   None
*******************************************************************************/

/******************************************************************************
 * INCLUDES
 *******************************************************************************/
#include "car_control.h"

/******************************************************************************
 * CONFIGURATION CONSTANTS
 *******************************************************************************/
#if(USING_FULLY_OUTPUT_PP_MODE)
// This mode control the motors completely with normal Output pushpull operation
#define MOTOR_LEFT1_START()                 (HAL_GPIO_WritePin(MOTOR_LEFT1_GPIO_Port, MOTOR_LEFT1_Pin, GPIO_PIN_SET))
#define MOTOR_LEFT1_STOP()                  (HAL_GPIO_WritePin(MOTOR_LEFT1_GPIO_Port, MOTOR_LEFT1_Pin, GPIO_PIN_RESET))
#define MOTOR_LEFT2_START()                 (HAL_GPIO_WritePin(MOTOR_LEFT2_GPIO_Port, MOTOR_LEFT2_Pin, GPIO_PIN_SET))
#define MOTOR_LEFT2_STOP()                  (HAL_GPIO_WritePin(MOTOR_LEFT2_GPIO_Port, MOTOR_LEFT2_Pin, GPIO_PIN_RESET))
#define MOTOR_RIGHT1_START()                (HAL_GPIO_WritePin(MOTOR_RIGHT1_GPIO_Port, MOTOR_RIGHT1_Pin, GPIO_PIN_SET))
#define MOTOR_RIGHT1_START()                (HAL_GPIO_WritePin(MOTOR_RIGHT1_GPIO_Port, MOTOR_RIGHT1_Pin, GPIO_PIN_RESET))
#define MOTOR_RIGHT2_START()                (HAL_GPIO_WritePin(MOTOR_RIGHT2_GPIO_Port, MOTOR_RIGHT2_Pin, GPIO_PIN_SET))
#define MOTOR_RIGHT2_START()                (HAL_GPIO_WritePin(MOTOR_RIGHT2_GPIO_Port, MOTOR_RIGHT2_Pin, GPIO_PIN_RESET))
#else
// This mode control the motors completely with PWM
#define MOTOR_LEFT1_CONTROL(x)     __HAL_TIM_SET_COMPARE(&car_control_tim, TIM_CHANNEL_1, CAR_CTRL_TIM_PERIOD * x / 100)
#define MOTOR_LEFT2_CONTROL(x)     __HAL_TIM_SET_COMPARE(&car_control_tim, TIM_CHANNEL_2, CAR_CTRL_TIM_PERIOD * x / 100)
#define MOTOR_RIGHT1_CONTROL(x)    __HAL_TIM_SET_COMPARE(&car_control_tim, TIM_CHANNEL_3, CAR_CTRL_TIM_PERIOD * x / 100)
#define MOTOR_RIGHT2_CONTROL(x)    __HAL_TIM_SET_COMPARE(&car_control_tim, TIM_CHANNEL_4, CAR_CTRL_TIM_PERIOD * x / 100)
#endif

/******************************************************************************
 * PREPROCESSOR MACROS
 *******************************************************************************/


/******************************************************************************
 * TYPEDEFS
 *******************************************************************************/


/******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************/
static void car_control_gpio_init(void);
static void car_control_gpio_deinit(void);
static void car_control_tim_init(void);
static void car_control_tim_deinit(void);

/******************************************************************************
 * VARIABLE DEFINITIONS
 *******************************************************************************/
TIM_HandleTypeDef car_control_tim;

/******************************************************************************
 * PUBLIC FUNCTIONS
 *******************************************************************************/
/******************************************************************************
* Function : void car_control_init(void)
* Brief    : Initialize the car control driver 
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void car_control_init(void)
{
  car_control_gpio_init();
  car_control_tim_init();
}

/******************************************************************************
* Function : void car_control_deinit(void)
* Brief    : DeInitialize the car control driver 
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void car_control_deinit(void)
{
  car_control_gpio_deinit();
  car_control_tim_deinit();
}

/******************************************************************************
* Function : void car_control_exec_cmd(car_ctrl_t cmd)
* Brief    : Control the RF car corresponding to command
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void car_control_exec_cmd(car_ctrl_t cmd)
{
  // TODO: Need to add Power param for control
  // the duty cycle of PWM for speed changing 
  // purpose

  switch(cmd)
  {
    case CAR_CTRL_MOVE_FORWARD:
      MOTOR_LEFT1_CONTROL(100);
      MOTOR_LEFT2_CONTROL(0);
      MOTOR_RIGHT1_CONTROL(100);
      MOTOR_RIGHT2_CONTROL(0);
      break;

    case CAR_CTRL_MOVE_BACKWARD:
      MOTOR_LEFT1_CONTROL(0);
      MOTOR_LEFT2_CONTROL(100);
      MOTOR_RIGHT1_CONTROL(0);
      MOTOR_RIGHT2_CONTROL(100);
      break;

    case CAR_CTRL_TURN_LEFT:
      MOTOR_LEFT1_CONTROL(100);
      MOTOR_LEFT2_CONTROL(0);
      // TODO: Need to adjust the duty cycle of MOTOR_RIGHT_1 for
      // best performance
      MOTOR_RIGHT1_CONTROL(50);
      MOTOR_RIGHT2_CONTROL(0);
      break;

    case CAR_CTRL_TURN_RIGHT:
      // TODO: Need to adjust the duty cycle of MOTOR_LEFT_1 for
      // best performance
      MOTOR_LEFT1_CONTROL(50);
      MOTOR_LEFT2_CONTROL(0);
      MOTOR_RIGHT1_CONTROL(100);
      MOTOR_RIGHT2_CONTROL(0);
      break;

    case CAR_CTRL_BACK_LEFT:
      MOTOR_LEFT1_CONTROL(0);
      MOTOR_LEFT2_CONTROL(100);
      MOTOR_RIGHT1_CONTROL(0);
      // TODO: Need to adjust the duty cycle of MOTOR_RIGHT_2 for
      // best performance
      MOTOR_RIGHT2_CONTROL(50);
      break;

    case CAR_CTRL_BACK_RIGHT:
      MOTOR_LEFT1_CONTROL(0);
      // TODO: Need to adjust the duty cycle of MOTOR_LEFT_2 for
      // best performance
      MOTOR_LEFT2_CONTROL(50);
      MOTOR_RIGHT1_CONTROL(0);
      MOTOR_RIGHT2_CONTROL(100);
      break;

    case CAR_CTRL_ROTATE_LEFT:
      MOTOR_LEFT1_CONTROL(100);
      MOTOR_LEFT2_CONTROL(0);
      MOTOR_RIGHT1_CONTROL(0);
      MOTOR_RIGHT2_CONTROL(100);
      break;

    case CAR_CTRL_ROTATE_RIGHT:
      MOTOR_LEFT1_CONTROL(0);
      MOTOR_LEFT2_CONTROL(100);
      MOTOR_RIGHT1_CONTROL(100);
      MOTOR_RIGHT2_CONTROL(0);
      break;

    default:
      // Stop the car module
      MOTOR_LEFT1_CONTROL(0);
      MOTOR_LEFT2_CONTROL(0);
      MOTOR_RIGHT1_CONTROL(0);
      MOTOR_RIGHT2_CONTROL(0);
      break;
  }
}

/******************************************************************************
* Function : void car_control_pwm_start(void)
* Brief    : Start the PWM module.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void car_control_pwm_start(void)
{
    HAL_TIM_PWM_Start(&car_control_tim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&car_control_tim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&car_control_tim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&car_control_tim, TIM_CHANNEL_4);
}

/******************************************************************************
* Function : void car_control_pwm_stop(void)
* Brief    : Stop the PWM module.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
void car_control_pwm_stop(void)
{
    HAL_TIM_PWM_Stop(&car_control_tim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&car_control_tim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&car_control_tim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&car_control_tim, TIM_CHANNEL_4);
}

/******************************************************************************
 * STATIC FUNCTIONS
 *******************************************************************************/
/******************************************************************************
* Function : void car_control_gpio_init(void)
* Brief    : Initialize GPIO for car control module.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void car_control_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = MOTOR_LEFT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MOTOR_LEFT1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MOTOR_LEFT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MOTOR_LEFT2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MOTOR_RIGHT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MOTOR_RIGHT1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MOTOR_RIGHT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MOTOR_RIGHT2_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
}

/******************************************************************************
* Function : void car_control_gpio_deinit(void)
* Brief    : DeInitialize GPIO for car control module.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void car_control_gpio_deinit(void)
{
    HAL_GPIO_DeInit(MOTOR_LEFT1_GPIO_Port, MOTOR_LEFT1_Pin);
    HAL_GPIO_DeInit(MOTOR_LEFT2_GPIO_Port, MOTOR_LEFT2_Pin);
    HAL_GPIO_DeInit(MOTOR_RIGHT1_GPIO_Port, MOTOR_RIGHT1_Pin);
    HAL_GPIO_DeInit(MOTOR_RIGHT2_GPIO_Port, MOTOR_RIGHT2_Pin);
}

/******************************************************************************
* Function : void car_control_tim_init(void)
* Brief    : Initialize the PWM channels corresponding to Timer.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void car_control_tim_init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();

    car_control_tim.Instance = TIM2;
    car_control_tim.Init.Prescaler = CAR_CTRL_TIM_PRESCALER - 1;
    car_control_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    // Since the period = 499
    // Prescaler = 47
    // => Ftimer = 48Mhz / (Period + 1) x (Prescaler + 1)
    // => Ftimer = 2kHz
    car_control_tim.Init.Period = CAR_CTRL_TIM_PERIOD - 1;
    car_control_tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    car_control_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&car_control_tim) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&car_control_tim, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    // Set duty cycle at default value
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&car_control_tim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&car_control_tim, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&car_control_tim, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&car_control_tim, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/******************************************************************************
* Function : void car_control_tim_deinit(void)
* Brief    : DeInitialize the PWM channels corresponding to Timer.
* Input    : None.
* Output   : None.
* Return   : None.
*******************************************************************************/
static void car_control_tim_deinit(void)
{
    HAL_TIM_PWM_DeInit(&car_control_tim);
    __HAL_RCC_TIM2_CLK_DISABLE();
}
