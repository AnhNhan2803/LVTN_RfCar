/*******************************************************************************
* Title                 :   Car control driver header file
* Filename              :   car_control.h
* Author                :   Nhan
* Origin Date           :   Oct 19th 2021
* Notes                 :   None
*******************************************************************************/
#ifndef CAR_CONTROL_H_
#define CAR_CONTROL_H_

/******************************************************************************
* INCLUDES
*******************************************************************************/
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "log_debug.h"

/******************************************************************************
 * CONFIGURATION CONSTANTS
 *******************************************************************************/
#define USING_FULLY_OUTPUT_PP_MODE    (0)
// Count up 1us each time
#define CAR_CTRL_TIM_PRESCALER        (48)
// Setup period counter of the PWM to 500us
// => PWM Frequency = 2Kz
#define CAR_CTRL_TIM_PERIOD           (500)

/******************************************************************************
 * MACROS
 *******************************************************************************/


/******************************************************************************
 * TYPEDEFS
 *******************************************************************************/
typedef enum {
    CAR_CTRL_MOVE_FORWARD = 0,
    CAR_CTRL_MOVE_BACKWARD,
    CAR_CTRL_TURN_LEFT,
    CAR_CTRL_TURN_RIGHT,
    CAR_CTRL_BACK_LEFT,
    CAR_CTRL_BACK_RIGHT,
    CAR_CTRL_ROTATE_LEFT,
    CAR_CTRL_ROTATE_RIGHT,
    CAR_CTRL_STOP
} car_ctrl_t;

typedef enum {
    MOTOR_LEFT_1 = 0,
    MOTOR_LEFT_2,
    MOTOR_RIGHT_1,
    MOTOR_RIGHT_2
} motor_t;

/******************************************************************************
 * VARIABLES DEFINITIONS
 *******************************************************************************/


/******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************/
void car_control_init(void);
void car_control_deinit(void);
void car_control_exec_cmd(car_ctrl_t cmd);
void car_control_pwm_start(void);
void car_control_pwm_stop(void);

#endif /* CAR_CONTROL_H_ */
/*** END OF FILE **************************************************************/
