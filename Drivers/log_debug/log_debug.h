/*******************************************************************************
* Title                 :   Print log debug header file
* Filename              :   log_debug.h
* Author                :   Nhan
* Origin Date           :   09/10/2021
* Notes                 :   None
*******************************************************************************/

#ifndef LOG_DEBUG_H_
#define LOG_DEBUG_H_

/******************************************************************************
* INCLUDES
*******************************************************************************/
#include <stdio.h>

/******************************************************************************
* CONFIGURATION CONSTANTS
*******************************************************************************/
#define LOG_DEBUG_EN          0 // 1: Enable debug log over UART
                                // 0: Disable debug log over UART

#define INFO_LOG_DEBUG_EN     1
#define ERROR_LOG_DEBUG_EN    1

/******************************************************************************
* MACROS
*******************************************************************************/
#if (INFO_LOG_DEBUG_EN && LOG_DEBUG_EN)
  #define PRINT_INFO_LOG(...)  printf(__VA_ARGS__)
  #define PRINT_INFO_LOG_LINE(...)  printf(__VA_ARGS__);printf("\r\n")
  #define Error_Handler() printf("Error Handler at:%s:%d\r\n", __FILE__, __LINE__)
#else
  #define PRINT_INFO_LOG(...) 
  #define PRINT_INFO_LOG_LINE(...) 
  #define Error_Handler(...)
#endif

#if (ERROR_LOG_DEBUG_EN && LOG_DEBUG_EN)
  #define PRINT_ERROR_LOG(...)  printf(__VA_ARGS__)
#else
  #define PRINT_ERROR_LOG(...)
#endif

/******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void log_debug_init(void);
void log_debug_deinit(void);

#endif // LOG_DEBUG_H_
/*** END OF FILE **************************************************************/
