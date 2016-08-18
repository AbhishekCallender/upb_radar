/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : Header File for main.c
  * Date Modified      : 14-12-2015
  ******************************************************************************

  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <arm_math.h>
#include <arm_const_structs.h>
#include <arm_common_tables.h>
#include <math.h>


/* Private variables ---------------------------------------------------------*/


//I2C_HandleTypeDef hi2c1;
//SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
TIM_HandleTypeDef Timer_GPIO_Handle;
//arm_cfft_instance_f32 S1;
//arm_cfft_radix4_instance_f32 S;


void SystemClock_Config(void);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
