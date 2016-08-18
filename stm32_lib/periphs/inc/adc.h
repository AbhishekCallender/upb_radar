/**
  ******************************************************************************
  * File Name          : adc.h
  * Description        : Header File for adc.c
  * Date Modified      : 14-12-2015
  ******************************************************************************

  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stm32f4xx.h>
#include <arm_math.h>

 void adc_interface_init(uint8_t samp);
 void adc_data(float32_t data_rx[], uint16_t samples, uint8_t opmode);

 

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H */
