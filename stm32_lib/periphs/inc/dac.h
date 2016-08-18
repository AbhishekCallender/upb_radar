/**
  ******************************************************************************
  * File Name          : dac.h
  * Description        : Header File for dac.c
  * Date Modified      : 09-12-2015
  ******************************************************************************

  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DAC_H
#define __DAC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stm32f4xx.h>
#include <arm_math.h>


void dac_interface_init(void);
void dac_configure(float32_t c4_offset_i, float32_t c5_offset_q, float32_t c6_cmq, float32_t c7_cmi);
void bias_cmq(float32_t* cm_q);
void bias_cmi(float32_t* cm_i);
#ifdef __cplusplus
}
#endif

#endif /* __LED_H */
