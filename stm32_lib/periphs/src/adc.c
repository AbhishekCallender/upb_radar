/**
  ******************************************************************************
  * File Name          : adc.c
  * Description        : Includes functions for configuring and reading ADC values
  * Date Modified      : 14-12-2015
  ******************************************************************************

  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <adc.h>
#define FFTSIZE 4096
//	    uint16_t fftin[2048];
		uint16_t i;
//		uint16_t pr; uint16_t pi;
		uint8_t dummy_send[FFTSIZE *5];
		uint8_t data_rx_dummy[FFTSIZE*5];

SPI_HandleTypeDef hspi4;


void adc_interface_init(uint8_t samp)
{

	  hspi4.Instance = SPI4;
	  hspi4.Init.Mode = SPI_MODE_MASTER;
	  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
	  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
	  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
	  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
	  hspi4.Init.NSS = SPI_NSS_SOFT;
	  switch (samp) {
		case 1:
			hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
			break;
		case 2:
			hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
			break;
		case 3:
			hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
			break;
		case 4:
			hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
			break;
		case 5:
			hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
			break;
		case 6:
			hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
			break;
		case 7:
			hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
			break;
		case 8:
			hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
			break;
		default:
			hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
			break;
	}

	  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
	  hspi4.Init.TIMode = SPI_TIMODE_DISABLED;
	  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
	  HAL_SPI_Init(&hspi4);
	  __HAL_SPI_ENABLE(&hspi4);


}

void adc_data(float32_t data_rx[], uint16_t samples, uint8_t opmode)
{

		data_rx_dummy[0]=0;
		data_rx[0]=0;
		dummy_send[0]=0x80;

		for(i=1;i<FFTSIZE*5;i++)
		{
			data_rx_dummy[i]=0;
			dummy_send[i]=0;
		}
		for(i=1;i<FFTSIZE;i++)
		{
			dummy_send[5*i]=0x80;
		}
		if (opmode==1) {
			startPll();
		}
		HAL_SPI_TransmitReceive_DMA(&hspi4, (uint8_t *) &dummy_send, (uint8_t *) &data_rx_dummy, (samples+1)*5);

		while (HAL_SPI_GetState(&hspi4) != HAL_SPI_STATE_READY) {

		}
		  if (opmode==1) {
			  stopPll();
		}
		for(i=0;i<samples+1;i++)
		{
		  	  data_rx[2*i-2]   = (float32_t)(((((data_rx_dummy[5*i] & 0x1F) << 9) | ((data_rx_dummy[5*i+1] & 0xFF) << 1)  | ((data_rx_dummy[5*i+2] & 0x80) >> 7)))/(float32_t)1);


		  	  data_rx[2*i-1] = (float32_t)(((((data_rx_dummy[5*i+2] & 0x1F) << 9) | ((data_rx_dummy[5*i+3] & 0xFF) << 1)	 | ((data_rx_dummy[5*i+4] & 0x80) >> 7)))/(float32_t)1);
		}



}
