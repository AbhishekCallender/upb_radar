/**
  ******************************************************************************
  * File Name          : dac.c
  * Description        : Configuration and setting functions of DAC
  * Date Modified      : 09-12-2015
  ******************************************************************************

  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <dac.h>

SPI_HandleTypeDef hspi2;

void dac_interface_init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  HAL_SPI_Init(&hspi2);
  __HAL_SPI_ENABLE(&hspi2);

  //Chip Select Configuration for DAC
  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  __GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //CS = High

}

void dac_configure(float32_t c4_offset_i, float32_t c5_offset_q, float32_t c6_cmq, float32_t c7_cmi)
{
	uint16_t wtm_mode = 36864; //0b1001000000000000

    float32_t vref = 3.3;

    float32_t channel[8];
    channel[0] = 0;
    channel[1] = 0;
    channel[2] = 0;
    channel[3] = 0;
    channel[4] = c4_offset_i; //Offset_I
    channel[5] = c5_offset_q; //Offset_Q
    channel[6] = c6_cmq;  //CommonModeQ
    channel[7] = c7_cmi; //CommonModeI

    float32_t voltperbit = (vref / (float)(pow(2,12) - 1));

    uint16_t dac_send;
    uint16_t i=0;

    //Transmit ControlBitString for WTM-Mode

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);//CS = Low
    HAL_SPI_Transmit (&hspi2, (uint8_t*) &wtm_mode, sizeof(wtm_mode)/2, 0x1000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //CS = High

    for (i = 0; i < 8; ++i)
    {
    	dac_send = ((uint16_t) (channel[i] / voltperbit)) & 0x0FFF; //Holding the last 12 bits
    	dac_send ^= (i << 12);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);//CS = Low
        HAL_SPI_Transmit (&hspi2, (uint8_t*) &dac_send, sizeof(dac_send)/2, 0x1000);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //CS = High
	}

}

void bias_cmq(float32_t* cm_q)
{
	uint16_t i=0;
	uint8_t data_rx[5];
	uint32_t biasparam = 0;
	uint16_t biasdata[300];
	float32_t vref = 3.3;
	uint16_t dac_send;
	uint8_t done=0;
	uint16_t over=0;
	float32_t voltperbit = (vref / (float)(pow(2,12) - 1));
	float32_t c6_cmq_test;
	for (c6_cmq_test = 1.65; (done==0) && (over !=3300); ) {
		dac_send = ((uint16_t) (c6_cmq_test / voltperbit)) & 0x0FFF; //Holding the last 12 bits
		    	dac_send ^= (6 << 12);
		        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);//CS = Low
		        HAL_SPI_Transmit (&hspi2, (uint8_t*) &dac_send, sizeof(dac_send)/2, 0x1000);
		        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //CS = High
		        biasparam = 0;
		        for(i=0;i<300;i++)
		        {

		        adc_data(data_rx, 1024,0);
		        			biasdata[i] = ((data_rx[2] & 0x1F) << 9)
		        				| ((data_rx[3] & 0xFF) << 1)
		        				| ((data_rx[4] & 0x80) >> 7);
		        			biasparam +=biasdata[i];

		        }
		        biasparam=biasparam/300;
		       if (((biasparam)<16050) && ((biasparam)>15950)) {
				done=1;
				*cm_q=c6_cmq_test;
			}
		       else if ((biasparam)<=15950)
		       {
		    	   c6_cmq_test +=0.001;
		    	   over++;

		       }
		       else
		{
		    	c6_cmq_test -=0.001;
		    	over++;
		}




	}
}

void bias_cmi(float32_t* cm_i)
{
	uint16_t i=0;
		uint8_t data_rx[5];
		uint32_t biasparam = 0;
		uint16_t biasdata[300];
		float32_t vref = 3.3;
		uint16_t dac_send;
		uint8_t done=0;
		uint16_t over=0;
	float32_t voltperbit = (vref / (float)(pow(2,12) - 1));
	float32_t c6_cmi_test;
	for (c6_cmi_test = 1.65; (done==0) && (over !=3300); ) {
		dac_send = ((uint16_t) (c6_cmi_test / voltperbit)) & 0x0FFF; //Holding the last 12 bits
		    	dac_send ^= (7 << 12);
		        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);//CS = Low
		        HAL_SPI_Transmit (&hspi2, (uint8_t*) &dac_send, sizeof(dac_send)/2, 0x1000);
		        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //CS = High
		        biasparam = 0;
		        		        for(i=0;i<300;i++)
		        		        {

		        		        adc_data(data_rx, 1024,0);
		        		        			biasdata[i] = ((data_rx[0] & 0x1F) << 9)
		        		        				| ((data_rx[1] & 0xFF) << 1)
		        		        				| ((data_rx[2] & 0x80) >> 7);
		        		        			biasparam +=biasdata[i];

		        		        }
		        		        biasparam=biasparam/300;
		        		       if (((biasparam)<16050) && ((biasparam)>15950)) {
		        				done=1;
		        				*cm_i=c6_cmi_test;
		        			}
		        		       else if ((biasparam)<=15950)
		        		       {
		        		    	   c6_cmi_test +=0.001;
		        		    	   over++;
		        		       }
		        		       else
		        		{
		        		    	c6_cmi_test -=0.001;
		        		    	over++;
		        		}


	}
}

