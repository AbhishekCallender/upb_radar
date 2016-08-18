
/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include <adc.h>
#include <dac.h>
#include <pll.h>

#include <cstdlib>
#include <unistd.h>
#include <uavcan/uavcan.hpp>
#include <stm32f4xx.h>

#include <uavcan/protocol/debug/KeyValue.hpp> 


extern SPI_HandleTypeDef hspi2;

I2C_HandleTypeDef  hi2c1;

#define I2C_ADDRESS		(uint8_t)0b01011110
#define MAX_GAIN 		100
#define DEFAULT_GAIN_I 	10
#define DEFAULT_GAIN_Q 	10

float32_t max_gain_ = MAX_GAIN;
float32_t channelI_ = DEFAULT_GAIN_I;
float32_t channelQ_ = DEFAULT_GAIN_Q;

//#define PI M_PI
#define FMCW //Comment this line for CW mode of operation
#define MAXELEMENTS 255
#define OP_FFTSIZE 4096
#define IP_FFTSIZE 8192

//*************For GUI***************
uint8_t basic=0;
uint8_t setvalue;
uint8_t isconnected=0;
uint8_t IN_BYTE = 0;
static uint16_t tempint16;
static uint8_t tempint8;
static float32_t tempFloat;
float distances_sim[MAXELEMENTS] = { 1.2, 2.4, 3.6};
uint32_t ib=0;
char *PointChar;
static uint8_t UsbTrans[1024];
uint8_t senddata =0;
float32_t mindist=0.3;
float32_t maxdist=15.0;
//******************Parameters for FFT*******************

//float32_t orip[IP_FFTSIZE];
uint8_t peak=0;
float32_t ip[IP_FFTSIZE];
float32_t op[OP_FFTSIZE];
float32_t op1[OP_FFTSIZE];
float32_t norm;
uint8_t ref=0;
uint16_t count=0;
float32_t maxValue;
float32_t minvalue;
float32_t maxValues[MAXELEMENTS];
uint32_t maxIndex;
uint32_t maxIndexes[MAXELEMENTS];
float32_t distances[MAXELEMENTS];
uint8_t data_rx[5];
uint8_t ibiased=0;
float32_t biasmin=0;
float32_t biasmax=3.3;
uint8_t qbiased=0;
uint16_t over=0;
uint8_t firstsample=0;
float32_t tempdistance[MAXELEMENTS];
uint8_t againi=0;
uint8_t againq=0;
uint8_t osd=0;
uint8_t fmcwop=1;

/****************************** Settings for RADAR Board ***************************/
uint16_t no_of_samples = OP_FFTSIZE;
uint16_t no_of_samples_osd = OP_FFTSIZE;
float32_t sampling_freq=525;
float32_t offset_i = 1.65;
float32_t offset_q = 1.65;
float32_t cm_q = 1.65; //Will be replaced by auto bias
float32_t cm_i = 1.65; //Will be replaced by auto bias
float32_t fstart = 120.4; //Will be replaced by calibration
float32_t fstop = 123.4; //Will be replaced by calibration
float32_t sweeptime = 7.5; //in ms
float32_t gain_i = 1;
float32_t gain_q = 1;
float32_t delta=1;
uint8_t sampled=0;

/****************************** Settings for PERIPHS Board ***************************/

void init_led()
{
	//Led on Radar Board (PIN-E3)initialization:
	/* -1- Enable GPIOD Clock (to be able to program the configuration registers) */
	__GPIOE_CLK_ENABLE();
	/* -2- Configure  IOs in output push-pull mode to drive external LEDs */
	GPIO_InitTypeDef  GPIO_InitStruct;
	/* Configure the GPIO_LED pin */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

void set_led()
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

void reset_led()
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
}

void toggle_led()
{
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
}

void init_all_peri()
{

  /* GPIO Ports Clock Enable */
	  __GPIOE_CLK_ENABLE();
	  __GPIOH_CLK_ENABLE();
	  __GPIOC_CLK_ENABLE();
	  __GPIOA_CLK_ENABLE();
	  __GPIOB_CLK_ENABLE();
	  __GPIOD_CLK_ENABLE();

  init_led();
  //can_init(); //can1
   //poti
  initPll(); //spi1
  dac_interface_init(); //spi2
  //MX_SPI3_Init();

}
void configurePoti();
void initI2c();
void setChannelIPoti(float32_t val);
void setChannelQPoti(float32_t val);
uint8_t getChannelQPoti();
uint8_t getChannelIPoti(); 

void initPoti(float32_t chI, float32_t chQ) {
	channelI_ = chI;
	channelQ_ = chQ;
	initI2c();
	configurePoti();
}


void initI2c() {

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 10000; //100kHz = max Speed for I2C
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0b01011110; //slave adress used, is this teh master-adress?
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	HAL_I2C_Init(&hi2c1); //ruft auch eigene GPIO Init und config in *hal_msp.c auf
	
}

void configurePoti() {
	setChannelIPoti(channelI_);
	setChannelQPoti(channelQ_);
}


uint8_t convertGainToBit(float32_t val) {

	if(val <  0 || val > max_gain_){
		val = DEFAULT_GAIN_I;
	}
	const uint8_t bitseq = (uint8_t)(255 * val / max_gain_) & 0x00FF;

	return bitseq;
}


uint8_t convertBitToGain(uint8_t bitseq) {

	return (bitseq / 255 * max_gain_);

}

void sendToPoti(float32_t val, uint16_t id) {
	// the first bits are to identify the channel
	uint8_t ident = (id << 7)| 0x00;
	uint8_t value = convertGainToBit(val);

	uint8_t tmp[2];
	tmp[0] = ident;
	tmp[1] = value;

	while((HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDRESS, (uint8_t*)&tmp, (uint16_t)sizeof(tmp), (uint32_t)1000))!= HAL_OK);
}

void setChannelIPoti(float32_t val) {
	channelI_ = val;
	sendToPoti(val, 0);
}

void setChannelQPoti(float32_t val) {
	channelQ_ = val;
	sendToPoti(val, 1);
}

uint8_t getChannelIPoti() {

	uint8_t tmp[2];
	setChannelIPoti(channelI_);
	while((HAL_I2C_Master_Receive(&hi2c1, I2C_ADDRESS, (uint8_t*)&tmp, (uint16_t)sizeof(tmp), (uint32_t)1000))!= HAL_OK);
	return (tmp[0]);

}

uint8_t getChannelQPoti() {

	uint8_t tmp[2];
	setChannelQPoti(channelQ_);
	while((HAL_I2C_Master_Receive(&hi2c1, I2C_ADDRESS, (uint8_t*)&tmp, (uint16_t)sizeof(tmp), (uint32_t)1000))!= HAL_OK);
	return (tmp[0]);

}

/****************************** Settings for PERIPHS ***************************/

/****************************** Settings for RADAR Board ***************************/
//*******************************************************

void bias_cmi_n();
void bias_cmq_n();
void detectinput();
void setdata();
void sendiq();
void processing();
void detrend(float32_t c[],uint16_t n);
void hanning_window(float32_t* w, uint16_t j, uint16_t n);
void win_blackman_harris(float32_t* w, uint16_t j, uint16_t n);
void win_blackman(float32_t* w, uint16_t j, uint16_t n);
void Find_FFT();
void peakdetect(uint16_t startpos, uint16_t stoppos);
void sendinitialdata();

/****************************** Settings for UAVCAN Node ***************************/
void CAN_PIN_Config()
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitDef;
	
	GPIO_InitDef.Pin = GPIO_PIN_8|GPIO_PIN_9;
	GPIO_InitDef.Mode = GPIO_MODE_AF_PP;
	GPIO_InitDef.Pull = GPIO_PULLUP;
	GPIO_InitDef.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitDef.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitDef);
}


#define node_id 74

extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();

const unsigned NodeMemoryPoolSize = 16384;

typedef uavcan::Node<NodeMemoryPoolSize> Node;
 
static Node& getNode()
{
    static Node node(getCanDriver(), getSystemClock());
    return node;
}


/****************************** Settings for UAVCAN Node ***************************/


int main(void)
{


  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  
  CAN_PIN_Config();
  
  
    /* Configure the UAVCAN Node */
    auto& node = getNode();
	node.setNodeID(node_id);
	node.setName("com.HNI");
	
	const int node_start_res = node.start();
	if (node_start_res < 0)
		{
		while(true){}
		}
	uavcan::Publisher<uavcan::protocol::debug::KeyValue> kv_pub(node);
	const int kv_pub_init_res = kv_pub.init();
    if (kv_pub_init_res < 0)
		{
		while(true){}
		}
	
	kv_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
	kv_pub.setPriority(uavcan::TransferPriority::NumericallyMin);
	
	node.setModeOperational();
		
	
	/* Configure the UAVCAN Node */

  /* Initialize all configured peripherals */

  init_all_peri();
  adc_interface_init(2); //spi4
  init_led();
  set_led();
  //MX_USB_DEVICE_Init();

  dac_configure(offset_i, offset_q, cm_q, cm_i);

  initPoti(gain_i,gain_q);


#ifdef FMCW
  startPll();
#endif

  //*********AUTOCALIBRATION********

  calibrate_pll(&fstart, &fstop, sweeptime);
  configPll(fstart,fstop,sweeptime);

  //********************************

  //*************AUTOBIAS***********
#ifdef FMCW
  stopPll();
#endif

  //bias_cmq(&cm_q);
  //bias_cmi(&cm_i);

//#ifdef FMCW
//  startPll();
//#endif
  //********************************

no_of_samples=(uint16_t)(sweeptime*sampling_freq);
no_of_samples_osd=no_of_samples;
  //Init_Timer();

  while (1)
  {
	   const int spin_res = node.spin(uavcan::MonotonicDuration::fromMSec(1000));
        if (spin_res < 0)
        {
        while(true){}
        }

	  adc_data(ip, no_of_samples, fmcwop);

	  	  		if (ibiased==1 && qbiased==1 && againi==1 && againq==1 && osd!=0) {
	  	  			uint8_t divider=(uint8_t)powf(4,osd);
	  	  			uint8_t divider2=(uint8_t)powf(2,osd);
	  	  		    for (count=0; count<no_of_samples/divider;count++)
	  	  		    {
	  	  		    	uint8_t i=0;
	  	  		    ip[2*count]=ip[2*divider*count]/divider2;
	  	  		ip[2*count+1]=ip[2*divider*count+1]/divider2;
	  	  		    	for (i = 1; i < divider; ++i) {


	  	  		        ip[2*count]+=(ip[2*divider*count+2*i])/divider2;
	  	  		        ip[2*count+1]+=(ip[2*divider*count+2*i+1])/divider2;

	  	  		    	}
	  	  		    }
	  	  	}
	  if (ibiased==1 && qbiased==1 && againi==1 && againq==1) {
	  toggle_led();
	  }

		  if(ibiased==0)
		  {
		  	 bias_cmi_n();
		  	 count=0;
		  	 firstsample=0;
		  }
		  else if(qbiased==0)
		   {
		  	  bias_cmq_n();
		  	  count=0;
		  	  firstsample=0;
		  	}
		  else if(againi==0)
		  {
			  minvalue=ip[0];
			  maxValue=ip[0];
			  for (count = 1; count < no_of_samples; ++count) {
				if ((ip[2*count]) < minvalue) {
					minvalue=ip[2*count];
				}
				if ((ip[2*count]) > maxValue) {
					maxValue=ip[2*count];
				}
			}
			  if ((maxValue==16383) || minvalue==0 ) {
			  					againi=1;
			  				}
			  				else {
			  					gain_i=gain_i+1;
			  					initPoti(gain_i,gain_q);
			  					ibiased=0;
			  				}
		  	  count=0;
		  	  firstsample=0;
		  	  //HAL_TIM_Base_Start_IT(&Timer_GPIO_Handle);
		  }
		  else if(againq==0)
		  {
			  minvalue=ip[1];
			  maxValue=ip[1];
			  for (count = 1; count < no_of_samples; ++count) {
				if ((ip[2*count+1]) < minvalue) {
					minvalue=ip[2*count+1];
				}
				if ((ip[2*count+1]) > maxValue) {
					maxValue=ip[2*count+1];
				}
			}
			  if ((maxValue==16383) || minvalue==0 ) {
			  					againq=1;
			  				}
			  				else {
			  					gain_q=gain_q+1;
			  					initPoti(gain_i,gain_q);
			  					qbiased=0;
			  				}
		  	  count=0;
		  	  firstsample=0;
		  }
		  else
		   {
		  	   processing();
		   }
		  	  
		  sampled=0;
	//UAVCAN Publisher
	
	uavcan::protocol::debug::KeyValue kv_msg;  // Always zero initialized
	kv_msg.value = distances[0];               // The distance of the nearest object 
	
	if (kv_msg.value > 0)
        {
		toggle_led(); 
		}
	
	const int pub_res = kv_pub.broadcast(kv_msg);
        if (pub_res < 0)
        {
           while(true){} 
        }
	}
}

void Init_Timer(void)
{


	__HAL_RCC_TIM2_CLK_ENABLE();

	Timer_GPIO_Handle.Instance = TIM2;
	Timer_GPIO_Handle.Init.Prescaler = 0;
	Timer_GPIO_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Timer_GPIO_Handle.Init.Period = 1749;
	HAL_TIM_Base_Init(&Timer_GPIO_Handle);

	HAL_TIM_Base_Start_IT(&Timer_GPIO_Handle);
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);


}

void TIM2_IRQHandler(void)
{

	HAL_TIM_IRQHandler(&Timer_GPIO_Handle);

}

void processing()
{
		for (count = 0; count < 2*no_of_samples_osd; ++count) {
			ip[count]=ip[count]/8192-1;
		}
	detrend(ip,no_of_samples_osd);
	for (count = 0; count < no_of_samples_osd; ++count) {

		float32_t window=0;
		if (ref==0) {
		hanning_window(&window,count,no_of_samples_osd);
		}
		else if (ref==1) {
			win_blackman(&window,count,no_of_samples_osd);
		}
		else {
			win_blackman_harris(&window,count,no_of_samples_osd);
		}
		//
		ip[2*count]=ip[2*count]*window;
		ip[2*count+1]=ip[2*count+1]*window;
		norm += window;

	}
	for (count = no_of_samples_osd; count < OP_FFTSIZE; ++count) {
		ip[2*count]=0;
		ip[2*count+1]=0;
	}
	norm=(float32_t)sqrtf(2.0/norm/no_of_samples_osd);
	count=0;
	firstsample=0;
	Find_FFT();
}

void Find_FFT()
{
	float32_t fftres=(float32_t)sampling_freq*1000/OP_FFTSIZE;
	uint16_t i=0;
	uint16_t yz=maxIndex;
	float32_t tempdist=0;
arm_cfft_f32(&arm_cfft_sR_f32_len4096, ip, 0, 1);
arm_cmplx_mag_f32(ip, op, OP_FFTSIZE);
arm_cmplx_mag_f32(ip, op1, OP_FFTSIZE);
for (yz = 0; yz < 0; ++yz) {
	op1[yz]=0;
}
for (yz = 1; yz < OP_FFTSIZE/2; ++yz) {
	op[yz] += op[OP_FFTSIZE-yz];
	op1[yz] += op1[OP_FFTSIZE-yz];
	op1[OP_FFTSIZE-yz]=0;
}
uint16_t startpos;
uint16_t stoppos;
for (i = 0; i < OP_FFTSIZE/2; ++i) {
	tempdist=(3*100000000)*(fftres*i)*(sweeptime*0.001)/(2*(fstop-fstart)*1000000000);
	if (tempdist>mindist) {
		startpos=i;
		break;
	}
}
for (i = 0; i < OP_FFTSIZE/2; ++i) {
	tempdist=(3*100000000)*(fftres*i)*(sweeptime*0.001)/(2*(fstop-fstart)*1000000000);
	if (tempdist>maxdist) {
		stoppos=i;
		break;
	}
	if (i==OP_FFTSIZE/2-1) {
		stoppos=i;
		maxdist=tempdist;
	}
}
peakdetect(startpos,stoppos);
for (i = 0; i <peak; ++i) {
	tempdist=(3*100000000)*(fftres*maxIndexes[i])*(sweeptime*0.001)/(2*(fstop-fstart)*1000000000);
	distances[i]=tempdist;
	}
for (i = peak; i < MAXELEMENTS; ++i) {
	distances[i]=0;
	maxIndexes[i]=0;
	maxValues[i]=0;
}
count=0;
}


void setdata()
{
	if (setvalue==1) {
		if (ib==4) {


			PointChar = (char*) &tempFloat;
						*PointChar = UsbTrans[0];
						PointChar++;
						*PointChar = UsbTrans[1];
						PointChar++;
						*PointChar = UsbTrans[2];
						PointChar++;
						*PointChar = UsbTrans[3];
						gain_i = tempFloat;
						ibiased=0;
						ib=0;
						setvalue=0;
						isconnected=1;
						initPoti(gain_i,gain_q);
						//CDC_Transmit_FS(UsbTrans, 1);

		}
	}
	else if (setvalue==2) {
			if (ib==4) {


				PointChar = (char*) &tempFloat;
							*PointChar = UsbTrans[0];
							PointChar++;
							*PointChar = UsbTrans[1];
							PointChar++;
							*PointChar = UsbTrans[2];
							PointChar++;
							*PointChar = UsbTrans[3];
							gain_q = tempFloat;
							setvalue=0;
							qbiased=0;
							ib=0;
							isconnected=1;
							initPoti(gain_i,gain_q);
							//CDC_Transmit_FS(UsbTrans, 1);
			}
		}
	else if (setvalue==3) {
			if (ib==4) {


				PointChar = (char*) &tempFloat;
							*PointChar = UsbTrans[0];
							PointChar++;
							*PointChar = UsbTrans[1];
							PointChar++;
							*PointChar = UsbTrans[2];
							PointChar++;
							*PointChar = UsbTrans[3];
							sweeptime = tempFloat;
							ibiased=0;
							qbiased=0;
							no_of_samples=sweeptime*sampling_freq;
							no_of_samples=powf(4,osd)*sweeptime*sampling_freq;
							no_of_samples_osd=no_of_samples/powf(4,osd);
							setvalue=0;
							ib=0;
							isconnected=1;
							configPll(fstart,fstop,sweeptime);
							stopPll();
							//1. No of Samples
							  tempint16 = no_of_samples_osd;
							  PointChar = (char*) &tempint16;
							  UsbTrans[ib++] = *PointChar++;
							  UsbTrans[ib++] = *PointChar;

							  //CDC_Transmit_FS(UsbTrans, 2);
							  ib=0;

			}
		}
	if (setvalue==4) {
		if (ib==4) {


			PointChar = (char*) &tempFloat;
						*PointChar = UsbTrans[0];
						PointChar++;
						*PointChar = UsbTrans[1];
						PointChar++;
						*PointChar = UsbTrans[2];
						PointChar++;
						*PointChar = UsbTrans[3];
						mindist = tempFloat;
						ib=0;
						setvalue=0;
						isconnected=1;
						//CDC_Transmit_FS(UsbTrans, 1);
		}
	}
	if (setvalue==5) {
		if (ib==4) {


			PointChar = (char*) &tempFloat;
						*PointChar = UsbTrans[0];
						PointChar++;
						*PointChar = UsbTrans[1];
						PointChar++;
						*PointChar = UsbTrans[2];
						PointChar++;
						*PointChar = UsbTrans[3];
						maxdist = tempFloat;
						ib=0;
						setvalue=0;
						isconnected=1;
						//CDC_Transmit_FS(UsbTrans, 1);
		}
	}

	if (setvalue==6) {
		if (ib==4) {


			PointChar = (char*) &tempFloat;
						*PointChar = UsbTrans[0];
						PointChar++;
						*PointChar = UsbTrans[1];
						PointChar++;
						*PointChar = UsbTrans[2];
						PointChar++;
						*PointChar = UsbTrans[3];
						delta = tempFloat;
						ib=0;
						setvalue=0;
						isconnected=1;
						//CDC_Transmit_FS(UsbTrans, 1);

		}
	}

	if (setvalue==7) {
		if (ib==4) {


			PointChar = (char*) &tempFloat;
						*PointChar = UsbTrans[0];
						PointChar++;
						*PointChar = UsbTrans[1];
						PointChar++;
						*PointChar = UsbTrans[2];
						PointChar++;
						*PointChar = UsbTrans[3];
						fstart = tempFloat;
						ib=0;
						setvalue=0;
						isconnected=1;
						configPll(fstart,fstop,sweeptime);
						stopPll();
						//CDC_Transmit_FS(UsbTrans, 1);

		}
	}

	if (setvalue==8) {
		if (ib==4) {


			PointChar = (char*) &tempFloat;
						*PointChar = UsbTrans[0];
						PointChar++;
						*PointChar = UsbTrans[1];
						PointChar++;
						*PointChar = UsbTrans[2];
						PointChar++;
						*PointChar = UsbTrans[3];
						fstop = tempFloat;
						ib=0;
						setvalue=0;
						isconnected=1;
						configPll(fstart,fstop,sweeptime);
						stopPll();
						//CDC_Transmit_FS(UsbTrans, 1);

		}
	}
	if (setvalue==9) {
		if (ib==1) {


			PointChar = (char*) &tempint8;
						*PointChar = UsbTrans[0];
						sampling_freq = (float32_t)(SystemCoreClock/2000/40/powf(2,tempint8)/powf(4,osd));
						no_of_samples=powf(4,osd)*sweeptime*sampling_freq;
						no_of_samples_osd=no_of_samples/powf(4,osd);
						ib=0;
						setvalue=0;
						isconnected=1;
						adc_interface_init(tempint8);

						//1. No of Samples
						  tempint16 = no_of_samples_osd;
						  PointChar = (char*) &tempint16;
						  UsbTrans[ib++] = *PointChar++;
						  UsbTrans[ib++] = *PointChar;

						  //CDC_Transmit_FS(UsbTrans, 2);
						  ib=0;
						  //2. Sampling Frequency
						  tempFloat = sampling_freq;
						  PointChar = (char*) &tempFloat;
						  UsbTrans[ib++] = *PointChar++;
						  UsbTrans[ib++] = *PointChar++;
						  UsbTrans[ib++] = *PointChar++;
						  UsbTrans[ib++] = *PointChar;

						  //CDC_Transmit_FS(UsbTrans, 4);
						  ib=0;

		}
	}
}
void sendinitialdata()
{

	//1. No of Samples
	  tempint16 = no_of_samples_osd;
	  PointChar = (char*) &tempint16;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar;

	  //CDC_Transmit_FS(UsbTrans, 2);
	  ib=0;

	  //2. Sampling Frequency
	  tempFloat = sampling_freq;
	  PointChar = (char*) &tempFloat;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar;

	  //CDC_Transmit_FS(UsbTrans, 4);
	  ib=0;


	  //3. Sweeptime
	  tempFloat = sweeptime;
	  PointChar = (char*) &tempFloat;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar;

	  //CDC_Transmit_FS(UsbTrans, 4);
	  ib=0;

	  //4. GainI
	  tempFloat = gain_i;
	  PointChar = (char*) &tempFloat;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar;

	  //CDC_Transmit_FS(UsbTrans, 4);
	  ib=0;

	  //5. GainQ

	  tempFloat = gain_q;
	  PointChar = (char*) &tempFloat;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar;

	  //CDC_Transmit_FS(UsbTrans, 4);
	  ib=0;

	  //6. fstart

	  tempFloat = fstart;
	  PointChar = (char*) &tempFloat;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar;

	  //CDC_Transmit_FS(UsbTrans, 4);
	  ib=0;

	  //7. fstop

	  tempFloat = fstop;
	  PointChar = (char*) &tempFloat;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar;

	  //CDC_Transmit_FS(UsbTrans, 4);
	  ib=0;

	  //8. CMI

	  tempFloat = cm_i;
	  PointChar = (char*) &tempFloat;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar;

	  //CDC_Transmit_FS(UsbTrans, 4);
	  ib=0;

	  //9. CMQ

	  tempFloat = cm_q;
	  PointChar = (char*) &tempFloat;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar;

	  //CDC_Transmit_FS(UsbTrans, 4);
	  ib=0;
}

void sendiq()
{
	  for (count = 0; count < no_of_samples_osd; ++count) {


	  tempFloat = ip[2*count];
	  PointChar = (char*) &tempFloat;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar;

	  tempFloat = ip[2*count+1];
	  PointChar = (char*) &tempFloat;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar++;
	  UsbTrans[ib++] = *PointChar;

	  //CDC_Transmit_FS(UsbTrans, 8);
	  ib=0;

	  }
}
void peakdetect(uint16_t startpos, uint16_t stoppos)
{
		uint16_t   i;
		float32_t  mx=-100000000;
	    float32_t  mn=100000000;
	    uint16_t   mx_pos = 0;
	    uint16_t   mn_pos = 0;
	    uint8_t lookmax=1;
	    peak=0;
	    for (i = startpos; i < stoppos; ++i) {
	    	if (op[i]>mx) {
				mx_pos=i;
				mx=op[i];
			}
	        if(op[i] < mn)
	        {
	            mn_pos = i;
	            mn = op[i];
	        }
	        if (lookmax==1) {
				if (op[i]< (mx-delta)) {
					if (mx_pos != startpos) {
					maxValues[peak]=mx;
					maxIndexes[peak]=mx_pos;
					peak++;
					}
					mn=op[i];
					mn_pos=i;
					lookmax=0;
				}
			}
	        else {
	        	if (op[i]> (mn+delta)) {
	        		mx=op[i];
	        		mx_pos=i;
	        		lookmax=1;
	        	}
			}

		}
}

void detrend(float32_t c[],uint16_t n)
{
    	    uint16_t i;
    	    float32_t a, b = 0.0, tsqsum = 0.0, ysum = 0.0, t;

    	    for (i = 0; i < n; i++)
    		ysum += c[2*i];



    	    ysum=0;
    	    for (i = 0; i < n; i++)
    		ysum += c[2*i];
    	    for (i = 0; i < n; i++) {
    		t = i - n/2 + 0.5;
    		tsqsum += t*t;
    		b += t*c[2*i];
    	    }
    	    b /= tsqsum;
    	    a = ysum/n - b*(n-1)/2.0;
    	    for (i = 0; i < n; i++)
    		c[2*i] -= a + b*i;

    	    a=0.0, b = 0.0, tsqsum = 0.0, ysum = 0.0, t=0.0;

    	    for (i = 0; i < n; i++)
    		ysum += c[2*i+1];
    	    for (i = 0; i < n; i++) {
    		t = i - n/2 + 0.5;
    		tsqsum += t*t;
    		b += t*c[2*i+1];
    	    }
    	    b /= tsqsum;
    	    a = ysum/n - b*(n-1)/2.0;
    	    for (i = 0; i < n; i++)
    		c[2*i+1] -= a + b*i;
}

void hanning_window(float32_t* w, uint16_t j, uint16_t n)
{
	float32_t a=2.0*PI/(n-1);
	*w = 0.5 - 0.5*(float32_t)cosf(a*j);
	}
void win_blackman_harris(float32_t* w, uint16_t j, uint16_t n)
{
    float32_t a = 2.0*PI/(n-1);

    *w = 0.35875 - 0.48829*cosf(a*j) + 0.14128*cosf(2*a*j) - 0.01168*cosf(3*a*j);
}

void win_blackman(float32_t* w, uint16_t j, uint16_t n)
{
    float32_t a = 2.0*PI/(n-1);

    *w = 0.42 - 0.5*cosf(a*j) + 0.08*cosf(2*a*j);

}
void bias_cmq_n()
{
	if (over==300) {
			        				qbiased=1;
			        				over=0;
			        				biasmin=0;
			        				biasmax=3.3;
			        				}
	else
		{
		uint16_t i=0;
		uint32_t biasparam = 0;
		float32_t vref = 3.3;
		uint16_t dac_send;


	float32_t voltperbit = (vref / (float)(pow(2,12) - 1));


		        		        for(i=0;i<no_of_samples;i++)
		        		        {

		        		              			biasparam +=ip[2*i+1];

		        		        }
		        		        biasparam=biasparam/no_of_samples;
		        		       if (((biasparam)<8242) && ((biasparam)>8142)) {
		        				qbiased=1;
		        				over=0;
		        				biasmin=0;
		        				biasmax=3.3;

		        				}
		        		       else if ((biasparam)<=8142)
		        		       {
		        		    	   biasmin=cm_q;
		        		    	   cm_q=(biasmin+biasmax)/2;
		        		    	   dac_send = ((uint16_t) (cm_q / voltperbit)) & 0x0FFF; //Holding the last 12 bits
		        		    	   		    	dac_send ^= (6 << 12);
		        		    	   		        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);//CS = Low
		        		    	   		        HAL_SPI_Transmit (&hspi2, (uint8_t*) &dac_send, sizeof(dac_send)/2, 0x1000);
		        		    	   		        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //CS = High
		        		    	   over++;
		        		       }
		        		       else
		        		{
			        		    	biasmax=cm_q;
			        		    	cm_q=(biasmin+biasmax)/2;
		        		    	dac_send = ((uint16_t) (cm_q / voltperbit)) & 0x0FFF; //Holding the last 12 bits
		        		    			    	dac_send ^= (6 << 12);
		        		    			        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);//CS = Low
		        		    			        HAL_SPI_Transmit (&hspi2, (uint8_t*) &dac_send, sizeof(dac_send)/2, 0x1000);
		        		    			        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //CS = High
		        		    	over++;
		        		}


	}
}

void bias_cmi_n()
{
	if (over==300) {
			        				ibiased=1;
			        				over=0;
			        				biasmin=0;
			        				biasmax=3.3;
			        				}
	else
		{
		uint16_t i=0;
		uint32_t biasparam = 0;
		float32_t vref = 3.3;
		uint16_t dac_send;


	float32_t voltperbit = (vref / (float)(pow(2,12) - 1));


		        		        for(i=0;i<no_of_samples;i++)
		        		        {

		        		              			biasparam +=ip[2*i];

		        		        }
		        		        biasparam=biasparam/no_of_samples;
		        		       if (((biasparam)<8242) && ((biasparam)>8142)) {
		        				ibiased=1;
		        				over=0;
		        				biasmin=0;
		        				biasmax=3.3;
		        				}
		        		       else if ((biasparam)<=8142)
		        		       {
		        		    	   biasmin=cm_i;
		        		    	   cm_i=(biasmin+biasmax)/2;
		        		    	   dac_send = ((uint16_t) (cm_i / voltperbit)) & 0x0FFF; //Holding the last 12 bits
		        		    	   		    	dac_send ^= (7 << 12);
		        		    	   		        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);//CS = Low
		        		    	   		        HAL_SPI_Transmit (&hspi2, (uint8_t*) &dac_send, sizeof(dac_send)/2, 0x1000);
		        		    	   		        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //CS = High
		        		    	   over++;
		        		       }
		        		       else
		        		{
		        		    	biasmax=cm_i;
		        		    	cm_i=(biasmin+biasmax)/2;
		        		    	dac_send = ((uint16_t) (cm_i / voltperbit)) & 0x0FFF; //Holding the last 12 bits
		        		    			    	dac_send ^= (7 << 12);
		        		    			        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);//CS = Low
		        		    			        HAL_SPI_Transmit (&hspi2, (uint8_t*) &dac_send, sizeof(dac_send)/2, 0x1000);
		        		    			        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //CS = High
		        		    	over++;
		        		}


	}
}

/** System Clock Configuration
*/

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}






/* SPI3 init function */
void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi3.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi3);

}


#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

