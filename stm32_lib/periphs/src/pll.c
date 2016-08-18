#include "pll.h"

SPI_HandleTypeDef  hspi1;

uint16_t prescaler_rf_ = 64;
uint16_t prescaler_steps_ = 100;
float32_t freq_start_ghz_ = 121.0;
float32_t freq_stop_ghz_ = 122.2;
uint16_t bandwidth_MHz_ = 1200;
float32_t sweeptime_ms_ = 10.0;

bool d_ = false_;
uint16_t r_ = 2;
bool t_ = false_;

uint16_t int_ = 63;
uint32_t frac_ = 699051;

uint32_t f_ref_ = 60000000;

bool ramp_ = false_;
uint32_t mux_out_ = 0b0110;

bool csr_ = false_;
uint16_t cp_i_ = 4;
bool prescaler_ = 0;

uint16_t clk1_ = 0;
bool NSEL_ = false_;
bool SD_Reset_ = true_;
uint8_t RampMode_ = 0b00;

bool PSK_ = false_;
bool FSK_ = false_;
bool LDP_ = 0;

bool phase_detector_ = 1;

bool PD_ = false_;
bool CP3_ = false_;
bool CntRes_ = false_;

bool LE_sel_ = 0;
bool ReadBack_ = false_;

uint8_t CLK_Div_ = 0b00;

uint32_t Div_ = 0;
bool Parabolic_Ramp_ = false_;
bool FSK_Ramp_ = false_;
bool Ramp2_  = false_;
uint32_t Deviation_Offset1_ = 0;
uint32_t Deviation1_ = 6991;
uint32_t Deviation_Offset2_ = 0;
uint32_t Deviation2_ = 0;
uint32_t StepWord1_ = 3000;
uint32_t StepWord2_ = 0;

bool RampDelay_FL_ = false_;
bool RampDelay_ = false_;
bool Delay_Clock_ = 0;

bool Delayed_Start_enable_ = false_;
bool Delayed_Start_ = 0;


void setRampSettingsPll() {

	float32_t fStartIF_GHz = freq_start_ghz_/prescaler_rf_;
	float32_t bandwidthIF_MHz = bandwidth_MHz_/((float32_t)prescaler_rf_);
	float32_t fpfd = 30000000.0;
	float32_t DIV = 1.0E9*fStartIF_GHz/fpfd;
	uint32_t NStep;
	float32_t fres;
	bool checkOK;
	uint32_t deviation;
	uint32_t dev_offset;
	float32_t fdev;
	float32_t tStep_ms;

	int_ = (uint16_t)(DIV);
	frac_ = (uint32_t)(((DIV - int_)*powf(2,25)));
	NStep = (uint32_t)(1+1e-3*sweeptime_ms_*fpfd);
	NStep = (uint32_t) (1+ NStep/prescaler_steps_);
	if( ((uint32_t) (1+ log2(NStep))) > 20){
		NStep = powf(2,20)-1;
	}

	fres = fpfd / powf(2,25);
	checkOK = false_;
	NStep *= 2;
	while (!checkOK) {
	    NStep /= 2;
	    fdev = 1E6*bandwidthIF_MHz/NStep;
	    dev_offset = (uint32_t)(1+ log2f(fdev/(fres*powf(2,15))));
	    if(dev_offset < 0)
	    	dev_offset = 0;
	    deviation = (uint32_t) (fdev/fres*powf(2,dev_offset));
	    if(deviation > 20)
	    	checkOK = true_;
	}

	Deviation_Offset1_ = dev_offset;
	Deviation1_ = deviation;
	StepWord1_ = (uint32_t)(NStep);

	tStep_ms = sweeptime_ms_ / StepWord1_;
	CLK_Div_ = 0b11;
	Div_ = 1;
	clk1_ = (uint16_t)(fpfd*1E-3*tStep_ms);

}

void initPll() {
	initSpiPll();
	initGpioPll();
}


void configPll(float32_t fStart, float32_t fStop, float32_t sweeptime) {

	freq_start_ghz_ = fStart;
	freq_stop_ghz_ = fStop;
	bandwidth_MHz_ = (freq_stop_ghz_ - freq_start_ghz_) * 1e3;
	sweeptime_ms_ = sweeptime;

	setRampSettingsPll();
	setRegsPll();

}

void setRegsPll() {
	setReg7();
	setReg6();
	setReg5();
	setReg4();
	setReg3();
	setReg2();
	setReg1();
	setReg0();
}

void initSpiPll() {

	hspi1.Instance   		=   SPI1 ; //PLL
    hspi1.Init.Mode   		=   SPI_MODE_MASTER ;
    hspi1.Init.Direction    =   SPI_DIRECTION_2LINES ; //Full-Duplex
    hspi1.Init.DataSize   	=   SPI_DATASIZE_16BIT ; //
    hspi1.Init.CLKPolarity  =   SPI_POLARITY_LOW ; 	// clock is low when idle
    hspi1.Init.CLKPhase     =   SPI_PHASE_1EDGE ;  //
    hspi1.Init.NSS   		=   SPI_NSS_SOFT ; //Chip select hardware/sofware
    hspi1.Init.BaudRatePrescaler =   SPI_BAUDRATEPRESCALER_32 ; //SPI1-freq is APB2-freq/2(clock/2=168/2/2)/Prescaler |84MHz/2=42MHz
    hspi1.Init.FirstBit   	=   SPI_FIRSTBIT_MSB ;
    hspi1.Init.TIMode   	=   SPI_TIMODE_DISABLED ;
    hspi1.Init.CRCCalculation    =   SPI_CRCCALCULATION_DISABLED ;
	//hspi1.Init.CRCPolynomial   =   7 ;
    HAL_SPI_Init(&hspi1);
    __HAL_SPI_ENABLE(&hspi1);

}


void initGpioPll(){
	
	// PLL Chip Select
    GPIO_InitTypeDef  GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    __GPIOA_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    // PLL digital lock detect
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    __GPIOD_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
}

void sendCommand(uint32_t command) {

	uint16_t i;
	swapEndian(&command);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	for(i=0;i<10;i++);
	HAL_SPI_Transmit( &hspi1, (uint8_t*) &command,  sizeof(command)/2, 0x1000);
	for(i=0;i<10;i++);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	for(i=0;i<10;i++);
}

void setReg0() {
	uint32_t frac_tmp = (frac_ & 0x3FFE000) >> 13;
	uint32_t reg0 = 	(ramp_ << 31)
					|	(mux_out_ << 27)
					|	(int_ << 15)
					|	(frac_tmp << 3)
					|	(0b000 << 2);
	sendCommand(reg0);
}

void setReg1() {
	uint32_t frac_tmp = (frac_ & 0x1FFF);
	uint32_t reg1 =		(frac_tmp << 15)
					|	(0b1);
	sendCommand(reg1);
}

void setReg2() {
	uint32_t reg2 = 	(csr_ << 28)
					|	(cp_i_ << 24)
					|	(prescaler_ << 22)
					|	(t_ << 21)
					|	(d_ << 20)
					|	(r_ << 15)
					|	(clk1_ << 3)
					| 	(0b010);
	sendCommand(reg2);
}

void setReg3() {
	uint32_t reg3 = 	(NSEL_ << 15)
					|	(SD_Reset_ << 14)
					|	(RampMode_ << 10)
					|	(PSK_ << 9)
					|	(FSK_ << 8)
					|	(LDP_ << 7)
					| 	(phase_detector_ << 6)
					|	(PD_ << 5)
					|	(CP3_ << 4)
					| 	(CntRes_ << 3)
					|	(0b11);
	sendCommand(reg3);
}

void setReg4() {
	uint32_t reg4 = 	(LE_sel_ << 31)
					|	(ReadBack_ << 21)
					|	(CLK_Div_ << 19)
					|	(Div_ << 6)
					|	(0b1 << 2);
	sendCommand(reg4);
}

void setReg5() {
	uint32_t reg5a = 	(Parabolic_Ramp_ << 28)	
					|	(FSK_Ramp_ << 25)
					|	(Ramp2_ << 24)
					|	(Deviation_Offset1_ << 19)
					|	(Deviation1_ << 3)
					|	(0b101);
	uint32_t reg5b = 	(Parabolic_Ramp_ << 28)	
					|	(FSK_Ramp_ << 25)
					|	(Ramp2_ << 24)
					|	(0b1 << 23)
					|	(Deviation_Offset2_ << 19)
					|	(Deviation2_ << 3)
					|	(0b101);	
	sendCommand(reg5a);
	sendCommand(reg5b);
}

void setReg6() {
	uint32_t reg6a =	(StepWord1_ << 3)
					|	(0b110);
	uint32_t reg6b =	(0b1 << 23)
					|	(StepWord2_ << 3)
					|	(0b110);
	sendCommand(reg6a);
	sendCommand(reg6b);
}

void setReg7() {
	uint32_t reg7 =		(RampDelay_FL_ << 18)
					|	(RampDelay_ << 17)
					|	(Delay_Clock_ << 16)
					|	(Delayed_Start_enable_ << 15)
					|	(Delayed_Start_ << 14)
					|	(0b111);
	sendCommand(reg7);
}

void startPll() {
	ramp_ = true_;
	setReg0();
}

void stopPll() {
	ramp_ = false_;
	setReg0();
}

void swapEndian(uint32_t *R)
{
	uint32_t tmp=0, tmp1;
	tmp = *R;
	tmp <<= 16;
	tmp1 = *R;
	tmp1 >>= 16;
	tmp ^= tmp1;
	*R = tmp;
}

void calibrate_pll(float32_t* fstart, float32_t* fstop, float32_t sweep)
{
	uint8_t lock_detection[150];
	float32_t fstart_cali = 115;
	fstart_cali = *fstart;
	float32_t sweep_cali= sweep;
	uint16_t config=0;
	for(config=0; fstart_cali < (*fstop);config++){
		configPll(fstart_cali,fstart_cali+0.1,sweep_cali);
		HAL_Delay(2);
		lock_detection[config] = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9);
		fstart_cali += 0.1;
	}
    uint16_t config1 = config;
	float32_t best_start=0;
	uint32_t best_bandwidth=0;
	for (config = 0; config <= config1; config++) {
		if (lock_detection[config]) {
			float32_t best_start_t=0;
			uint32_t best_bandwidth_t=0;
			best_start_t= (*fstart) + 0.1*config;
			config++;
			while(lock_detection[config] && config<150)
			{
				best_bandwidth_t += 100;
				config++;
			}
			if (best_bandwidth_t > best_bandwidth) {
				best_bandwidth=best_bandwidth_t;
				best_start = best_start_t;
			}
		}
	}
	*fstart=best_start;
	*fstop=best_start+(best_bandwidth/1000);

}
