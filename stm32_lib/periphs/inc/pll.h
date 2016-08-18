#ifndef PLL_H_
#define PLL_H_

#ifdef __cplusplus
 extern "C" {
#endif

#define NR_FREQ_CHECK_SAMPLES 200

#include <stm32f4xx.h>
#include "arm_math.h"

typedef enum { false_=0, true_=1 } bool; //Erstelle Datentyp bool !!!


void initPll();

void initSpi();

void initGPIO();

/*uint8_t setMaxFreqRangePll(struct uc_rapid *rapid);*/
void calibrate_pll(float32_t* fstart, float32_t* fstop, float32_t sweep);

void configPll(float32_t fStart, float32_t fStop, float32_t sweeptime);

void setRampSettingsPll();

void setRegsPll();

void startPll();

void stopPll();

void sendCommand(uint32_t command);

void setReg0();

void setReg1();

void setReg2();

void setReg3();

void setReg4();

void setReg5();

void setReg6();

void setReg7();

void swapEndian(uint32_t *R);
	
#ifdef __cplusplus
}
#endif


#endif /* PLL_H_ */
