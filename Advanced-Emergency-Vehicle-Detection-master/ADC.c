#include <fsl_device_registers.h>
#include "ADC.h"

unsigned short ADC_read16b(void)
{
	ADC0_SC1A = 0 & ADC_SC1_ADCH_MASK; //Write to SC1A to start conversion
	while(ADC0_SC2 & ADC_SC2_ADACT_MASK); //Conversion in progress
	while(!(ADC0_SC1A & ADC_SC1_COCO_MASK)); //Wait until conversion complete
	return ADC0_RA;
}

void DelayFunction (void)
{
	unsigned long Counter = 0xFFFFF;
	do
	{
		Counter--;
	}while(Counter);
}

void initADC(void)
{
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
	ADC0_CFG1 |= ADC_CFG1_MODE(3); //16 bit single-ended
	ADC0_SC1A |= ADC_SC1_ADCH(0); 
	
	SIM_SCGC3 |= SIM_SCGC3_ADC1_MASK;
	ADC1_CFG1 |= ADC_CFG1_MODE(3); //16 bit single-ended
	ADC1_SC1A |= ADC_SC1_ADCH(0); 
	
}
