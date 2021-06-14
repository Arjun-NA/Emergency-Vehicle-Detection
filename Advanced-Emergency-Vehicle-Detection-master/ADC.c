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
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;	// System Integration Module -> System Clock Gating Control Register	
	SIM_SCGC3 |= SIM_SCGC3_ADC1_MASK;	// System Integration Module -> System Clock Gating Control Register

	ADC0_SC1A |= ADC_SC1_ADCH(0); 		// Channel 0
	ADC1_SC1A |= ADC_SC1_ADCH(0); 		// Channel 0

	ADC0_SC1A |= ADC_SC1_DIFF(0); 		// Single Ended
	ADC1_SC1A |= ADC_SC1_DIFF(0); 		// Single Ended

	ADC0_SC1A |= ADC_SC1_AIEN(0); 		// Interrupt Disable
	ADC1_SC1A |= ADC_SC1_AIEN(0); 		// Interrupt Disable
	
	ADC0_CFG1 |= ADC_CFG1_ADICLK(0); 	// Bus clock
	ADC1_CFG1 |= ADC_CFG1_ADICLK(0); 	// Bus clock

	ADC0_CFG1 |= ADC_CFG1_MODE(3); 		//16 bit 
	ADC1_CFG1 |= ADC_CFG1_MODE(3); 		//16 bit 
		
	ADC0_CFG1 |= ADC_CFG1_ADLSMP(0);  	// short sample time
	ADC1_CFG1 |= ADC_CFG1_ADLSMP(0);	// short sample time


	ADC0_CFG2 |= ADC_CFG2_ADHSC(1);   	// high speed conversion
	ADC1_CFG2 |= ADC_CFG2_ADHSC(1);   	// high speed conversion
	ADC0_CFG2 |= ADC_CFG2_MUXSEL(0);   	// ADC Mux Select channel A
	ADC1_CFG2 |= ADC_CFG2_MUXSEL(0);   	// ADC Mux Select channel A
	
}
