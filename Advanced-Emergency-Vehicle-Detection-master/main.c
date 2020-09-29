#include <fsl_device_registers.h>
#include "ADC.h"
#include <stdlib.h>
#include "board.h"
#include <math.h>
#define ARM_MATH_CM4
#include <arm_math.h>
#include "fsl_debug_console.h"

uint32_t maxVal_Index;
unsigned int INPUT_LEN = 2000;
unsigned int Correlation_length = 3999;
float32_t maxVal;
float32_t time_Delay;
float32_t sampling_T = 1.7644E-6f;
float32_t theta;
float32_t dist = 0.05f;

void AudioAcquire(float32_t * mic0, float32_t * mic1);
void printTerminal(float32_t* data);    	 
void LED_init();    	 
void LED_out(float32_t theta);

int main()
{
    hardware_init();
    initADC();
    LED_init();

    //init data arrays
    float32_t *mic0Data = (float32_t*)
    malloc(INPUT_LEN*sizeof(float32_t));
    if (!mic0Data) 
    {
	debug_printf("memory allocation failed!");
  	return -1;
    }

    float32_t *mic1Data = (float32_t*)
    malloc(INPUT_LEN*sizeof(float32_t));
    if (!mic1Data)
    {
debug_printf("memory allocation failed!");
return -1;
    }
    
float32_t *correlationData = (float32_t*)
malloc(Correlation_length*sizeof(float32_t));
    if (!correlationData) 
{
debug_printf("memory allocation failed!");
return -1;
    }
    
while(1)
{
AudioAcquire(mic0Data, mic1Data);
arm_correlate_f32(mic0Data, INPUT_LEN, mic1Data, INPUT_LEN,
correlationData);
arm_max_f32(correlationData, Correlation_length, &maxVal,
&maxVal_Index);
debug_printf("%d %d\r\n", (unsigned)maxVal,
(unsigned)maxVal_Index);
time_Delay = sampling_T*((int)maxVal_Index - (int)INPUT_LEN);
theta = asinf(343*time_Delay/dist)*180/3.14159265;
debug_printf("angle: %d \r\n", (int)theta);
LED_out(theta);
    }
}

void LED_init() 
{
    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    PORTC->PCR[8] = PORT_PCR_MUX(001);
    PTC->PDDR |= (1<<8);
    PORTC->PCR[9] = PORT_PCR_MUX(001);
    PTC->PDDR |= (1<<9);
    PORTC->PCR[0] = PORT_PCR_MUX(001);
    PTC->PDDR |= (1<<0);
    PORTC->PCR[5] = PORT_PCR_MUX(001);
    PTC->PDDR |= (1 << 5);
    PORTC->PCR[7] = PORT_PCR_MUX(001);
    PTC->PDDR |= (1<<7);
    PTC->PTOR |= 0x00003A1;
    DelayFunction();
    PTC->PTOR |= 0x00003A1;
}

void LED_out(float32_t theta) 
{
    PTC->PCOR |= 0xFFFFFFFF;
    if(theta < -50)
   	 PTC->PSOR = (1 << 8);
    else if(theta < -20)
   	 PTC->PSOR = (1 << 9);
    else if(theta < 20)
   	 PTC->PSOR = (1 << 0);
    else if(theta < 50)
   	 PTC->PSOR = (1 << 7);
    else if (theta < 90)
   	 PTC->PSOR = (1 << 5);    
}

void AudioAcquire(float32_t* mic0, float32_t * mic1)
{
    int i;
    float32_t mic0Mean;
    float32_t mic1Mean;
    
	for (i = 0; i < INPUT_LEN; i++)
    {
   	 ADC0_SC1A = 0 & ADC_SC1_ADCH_MASK;
   	 while(ADC0_SC2 & ADC_SC2_ADACT_MASK);
   	 while(!(ADC0_SC1A & ADC_SC1_COCO_MASK));
   	 mic0[i] = (((float32_t)ADC0_RA)/1000);

   	 ADC1_SC1A = 0 & ADC_SC1_ADCH_MASK;
   	 while(ADC1_SC2 & ADC_SC2_ADACT_MASK);
   	 while(!(ADC1_SC1A & ADC_SC1_COCO_MASK));
   	 mic1[i] = (((float32_t)ADC1_RA)/1000);
    }
    arm_mean_f32(mic0, INPUT_LEN, &mic0Mean);
    arm_mean_f32(mic1, INPUT_LEN, &mic1Mean);
    arm_offset_f32(mic0, -mic0Mean, mic0, INPUT_LEN);
    arm_offset_f32(mic1, -mic1Mean, mic1, INPUT_LEN);
}

void printTerminal(float32_t * data)
{
    int i;
    for (i = 0; i < Correlation_length; i++)
   	 debug_printf("%d\r\n", (unsigned)data[i]);
}