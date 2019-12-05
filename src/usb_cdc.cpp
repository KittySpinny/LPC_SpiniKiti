/*
===============================================================================
 Name        : main.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
 */

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// TODO: insert other include files here
#include "FreeRTOS.h"
#include "task.h"
#include "ITM_write.h"
#include <mutex>
#include "Fmutex.h"
#include "user_vcom.h"

#define TICKRATE_HZ 1000
#define black_th 3550


// TODO: insert other definitions and declarations here

int rotation(uint32_t reading, int count);
/* the following is required if runtime statistics are to be collected */
extern "C" {

void vConfigureTimerForRunTimeStats( void ) {
	Chip_SCT_Init(LPC_SCTSMALL1);
	LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
	LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
}

}
/* end runtime statictics collection */

/* Sets up system hardware */

void ADC_StartCalibration(LPC_ADC_T *pADC)
{
	// clock divider is the lowest 8 bits of the control register
	/* Setup ADC for about 500KHz (per UM) */
	uint32_t ctl = (Chip_Clock_GetSystemClockRate() / 500000) - 1;
	/* Set calibration mode */
	ctl |= ADC_CR_CALMODEBIT;
	pADC->CTRL = ctl;
	/* Calibration is only complete when ADC_CR_CALMODEBIT bit has cleared */
	while(pADC->CTRL & ADC_CR_CALMODEBIT) { };
}

static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();

}

static void ADC_setup(void)
{
	//SETUP ADC
	Chip_ADC_Init(LPC_ADC0, 0);

	/* Setup for ADC clock rate */
	Chip_ADC_SetClockRate(LPC_ADC0, 500000);
	/* For ADC0, sequencer A will be used without threshold events.
		 It will be triggered manually, convert CH8 and CH10 in the sequence */
	Chip_ADC_SetupSequencer(LPC_ADC0, ADC_SEQA_IDX, (ADC_SEQ_CTRL_CHANSEL(0) | ADC_SEQ_CTRL_CHANSEL(8) | ADC_SEQ_CTRL_CHANSEL(10) |
			ADC_SEQ_CTRL_MODE_EOS));

	/* For ADC0, select analog input pin for channel 0 on ADC0 */
	Chip_ADC_SetADC0Input(LPC_ADC0, 0);
	/* Use higher voltage trim for both ADC */
	Chip_ADC_SetTrim(LPC_ADC0, ADC_TRIM_VRANGE_HIGHV);
	/* Assign ADC0_8 to PIO1_0 via SWM (fixed pin) and ADC0_10 to PIO0_0 */
	Chip_SWM_EnableFixedPin(SWM_FIXED_ADC0_0);
	Chip_SWM_EnableFixedPin(SWM_FIXED_ADC0_8);
	Chip_SWM_EnableFixedPin(SWM_FIXED_ADC0_10);
	/* Need to do a calibration after initialization and trim */
	//while (!(Chip_ADC_IsCalibrationDone(LPC_ADC0))); // The NXP library function violates their own access rules given in data sheet so we can't use it
	ADC_StartCalibration(LPC_ADC0);
	/* Set maximum clock rate for ADC */
	/* Our CPU clock rate is 72 MHz and ADC clock needs to be 50 MHz or less
	 * so the divider must be at least two. The real divider used is the value below + 1
	 */
	Chip_ADC_SetDivider(LPC_ADC0, 1);
	/* Chip_ADC_SetClockRate set the divider but due to rounding error it sets the divider too low
	 * which results in a clock rate that is out of allowed range
	 */
	//Chip_ADC_SetClockRate(LPC_ADC0, 500000); // does not work with 72 MHz clock when we want maximum frequency
	/* Clear all pending interrupts and status flags */
	Chip_ADC_ClearFlags(LPC_ADC0, Chip_ADC_GetFlags(LPC_ADC0));
	/* Enable sequence A completion interrupts for ADC0 */
	Chip_ADC_EnableInt(LPC_ADC0, ADC_INTEN_SEQA_ENABLE);
	/* We don't enable the corresponding interrupt in NVIC so the flag is set but no interrupt is
		triggered */
	/* Enable sequencer */
	Chip_ADC_EnableSequencer(LPC_ADC0, ADC_SEQA_IDX);
	/* Configure systick timer */
	SysTick_Config(Chip_Clock_GetSysTickClockRate() / TICKRATE_HZ);
}



static void receive_task(void *pvParameters) {
	bool LedState = false;

	while (1) {
		char str[80];
		uint32_t len = USB_receive((uint8_t *)str, 79);
		str[len] = 0; /* make sure we have a zero at the end so that we can print the data */
		ITM_write(str);

		Board_LED_Set(1, LedState);
		LedState = (bool) !LedState;
	}
}

static void sensor_test_task(void *pvParameters)
{

	uint32_t a0 = 0;
	uint32_t a1 = 0;
	uint32_t s1 = 0;
	uint32_t s2 = 0;

	char test_S1[10];
	char test_S2[10];
	char distance[10];

	int timeout  = 0;

	int dir = 0;
	int count = 0;

	bool hit = false;

	vTaskDelay(1000);

	while(1)
	{
		Chip_ADC_StartSequencer(LPC_ADC0, ADC_SEQA_IDX); // poll sequence complete flag
		while(!(Chip_ADC_GetFlags(LPC_ADC0) & ADC_FLAGS_SEQA_INT_MASK));
		// clear the flags
		Chip_ADC_ClearFlags(LPC_ADC0, Chip_ADC_GetFlags(LPC_ADC0));
		// get data from ADC channels
		a0 = Chip_ADC_GetDataReg(LPC_ADC0, 0); // raw value
		a1 = Chip_ADC_GetDataReg(LPC_ADC0, 8); // raw value

		s1 = ADC_DR_RESULT(a0);
		s2 = ADC_DR_RESULT(a1);


		//DETERMINE WHICH SENSOR
		if(s1 > 3900 && dir != 2 && dir != 1)
		{
			dir = 1;
		}
	/*	else if(s2 > 3900 && dir != 1 && dir != 2)
		{
			dir = 2;
		}*/

		//READ SENSOR DEPENDING ON DIRECTION
		if(dir == 1) {
			if(s1 > 2900 && hit == false)
			{
				count++;
				timeout = xTaskGetTickCount();
				hit = true;
			}
			else if(s1 < 3900){hit = false;}

		}
		else if(dir == 2){

			if(s2 > 3900 && hit == false)
			{
				//count++;
				timeout = xTaskGetTickCount();
				hit = true;

			}
			else if(s2 < 3900){hit = false;}
		}
		if((xTaskGetTickCount() - timeout) >= 10000 && s1 < 3900 /*&& s2 < 3900*/)
		{
			//strcpy(distance, std::to_string(count).c_str());
			int color = sprintf(distance, "%d\r\n", count);
		//	USB_send((uint8_t*)distance, color);
			dir = 0;
			count = 0;
			timeout = 0;
		}


		sprintf(test_S1, "%d ", s1);
		sprintf(test_S2, "%d	", s2);
		sprintf(distance, "%d\r\n", count);
		ITM_write(test_S1);
		ITM_write(test_S2);

		ITM_write(distance);

		vTaskDelay(100);

		/*else
		{
			int color = sprintf(str, white);
			USB_send((uint8_t*)str, color);
			sprintf(test_S1, "%d\r\n", s1);
			sprintf(test_S2, "%d\r\n", (int) s2);
			ITM_write(test_S1);
			ITM_write(test_S2);
			vTaskDelay(10);
		}*/
	}


}

int main(void) {





    prvSetupHardware();
	ADC_setup();

	ITM_init();


	xTaskCreate(sensor_test_task, "sensor_test",
			configMINIMAL_STACK_SIZE * 3, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	/*	xTaskCreate(send_task, "Tx",
				configMINIMAL_STACK_SIZE * 3, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);
	 */

	xTaskCreate(receive_task, "Rx",
			configMINIMAL_STACK_SIZE * 3, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);


	xTaskCreate(cdc_task, "CDC",
			configMINIMAL_STACK_SIZE * 3, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);




	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}

int rotation(uint32_t reading, int count)
{
	bool hit = false;
	if(reading > 3900 && hit == false)
	{
		count++;
		hit == true;
	}
	else{hit = false;}
	return count;
}
