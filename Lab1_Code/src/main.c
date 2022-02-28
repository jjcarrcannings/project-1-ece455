/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wwrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
This project simulates a traffic buildup at a traffic light. Here is a description of the project's functionality:

The main() Function:
main() performs initialization and the creates the tasks and software timers described in this section, before
starting the scheduler.




*/

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "stm32f4_discovery.h"

/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

/*-----------------------------------------------------------*/
#define mainQUEUE_LENGTH 100

#define GREEN_LIGHT_PIN GPIO_Pin_0
#define YELLOW_LIGHT_PIN GPIO_Pin_1
#define RED_LIGHT_PIN GPIO_Pin_2
#define ADC_PIN GPIO_Pin_3

#define Data_Pin	GPIO_Pin_6
#define Clock_Pin	GPIO_Pin_7
#define Reset_Pin	GPIO_Pin_8

#undef configUSE_TICK_HOOK
#define configUSE_TICK_HOOK 1

/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );

/*
 * Task declarations.
 */
static void Potentiometer_Read_Task( void *pvParameters );
static void Traffic_Generator_Task( void *pvParameters );
static void Traffic_Light_State_Task( TimerHandle_t );
static void System_Display_Task( void *pvParameters );

/*
 * Global queue handles.
 */
xQueueHandle xQueue_potentiometer = 0;
xQueueHandle xQueue_traffic_light = 0;
xQueueHandle xQueue_car_generation = 0;
xTimerHandle xTimer_traffic_light_state = 0;


/*-----------------------------------------------------------*/
/*-------Function Definitions--------------------------------*/
/*-----------------------------------------------------------*/

int init(void){
	GPIO_InitTypeDef gpio_traffic_light;
	GPIO_InitTypeDef gpio_adc;
	ADC_InitTypeDef adc_itd;
	TimerHandle_t timer_handle;

	//Enable Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	//Configure GPIO traffic light pins
	gpio_traffic_light.GPIO_Mode = GPIO_Mode_OUT;
	gpio_traffic_light.GPIO_OType = GPIO_OType_PP;
	gpio_traffic_light.GPIO_Pin = 
		GREEN_LIGHT_PIN | YELLOW_LIGHT_PIN | RED_LIGHT_PIN | Data_Pin | Clock_Pin | Reset_Pin;
	gpio_traffic_light.GPIO_PuPd = GPIO_PuPd_DOWN;
	gpio_traffic_light.GPIO_Speed = GPIO_Speed_100MHz;
	
	//Configure GPIO ADC pin
	gpio_adc.GPIO_Mode = GPIO_Mode_AN;
	gpio_adc.GPIO_Pin = ADC_PIN;
	gpio_adc.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio_adc.GPIO_Speed = GPIO_Speed_100MHz;

	//Initialize GPIOC for traffic light and ADC pins
	GPIO_Init(GPIOC, &gpio_traffic_light);
	GPIO_Init(GPIOC, &gpio_adc);

	//Configure ADC
	adc_itd.ADC_ContinuousConvMode = 0;
	adc_itd.ADC_DataAlign = ADC_DataAlign_Right;
	adc_itd.ADC_ExternalTrigConv = DISABLE;
	adc_itd.ADC_Resolution = ADC_Resolution_10b;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	//Initialize ADC
	ADC_Init(ADC1, &adc_itd);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);

	ADC_Cmd(ADC1, ENABLE);
	ADC_SoftwareStartConv(ADC1);

	/*//Create timer
	TimerHandle_t timer_handle = xTimerCreate("Traffic Light Timer", 1000, pdTRUE, NULL, Traffic_Light_State_Task);
	if(timer_handle == NULL) {
		fprintf(stderr, "Timer not created\n");
		return 1;
	}

	//Start timer
	if(xTimerStart( timer_handle, 0 ) != pdPASS )
	{
		printf("The timer could not be set into the Active state.\n");
		return 2;
	}*/


	//Provide seed for random number generator
	time_t t;
	srand((unsigned) time(&t));

	return 0;
}


int main(void)
{
	//Initialize hardware
	if(!init()){
		printf("Hardware initialized\n");
	}
	else{
		printf("Hardware initialization failed\n");
		return 1;
	}

	prvSetupHardware();

	//Create queues
	xQueue_potentiometer = xQueueCreate(mainQUEUE_LENGTH, sizeof(uint16_t));
	xQueue_car_generation = xQueueCreate( 	mainQUEUE_LENGTH, sizeof(uint16_t));

	//Create tasks
	vQueueAddToRegistry( xQueue_potentiometer, "PotentiometerQueue" );
	vQueueAddToRegistry( xQueue_car_generation, "CarGenerationQueue" );

	//Create tasks
	xTaskCreate( Potentiometer_Read_Task, "Potentiometer_Read", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Traffic_Generator_Task, "Traffic_Generator", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( System_Display_Task, "System_Display", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	xTimer_traffic_light_state = xTimerCreate("Traffic Light Timer", 1000, pdTRUE, NULL, Traffic_Light_State_Task);
	xTimerStart(xTimer_traffic_light_state, 0);

	//Start the tasks and timer.
	vTaskStartScheduler();

	return 0;
}

int traffic_light_is_green()
{
	return GPIO_ReadInputDataBit(GPIOC, GREEN_LIGHT_PIN);
}

int traffic_light_is_yellow()
{
	return GPIO_ReadInputDataBit(GPIOC, YELLOW_LIGHT_PIN);
}

int traffic_light_is_red()
{
	return GPIO_ReadInputDataBit(GPIOC, RED_LIGHT_PIN);
}

/*-----------------------------------------------------------
/*
 * Potentionmeter Read Task
 * Description: Reads potentiometer value and puts it in queue, then sleeps for a half a second, and
 * removes the value from the queue.
 * Input: void *pvParameters - pointer to parameters
 */

static void Potentiometer_Read_Task( void *pvParameters )
{
	uint16_t adc_val, old_adc_val;
	for(;;) {
		ADC_SoftwareStartConv(ADC1);

		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

		//Divide value by 10 so that it is in the range of 0-100
		adc_val = ADC_GetConversionValue(ADC1) / 10;

		//Pop old value of the queue
		xQueueReceive(xQueue_potentiometer, NULL, 10);

		//Send new adc value into the queue (value not used)
		xQueueSend(xQueue_potentiometer,&adc_val,1000);

		//Sleep for half a second
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}


/*-----------------------------------------------------------*/
/*
 * Traffic Generator Task
 * Description: Reads potentiometer value from queue, and then uses that value in addition to a
 * random number to determine whether a car should be generated. The higher the value of the 
 * potentiometer, the more likely a car will be generated.
 * Input: void *pvParameters - pointer to parameters
 * Output: void
 */
static void  Traffic_Generator_Task( void *pvParameters )
{
	uint16_t potentiometer_val = 0;
	for(;;) {
		//Read potentiometer value from queue
		xQueuePeek(xQueue_potentiometer, &potentiometer_val, 1000);
		
		//Shift the potentiometer value up by 20 and scale so that max stays at 100
		potentiometer_val += 20;
		potentiometer_val = (int)((double)potentiometer_val/1.2);

		//Generate random number between 0 and 100
		int rand_val = (rand()*100.0) / RAND_MAX;
		printf("rand: %d --- adc: %u \n", rand_val, potentiometer_val);
		
		//If random number is greater than potentiometer value, generate car
		//By sending a 1 to the queue, the car generation task will generate a car
		//If random number is less than potentiometer value, do not generate car
		//By sending a 0 to the queue, the car generation task will not generate a car
		if(rand_val <= potentiometer_val) { //Create new car
			//Send car to new queue
			int new_car_val = 1;
			xQueueSend(xQueue_car_generation, &new_car_val, 1000);
		} else{
			int new_car_val = 0;
			xQueueSend(xQueue_car_generation, &new_car_val, 1000);
		}
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

/*-----------------------------------------------------------*/

/*
 * Traffic Light State Task
 * Description: Timer triggered task that changes the state of the traffic light when called.
 * Modifies timer period based on potentiometer value and new traffic light state.
 * Input: TimerHandle_t xTimer - timer handle
 * Output: void
 */
void Traffic_Light_State_Task( TimerHandle_t xTimer ) {
	uint16_t potentiometer_val = 0;

	//Read potentiometer value from queue
	xQueuePeek(xQueue_potentiometer, &potentiometer_val, 1000);

	//The period that the light is green should be proportional to the potentiometer value
	//The period that the light is yellow should be constant
	//The period that the light is red should be inversly proportional to the potentiometer value
	//If the potentiometer value is 0, the red light should be on twice as long as the green light
	//If the potentiometer value is 100, the green light should be on twice as long as the red light

	xTimerChangePeriod( xTimer, pdMS_TO_TICKS(ms_to_delay), 1000);

	if(traffic_light_is_green()) {
		//Set light to yellow
		GPIO_ResetBits(GPIOC, GREEN_LIGHT_PIN);
		GPIO_SetBits(GPIOC, YELLOW_LIGHT_PIN);

		//Set timer period to 3 seconds
		int ms_to_delay = 3000;

		xTimerChangePeriod( xTimer, pdMS_TO_TICKS(ms_to_delay), 1000);

		printf("Traffic Light Changed to Yellow\n");
	} else if(traffic_light_is_yellow()) {
		//Set light to red
		GPIO_ResetBits(GPIOC, YELLOW_LIGHT_PIN);
		GPIO_SetBits(GPIOC, RED_LIGHT_PIN);

		//Set timer period to be inverse proportional to potentiometer value
		//When potentiometer value is 0, timer period is 2 seconds
		//When potentiometer value is 100, timer period is 1 second
		int ms_to_delay = 2000 - potentiometer_val*10;


		printf("Traffic Light Changed to Red\n");
	} else if (traffic_light_is_red()) {
		//Set light to green
		GPIO_ResetBits(GPIOC, RED_LIGHT_PIN);
		GPIO_SetBits(GPIOC, GREEN_LIGHT_PIN);

		//Set timer period to be proportional to potentiometer value
		//When potentiometer value is 0, timer period is 1 second
		//When potentiometer value is 100, timer period is 2 seconds
		int ms_to_delay = 1000 + potentiometer_val*10;

		printf("Traffic Light Changed to Green\n");
	} else {
		printf("Traffic Light Error\n");
	}
}



/*-----------------------------------------------------------*/
/*
 * Update Traffic Display Task
 * Description: task that updates the traffic display based on the car array
 * Input: int traffic_pos[NUM_CAR_SPOTS] - array of car positions
 * Output: void
 */
#define NUM_CAR_SPOTS 19
#define STOP_POSITION 9	//Position where cars should stop if light turns yellow
#define YELLOW_LIGHT_POSITION 6	//Position where light turns yellow
void update_traffic_display(int traffic_pos[NUM_CAR_SPOTS])
{
	//Reset all the car spots
	GPIO_ResetBits(GPIOC, Reset_Pin);
	GPIO_SetBits(GPIOC, Reset_Pin);

	for(int i=NUM_CAR_SPOTS-1; i>=0; --i) {
		//If there is a car in the spot, set the data pin high
		if(traffic_pos[i]){
			GPIO_SetBits(GPIOC, Data_Pin);
		}
		GPIO_SetBits(GPIOC, Clock_Pin);
		GPIO_ResetBits(GPIOC, Data_Pin | Clock_Pin);
	}
}

/*-----------------------------------------------------------*/
/*
 * System Display Task
 * Description: task that updates the system display based on the traffic light state and the car queue.
 * Input: void *pvParameters - pointer to parameters
 * Output: void
 */
static void System_Display_Task( void *pvParameters )
{
	int new_car_val = 0;
	int traffic_pos[19] = {};

	for(;;)
		if(xQueueReceive(xQueue_car_generation, &new_car_val, 1000)) { // Update traffic

			// Start with traffic beyond stop line, always shift
			for(int i=NUM_CAR_SPOTS-1; i>=STOP_POSITION; --i) {
				traffic_pos[i] = traffic_pos[i-1];
			}

			// Traffic past yellow light position can go if light is yellow
			if(traffic_light_is_yellow()) {
				for (int i=STOP_POSITION-1; i>=YELLOW_LIGHT_POSITION; --i) {
					traffic_pos[i] = traffic_pos[i-1];
				}
			}

			// Traffic at stop line, shift when traffic light is green
			if(traffic_light_is_green()){
				for(int i=YELLOW_LIGHT_POSITION-1; i>=1; --i) {
					traffic_pos[i] = traffic_pos[i-1];
				}
				traffic_pos[0] = 0;
			} 
			//else { // shift only up to stop line
			//	for(int i=6; i>=0; i--){
			//		if(!traffic_pos[i+1]){ // can only shift forward if no traffic in front
			//			traffic_pos[i+1] = traffic_pos[i];
			//			traffic_pos[i] = 0;
			//		}
			//	}
			}//

			// Add new car value if no traffic jam and new car value is 1
			traffic_pos[0] = new_car_val && !traffic_pos[0];
		}
		update_traffic_display(traffic_pos);
	}
}


/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software 
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}

