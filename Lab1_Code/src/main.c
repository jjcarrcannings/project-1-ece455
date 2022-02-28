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
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.

This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.

This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.

Here is a description of the project's functionality:

The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.

The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html

The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html

An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html

The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.

The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().

The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
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

#define amber  	0
#define green  	1
#define red  	2
#define blue  	3

#define amber_led	LED3
#define green_led	LED4
#define red_led		LED5
#define blue_led	LED6

#define Reset_Pin	GPIO_Pin_8
#define Data_Pin	GPIO_Pin_6
#define Clock_Pin	GPIO_Pin_7

#undef configUSE_TICK_HOOK
#define configUSE_TICK_HOOK 1


/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );

/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */
static void Potentiometer_Read_Task( void *pvParameters );
static void Traffic_Generator_Task( void *pvParameters );
static void Traffic_Light_State_Task( TimerHandle_t );
static void System_Display_Task( void *pvParameters );

xQueueHandle xQueue_potentiometer = 0;
xQueueHandle xQueue_traffic_light = 0;
xQueueHandle xQueue_car_generation = 0;


/*-----------------------------------------------------------*/

void init(void){
	//Step 1: Initialize GPIO

	//Enable Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);


	GPIO_InitTypeDef gpio_traffic_light;

	gpio_traffic_light.GPIO_Mode = GPIO_Mode_OUT;
	gpio_traffic_light.GPIO_OType = GPIO_OType_PP;
	gpio_traffic_light.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1
			| GPIO_Pin_2 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;

	gpio_traffic_light.GPIO_PuPd = GPIO_PuPd_DOWN;
	gpio_traffic_light.GPIO_Speed = GPIO_Speed_100MHz;

	//Set Gpio bits
	GPIO_Init(GPIOC, &gpio_traffic_light);
	GPIO_SetBits(GPIOC, GPIO_Pin_2);


	GPIO_InitTypeDef gpio_adc;

	gpio_adc.GPIO_Mode = GPIO_Mode_AN;
	//gpio_adc.GPIO_OType = GPIO_OType_PP;
	gpio_adc.GPIO_Pin = GPIO_Pin_3;
	gpio_adc.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio_adc.GPIO_Speed = GPIO_Speed_100MHz;


	GPIO_Init(GPIOC, &gpio_adc);

	ADC_InitTypeDef adc_itd;

	adc_itd.ADC_ContinuousConvMode = 0;
	adc_itd.ADC_DataAlign = ADC_DataAlign_Right;
	adc_itd.ADC_ExternalTrigConv = DISABLE;
	//adc_itd.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;
	//adc_itd.ADC_NbrOfConversion = 3;
	adc_itd.ADC_Resolution = ADC_Resolution_10b;
	//adc_itd.ADC_ScanConvMode = 0;

	//Enable Clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_Init(ADC1, &adc_itd);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);

	time_t t;
	srand((unsigned) time(&t));
}


int main(void)
{

	init();

	/* Initialize LEDs */
	STM_EVAL_LEDInit(amber_led);
	STM_EVAL_LEDInit(green_led);
	STM_EVAL_LEDInit(red_led);
	STM_EVAL_LEDInit(blue_led);

	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();

	ADC_Cmd (ADC1, ENABLE);
	ADC_SoftwareStartConv(ADC1);

	/*Initialize timer*/
	void * const pvTimerID = 0;
	TimerHandle_t timer_handle = xTimerCreate("Traffic Light Timer", 1000, pdTRUE, pvTimerID, Traffic_Light_State_Task);
	if( timer_handle == NULL) {
		fprintf(stderr, "Timer not created\n");
		return 1;
	}

	/* Create the queue used by the queue send and queue receive tasks.
	http://www.freertos.org/a00116.html */
	xQueue_potentiometer = xQueueCreate( 	mainQUEUE_LENGTH,		/* The number of items the queue can hold. */
							sizeof( uint16_t ) );	/* The size of each item the queue holds. */

	xQueue_car_generation = xQueueCreate( 	mainQUEUE_LENGTH,		/* The number of items the queue can hold. */
							sizeof( uint16_t ) );	/* The size of each item the queue holds. */
	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( xQueue_potentiometer, "PotentiometerQueue" );
	vQueueAddToRegistry( xQueue_car_generation, "CarGenerationQueue" );

	xTaskCreate( Potentiometer_Read_Task, "Potentiometer_Read", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Traffic_Generator_Task, "Traffic_Generator", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	//xTaskCreate( Traffic_Light_State_Task, "Traffic_Light_State", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( System_Display_Task, "System_Display", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	/* Start the timer.  No block time is specified, and
	 even if one was it would be ignored because the RTOS
	 scheduler has not yet been started. */
	 if( xTimerStart( timer_handle, 0 ) != pdPASS )
	 {
		 printf("The timer could not be set into the Active state.\n");
		 return 2;
	 }

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
}

/*-----------------------------------------------------------*/

static void Potentiometer_Read_Task( void *pvParameters )
{
	uint16_t adc_val;
	uint16_t old_adc_val;
	for(;;) {
		ADC_SoftwareStartConv(ADC1);

		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

		adc_val = ADC_GetConversionValue(ADC1) / 10;

		//Pop old value of the queue
		xQueueReceive(xQueue_potentiometer, &old_adc_val, 10);
		//Send new adc value into the queue
		xQueueSend(xQueue_potentiometer,&adc_val,1000);

		vTaskDelay(pdMS_TO_TICKS(500));
	}
}


/*-----------------------------------------------------------*/

static void  Traffic_Generator_Task( void *pvParameters )
{
	uint16_t potentiometer_val = 0;
	while(1)
	{
		xQueuePeek(xQueue_potentiometer, &potentiometer_val, 1000);
		potentiometer_val += 20;
		int rand_val = (rand()*100.0) / RAND_MAX;
		printf("rand: %d --- adc: %u \n", rand_val, potentiometer_val);
		int new_car_val = 0;
		if(rand_val <= potentiometer_val) { //Create new car
			//Send car to new queue
			new_car_val = 1;
			xQueueSend(xQueue_car_generation, &new_car_val, 1000);
			printf("Sending car\n");
		} else{
			xQueueSend(xQueue_car_generation, &new_car_val, 1000);
		}
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

/*-----------------------------------------------------------*/

void Traffic_Light_State_Task( TimerHandle_t xTimer ) {
	uint16_t potentiometer_val = 0;

	xQueuePeek(xQueue_potentiometer, &potentiometer_val, 1000);

	if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0)) { // traffic light is red
		 //Set light to green
		 GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2);
		 GPIO_SetBits(GPIOC, GPIO_Pin_2);

		 xTimerChangePeriod( xTimer, pdMS_TO_TICKS(15*(potentiometer_val+1)), 1000);

		 //printf("Green\n");
	}else if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1)) { // traffic light is yellow
		 //Set light to red
		 GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2);
		 GPIO_SetBits(GPIOC, GPIO_Pin_0);

		 //Change timer period
		 //xTimerChangePeriod( xTimer, pdMS_TO_TICKS(8000/(potentiometer_val+1)), 1000);
		 xTimerChangePeriod( xTimer, pdMS_TO_TICKS(100000), 1000);

		 //printf("Red\n");
	}else if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)){ // traffic light is green
		 //Set light to yellow
		 GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2);
		 GPIO_SetBits(GPIOC, GPIO_Pin_1);

		 // Set to constant yellow light period
		 xTimerChangePeriod( xTimer, pdMS_TO_TICKS(1000), 1000);

		 //printf("Yellow\n");
	} else {
		printf("No light state (error)\n");
	}
}



/*-----------------------------------------------------------*/

void update_traffic_display(int traffic_pos[19])
{

	GPIO_ResetBits(GPIOC, Reset_Pin);
	GPIO_SetBits(GPIOC, Reset_Pin);

	for(int i=18; i>=0; i--)
	{
		if(traffic_pos[i]){
			GPIO_SetBits(GPIOC, Data_Pin);
		}
		GPIO_SetBits(GPIOC, Clock_Pin);
		GPIO_ResetBits(GPIOC, Data_Pin | Clock_Pin);
	}

}

int traffic_light_is_green()
{
	return GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
}


static void System_Display_Task( void *pvParameters )
{
	int new_car_val = 0;
	int traffic_pos[19] = {};
	while(1)
	{
		if(xQueueReceive(xQueue_car_generation, &new_car_val, 1000)){ // Update traffic

			// Start with traffic beyond stop line, always shift
			for(int i=18; i>=9; i--){
				traffic_pos[i] = traffic_pos[i-1];
			}
			// Traffic at stop line, shift when traffic light is green
			if(traffic_light_is_green()){
				for(int i=8; i>=1; i--){
					traffic_pos[i] = traffic_pos[i-1];
				}
				traffic_pos[0] = 0;
			} else{ // shift only up to stop line
				for(int i=6; i>=0; i--){
					if(!traffic_pos[i+1]){ // can only shift forward if no traffic in front
						traffic_pos[i+1] = traffic_pos[i];
						traffic_pos[i] = 0;
					}
				}
			}

			// Add new car value if no traffic jam
			if(!traffic_pos[0]){
				traffic_pos[0] = new_car_val;
			}
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

