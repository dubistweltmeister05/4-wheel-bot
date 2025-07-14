/*
 * self_msp.c
 *
 *  Created on: Jan 4, 2024
 *      Author: wardawg
 */



#include "main.h"
extern TIM_HandleTypeDef htimer4;
void HAL_MspInit(void) {
	//Here,we do the Low Level Processor Initializations.

	//1. Set up the priority grouping of Processor
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	//2. Enable the required exceptions (system) of the Arm Cortex Mx Processor
	SCB->SHCSR |= 0x7 << 16; //For setting the usage fault, bus fault and memory fault system exceptions

	//3. Configure the priority for the system initializations.
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim) {

	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	//1. Configuring the GPIOs in their alternate functions
	GPIO_InitTypeDef timer4_gpio;
	timer4_gpio.Mode = GPIO_MODE_AF_PP;
	timer4_gpio.Alternate = GPIO_AF2_TIM4;
	timer4_gpio.Pull = GPIO_NOPULL;
	timer4_gpio.Speed = GPIO_SPEED_FAST;
	timer4_gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;

	HAL_GPIO_Init(GPIOD,&timer4_gpio);

	 HAL_NVIC_SetPriority(TIM4_IRQn,15,0);
	 HAL_NVIC_EnableIRQ(TIM4_IRQn);

}
