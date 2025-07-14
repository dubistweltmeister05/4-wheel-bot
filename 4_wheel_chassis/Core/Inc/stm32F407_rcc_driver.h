/*
 * stm32F407_rcc.h
 *
 *  Created on: Dec 10, 2023
 *      Author: wardawg
 */

#ifndef STM32F407_RCC_H_
#define STM32F407_RCC_H_

#include "stm32F407xx.h"

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);

#endif /* STM32F407_RCC_H_ */
