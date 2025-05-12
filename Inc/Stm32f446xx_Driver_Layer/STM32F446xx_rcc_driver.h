/*
 * STM32F446xx_rcc_driver.h
 *
 *  Created on: Oct 12, 2024
 *      Author: DELL
 */

#ifndef STM32F446XX_DRIVER_LAYER_STM32F446XX_RCC_DRIVER_H_
#define STM32F446XX_DRIVER_LAYER_STM32F446XX_RCC_DRIVER_H_

#include"stm32f446xx.h"

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);

#endif /* STM32F446XX_DRIVER_LAYER_STM32F446XX_RCC_DRIVER_H_ */
