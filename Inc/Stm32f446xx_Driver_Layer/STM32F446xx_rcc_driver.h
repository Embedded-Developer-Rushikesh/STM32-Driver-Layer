/**
 ******************************************************************************
 * @file    STM32F446xx_rcc_driver.h
 * @author  Embedded Galaxy
 * @version V1.0
 * @date    May 12,2025
 * @brief   This file provides function prototypes for managing the Reset and
 *          Clock Control (RCC) peripheral of the STM32F446xx microcontroller.
 *          It includes functions for retrieving clock values for APB1, APB2,
 *          and PLL output clocks.
 *
 * @note    The RCC peripheral is used to configure and manage clock sources
 *          for the entire microcontroller, including APB and AHB bus clocks,
 *          system clock, and PLLs. This driver provides essential functions
 *          to get the current clock frequencies for the system's busses and PLL.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef STM32F446XX_DRIVER_LAYER_STM32F446XX_RCC_DRIVER_H_
#define STM32F446XX_DRIVER_LAYER_STM32F446XX_RCC_DRIVER_H_

#include "stm32f446xx.h"

/**
 * @brief  This function returns the current value of the APB1 clock (PCLK1) in Hz.
 * @return The current APB1 clock value.
 */
uint32_t RCC_GetPCLK1Value(void);

/**
 * @brief  This function returns the current value of the APB2 clock (PCLK2) in Hz.
 * @return The current APB2 clock value.
 */
uint32_t RCC_GetPCLK2Value(void);

/**
 * @brief  This function returns the current PLL output clock value in Hz.
 * @return The current PLL output clock value.
 */
uint32_t RCC_GetPLLOutputClock(void);

#endif /* STM32F446XX_DRIVER_LAYER_STM32F446XX_RCC_DRIVER_H_ */
