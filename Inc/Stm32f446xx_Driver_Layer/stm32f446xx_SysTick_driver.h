/********************************************************************************
  * @file    stm32f446xx_SysTick_driver.c
  * @author  Embedded Galaxy
  * @version V1.0
  * @date    May 5,2025
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
#ifndef STM32F446XX_DRIVER_LAYER_STM32F446XX_SYSTICK_DRIVER_H_
#define STM32F446XX_DRIVER_LAYER_STM32F446XX_SYSTICK_DRIVER_H_

#include "stm32f446xx.h"

/*
 * SysTick Load Value for 1ms Delay.
 * This value is used for configuring the SysTick timer to generate a 1ms interrupt
 * based on the system clock (assuming 16 MHz system clock).
 */
#define SYSTICK_LOAD_VAL            16000 /*!< Load value for generating a 1ms delay */

/*
 * Control Register Bit Definitions
 * These define various bits within the SysTick control register (SysTick_CTRL).
 */
#define CTRL_ENABLE                 (1U << 0)  /*!< Enables the SysTick counter */
#define CTRL_CLKSRC                 (1U << 2)  /*!< Selects the SysTick clock source (1 for system clock, 0 for external clock) */
#define CTRL_COUNTFLAG              (1U << 16) /*!< Indicates if the SysTick counter has counted to zero */
#define CTRL_TICKINT                (1U << 1)  /*!< Enables the SysTick interrupt */

/*
 * One Second Load Value
 * This defines the value required to configure the SysTick timer for a 1Hz interrupt,
 * which is a 1-second delay, based on a 16 MHz system clock.
 */
#define ONE_SEC_LOAD                16000000 /*!< Load value for generating a 1 second delay (1Hz) */

/**
 * @brief Generates a delay in milliseconds using SysTick.
 * This function uses the SysTick timer to generate a delay in milliseconds.
 * The SysTick timer is configured for 1ms ticks.
 *
 * @param delay The delay in milliseconds.
 */
void DelayMS(int delay);

/**
 * @brief Configures SysTick for a 1Hz interrupt (1-second tick).
 * This function sets up the SysTick timer to generate an interrupt at 1Hz,
 * which means it will generate an interrupt every second.
 *
 * The interrupt can be used to trigger a periodic event (such as toggling an LED every second).
 */
void systick_1hz_interrupt(void);

#endif /* STM32F446XX_DRIVER_LAYER_STM32F446XX_SYSTICK_DRIVER_H_ */
