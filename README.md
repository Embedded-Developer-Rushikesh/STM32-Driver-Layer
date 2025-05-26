/**
  ******************************************************************************
  * @file    stm32f446xx_gpio_driver.h
  * @author  Embedded Galaxy
  * @version V1.0
  * @date    May 5,2025
  * @brief   GPIO driver implementation for STM32F446xx microcontroller.
  *          This file provides firmware functions to manage:
  *           - GPIO initialization and configuration
  *           - GPIO read/write operations
  *           - GPIO interrupt handling
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
#ifndef STM32F446XX_DRIVER_LAYER_STM32F446XX_GPIO_DRIVER_H_
#define STM32F446XX_DRIVER_LAYER_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/**************************************
 *        GPIO Pin Configuration      *
 **************************************/
typedef struct
{
	uint8_t GPIO_PinNumber;        /*!< GPIO pin number (0 to 15) */
	uint8_t GPIO_PinMode;          /*!< GPIO pin mode. Refer @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;         /*!< GPIO pin speed. Refer @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPdControl;   /*!< GPIO pull-up/pull-down configuration */
	uint8_t GPIO_PinOPType;        /*!< GPIO output type: push-pull or open-drain */
	uint8_t GPIO_PinAltFunMode;    /*!< GPIO alternate function mode (0 to 15) */
} GPIO_PinConfig_t;

/**************************************
 *         GPIO Handle Structure      *
 **************************************/
typedef struct
{
	GPIO_RegDef_t *pGPIOx;               /*!< Pointer to GPIO port base address */
	GPIO_PinConfig_t GPIO_PinConfig;    /*!< GPIO pin configuration settings */
} GPIO_Handle_t;

/**************************************
 *       GPIO Pin Number Macros       *
 **************************************/
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15

/**************************************
 *         GPIO Mode Macros           *
 **************************************/
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT     4  /*!< Interrupt falling edge trigger */
#define GPIO_MODE_IT_RT     5  /*!< Interrupt rising edge trigger */
#define GPIO_MODE_IT_RFT    6  /*!< Interrupt rising/falling edge trigger */

/**************************************
 *        GPIO Output Type Macros     *
 **************************************/
#define GPIO_OP_TYPE_PP     0  /*!< Push-pull */
#define GPIO_OP_TYPE_OD     1  /*!< Open-drain */

/**************************************
 *         GPIO Speed Macros          *
 **************************************/
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPOI_SPEED_HIGH		3

/**************************************
 *   GPIO Pull-Up/Pull-Down Macros    *
 **************************************/
#define GPIO_NO_PUPD   		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/**************************************
 *       GPIO Driver API Prototypes   *
 **************************************/

/**
 * @brief Enables or disables peripheral clock for given GPIO port
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/**
 * @brief Initializes the GPIO pin with specified configurations
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);

/**
 * @brief Deinitializes the GPIO port (resets it)
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/**
 * @brief Reads the state of a specific GPIO input pin
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/**
 * @brief Reads the state of the entire GPIO input port
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/**
 * @brief Writes a value to a specific GPIO output pin
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);

/**
 * @brief Writes a value to the entire GPIO output port
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);

/**
 * @brief Toggles the state of a GPIO output pin
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/**
 * @brief Configures interrupt settings for a GPIO pin
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief Sets the priority of a given GPIO interrupt
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @brief Handles the GPIO interrupt for a given pin number
 */
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* STM32F446XX_DRIVER_LAYER_STM32F446XX_GPIO_DRIVER_H_ */
