/*
 * stm32f446xx_systeck_driver.h
 *
 *  Created on: Oct 13, 2024
 *      Author: DELL
 */

#ifndef STM32F446XX_DRIVER_LAYER_STM32F446XX_SYSTICK_DRIVER_H_
#define STM32F446XX_DRIVER_LAYER_STM32F446XX_SYSTICK_DRIVER_H_
#include"stm32f446xx.h"
#define SYSTICK_LOAD_VAL			16000
#define	CTRL_ENABLE					(1U<<0)
#define CTRL_CLKSRC					(1U<<2)
#define CTRL_COUNTFLAG				(1U<<16)
#define CTRL_TICKINT				(1U<<1)

#define ONE_SEC_LOAD				16000000

void systickDelayMs(int delay);
void systick_1hz_interrupt(void);

#endif /* STM32F446XX_DRIVER_LAYER_STM32F446XX_SYSTICK_DRIVER_H_ */
