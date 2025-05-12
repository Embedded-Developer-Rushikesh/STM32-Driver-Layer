/*
 * Led_Toggle.c
 *
 *  Created on: Oct 4, 2024
 *      Author: DELL
 */


#include"stm32f446xx.h"
#include<string.h>

int main()
{
	/*
	 * Led PA5
	 */

	GPIO_Handle_t GpioLed;
	//memset(&GpioLed,0,sizeof(GpioLed));
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioLed);
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		systickDelayMs(5000);
	}
}
