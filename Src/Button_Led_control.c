/*
 * Button_Led_control.c
 *
 *  Created on: Oct 5, 2024
 *      Author: DELL
 */
#include"stm32f446xx.h"
#include<string.h>
void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}
int main()
{
	//LED PA5
	GPIO_Handle_t GpioLed;
	memset(&GpioLed,0,sizeof(GpioLed));
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GpioLed);


	//Button PC13
	GPIO_Handle_t GpioButton;
	memset(&GpioButton,0,sizeof(GpioButton));
	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;
	GPIO_Init(&GpioButton);


	while(1)
	{
		if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == 1) {
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, ENABLE);
			delay();
		} else {
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, DISABLE);
			delay();
		}
	}

}

