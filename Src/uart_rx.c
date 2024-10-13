/*
 * uart_rx.c
 *
 *  Created on: Oct 13, 2024
 *      Author: DELL
 */


#include <stdint.h>
#include"stm32f446xx.h"
#include"string.h"
char msg[1024] = "Uart Driver test...\n\r";
#define SYS_FREQ		16000000
#define APB1_CLK		SYS_FREQ
#define UART_BAUDRATE		115200

char rx_buf[3] ;
USART_Handle_t usart2_handle;
//baud Rate
static void uart_set_baudrate(USART_RegDef_t *pUSARTx, uint32_t PeriphClk,  uint32_t BaudRate);
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);
void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;
	//usart2_handle.USART_Config.USART_Baud =USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}
void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode =7;

	//USART2 TX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpios);

	//USART2 RX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpios);


}
void LED1_Init(void)
{
	GPIO_Handle_t GpioLed;
	memset(&GpioLed,0,sizeof(GpioLed));
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_Init(&GpioLed);
}
int main(void)
{
	USART2_GPIOInit();
	/*
	 * Led PA5
	 */
	LED1_Init();


    USART2_Init();
	uart_set_baudrate(USART2,APB1_CLK,UART_BAUDRATE);
    USART_PeripheralControl(USART2,ENABLE);

    while(1)
    {

		USART_ReceiveData(&usart2_handle,(uint8_t *)rx_buf,sizeof(rx_buf));
		if(rx_buf[0]=='1')
		{
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, ENABLE);
			systickDelayMs(1000);
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, DISABLE);
		}
		else{
			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, DISABLE);
			systickDelayMs(1000);
		}

    }
}
 void uart_set_baudrate(USART_RegDef_t *pUSARTx, uint32_t PeriphClk,  uint32_t BaudRate)
{
	pUSARTx->BRR =  compute_uart_bd(PeriphClk,BaudRate);
}

 uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate)
{
	return ((PeriphClk + (BaudRate/2U))/BaudRate);
}
