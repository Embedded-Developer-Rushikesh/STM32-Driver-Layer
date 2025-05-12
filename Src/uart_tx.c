/*
 * uart_tx_polling_mode.c
 *
 *  Created on: Oct 13, 2024
 *      Author: DELL
 */


/*
 * Transmission_uart.c
 *
 *  Created on: Oct 12, 2024
 *      Author: DELL
 */
#include <stdint.h>
#include"stm32f446xx.h"
#include"string.h"
char msg[1024] = "Uart Driver test...\n\r";

#define SYS_FREQ		16000000
#define APB1_CLK		SYS_FREQ
#define UART_BAUDRATE		115200

USART_Handle_t usart2_handle;
//baud Rate
static void uart_set_baudrate(USART_RegDef_t *pUSARTx, uint32_t PeriphClk,  uint32_t BaudRate);
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);
void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud =USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}
void 	USART2_GPIOInit(void)
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

int main(void)
{
	USART2_GPIOInit();
    USART2_Init();
	uart_set_baudrate(USART2,APB1_CLK,UART_BAUDRATE);
    USART_PeripheralControl(USART2,ENABLE);

    while(1)
    {
    	systickDelayMs(1000);

    	USART_SendData(&usart2_handle,(uint8_t*)msg,strlen(msg));
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
