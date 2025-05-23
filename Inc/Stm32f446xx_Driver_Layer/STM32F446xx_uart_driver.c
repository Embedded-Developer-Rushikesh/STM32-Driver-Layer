/********************************************************************************
  * @file    stm32f446xx_uart_driver.c
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
#include"STM32F446xx_uart_driver.h"
#include"STM32F446xx_rcc_driver.h"



void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event);


/* -----------------------------------------------------------------------------
* @brief  Enables or disables peripheral clock for the given UART
*
* @param[in] pUSARTx: UART peripheral base address (USART1-USART6)
* @param[in] EnOrDi: Enable or disable macro (ENABLE/DISABLE)
*
* @note  Controls clock for all supported UART peripherals (USART1-6)
*        Uses RCC registers to enable/disable clocks
*/
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE) {
		if(pUSARTx==USART1)
		{
			USART1_PCCK_EN();
		}else if(pUSARTx==USART2)
		{
			USART2_PCCK_EN();
		}else if(pUSARTx==USART3)
		{
			USART3_PCCK_EN();
		}else if(pUSARTx==UART4)
		{
			UART4_PCCK_EN();
		}else if(pUSARTx==UART5)
		{
			UART5_PCCK_EN();
		}else if(pUSARTx==USART6){
			 USART6_PCCK_EN();
		}else
		{
			;
		}
	}else
	{
		if (pUSARTx == USART1) {
			USART1_PCCK_DI();
		} else if (pUSARTx == USART2) {
			USART2_PCCK_DI();
		} else if (pUSARTx == USART3) {
			USART3_PCCK_DI();
		} else if (pUSARTx == UART4) {
			UART4_PCCK_DI();
		} else if (pUSARTx == UART5) {
			UART5_PCCK_DI();
		} else if (pUSARTx == USART6) {
			USART6_PCCK_DI();
		} else {
			;
		}
	}
}


/* -----------------------------------------------------------------------------
* @brief  Calculates and sets the baud rate for UART communication
*
* @param[in] pUSARTx: UART peripheral base address
* @param[in] BaudRate: Desired baud rate (e.g., 9600, 115200)
*
* @note  Handles both oversampling by 8 and 16 modes
*        Calculates mantissa and fraction values for BRR register
*        Uses APB bus clock frequencies for calculation
*/
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   //over sampling by 16
	   usartdiv = ((25 * PCLKx) / (4 *BaudRate));
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->BRR = tempreg;
}


/* -----------------------------------------------------------------------------
* @brief  Initializes UART peripheral with specified configuration
*
* @param[in] pUSARTHandle: Pointer to UART handle containing:
*            - pUSARTx: UART peripheral base address
*            - USART_Config: Configuration structure with:
*              * USART_Mode: TX, RX or both
*              * USART_Baud: Baud rate
*              * USART_WordLength: 8 or 9 bits
*              * USART_StopBits: Stop bits configuration
*              * USART_ParityControl: Parity enable/disable
*              * USART_HWFlowControl: Hardware flow control settings
*
* @note  Configures all UART parameters including:
*        - Mode (TX/RX/both)
*        - Word length
*        - Stop bits
*        - Parity
*        - Hardware flow control
*        - Baud rate (via USART_SetBaudRate)
*/
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	USART_PeriClockControl(pUSARTHandle->pUSARTx,ENABLE);
	switch(pUSARTHandle->USART_Config.USART_Mode)
	{
	case USART_MODE_ONLY_RX:
	{
		pUSARTHandle->pUSARTx->CR1|= (1 << USART_CR1_RE);
		break;
	}
	case USART_MODE_ONLY_TX:
	{
		pUSARTHandle->pUSARTx->CR1= (1 << USART_CR1_TE);
		break;
	}
	case USART_MODE_TXRX:
	{
		pUSARTHandle->pUSARTx->CR1|= ((1 << USART_CR1_RE)|(1 << USART_CR1_TE));
		break;
	}
	default:
	{
		break;
	}
	}
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_PCE);
		pUSARTHandle->pUSARTx->CR1 |= ( 0 << USART_CR1_PS);
	}
	else if(pUSARTHandle->USART_Config.USART_ParityControl ==USART_PARITY_EN_ODD)
	{
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_PCE);
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_PS);
	}else
	{
		;
	}
	 //Implement the code to configure the Word length configuration item
	pUSARTHandle->pUSARTx->CR1 |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	switch(pUSARTHandle->USART_Config.USART_NoOfStopBits)
	{
	case USART_STOPBITS_1:
	{
		pUSARTHandle->pUSARTx->CR2 |=0<< USART_CR2_STOP;
		break;
	}
	case USART_STOPBITS_0_5:
	{
		pUSARTHandle->pUSARTx->CR2 |=1<< USART_CR2_STOP;
		break;
	}
	case USART_STOPBITS_2:
	{
		pUSARTHandle->pUSARTx->CR2 |=2<< USART_CR2_STOP;
		break;
	}
	case USART_STOPBITS_1_5:
	{
		pUSARTHandle->pUSARTx->CR2 |=3<< USART_CR2_STOP;
		break;
	}
	default:
	{
		break;
	}
	}


	//Configuration of USART hardware flow control
	switch(pUSARTHandle->USART_Config.USART_HWFlowControl)
		{
		case USART_HW_FLOW_CTRL_CTS:
		{
			pUSARTHandle->pUSARTx->CR3 |= ( 1 << USART_CR3_CTSE);
			break;
		}
		case USART_HW_FLOW_CTRL_RTS:
		{
			pUSARTHandle->pUSARTx->CR3 |=( 1 << USART_CR3_RTSE);
			break;
		}
		case USART_HW_FLOW_CTRL_CTS_RTS:
		{
			pUSARTHandle->pUSARTx->CR3 |=( 1 << USART_CR3_CTSE)|( 1 << USART_CR3_RTSE);
			break;
		}
		default:
		{
			break;
		}
		}

	/******************************** Configuration of BRR(Baudrate register)******************************************/

		//Implement the code to configure the baud rate
		//We will cover this in the lecture. No action required here
	//	USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);
}


/* -----------------------------------------------------------------------------
* @brief  Enables or disables UART peripheral
*
* @param[in] pUSARTx: UART peripheral base address
* @param[in] Cmd: Enable or disable command (ENABLE/DISABLE)
*
* @note  Controls the UE (USART Enable) bit in CR1 register
*        Must be called after initialization for peripheral to operate
*/
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t Cmd)
{
	if(Cmd == ENABLE)
	{
		Set_Bit(pUSARTx->CR1,13);
	}else
	{
		Reset_Bit(pUSARTx->CR1,13);
	}

}



/* -----------------------------------------------------------------------------
* @brief  Checks the status of a specific UART flag
*
* @param[in] pUSARTx: UART peripheral base address
* @param[in] StatusFlagName: Flag to check (e.g., USART_FLAG_TXE)
*
* @return uint8_t: Status of flag (SET or RESET)
*
* @note  Reads the status register (SR) and checks specified flag
*        Common flags: TXE, TC, RXNE, ORE, etc.
*/
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
    if(pUSARTx->SR & StatusFlagName)
    {
    	return SET;
    }

   return RESET;
}


/* -----------------------------------------------------------------------------
* @brief  Transmits data over UART
*
* @param[in] pUSARTHandle: Pointer to UART handle
* @param[in] pTxBuffer: Pointer to transmit data buffer
* @param[in] Len: Length of data to transmit
*
* @note  Handles both 8-bit and 9-bit data formats
*        Waits for TXE flag before sending each byte
*        Waits for TC flag at end of transmission
*        Supports parity mode configurations
*/
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{uint16_t *pdata;

//Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

		//Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer , so 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));

}



/* -----------------------------------------------------------------------------
* @brief  Receives data from UART
*
* @param[in] pUSARTHandle: Pointer to UART handle
* @param[in] pRxBuffer: Pointer to receive data buffer
* @param[in] Len: Length of data to receive
*
* @note  Handles both 8-bit and 9-bit data formats
*        Waits for RXNE flag before reading each byte
*        Supports parity mode configurations
*        Adjusts buffer pointer based on word length
*/
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	//Loop over until "Len" number of bytes are transferred
		for(uint32_t i = 0 ; i < Len; i++)
		{
			//Implement the code to wait until RXNE flag is set in the SR
			while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

			//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
			if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
			{
				//We are going to receive 9bit data in a frame

				//Now, check are we using USART_ParityControl control or not
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used , so all 9bits will be of user data

					//read only first 9 bits so mask the DR with 0x01FF
					*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

					//Now increment the pRxBuffer two times
					pRxBuffer++;
					pRxBuffer++;
				}
				else
				{
					//Parity is used, so 8bits will be of user data and 1 bit is parity
					 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					 pRxBuffer++;
				}
			}
			else
			{
				//We are going to receive 8bit data in a frame

				//Now, check are we using USART_ParityControl control or not
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//No parity is used , so all 8bits will be of user data

					//read 8 bits from DR
					 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				}

				else
				{
					//Parity is used, so , 7 bits will be of user data and 1 bit is parity

					//read only 7 bits , hence mask the DR with 0X7F
					 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

				}

				//Now , increment the pRxBuffer
				pRxBuffer++;
			}
		}
}

/* -----------------------------------------------------------------------------
* @brief  Configures UART interrupts in NVIC
*
* @param[in] IRQNumber: UART interrupt number
* @param[in] EnorDi: Enable or disable interrupt (ENABLE/DISABLE)
*
* @note  Handles interrupt configuration for all NVIC groups
*        Supports IRQ numbers 0-95
*        Uses ISER/ICER registers for enable/disable
*/
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				//program ISER0 register
				Set_Bit(*NVIC_ISER0,IRQNumber);
			}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
			{
				//program ISER1 register
				Set_Bit(*NVIC_ISER1,(IRQNumber % 32));
			}
			else if(IRQNumber >= 64 && IRQNumber < 96 )
			{
				//program ISER2 register //64 to 95
				Set_Bit(*NVIC_ISER3,(IRQNumber % 64));
			}
		}else
		{
			if(IRQNumber <= 31)
			{
				//program ICER0 register
				Set_Bit(*NVIC_ICER0,IRQNumber);
			}else if(IRQNumber > 31 && IRQNumber < 64 )
			{
				//program ICER1 register
				Set_Bit(*NVIC_ICER1,(IRQNumber % 32));
			}
			else if(IRQNumber >= 6 && IRQNumber < 96 )
			{
				//program ICER2 register
				Set_Bit(*NVIC_ICER3,(IRQNumber % 64));
			}
		}

}



/* -----------------------------------------------------------------------------
* @brief  Sets priority for UART interrupts
*
* @param[in] IRQNumber: UART interrupt number
* @param[in] IRQPriority: Interrupt priority value
*
* @note  Calculates correct NVIC priority register and bit position
*        Handles STM32's priority bits implementation
*        Supports all NVIC priority groups
*/
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}


/* -----------------------------------------------------------------------------
* @brief  Handles UART interrupts and events
*
* @param[in] pUSARTHandle: Pointer to UART handle
*
* @note  Processes all UART interrupt sources:
*        - Transmission Complete (TC)
*        - Transmit Data Register Empty (TXE)
*        - Receive Data Ready (RXNE)
*        - Clear To Send (CTS)
*        - Idle Line Detection (IDLE)
*        - Overrun Error (ORE)
*        - Framing Error (FE)
*        - Noise Error (NE)
*        Calls application callback for each event
*/
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1 , temp2;
	uint16_t *pdata;
	/*************************Check for TC flag ********************************************/
	 //Implement the code to check the state of TC bit in the SR
		temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);
		 //Implement the code to check the state of TCEIE bit
		temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);
		if(temp1 && temp2 )
		{
			//this interrupt is because of TC
			//close transmission and call application callback if TxLen is zero
			if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
			{
			//Implement the code to clear the TC flag
			pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);

			//Implement the code to clear the TCIE control bit

			//Reset the application state
			pUSARTHandle->TxBusyState = USART_READY;

			//Reset Buffer address to NULL
			pUSARTHandle->pTxBuffer = NULL;

			//Reset the length to zero
			pUSARTHandle->TxLen = 0;

			//Call the application call back with event USART_EVENT_TX_CMPLT
			USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);

			}
		}
	/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);

	if (temp1 && temp2) {
		//this interrupt is because of TXE

		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX) {
			//Keep sending data until Txlen reaches to zero
			if (pUSARTHandle->TxLen > 0) {
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if (pUSARTHandle->USART_Config.USART_WordLength
						== USART_WORDLEN_9BITS) {
					//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t) 0x01FF);

					//check for USART_ParityControl
					if (pUSARTHandle->USART_Config.USART_ParityControl== USART_PARITY_DISABLE) {
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= 2;
					} else {
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= 1;
					}
				} else {
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer& (uint8_t) 0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen -= 1;
				}

			}
			if (pUSARTHandle->TxLen == 0) {
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

	/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if (temp1 && temp2) {
		//this interrupt is because of rxne
		if (pUSARTHandle->RxBusyState == USART_BUSY_IN_RX) {
			if (pUSARTHandle->RxLen > 0) {
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if (pUSARTHandle->USART_Config.USART_WordLength== USART_WORDLEN_9BITS) {
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if (pUSARTHandle->USART_Config.USART_ParityControl== USART_PARITY_DISABLE) {
						//No parity is used , so all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) =(pUSARTHandle->pUSARTx->DR & (uint16_t) 0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 2;
					} else {
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR& (uint8_t) 0xFF);
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 1;
					}
				} else {
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if (pUSARTHandle->USART_Config.USART_ParityControl
							== USART_PARITY_DISABLE) {
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						*pUSARTHandle->pRxBuffer =(uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);

					}

					else {
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						*pUSARTHandle->pRxBuffer =(uint8_t) (pUSARTHandle->pUSARTx->DR& (uint8_t) 0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen -= 1;
				}

			}				//if of >0

			if (!pUSARTHandle->RxLen) {
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}
	/*************************Check for CTS flag ********************************************/
	//Note : CTS feature is not applicable for UART4 and UART5

		//Implement the code to check the status of CTS bit in the SR
		temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS);

		//Implement the code to check the state of CTSE bit in CR1
		temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);



		if(temp1  && temp2 )
		{
			//Implement the code to clear the CTS flag in SR
			pUSARTHandle->pUSARTx->SR &=  ~( 1 << USART_SR_CTS);

			//this interrupt is because of cts
			USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
		}

	/*************************Check for IDLE detection flag ********************************************/

		//Implement the code to check the status of IDLE flag bit in the SR
		temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE);

		//Implement the code to check the state of IDLEIE bit in CR1
		temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);


		if(temp1 && temp2)
		{
			//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
			temp1 = pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_IDLE);

			//this interrupt is because of idle
			USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
		}

	/*************************Check for Overrun detection flag ********************************************/

		//Implement the code to check the status of ORE flag  in the SR
		temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

		//Implement the code to check the status of RXNEIE  bit in the CR1
		temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


		if(temp1  && temp2 )
		{
			//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

			//this interrupt is because of Overrun error
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}



	/*************************Check for Error Flag ********************************************/

	//Noise Flag, Overrun error and Framing Error in multibuffer communication
	//We dont discuss multibuffer communication in this course. please refer to the RM
	//The blow code will get executed in only if multibuffer mode is used.

		temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

		if(temp2 )
		{
			temp1 = pUSARTHandle->pUSARTx->SR;
			if(temp1 & ( 1 << USART_SR_FE))
			{
				/*
					This bit is set by hardware when a de-synchronization, excessive noise or a break character
					is detected. It is cleared by a software sequence (an read to the USART_SR register
					followed by a read to the USART_DR register).
				*/
				USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
			}

			if(temp1 & ( 1 << USART_SR_NE) )
			{
				/*
					This bit is set by hardware when noise is detected on a received frame. It is cleared by a
					software sequence (an read to the USART_SR register followed by a read to the
					USART_DR register).
				*/
				USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
			}

			if(temp1 & ( 1 << USART_SR_ORE) )
			{
				USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
			}
		}

}


/* -----------------------------------------------------------------------------
* @brief  Default application callback for UART events (weak implementation)
*
* @param[in] pUSARTHandle: Pointer to UART handle structure
* @param[in] event: UART event that occurred
*
* @note  This weak implementation can be overridden by user application
*        Blinks GPIO pin PA5 when called as example implementation
*/
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
{
		GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, ENABLE);
		DelayMS(1000);
		GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, DISABLE);
}
