#include "stm32f446xx_spi_driver.h"

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
/* -----------------------------------------------------------------------------
* @brief  Enables or disables peripheral clock for the given SPI
*
* @param[in] pSPIx: SPI peripheral base address (SPI1-SPI3)
* @param[in] EnOrDi: Enable or disable macro (ENABLE/DISABLE)
*
* @note  Controls clock for all supported SPI peripherals (SPI1-SPI3)
*        Uses RCC registers to enable/disable clocks
*/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}
	}
}


/* -----------------------------------------------------------------------------
* @brief  Initializes the SPI peripheral with given configuration
*
* @param[in] pSPIHandle: Pointer to SPI handle structure containing:
*            - pSPIx: SPI peripheral base address
*            - SPIConfig: SPI configuration structure
*
* @note  Configures SPI mode, bus config, clock speed, DFF, CPOL/CPHA, SSM
*        Enables peripheral clock before configuration
*/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	//peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first lets configure the SPI_CR1 register

	uint32_t tempreg = 0;
	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR ;

	//2. Configure the bus configuration
	switch(pSPIHandle->SPIConfig.SPI_BusConfig)
	{
		case SPI_BUS_CONFIG_FD:
		{
			tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
			break;
		}
		case SPI_BUS_CONFIG_HD:
		{
			tempreg |= ( 1 << SPI_CR1_BIDIMODE);
			break;
		}
		case SPI_BUS_CONFIG_SIMPLEX_RXONLY:
		{
			tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
			tempreg |= ( 1 << SPI_CR1_RXONLY);
			break;
		}
		default:
		{
			break;
		}
	}
	// 3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.  Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6 . configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;

}

/* -----------------------------------------------------------------------------
* @brief  Resets the SPI peripheral registers to default values
*
* @param[in] pSPIx: SPI peripheral base address (SPI1-SPI3)
*
* @note  Uses RCC reset registers to perform peripheral reset
*        All configuration will be lost after this operation
*/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx==SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx==SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx==SPI3)
	{
		SPI3_REG_RESET();
	}else
	{
		;
	}
}


/* -----------------------------------------------------------------------------
* @brief  Gets the status of specified SPI flag
*
* @param[in] pSPIx: SPI peripheral base address
* @param[in] FlagName: Flag to check (e.g. SPI_TXE_FLAG, SPI_RXNE_FLAG)
*
* @return Flag status (SET or RESET)
*
* @note  Checks status register (SR) for the specified flag
*/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/* -----------------------------------------------------------------------------
* @brief  Transmits data in blocking (polling) mode
*
* @param[in] pSPIx: SPI peripheral base address
* @param[in] pTxBuffer: Pointer to transmit data buffer
* @param[in] Len: Length of data to transmit (in bytes for 8-bit, words for 16-bit)
*
* @note  Waits for TXE flag before each transmission
*        Handles both 8-bit and 16-bit data frames
*        Blocks until all data is transmitted
*/
void SPI_SendData(SPI_RegDef_t *pSPIx,const uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)  == FLAG_RESET );

		//2. check the DFF bit in CR1
		if(isBit_Set(pSPIx->CR1,SPI_CR1_DFF))
		{
			//16 bit DFF
			//1. load the data in to the DR
			pSPIx->DR =   *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}

}

/* -----------------------------------------------------------------------------
* @brief  Receives data in blocking (polling) mode
*
* @param[in] pSPIx: SPI peripheral base address
* @param[in] pRxBuffer: Pointer to receive data buffer
* @param[in] Len: Length of data to receive (in bytes for 8-bit, words for 16-bit)
*
* @note  Waits for RXNE flag before each reception
*        Handles both 8-bit and 16-bit data frames
*        Blocks until all data is received
*/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//1. wait until RXNE is set
			while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == (uint8_t)FLAG_RESET );

			//2. check the DFF bit in CR1
			if(isBit_Set(pSPIx->CR1,SPI_CR1_DFF))
			{
				//16 bit DFF
				//1. load the data from DR to Rxbuffer address
				 *((uint16_t*)pRxBuffer) = pSPIx->DR ;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}else
			{
				//8 bit DFF
				*(pRxBuffer) = pSPIx->DR ;
				Len--;
				pRxBuffer++;
			}
		}

}


/* -----------------------------------------------------------------------------
* @brief  Enables or disables the SPI peripheral
*
* @param[in] pSPIx: SPI peripheral base address
* @param[in] EnOrDi: Enable or disable macro (ENABLE/DISABLE)
*
* @note  Controls the SPE bit in CR1 register
*        Must be enabled before any communication
*/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		Set_Bit(pSPIx->CR1,SPI_CR1_SPE);
	}else
	{
		Reset_Bit(pSPIx->CR1,SPI_CR1_SPE);
	}


}


/* -----------------------------------------------------------------------------
* @brief  Configures internal slave select (SSI bit)
*
* @param[in] pSPIx: SPI peripheral base address
* @param[in] EnOrDi: Enable or disable macro (ENABLE/DISABLE)
*
* @note  Used in software slave management mode (SSM=1)
*        Controls SSI bit in CR1 register
*/
void  SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		Set_Bit(pSPIx->CR1,SPI_CR1_SSI);
	}else
	{
		Reset_Bit(pSPIx->CR1,SPI_CR1_SSI);
	}


}


/* -----------------------------------------------------------------------------
* @brief  Configures slave select output enable (SSOE bit)
*
* @param[in] pSPIx: SPI peripheral base address
* @param[in] EnOrDi: Enable or disable macro (ENABLE/DISABLE)
*
* @note  Controls SSOE bit in CR2 register
*        When enabled, NSS output is driven low in master mode
*/
void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		Set_Bit(pSPIx->CR2,SPI_CR2_SSOE);
	}else
	{
		Reset_Bit(pSPIx->CR2,SPI_CR2_SSOE);
	}


}



/* -----------------------------------------------------------------------------
* @brief  Enables or disables SPI interrupt in NVIC
*
* @param[in] IRQNumber: SPI interrupt number (IRQn_Type)
* @param[in] EnOrDi: Enable or disable macro (ENABLE/DISABLE)
*
* @note  Configures NVIC ISER/ICER registers
*        Must be called before using interrupt mode
*/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
* @brief  Sets priority for SPI interrupt
*
* @param[in] IRQNumber: SPI interrupt number (IRQn_Type)
* @param[in] IRQPriority: Interrupt priority value
*
* @note  Configures NVIC priority registers
*        Lower values indicate higher priority
*/
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

/* -----------------------------------------------------------------------------
* @brief  Starts non-blocking data transmission
*
* @param[in] pSPIHandle: Pointer to SPI handle structure
* @param[in] pTxBuffer: Pointer to transmit data buffer
* @param[in] Len: Length of data to transmit
*
* @return Current SPI state (SPI_READY or SPI_BUSY_IN_TX)
*
* @note  Enables TXE interrupt
*        Stores buffer info in handle for interrupt processing
*        Returns immediately after starting transmission
*/
SPI_State_en SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	SPI_State_en SPI_State = pSPIHandle->TxState;

	if(SPI_State != SPI_BUSY_IN_TX)
	{
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		SPI_State = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		Set_Bit(pSPIHandle->pSPIx->CR2,SPI_CR2_TXEIE);

	}


	return SPI_State;
}

/* -----------------------------------------------------------------------------
* @brief  Starts non-blocking data reception
*
* @param[in] pSPIHandle: Pointer to SPI handle structure
* @param[in] pRxBuffer: Pointer to receive data buffer
* @param[in] Len: Length of data to receive
*
* @return Current SPI state (SPI_READY or SPI_BUSY_IN_RX)
*
* @note  Enables RXNE interrupt
*        Stores buffer info in handle for interrupt processing
*        Returns immediately after starting reception
*/
SPI_State_en SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	SPI_State_en SPI_State = pSPIHandle->RxState;

	if(SPI_State != SPI_BUSY_IN_RX)
	{
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		Set_Bit(pSPIHandle->pSPIx->CR2,SPI_CR2_RXNEIE);

	}


	return SPI_State;

}


/* -----------------------------------------------------------------------------
* @brief  Handles SPI interrupts and dispatches to appropriate handlers
*
* @param[in] pHandle: Pointer to SPI handle structure
*
* @note  Checks and handles:
*        - TXE (Transmit) interrupts
*        - RXNE (Receive) interrupts
*        - OVR (Overrun) errors
*        Calls application callback on completion/error
*/


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

	uint8_t temp1 , temp2;
	//first lets check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for ovr flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}


}


//some helper function implementations

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if(isBit_Set(pSPIHandle->pSPIx->CR1,SPI_CR1_DFF))
	{
		//16 bit DFF
		//1. load the data in to the DR
		pSPIHandle->pSPIx->DR =   *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR =   *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}


static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//do rxing as per the dff
	if(isBit_Set(pSPIHandle->pSPIx->CR1,SPI_CR1_DFF))
	{
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;

	}else
	{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen)
	{
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}

/* -----------------------------------------------------------------------------
* @brief  Clears SPI overrun flag
*
* @param[in] pSPIx: SPI peripheral base address
*
* @note  Reads DR then SR registers to clear OVR flag
*        Required after overrun error occurs
*/
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}


void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	Reset_Bit(pSPIHandle->pSPIx->CR2,SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	Reset_Bit(pSPIHandle->pSPIx->CR2,SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}



void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}


/* -----------------------------------------------------------------------------
* @brief  Application callback for SPI events
*
* @param[in] pSPIHandle: Pointer to SPI handle structure
* @param[in] AppEv: SPI event that occurred
*            - SPI_EVENT_TX_CMPLT: Transmission complete
*            - SPI_EVENT_RX_CMPLT: Reception complete
*            - SPI_EVENT_OVR_ERR: Overrun error occurred
*
* @note  Weak implementation - should be overridden by application
*        Called from interrupt context
*/
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}





