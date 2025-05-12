#include "stm32f446xx_i2c_driver.h"
static void  I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle );
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle );

/* -----------------------------------------------------------------------------
* @brief  Generates START condition on I2C bus
*
* @param[in] pI2Cx: I2C peripheral base address
*
* @note  Sets START bit in CR1 register
*        Must be called in master mode
*/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	Set_Bit(pI2Cx->CR1,I2C_CR1_START);
}

/* -----------------------------------------------------------------------------
* @brief  Executes address phase for write operation
*
* @param[in] pI2Cx: I2C peripheral base address
* @param[in] SlaveAddr: 7-bit slave address
*
* @note  Shifts address left by 1 and clears LSB for write operation
*        Loads address into DR register
*/

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}

/* -----------------------------------------------------------------------------
* @brief  Executes address phase for read operation
*
* @param[in] pI2Cx: I2C peripheral base address
* @param[in] SlaveAddr: 7-bit slave address
*
* @note  Shifts address left by 1 and sets LSB for read operation
*        Loads address into DR register
*/
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->DR = SlaveAddr;
}

/* -----------------------------------------------------------------------------
* @brief  Clears ADDR flag after address phase
*
* @param[in] pI2CHandle: Pointer to I2C handle structure
*
* @note  Implements proper sequence to clear ADDR flag
*        Handles special cases for single byte reception
*/
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle )
{
	uint32_t dummy_read;
	I2C_State_en I2C_state=pI2CHandle->TxRxState;
	//check for device mode
	if(isBit_Set(pI2CHandle->pI2Cx->SR2,I2C_SR2_MSL))
	{
		//device is in master mode
		if( I2C_state== I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize  == 1)
			{
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}

		}
		else
		{
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}

	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}


}

/* -----------------------------------------------------------------------------
* @brief  Generates STOP condition on I2C bus
*
* @param[in] pI2Cx: I2C peripheral base address
*
* @note  Sets STOP bit in CR1 register
*        Can be called in master or slave mode
*/
 void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	 Set_Bit(pI2Cx->CR1,I2C_CR1_STOP);
}

 /* -----------------------------------------------------------------------------
 * @brief  Enables/disables slave callback events
 *
 * @param[in] pI2Cx: I2C peripheral base address
 * @param[in] EnorDi: Enable/disable macro (ENABLE/DISABLE)
 *
 * @note  Controls ITEVTEN bit in CR2 register
 *        When enabled, slave can generate event interrupts
 */
 void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
 {
	 if(EnorDi == ENABLE)
	 {
		 Set_Bit(pI2Cx->CR2,I2C_CR2_ITEVTEN);
	 }else
	 {
		 Reset_Bit(pI2Cx->CR2,I2C_CR2_ITEVTEN);
	 }

 }

 /* -----------------------------------------------------------------------------
 * @brief  Enables/disables I2C peripheral
 *
 * @param[in] pI2Cx: I2C peripheral base address
 * @param[in] EnOrDi: Enable/disable macro (ENABLE/DISABLE)
 *
 * @note  Controls PE bit in CR1 register
 *        Must be enabled before any communication
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		Set_Bit(pI2Cx->CR1,I2C_CR1_PE);
		//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
	}else
	{
		Reset_Bit(pI2Cx->CR1,I2C_CR1_PE);
	}

}


/* -----------------------------------------------------------------------------
* @brief  Enables or disables peripheral clock for the given I2C
*
* @param[in] pI2Cx: I2C peripheral base address (I2C1-I2C3)
* @param[in] EnOrDi: Enable or disable macro (ENABLE/DISABLE)
*
* @note  Controls clock for all supported I2C peripherals (I2C1-I2C3)
*        Uses RCC registers to enable/disable clocks
*/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}

}




/* -----------------------------------------------------------------------------
* @brief  Initializes I2C peripheral with given configuration
*
* @param[in] pI2CHandle: Pointer to I2C handle structure containing:
*            - pI2Cx: I2C peripheral base address
*            - I2C_Config: Configuration structure
*
* @note  Configures:
*        - Clock speed (standard/fast mode)
*        - Own address
*        - ACK control
*        - Duty cycle (fast mode only)
*        - Rise time
* Enables peripheral clock before configuration
*/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0 ;

	//enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() /1000000U ;
	pI2CHandle->pI2Cx->CR2 =  (tempreg & 0x3F);

   //program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//mode is fast mode
		tempreg |= ( 1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}else
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode

		tempreg = (RCC_GetPCLK1Value() /1000000U) + 1 ;

	}else
	{
		//mode is fast mode
		tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;

	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}


/* -----------------------------------------------------------------------------
* @brief  Deinitializes I2C peripheral registers
*
* @param[in] pI2Cx: I2C peripheral base address
*
* @note  Resets all registers to default values
*        Uses RCC reset registers
*/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1) {
		I2C1_REG_RESET();
	} else if (pI2Cx == I2C2) {
		I2C2_REG_RESET();
	} else if (pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}

/* -----------------------------------------------------------------------------
* @brief  Gets status of specified I2C flag
*
* @param[in] pI2Cx: I2C peripheral base address
* @param[in] FlagName: Flag to check (e.g. I2C_FLAG_SB, I2C_FLAG_ADDR)
*
* @return Flag status (SET or RESET)
*
* @note  Checks status register (SR1) for specified flag
*/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/* -----------------------------------------------------------------------------
* @brief  Master transmit data in blocking mode
*
* @param[in] pI2CHandle: Pointer to I2C handle structure
* @param[in] pTxbuffer: Pointer to transmit data buffer
* @param[in] Len: Length of data to transmit
* @param[in] SlaveAddr: 7-bit slave address
* @param[in] Sr: Repeated start control (I2C_ENABLE_SR/I2C_DISABLE_SR)
*
* @note  Implements complete I2C master transmit sequence:
*        1. START condition
*        2. Address phase
*        3. Data transmission
*        4. STOP condition (if Sr=I2C_DISABLE_SR)
* Blocks until all data is transmitted
*/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)   );

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)   );

	//5. clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6. send the data until len becomes 0

	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) ); //Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE) );

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF) );


	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_DISABLE_SR )
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

/* -----------------------------------------------------------------------------
* @brief  Master receive data in blocking mode
*
* @param[in] pI2CHandle: Pointer to I2C handle structure
* @param[in] pRxBuffer: Pointer to receive data buffer
* @param[in] Len: Length of data to receive
* @param[in] SlaveAddr: 7-bit slave address
* @param[in] Sr: Repeated start control (I2C_ENABLE_SR/I2C_DISABLE_SR)
*
* @note  Implements complete I2C master receive sequence:
*        1. START condition
*        2. Address phase
*        3. Data reception
*        4. Special handling for 1-byte and 2-byte reception
*        5. STOP condition (if Sr=I2C_DISABLE_SR)
* Blocks until all data is received
*/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)   );

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)   );


	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);


		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until  RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

		//generate STOP condition
		if(Sr == I2C_DISABLE_SR )
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}


    //procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				//generate STOP condition
				if(Sr == I2C_DISABLE_SR )
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;

		}

	}

	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}

}

/* -----------------------------------------------------------------------------
* @brief  Controls ACK generation
*
* @param[in] pI2Cx: I2C peripheral base address
* @param[in] EnorDi: Enable/disable macro (ENABLE/DISABLE)
*
* @note  Controls ACK bit in CR1 register
*        Used to enable/disable acknowledgment after byte reception
*/
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		Set_Bit(pI2Cx->CR1,I2C_CR1_ACK);
	}else
	{
		//disable the ack
		Reset_Bit(pI2Cx->CR1 ,I2C_CR1_ACK);
	}
}

/* -----------------------------------------------------------------------------
* @brief  Configures I2C interrupt in NVIC
*
* @param[in] IRQNumber: I2C interrupt number (IRQn_Type)
* @param[in] EnorDi: Enable/disable macro (ENABLE/DISABLE)
*
* @note  Configures NVIC ISER/ICER registers
*        Must be called before using interrupt mode
*/
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
* @brief  Sets priority for I2C interrupt
*
* @param[in] IRQNumber: I2C interrupt number (IRQn_Type)
* @param[in] IRQPriority: Interrupt priority value
*
* @note  Configures NVIC priority registers
*        Lower values indicate higher priority
*/
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

/* -----------------------------------------------------------------------------
* @brief  Master transmit data in interrupt mode
*
* @param[in] pI2CHandle: Pointer to I2C handle structure
* @param[in] pTxBuffer: Pointer to transmit data buffer
* @param[in] Len: Length of data to transmit
* @param[in] SlaveAddr: 7-bit slave address
* @param[in] Sr: Repeated start control (I2C_ENABLE_SR/I2C_DISABLE_SR)
*
* @return Current I2C state (I2C_READY or I2C_BUSY_IN_TX)
*
* @note  Enables I2C event and buffer interrupts
*        Starts transmission by generating START condition
*        Returns immediately after starting transmission
*        Completion notified via callback
*/
I2C_State_en I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	I2C_State_en busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		busystate = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		Set_Bit(pI2CHandle->pI2Cx->CR2 ,I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		Set_Bit(pI2CHandle->pI2Cx->CR2,I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		Set_Bit(pI2CHandle->pI2Cx->CR2,I2C_CR2_ITERREN);

	}

	return busystate;
}


/* -----------------------------------------------------------------------------
* @brief  Master receive data in interrupt mode
*
* @param[in] pI2CHandle: Pointer to I2C handle structure
* @param[in] pRxBuffer: Pointer to receive data buffer
* @param[in] Len: Length of data to receive
* @param[in] SlaveAddr: 7-bit slave address
* @param[in] Sr: Repeated start control (I2C_ENABLE_SR/I2C_DISABLE_SR)
*
* @return Current I2C state (I2C_READY or I2C_BUSY_IN_RX)
*
* @note  Enables I2C event and buffer interrupts
*        Starts reception by generating START condition
*        Returns immediately after starting reception
*        Completion notified via callback
*/
I2C_State_en I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	I2C_State_en I2C_State = pI2CHandle->TxRxState;

	if( (I2C_State != I2C_BUSY_IN_TX) && (I2C_State != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		I2C_State = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		Set_Bit(pI2CHandle->pI2Cx->CR2 ,I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		Set_Bit(pI2CHandle->pI2Cx->CR2,I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		Set_Bit(pI2CHandle->pI2Cx->CR2,I2C_CR2_ITERREN);
	}

	return I2C_State;
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle )
{

	if(pI2CHandle->TxLen > 0)
	{
		//1. load the data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;

	}

}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle )
{
	//We have to do the data reception
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;

	}


	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
		}

			//read DR
			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0 )
	{
		//close the I2C data reception and notify the application

		//1. generate the stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2 . Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}

/* -----------------------------------------------------------------------------
* @brief  Closes I2C data reception
*
* @param[in] pI2CHandle: Pointer to I2C handle structure
*
* @note  Disables I2C buffer and event interrupts
*        Resets handle state to ready
*        Re-enables ACK if configured
*/
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	Reset_Bit(pI2CHandle->pI2Cx->CR2 ,I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	Reset_Bit(	pI2CHandle->pI2Cx->CR2,I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}

}
/* -----------------------------------------------------------------------------
* @brief  Closes I2C data transmission
*
* @param[in] pI2CHandle: Pointer to I2C handle structure
*
* @note  Disables I2C buffer and event interrupts
*        Resets handle state to ready
*/
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	Reset_Bit(pI2CHandle->pI2Cx->CR2 ,I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	Reset_Bit(	pI2CHandle->pI2Cx->CR2,I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

/* -----------------------------------------------------------------------------
* @brief  Slave transmit data
*
* @param[in] pI2C: I2C peripheral base address
* @param[in] data: Data byte to transmit
*
* @note  Loads data into DR register
*        Called when slave is in transmitter mode
*/
void I2C_SlaveSendData(I2C_RegDef_t *pI2C,const uint8_t data)
{
	pI2C->DR = data;
}
/* -----------------------------------------------------------------------------
* @brief  Slave receive data
*
* @param[in] pI2C: I2C peripheral base address
*
* @return Received data byte
*
* @note  Reads data from DR register
*        Called when slave is in receiver mode
*/
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
    return (uint8_t) pI2C->DR;
}


/* -----------------------------------------------------------------------------
* @brief  Handles I2C event interrupts
*
* @param[in] pI2CHandle: Pointer to I2C handle structure
*
* @note  Handles following events:
*        - Start condition generated (SB)
*        - Address sent/matched (ADDR)
*        - Byte transfer finished (BTF)
*        - Stop detected (STOPF) - slave mode only
*        - Transmit buffer empty (TXE)
*        - Receive buffer not empty (RXNE)
* Calls appropriate handlers for master/slave modes
*/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device

	uint32_t temp1, temp2, temp3;

	temp1   = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN) ;
	temp2   = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN) ;

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB);
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets executed the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		// interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure that TXE is also set .
			if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE) )
			{
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0 )
				{
					//1. generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2. reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//3. notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);

				}
			}

		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			;
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	//The below code block will not be executed by the master since STOPF will not set in master mode
	if(temp1 && temp3)
	{
		//STOF flag is set
		//Clear the STOPF ( i.e 1) read SR1 2) Write to CR1 )

		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
	}


	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//Check for device mode
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//TXE flag is set
			//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			//slave
			//make sure that the slave is really in transmitter mode
		    if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
		    {
		    	I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
		    }
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		//check device mode .
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//The device is master

			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);

			}

		}else
		{
			//slave
			//make sure that the slave is really in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
			}
		}
	}
}



/* -----------------------------------------------------------------------------
* @brief  Handles I2C error interrupts
*
* @param[in] pI2CHandle: Pointer to I2C handle structure
*
* @note  Handles following error conditions:
*        - Bus error (BERR)
*        - Arbitration lost (ARLO)
*        - Acknowledge failure (AF)
*        - Overrun/underrun (OVR)
*        - Timeout error (TIMEOUT)
* Notifies application via error callback
*/
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}
