#include"stm32f446xx_gpio_driver.h"

/**
  * @brief  Initializes the GPIOx peripheral according to specified parameters
  * @param  pGPIOHandle: Pointer to GPIO_Handle_t structure
  * @retval None
  * @note   This function configures:
  *         - GPIO pin mode (input, output, alternate function, analog)
  *         - GPIO output type (push-pull/open-drain)
  *         - GPIO speed
  *         - GPIO pull-up/pull-down
  *         - Alternate function selection
  *         - Interrupt configuration if applicable
  */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	 uint32_t temp=0;
	 GPIO_DeInit(pGPIOHandle->pGPIOx);
	 GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//Pin Configuration
	switch (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) {
	case GPIO_MODE_ANALOG: {
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<< (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))); // Clearing the position
		pGPIOHandle->pGPIOx->MODER |= (0x3<< (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
		break;
	}
	case GPIO_MODE_ALTFN: {
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<< (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))); // Clearing the position
		pGPIOHandle->pGPIOx->MODER |= (0x2<< (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
		break;
	}
	case GPIO_MODE_IN:
	{
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<< (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))); // Clearing the position
		pGPIOHandle->pGPIOx->MODER &= ~(0x1<< (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
		break;
	}
	case GPIO_MODE_OUT:
	{
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<< (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber))); // Clearing the position
		pGPIOHandle->pGPIOx->MODER |= (0x1<< (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
		break;
	}
	case GPIO_MODE_IT_FT:
	{
		EXTI->FTSR|=(0x1<< ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
		EXTI->RTSR&=~(0x1<< ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));//Clearing Rising Intrupt line
		break;
	}
	case GPIO_MODE_IT_RT:
	{
		EXTI->RTSR|=(0x1<< ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
		EXTI->FTSR&=~(0x01<< ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));// Clearing the falling intrupt line
		break;
	}
	case GPIO_MODE_IT_RFT:
	{
		EXTI->RTSR|=(0x1<< ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
		EXTI->FTSR|=(0x1<< ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));// Clearing the falling intrupt line
	}
	default: {
		break;
	}

	}
	//2. configure the speed

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	(pGPIOHandle->pGPIOx->OSPEEDR) &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//Pull Push Configuration
	temp=0;
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx->PUPDR|= ~(0x3<< (2 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
	pGPIOHandle->pGPIOx->PUPDR|=temp;

//	//Port output type
//	temp=0;
//	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
//	pGPIOHandle->pGPIOx->OTYPER|= ~(0x1<< ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
//	pGPIOHandle->pGPIOx->OTYPER|=temp;
//
	//Alternative Functionality Mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode==GPIO_MODE_ALTFN)
	{
		pGPIOHandle->pGPIOx->AFR[pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8]&=~(0xF<<(4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %8)));
		pGPIOHandle->pGPIOx->AFR[pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8]|=(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)<<(4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %8));
	}
	//2. configure the GPIO port selection in SYSCFG_EXTICR
	uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
	uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
	uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
	SYSCFG_PCLK_EN();
	SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);

	//3 . enable the exti interrupt delivery using IMR
	EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
}



/* -----------------------------------------------------------------------------
* @brief  Controls the peripheral clock for the given GPIO port
*
* @param[in] pGPIOx: GPIO port base address (GPIOA, GPIOB, etc.)
* @param[in] EnorDi: Enable or disable macro (ENABLE/DISABLE)
*
* @note  Enables or disables the clock for the specified GPIO port
*        using RCC registers. Supports all GPIO ports (A-I)
*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		}
	}

}


/* -----------------------------------------------------------------------------
* @brief  Resets all registers of the given GPIO port to default values
*
* @param[in] pGPIOx: GPIO port base address to reset
*
* @note  Performs a peripheral reset through the RCC registers
*        Supports all GPIO ports (A-I)
*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}

}

/* -----------------------------------------------------------------------------
* @brief  Reads the value of a specific input pin
*
* @param[in] pGPIOx: GPIO port base address
* @param[in] PinNumber: Pin number to read (0-15)
*
* @return uint8_t: Current state of the pin (0 or 1)
*
* @note  Reads from the GPIO input data register (IDR)
*        Masks and shifts to get only the specified pin's state
*/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t data=(uint8_t)(((pGPIOx->IDR)>>PinNumber)&0x01);
	return data;
}


/* -----------------------------------------------------------------------------
* @brief  Reads the entire input port value
*
* @param[in] pGPIOx: GPIO port base address
*
* @return uint16_t: 16-bit value of the entire port
*
* @note  Directly returns the GPIO input data register (IDR) value
*        All 16 pins are read simultaneously
*/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t data=(uint16_t)(pGPIOx->IDR);
	return data;
}


/* -----------------------------------------------------------------------------
* @brief  Writes a value to a specific output pin
*
* @param[in] pGPIOx: GPIO port base address
* @param[in] PinNumber: Pin number to write (0-15)
* @param[in] Value: Value to write (ENABLE/DISABLE or SET/RESET)
*
* @note  Uses bit manipulation to set or clear the pin in ODR
*        Does not affect other pins in the port
*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value==ENABLE)
	{
		Set_Bit(pGPIOx->ODR,PinNumber);
	}else
	{
		Reset_Bit(pGPIOx->ODR,PinNumber);
	}



}

/* -----------------------------------------------------------------------------
* @brief  Writes a 16-bit value to the entire output port
*
* @param[in] pGPIOx: GPIO port base address
* @param[in] Value: 16-bit value to write to the port
*
* @note  Directly writes to the GPIO output data register (ODR)
*        All 16 pins are written simultaneously
*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR|=Value;
}


/* -----------------------------------------------------------------------------
* @brief  Toggles the state of a specific output pin
*
* @param[in] pGPIOx: GPIO port base address
* @param[in] PinNumber: Pin number to toggle (0-15)
*
* @note  Uses XOR operation to flip the pin state
*        If pin was high, becomes low and vice versa
*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR^=(1<<PinNumber);
}


/* -----------------------------------------------------------------------------
* @brief  Configures GPIO interrupt in the NVIC
*
* @param[in] IRQNumber: Interrupt number to configure
* @param[in] EnorDi: Enable or disable the interrupt (ENABLE/DISABLE)
*
* @note  Handles NVIC configuration for:
*        - IRQ numbers 0-31 (ISER0/ICER0)
*        - IRQ numbers 32-63 (ISER1/ICER1)
*        - IRQ numbers 64-95 (ISER2/ICER2)
*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
* @brief  Sets the priority of a GPIO interrupt
*
* @param[in] IRQNumber: Interrupt number to configure
* @param[in] IRQPriority: Priority value to set
*
* @note  Calculates the correct NVIC priority register and bit position
*        Handles STM32's priority bits implementation
*/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}


/* -----------------------------------------------------------------------------
* @brief  Handles the GPIO interrupt by clearing the pending bit
*
* @param[in] PinNumber: Pin number that triggered the interrupt
*
* @note  Clears the pending bit in EXTI_PR register
*        Must be called in the ISR to prevent repeated triggering
*/

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the EXTI pr register corresponding to the pin number
	if(isBit_Set(EXTI->PR,PinNumber))
	{
		//clear
		Set_Bit(EXTI->PR,PinNumber);
	}

}
