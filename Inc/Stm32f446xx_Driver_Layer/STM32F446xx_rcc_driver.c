#include"STM32F446xx_rcc_driver.h"
uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};

/* -----------------------------------------------------------------------------
* @brief  Gets the PCLK1 (APB1 peripheral clock) value
*
* @return PCLK1 frequency in Hz
*
* @note  Calculates PCLK1 frequency based on:
*        - System clock source (HSI, HSE, or PLL)
*        - AHB prescaler
*        - APB1 prescaler
*        Handles all possible prescaler configurations
*        Defaults to 16MHz (HSI) if clock source not configured
*/

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;

	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0 )
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if (clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	//for ahb
	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}



	//apb1
	temp = ((RCC->CFGR >> 10 ) & 0x7);

	if(temp < 4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 =  (SystemClk / ahbp) /apb1p;

	return pclk1;
}


/* -----------------------------------------------------------------------------
* @brief  Gets the PCLK2 (APB2 peripheral clock) value
*
* @return PCLK2 frequency in Hz
*
* @note  Calculates PCLK2 frequency based on:
*        - System clock source (HSI, HSE - PLL not handled in this implementation)
*        - AHB prescaler
*        - APB2 prescaler
*        Handles all possible prescaler configurations
*        Defaults to 16MHz (HSI) if clock source not configured
*/
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t SystemClock=0,tmp,pclk2;
	uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

	uint8_t ahbp,apb2p;

	if(clk_src == 0)
	{
		SystemClock = 16000000;
	}else
	{
		SystemClock = 8000000;
	}
	tmp = (RCC->CFGR >> 4 ) & 0xF;

	if(tmp < 0x08)
	{
		ahbp = 1;
	}else
	{
       ahbp = AHB_PreScaler[tmp-8];
	}

	tmp = (RCC->CFGR >> 13 ) & 0x7;
	if(tmp < 0x04)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB1_PreScaler[tmp-4];
	}

	pclk2 = (SystemClock / ahbp )/ apb2p;

	return pclk2;
}

/* -----------------------------------------------------------------------------
* @brief  Gets the PLL output clock value
*
* @return PLL output frequency in Hz
*
* @note  Currently returns 0 as placeholder
*        Should be implemented to calculate actual PLL output based on:
*        - PLL source (HSI/HSE)
*        - PLLM, PLLN, PLLP, PLLQ factors
*        - PLL configuration bits in RCC registers
*/
uint32_t  RCC_GetPLLOutputClock()
{

	return 0;
}


