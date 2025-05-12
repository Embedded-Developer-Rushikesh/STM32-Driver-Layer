
#include"stm32f446xx_SysTick_driver.h"
/**
  * @brief  Creates a blocking delay in milliseconds using the SysTick timer.
  * @param  delay: Duration of the delay in milliseconds.
  * @retval None
  * @note   This function uses the SysTick timer to create a software delay.
  *         It reloads the SysTick counter each millisecond and waits for the
  *         COUNTFLAG to be set. The system clock must be configured correctly
  *         for the delay to be accurate.
  */

void DelayMS(int delay)
{
	/*Reload with number of clocks per millisecond*/
	SysTick->LOAD	= SYSTICK_LOAD_VAL;

	/*Clear systick current value register */
	SysTick->VAL = 0;

	/*Enable systick and select internal clk src*/
	SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;
	for(int i=0; i<delay ; i++){

		/*Wait until the COUNTFLAG is set*/
		while((SysTick->CTRL &  CTRL_COUNTFLAG) == 0){}
	}
	SysTick->CTRL = 0;
}

/**
  * @brief  Configures the SysTick timer to generate an interrupt every 1 second.
  * @param  None
  * @retval None
  * @note   This function sets up the SysTick timer for 1 Hz periodic interrupts.
  *         It uses the internal clock source and enables the SysTick interrupt.
  *         The interrupt handler must be defined separately to handle the event.
  */
void systick_1hz_interrupt(void)
{
	/*Reload with number of clocks persecond*/
	SysTick->LOAD  = ONE_SEC_LOAD - 1;
	/*Clear systick current value register */
	SysTick->VAL = 0;

	/*Enable systick and select internal clk src*/
	SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC ;

	/*Enable systick interrupt*/
	SysTick->CTRL  |= CTRL_TICKINT;
}


