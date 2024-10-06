<h1 align="center">STM32F4xx GPIO API</h1>

###

<h5 align="left">typedef struct<br>{<br>	__vo uint32_t MODER;                        /*!< GPIO port mode register,                    	Address offset: 0x00      */<br>	__vo uint32_t OTYPER;                       /*!<    										Address offset: 0x04      */<br>	__vo uint32_t OSPEEDR;<br>	__vo uint32_t PUPDR;<br>	__vo uint32_t IDR;<br>	__vo uint32_t ODR;<br>	__vo uint32_t BSRR;<br>	__vo uint32_t LCKR;<br>	__vo uint32_t AFR[2];					 /*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */<br>}GPIO_RegDef_t;</h5>

###

<h5 align="left">typedef struct<br>{<br>	uint8_t GPIO_PinNumber;<br>	uint8_t GPIO_PinMode;			/*!< possible values from @GPIO_PIN_MODES >*/<br>	uint8_t GPIO_PinSpeed;			/*!< possible values from @GPIO_PIN_SPEED >*/<br>	uint8_t GPIO_PinPuPdControl;<br>	uint8_t GPIO_PinOPType;<br>	uint8_t GPIO_PinAltFunMode;<br>}GPIO_PinConfig_t;</h5>

###

<h5 align="left">This is a Handle structure for a GPIO pin<br><br>typedef struct<br>{<br>	GPIO_RegDef_t *pGPIOx;       		/*!< This holds the base address of the GPIO port to which the pin belongs >*/<br>	GPIO_PinConfig_t GPIO_PinConfig;   /*!< This holds GPIO pin configuration settings >*/<br><br>}GPIO_Handle_t;</h5>

###

<h5 align="left">void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);<br>void GPIO_Init(GPIO_Handle_t *pGPIOHandle);<br>void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);<br>uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);<br>uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);<br>void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);<br>void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);<br>void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);<br>void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);<br>void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);<br>void GPIO_IRQHandling(uint8_t PinNumber);</h5>

###
