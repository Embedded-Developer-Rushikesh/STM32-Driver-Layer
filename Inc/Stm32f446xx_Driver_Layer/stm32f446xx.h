/**
  ******************************************************************************
  * @file    stm32f446xx.h
  * @brief   This file contains:
  *          - Processor and peripheral register definitions
  *          - Memory map and base addresses
  *          - Clock enable/disable macros
  *          - Peripheral register structures
  *          - Interrupt numbers and priority levels
  *          - Bit position definitions for various peripherals
  *
  * @note    This file should be included in all driver source files
  *          for proper register access and peripheral control
  ******************************************************************************
  */

#ifndef STM32F446XX_DRIVER_LAYER_STM32F446XX_H_
#define STM32F446XX_DRIVER_LAYER_STM32F446XX_H_
#include <stddef.h>
#include <stdint.h>

/* Macros for volatile and weak attributes */
#define __vo volatile      /*!< Macro to denote volatile variables */
#define __weak __attribute__((weak)) /*!< Macro for weak function definitions */

/********************************** Processor Specific Details **********************************/

/* ARM Cortex Mx Processor NVIC Register Addresses */
#define NVIC_ISER0          ((__vo uint32_t*)0xE000E100) /*!< Interrupt Set Enable Register 0 */
#define NVIC_ISER1          ((__vo uint32_t*)0xE000E104) /*!< Interrupt Set Enable Register 1 */
#define NVIC_ISER2          ((__vo uint32_t*)0xE000E108) /*!< Interrupt Set Enable Register 2 */
#define NVIC_ISER3          ((__vo uint32_t*)0xE000E10C) /*!< Interrupt Set Enable Register 3 */


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0         ((__vo uint32_t*)0xE000E180) /*!< Interrupt Clear Enable Register 0 */
#define NVIC_ICER1         ((__vo uint32_t*)0xE000E184) /*!< Interrupt Clear Enable Register 1 */
#define NVIC_ICER2         ((__vo uint32_t*)0xE000E188) /*!< Interrupt Clear Enable Register 2 */
#define NVIC_ICER3         ((__vo uint32_t*)0xE000E18C) /*!< Interrupt Clear Enable Register 3 */

/* ARM Cortex Mx Processor SysTick Registers */
#define SCS_BASE           (0xE000E000UL)              /*!< System Control Space Base Address */
#define SysTick_BASE       (SCS_BASE + 0x0010UL)       /*!< SysTick Base Address */

/* ARM Cortex Mx Processor Priority Register */
#define NVIC_PR_BASE_ADDR  ((__vo uint32_t*)0xE000E400) /*!< NVIC Priority Register Base */
#define NO_PR_BITS_IMPLEMENTED 4                       /*!< Number of implemented priority bits */

/* Memory Base Addresses */
#define FLASH_BASEADDR     0x08000000U  /*!< Flash memory base address */
#define SRAM1_BASEADDR     0x20000000U  /*!< SRAM1 (112KB) base address */
#define SRAM2_BASEADDR     0x2001C000U  /*!< SRAM2 (16KB) base address */
#define ROM_BASEADDR       0x1FFF0000U  /*!< System memory (ROM) base address */
#define SRAM               SRAM1_BASEADDR /*!< Default SRAM selection */

/* Peripheral Bus Base Addresses */
#define PERIPH_BASEADDR    0x40000000U  /*!< Peripheral base address */
#define APB1PERIPH_BASEADDR PERIPH_BASEADDR /*!< APB1 peripheral base address */
#define APB2PERIPH_BASEADDR 0x40010000U /*!< APB2 peripheral base address */
#define AHB1PERIPH_BASEADDR 0x40020000U /*!< AHB1 peripheral base address */
#define AHB2PERIPH_BASEADDR 0x50000000U /*!< AHB2 peripheral base address */

/* AHB1 Peripheral Base Addresses */
#define GPIOA_BASEADDR     (AHB1PERIPH_BASEADDR + 0x0000) /*!< GPIOA base address */
#define GPIOB_BASEADDR     (AHB1PERIPH_BASEADDR + 0x0400) /*!< GPIOB base address */
#define GPIOC_BASEADDR     (AHB1PERIPH_BASEADDR + 0x0800) /*!< GPIOC base address */
#define GPIOD_BASEADDR     (AHB1PERIPH_BASEADDR + 0x0C00) /*!< GPIOD base address */
#define GPIOE_BASEADDR     (AHB1PERIPH_BASEADDR + 0x1000) /*!< GPIOE base address */
#define GPIOF_BASEADDR     (AHB1PERIPH_BASEADDR + 0x1400) /*!< GPIOF base address */
#define GPIOG_BASEADDR     (AHB1PERIPH_BASEADDR + 0x1800) /*!< GPIOG base address */
#define GPIOH_BASEADDR     (AHB1PERIPH_BASEADDR + 0x1C00) /*!< GPIOH base address */
#define GPIOI_BASEADDR     (AHB1PERIPH_BASEADDR + 0x2000) /*!< GPIOI base address */
#define RCC_BASEADDR       (AHB1PERIPH_BASEADDR + 0x3800) /*!< RCC base address */

/* APB1 Peripheral Base Addresses */
#define I2C1_BASEADDR      (APB1PERIPH_BASEADDR + 0x5400) /*!< I2C1 base address */
#define I2C2_BASEADDR      (APB1PERIPH_BASEADDR + 0x5800) /*!< I2C2 base address */
#define I2C3_BASEADDR      (APB1PERIPH_BASEADDR + 0x5C00) /*!< I2C3 base address */
#define SPI2_BASEADDR      (APB1PERIPH_BASEADDR + 0x3800) /*!< SPI2 base address */
#define SPI3_BASEADDR      (APB1PERIPH_BASEADDR + 0x3C00) /*!< SPI3 base address */
#define USART2_BASEADDR    (APB1PERIPH_BASEADDR + 0x4400) /*!< USART2 base address */
#define USART3_BASEADDR    (APB1PERIPH_BASEADDR + 0x4800) /*!< USART3 base address */
#define UART4_BASEADDR     (APB1PERIPH_BASEADDR + 0x4C00) /*!< UART4 base address */
#define UART5_BASEADDR     (APB1PERIPH_BASEADDR + 0x5000) /*!< UART5 base address */

/* APB2 Peripheral Base Addresses */
#define EXTI_BASEADDR      (APB2PERIPH_BASEADDR + 0x3C00) /*!< EXTI base address */
#define SPI1_BASEADDR      (APB2PERIPH_BASEADDR + 0x3000) /*!< SPI1 base address */
#define SYSCFG_BASEADDR    (APB2PERIPH_BASEADDR + 0x3800) /*!< SYSCFG base address */
#define USART1_BASEADDR    (APB2PERIPH_BASEADDR + 0x1000) /*!< USART1 base address */
#define USART6_BASEADDR    (APB2PERIPH_BASEADDR + 0x1400) /*!< USART6 base address */

/* SysTick Configuration Structure */
#define SysTick            ((SysTick_Type*)SysTick_BASE) /*!< SysTick configuration struct */

/********************************** Peripheral Register Definition Structures **********************************/

/**
  * @brief GPIO Register Definition Structure
  *
  * @note This structure contains all registers for GPIO peripheral
  *       Each register controls specific aspects of GPIO functionality
  */
typedef struct
{
    __vo uint32_t MODER;    /*!< GPIO port mode register - Configures I/O direction (input/output/alternate/analog)
                                Address offset: 0x00 */
    __vo uint32_t OTYPER;   /*!< GPIO output type register - Configures output type (push-pull/open-drain)
                                Address offset: 0x04 */
    __vo uint32_t OSPEEDR;  /*!< GPIO output speed register - Configures I/O speed (low/medium/high/very high)
                                Address offset: 0x08 */
    __vo uint32_t PUPDR;    /*!< GPIO pull-up/pull-down register - Configures pull-up/pull-down resistors
                                Address offset: 0x0C */
    __vo uint32_t IDR;      /*!< GPIO input data register - Contains input pin values
                                Address offset: 0x10 */
    __vo uint32_t ODR;      /*!< GPIO output data register - Controls output pin states
                                Address offset: 0x14 */
    __vo uint32_t BSRR;     /*!< GPIO bit set/reset register - Atomic set/reset of output pins
                                Address offset: 0x18 */
    __vo uint32_t LCKR;     /*!< GPIO configuration lock register - Locks port configuration
                                Address offset: 0x1C */
    __vo uint32_t AFR[2];   /*!< GPIO alternate function registers - Configures alternate functions
                                AFR[0]: Alternate function low register (pins 0-7), Address offset: 0x20
                                AFR[1]: Alternate function high register (pins 8-15), Address offset: 0x24 */
} GPIO_RegDef_t;

/**
  * @brief SysTick Register Definition Structure
  *
  * @note Contains registers for ARM Cortex-M SysTick timer peripheral
  */
typedef struct
{
    __vo uint32_t CTRL;     /*!< SysTick Control and Status Register - Enables timer and checks status
                                Offset: 0x000 (R/W) */
    __vo uint32_t LOAD;     /*!< SysTick Reload Value Register - Sets reload value for timer
                                Offset: 0x004 (R/W) */
    __vo uint32_t VAL;      /*!< SysTick Current Value Register - Shows current timer value
                                Offset: 0x008 (R/W) */
    __vo uint32_t CALIB;    /*!< SysTick Calibration Register - Contains calibration value
                                Offset: 0x00C (R/ ) */
} SysTick_Type;

/**
  * @brief RCC Register Definition Structure
  *
  * @note Contains all registers for Reset and Clock Control (RCC) peripheral
  *       Controls system clock configuration and peripheral clock gating
  */
typedef struct
{
    __vo uint32_t CR;            /*!< Clock control register - Controls internal/external oscillators and PLLs
                                     Address offset: 0x00 */
    __vo uint32_t PLLCFGR;       /*!< PLL configuration register - Configures main PLL parameters
                                     Address offset: 0x04 */
    __vo uint32_t CFGR;          /*!< Clock configuration register - Configures clock dividers and sources
                                     Address offset: 0x08 */
    __vo uint32_t CIR;           /*!< Clock interrupt register - Manages clock-related interrupts
                                     Address offset: 0x0C */
    __vo uint32_t AHB1RSTR;      /*!< AHB1 peripheral reset register - Resets AHB1 peripherals
                                     Address offset: 0x10 */
    __vo uint32_t AHB2RSTR;      /*!< AHB2 peripheral reset register - Resets AHB2 peripherals
                                     Address offset: 0x14 */
    __vo uint32_t AHB3RSTR;      /*!< AHB3 peripheral reset register - Resets AHB3 peripherals
                                     Address offset: 0x18 */
    uint32_t      RESERVED0;     /*!< Reserved register
                                     Address offset: 0x1C */
    __vo uint32_t APB1RSTR;      /*!< APB1 peripheral reset register - Resets APB1 peripherals
                                     Address offset: 0x20 */
    __vo uint32_t APB2RSTR;      /*!< APB2 peripheral reset register - Resets APB2 peripherals
                                     Address offset: 0x24 */
    uint32_t      RESERVED1[2];  /*!< Reserved registers
                                     Address offset: 0x28-0x2C */
    __vo uint32_t AHB1ENR;       /*!< AHB1 peripheral clock enable register - Enables AHB1 peripheral clocks
                                     Address offset: 0x30 */
    __vo uint32_t AHB2ENR;       /*!< AHB2 peripheral clock enable register - Enables AHB2 peripheral clocks
                                     Address offset: 0x34 */
    __vo uint32_t AHB3ENR;       /*!< AHB3 peripheral clock enable register - Enables AHB3 peripheral clocks
                                     Address offset: 0x38 */
    uint32_t      RESERVED2;     /*!< Reserved register
                                     Address offset: 0x3C */
    __vo uint32_t APB1ENR;       /*!< APB1 peripheral clock enable register - Enables APB1 peripheral clocks
                                     Address offset: 0x40 */
    __vo uint32_t APB2ENR;       /*!< APB2 peripheral clock enable register - Enables APB2 peripheral clocks
                                     Address offset: 0x44 */
    uint32_t      RESERVED3[2];  /*!< Reserved registers
                                     Address offset: 0x48-0x4C */
    __vo uint32_t AHB1LPENR;     /*!< AHB1 peripheral clock enable in low power mode register
                                     Address offset: 0x50 */
    __vo uint32_t AHB2LPENR;     /*!< AHB2 peripheral clock enable in low power mode register
                                     Address offset: 0x54 */
    __vo uint32_t AHB3LPENR;     /*!< AHB3 peripheral clock enable in low power mode register
                                     Address offset: 0x58 */
    uint32_t      RESERVED4;     /*!< Reserved register
                                     Address offset: 0x5C */
    __vo uint32_t APB1LPENR;     /*!< APB1 peripheral clock enable in low power mode register
                                     Address offset: 0x60 */
    __vo uint32_t APB2LPENR;     /*!< APB2 peripheral clock enable in low power mode register
                                     Address offset: 0x64 */
    uint32_t      RESERVED5[2];  /*!< Reserved registers
                                     Address offset: 0x68-0x6C */
    __vo uint32_t BDCR;          /*!< Backup domain control register - Controls RTC and backup domain
                                     Address offset: 0x70 */
    __vo uint32_t CSR;           /*!< Control/status register - Manages low-power modes and reset status
                                     Address offset: 0x74 */
    uint32_t      RESERVED6[2];  /*!< Reserved registers
                                     Address offset: 0x78-0x7C */
    __vo uint32_t SSCGR;         /*!< Spread spectrum clock generation register - Controls SSCG modulation
                                     Address offset: 0x80 */
    __vo uint32_t PLLI2SCFGR;    /*!< PLLI2S configuration register - Configures PLLI2S parameters
                                     Address offset: 0x84 */
    __vo uint32_t PLLSAICFGR;    /*!< PLLSAI configuration register - Configures PLLSAI parameters
                                     Address offset: 0x88 */
    __vo uint32_t DCKCFGR;       /*!< Dedicated clocks configuration register - Configures special clocks
                                     Address offset: 0x8C */
    __vo uint32_t CKGATENR;      /*!< Clocks gated enable register - Controls clock gating
                                     Address offset: 0x90 */
    __vo uint32_t DCKCFGR2;      /*!< Dedicated clocks configuration register 2 - Additional clock config
                                     Address offset: 0x94 */
} RCC_RegDef_t;

/**
  * @brief EXTI Register Definition Structure
  *
  * @note Contains registers for External Interrupt/Event Controller (EXTI)
  */
typedef struct
{
    __vo uint32_t IMR;    /*!< Interrupt mask register - Enables interrupt lines
                              Address offset: 0x00 */
    __vo uint32_t EMR;    /*!< Event mask register - Enables event lines
                              Address offset: 0x04 */
    __vo uint32_t RTSR;   /*!< Rising trigger selection register - Configures rising edge triggers
                              Address offset: 0x08 */
    __vo uint32_t FTSR;   /*!< Falling trigger selection register - Configures falling edge triggers
                              Address offset: 0x0C */
    __vo uint32_t SWIER;  /*!< Software interrupt event register - Generates software interrupts
                              Address offset: 0x10 */
    __vo uint32_t PR;     /*!< Pending register - Shows and clears pending interrupts
                              Address offset: 0x14 */
} EXTI_RegDef_t;

/**
  * @brief SPI Register Definition Structure
  *
  * @note Contains all registers for Serial Peripheral Interface (SPI) peripheral
  */
typedef struct
{
    __vo uint32_t CR1;        /*!< Control register 1 - Configures SPI communication parameters
                                  Address offset: 0x00 */
    __vo uint32_t CR2;        /*!< Control register 2 - Configures interrupts and DMA
                                  Address offset: 0x04 */
    __vo uint32_t SR;         /*!< Status register - Shows SPI status flags
                                  Address offset: 0x08 */
    __vo uint32_t DR;         /*!< Data register - Holds transmitted/received data
                                  Address offset: 0x0C */
    __vo uint32_t CRCPR;      /*!< CRC polynomial register - Configures CRC calculation
                                  Address offset: 0x10 */
    __vo uint32_t RXCRCR;     /*!< RX CRC register - Contains received CRC value
                                  Address offset: 0x14 */
    __vo uint32_t TXCRCR;     /*!< TX CRC register - Contains transmitted CRC value
                                  Address offset: 0x18 */
    __vo uint32_t I2SCFGR;    /*!< I2S configuration register - Configures I2S mode (if supported)
                                  Address offset: 0x1C */
    __vo uint32_t I2SPR;      /*!< I2S prescaler register - Configures I2S clock prescaler
                                  Address offset: 0x20 */
} SPI_RegDef_t;

/**
  * @brief SYSCFG Register Definition Structure
  *
  * @note Contains registers for System Configuration Controller (SYSCFG)
  */
typedef struct
{
    __vo uint32_t MEMRMP;       /*!< Memory remap register - Controls memory mapping
                                    Address offset: 0x00 */
    __vo uint32_t PMC;          /*!< Peripheral mode configuration register - Configures peripheral modes
                                    Address offset: 0x04 */
    __vo uint32_t EXTICR[4];    /*!< External interrupt configuration registers - Maps EXTI lines to GPIO ports
                                    EXTICR[0]: Address offset: 0x08 (EXTI0-3)
                                    EXTICR[1]: Address offset: 0x0C (EXTI4-7)
                                    EXTICR[2]: Address offset: 0x10 (EXTI8-11)
                                    EXTICR[3]: Address offset: 0x14 (EXTI12-15) */
    uint32_t      RESERVED1[2]; /*!< Reserved registers
                                    Address offset: 0x18-0x1C */
    __vo uint32_t CMPCR;        /*!< Compensation cell control register - Controls I/O compensation
                                    Address offset: 0x20 */
    uint32_t      RESERVED2[2]; /*!< Reserved registers
                                    Address offset: 0x24-0x28 */
    __vo uint32_t CFGR;         /*!< Configuration register - Additional system configuration
                                    Address offset: 0x2C */
} SYSCFG_RegDef_t;

/**
  * @brief I2C Register Definition Structure
  *
  * @note Contains all registers for Inter-Integrated Circuit (I2C) peripheral
  */
typedef struct
{
    __vo uint32_t CR1;        /*!< Control register 1 - Configures I2C communication parameters
                                  Address offset: 0x00 */
    __vo uint32_t CR2;        /*!< Control register 2 - Configures clock and interrupts
                                  Address offset: 0x04 */
    __vo uint32_t OAR1;       /*!< Own address register 1 - Configures device's own address
                                  Address offset: 0x08 */
    __vo uint32_t OAR2;       /*!< Own address register 2 - Configures secondary address (if needed)
                                  Address offset: 0x0C */
    __vo uint32_t DR;         /*!< Data register - Holds transmitted/received data
                                  Address offset: 0x10 */
    __vo uint32_t SR1;        /*!< Status register 1 - Shows I2C status flags
                                  Address offset: 0x14 */
    __vo uint32_t SR2;        /*!< Status register 2 - Additional I2C status flags
                                  Address offset: 0x18 */
    __vo uint32_t CCR;        /*!< Clock control register - Configures clock timing
                                  Address offset: 0x1C */
    __vo uint32_t TRISE;      /*!< TRISE register - Configures rise time
                                  Address offset: 0x20 */
    __vo uint32_t FLTR;       /*!< FLTR register - Configures digital noise filter
                                  Address offset: 0x24 */
} I2C_RegDef_t;

/**
  * @brief USART Register Definition Structure
  *
  * @note Contains all registers for Universal Synchronous/Asynchronous Receiver/Transmitter (USART)
  */
typedef struct
{
    __vo uint32_t SR;         /*!< Status register - Shows USART status flags
                                  Address offset: 0x00 */
    __vo uint32_t DR;         /*!< Data register - Holds transmitted/received data
                                  Address offset: 0x04 */
    __vo uint32_t BRR;        /*!< Baud rate register - Configures communication speed
                                  Address offset: 0x08 */
    __vo uint32_t CR1;        /*!< Control register 1 - Configures basic USART parameters
                                  Address offset: 0x0C */
    __vo uint32_t CR2;        /*!< Control register 2 - Configures advanced features
                                  Address offset: 0x10 */
    __vo uint32_t CR3;        /*!< Control register 3 - Configures interrupts and DMA
                                  Address offset: 0x14 */
    __vo uint32_t GTPR;       /*!< Guard time and prescaler register - Configures smartcard mode
                                  Address offset: 0x18 */
} USART_RegDef_t;


/* Bit Manipulation Macros */
#define Set_Bit(bit,position)    (bit |= (1 << position))  /*!< Set specific bit in a register */
#define Reset_Bit(bit,position)  (bit &= ~(1 << position)) /*!< Clear specific bit in a register */
#define isBit_Set(bit,position)  (bit & (1 << position))   /*!< Check if specific bit is set */

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1  				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2  				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3  				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  				((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))



/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCCK_EN()  (RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()  (RCC->APB1ENR |= (1 << 20))
#define USART6_PCCK_EN() (RCC->APB1ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()    	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock Disable Macros for SPIx peripherals
 */

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCCK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCCK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCCK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCCK_DI()  (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCCK_DI()  (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCCK_DI() (RCC->APB1ENR &= ~(1 << 5))

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1 << 13))
/*
 * Clock Disable Macros for SPIx peripherals
 */
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))
/*
 * Clock Disable Macros for SYSCFG peripheral
 */


/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               	do{Set_Bit(RCC->AHB1RSTR,0 ); Reset_Bit(RCC->AHB1RSTR,0);}while(0)
#define GPIOB_REG_RESET()               	do{Set_Bit(RCC->AHB1RSTR,1); Reset_Bit(RCC->AHB1RSTR,1); }while(0)
#define GPIOC_REG_RESET()               	do{Set_Bit(RCC->AHB1RSTR,2); Reset_Bit(RCC->AHB1RSTR,2); }while(0)
#define GPIOD_REG_RESET()               	do{Set_Bit(RCC->AHB1RSTR,3); Reset_Bit(RCC->AHB1RSTR,3); }while(0)
#define GPIOE_REG_RESET()               	do{Set_Bit(RCC->AHB1RSTR,4); Reset_Bit(RCC->AHB1RSTR,4); }while(0)
#define GPIOF_REG_RESET()               	do{Set_Bit(RCC->AHB1RSTR,5); Reset_Bit(RCC->AHB1RSTR,5); }while(0)
#define GPIOG_REG_RESET()               	do{Set_Bit(RCC->AHB1RSTR,6); Reset_Bit(RCC->AHB1RSTR,6); }while(0)
#define GPIOH_REG_RESET()               	do{Set_Bit(RCC->AHB1RSTR,7); Reset_Bit(RCC->AHB1RSTR,7); }while(0)
#define GPIOI_REG_RESET()               	do{Set_Bit(RCC->AHB1RSTR,8); Reset_Bit(RCC->AHB1RSTR,8); }while(0)

/*
 *  Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()              		do{Set_Bit(RCC->AHB2RSTR,12); Reset_Bit(RCC->AHB2RSTR,12);}while(0)
#define SPI2_REG_RESET()              		do{Set_Bit(RCC->AHB1RSTR,14);Reset_Bit(RCC->AHB2RSTR,14);}while(0)
#define SPI3_REG_RESET()              		do{Set_Bit(RCC->AHB1RSTR,15); Reset_Bit(RCC->AHB2RSTR,15);}while(0)
/*
 *  Macros to reset I2Cx peripherals
 */
#define I2C1_REG_RESET()					do{Set_Bit(RCC->AHB1RSTR ,21);Reset_Bit(RCC->AHB1RSTR,21);}while(0)
#define I2C2_REG_RESET()					do{Set_Bit(RCC->AHB1RSTR ,22);Reset_Bit(RCC->AHB1RSTR,22);}while(0)
#define I2C3_REG_RESET()					do{Set_Bit(RCC->AHB1RSTR ,23);Reset_Bit(RCC->AHB1RSTR,23);}while(0)
/*
 *  returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7: \
								        (x == GPIOI)?8:0)


/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI2    2
#define NVIC_IRQ_PRI15   15


//some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET         RESET
#define FLAG_SET 			SET


/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

#include "stm32f446xx_gpio_driver.h"
#include"STM32F446xx_rcc_driver.h"
#include"STM32F446xx_uart_driver.h"
#include"stm32f446xx_SysTick_driver.h"
#include"stm32f446xx_spi_driver.h"
#include"stm32f446xx_i2c_driver.h"
#endif /* STM32F446XX_DRIVER_LAYER_STM32F446XX_H_ */
