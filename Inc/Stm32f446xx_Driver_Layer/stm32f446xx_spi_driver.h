/**
 *******************************************************************************
 * @file    stm32f446xx_spi_driver.h
 * @author  Embedded Galaxy
 * @version V1.0
 * @date    May 12, 2025
 * @brief   This file provides all the necessary configurations, structures,
 *          and function prototypes to manage the SPI peripheral on the
 *          STM32F446xx microcontroller. It includes support for:
 *          - Configuring the SPI peripheral (mode, baud rate, data width)
 *          - Data transmission and reception in master and slave modes
 *          - Interrupt handling and state management
 *          - Error detection and handling for common SPI errors
 *
 * @note    The driver requires the STM32F446xx hardware abstraction layer (HAL)
 *          and CMSIS libraries for full functionality. This driver is ideal for
 *          communication between STM32F446xx microcontrollers and other SPI devices.
 *******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 *******************************************************************************
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/**
 * @brief  Configuration structure for SPIx peripheral.
 *         This structure holds the configuration parameters for initializing the SPI peripheral.
 */
typedef struct
{
	uint8_t SPI_DeviceMode;        /*!< Specifies the SPI device mode (Master/Slave) */
	uint8_t SPI_BusConfig;         /*!< Specifies the SPI bus configuration (Full-Duplex, Half-Duplex, Simplex) */
	uint8_t SPI_SclkSpeed;         /*!< Specifies the SCK speed (Baud rate prescaler) */
	uint8_t SPI_DFF;               /*!< Specifies the Data Frame Format (8-bit or 16-bit) */
	uint8_t SPI_CPOL;              /*!< Specifies the Clock Polarity (Idle state of the clock) */
	uint8_t SPI_CPHA;              /*!< Specifies the Clock Phase (Data valid edge) */
	uint8_t SPI_SSM;               /*!< Enables or disables software slave management */
} SPI_Config_t;

/**
 * @brief  Handle structure for SPIx peripheral.
 *         This structure holds the SPI configuration, buffer addresses, and communication state.
 */
typedef struct
{
	SPI_RegDef_t 	*pSPIx;          /*!< Pointer to the base address of the SPI peripheral */
	SPI_Config_t 	SPIConfig;      /*!< SPI configuration settings */
	uint8_t 		*pTxBuffer;      /*!< Pointer to the transmission buffer */
	uint8_t 		*pRxBuffer;      /*!< Pointer to the reception buffer */
	uint32_t 		TxLen;           /*!< Length of the data to be transmitted */
	uint32_t 		RxLen;           /*!< Length of the data to be received */
	uint8_t 		TxState;         /*!< State of the transmission */
	uint8_t 		RxState;         /*!< State of the reception */
} SPI_Handle_t;

/**
 * @brief  Enumeration for SPI application states.
 */
typedef enum
{
	SPI_READY=0,     /*!< SPI peripheral is ready for communication */
	SPI_BUSY_IN_RX,  /*!< SPI peripheral is busy receiving data */
	SPI_BUSY_IN_TX   /*!< SPI peripheral is busy transmitting data */
} SPI_State_en;

/**
 * @brief  SPI Device Modes.
 */
#define SPI_DEVICE_MODE_MASTER   1  /*!< Master mode */
#define SPI_DEVICE_MODE_SLAVE    0  /*!< Slave mode */

/**
 * @brief  SPI Bus Configurations.
 */
#define SPI_BUS_CONFIG_FD                 1  /*!< Full-Duplex communication */
#define SPI_BUS_CONFIG_HD                 2  /*!< Half-Duplex communication */
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY     3  /*!< Simplex communication (Receive only) */

/**
 * @brief  SPI Clock Speed (Baud Rate Prescaler).
 */
#define SPI_SCLK_SPEED_DIV2             0  /*!< Clock Speed Prescaler / 2 */
#define SPI_SCLK_SPEED_DIV4             1  /*!< Clock Speed Prescaler / 4 */
#define SPI_SCLK_SPEED_DIV8             2  /*!< Clock Speed Prescaler / 8 */
#define SPI_SCLK_SPEED_DIV16            3  /*!< Clock Speed Prescaler / 16 */
#define SPI_SCLK_SPEED_DIV32            4  /*!< Clock Speed Prescaler / 32 */
#define SPI_SCLK_SPEED_DIV64            5  /*!< Clock Speed Prescaler / 64 */
#define SPI_SCLK_SPEED_DIV128           6  /*!< Clock Speed Prescaler / 128 */
#define SPI_SCLK_SPEED_DIV256           7  /*!< Clock Speed Prescaler / 256 */

/**
 * @brief  SPI Data Frame Formats.
 */
#define SPI_DFF_8BITS    0  /*!< 8-bit data frame */
#define SPI_DFF_16BITS   1  /*!< 16-bit data frame */

/**
 * @brief  SPI Clock Polarity.
 */
#define SPI_CPOL_HIGH    1  /*!< Idle High clock polarity */
#define SPI_CPOL_LOW     0  /*!< Idle Low clock polarity */

/**
 * @brief  SPI Clock Phase.
 */
#define SPI_CPHA_HIGH    1  /*!< Data is valid on the second edge of the clock */
#define SPI_CPHA_LOW     0  /*!< Data is valid on the first edge of the clock */

/**
 * @brief  SPI Software Slave Management.
 */
#define SPI_SSM_EN        1  /*!< Enable software slave management */
#define SPI_SSM_DI        0  /*!< Disable software slave management */

/**
 * @brief  SPI status flags.
 *         These flags are set by hardware to indicate the status of SPI operations.
 */
#define SPI_TXE_FLAG     (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG    (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG    (1 << SPI_SR_BSY)

/**
 * @brief  SPI application events.
 *         These events are generated during SPI communication for monitoring.
 */
#define SPI_EVENT_TX_CMPLT   1  /*!< Transmission complete event */
#define SPI_EVENT_RX_CMPLT   2  /*!< Reception complete event */
#define SPI_EVENT_OVR_ERR    3  /*!< Overrun error event */
#define SPI_EVENT_CRC_ERR    4  /*!< CRC error event */

/******************************************************************************************
 *                              APIs supported by this driver
 * For more information about the APIs, check the function definitions below.
 ******************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Initialization and De-initialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, const uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
SPI_State_en SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
SPI_State_en SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application Callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
