/**
 ******************************************************************************
 * @file    stm32f446xx_i2c_driver.h
 * @author  Embedded Galaxy
 * @version V1.0
 * @date    May 12, 2025
 * @brief   This file provides all the necessary configurations, structures,
 *          and function prototypes to manage the I2C peripheral on the
 *          STM32F446xx microcontroller. It includes support for:
 *          - Configuring the I2C peripheral (SCL speed, device address, acknowledgment)
 *          - Data transmission and reception in master and slave modes
 *          - Interrupt handling and state management
 *          - Error detection and handling for common I2C errors
 *
 * @note    The driver requires the STM32F446xx hardware abstraction layer (HAL)
 *          and CMSIS libraries for full functionality. This driver is ideal for
 *          communication between STM32F446xx microcontrollers and other I2C devices.
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

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"

/**
 * @brief  Configuration structure for I2Cx peripheral.
 *         This structure holds the configuration parameters for initializing the I2C peripheral.
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;         /*!< Specifies the I2C SCL speed (Standard Mode, Fast Mode, etc.) */
	uint8_t  I2C_DeviceAddress;    /*!< Specifies the I2C device address (7-bit or 10-bit) */
	uint8_t  I2C_AckControl;       /*!< Enables or disables acknowledgment in I2C communication */
	uint8_t  I2C_FMDutyCycle;      /*!< Specifies the duty cycle for Fast Mode */
} I2C_Config_t;

/**
 * @brief  Handle structure for I2Cx peripheral.
 *         This structure holds the I2C configuration, buffer addresses, and communication state.
 */
typedef struct
{
	I2C_RegDef_t 	*pI2Cx;         /*!< Pointer to the base address of the I2C peripheral */
	I2C_Config_t 	I2C_Config;     /*!< I2C configuration settings */
	uint8_t 		*pTxBuffer;     /*!< Pointer to the transmission buffer */
	uint8_t 		*pRxBuffer;     /*!< Pointer to the reception buffer */
	uint32_t 		TxLen;          /*!< Length of the data to be transmitted */
	uint32_t 		RxLen;          /*!< Length of the data to be received */
	uint8_t 		TxRxState;      /*!< State of the communication (transmission or reception) */
	uint8_t 		DevAddr;        /*!< Address of the I2C slave device */
    uint32_t        RxSize;         /*!< Size of the reception buffer */
    uint8_t         Sr;             /*!< Repeated Start condition flag */
} I2C_Handle_t;

/**
 * @brief  Enumeration for I2C application states.
 */
typedef enum
{
	I2C_READY=0,    /*!< I2C peripheral is ready for communication */
	I2C_BUSY_IN_RX, /*!< I2C peripheral is busy in receiving data */
	I2C_BUSY_IN_TX  /*!< I2C peripheral is busy in transmitting data */
} I2C_State_en;

/**
 * @brief  I2C SCL speed configurations.
 */
#define I2C_SCL_SPEED_SM  	100000     /*!< Standard Mode (100kHz) */
#define I2C_SCL_SPEED_FM4K 	400000     /*!< Fast Mode (400kHz) */
#define I2C_SCL_SPEED_FM2K  200000     /*!< Fast Mode (200kHz) */

/**
 * @brief  I2C Acknowledgment control.
 */
#define I2C_ACK_ENABLE        1  /*!< Enable acknowledgment */
#define I2C_ACK_DISABLE       0  /*!< Disable acknowledgment */

/**
 * @brief  Fast mode duty cycle configuration.
 */
#define I2C_FM_DUTY_2        0  /*!< 2-cycle duty cycle for Fast Mode */
#define I2C_FM_DUTY_16_9     1  /*!< 16/9-cycle duty cycle for Fast Mode */

/**
 * @brief  I2C status flags.
 *         These flags are set by the hardware to indicate the status of I2C operations.
 */
#define I2C_FLAG_TXE   		( 1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE   	( 1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB			( 1 << I2C_SR1_SB)
#define I2C_FLAG_OVR  		( 1 << I2C_SR1_OVR)
#define I2C_FLAG_AF   		( 1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO 		( 1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR 		( 1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF 		( 1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 		( 1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF  		( 1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR 		( 1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 	( 1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET

/**
 * @brief  I2C application events.
 *         These events are generated during I2C communication for monitoring.
 */
#define I2C_EV_TX_CMPLT  	 	0  /*!< Transmission complete event */
#define I2C_EV_RX_CMPLT  	 	1  /*!< Reception complete event */
#define I2C_EV_STOP       		2  /*!< Stop condition event */
#define I2C_ERROR_BERR 	 		3  /*!< Bus error event */
#define I2C_ERROR_ARLO  		4  /*!< Arbitration lost event */
#define I2C_ERROR_AF    		5  /*!< Acknowledge failure event */
#define I2C_ERROR_OVR   		6  /*!< Overrun/Underrun event */
#define I2C_ERROR_TIMEOUT 		7  /*!< Timeout event */
#define I2C_EV_DATA_REQ         8  /*!< Data request event */
#define I2C_EV_DATA_RCV         9  /*!< Data received event */

/******************************************************************************************
 *                              APIs supported by this driver
 * For more information about the APIs, check the function definitions below.
 ******************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Initialization and De-initialization
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);
I2C_State_en I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
I2C_State_en I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

void I2C_SlaveSendData(I2C_RegDef_t *pI2C, const uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

/*
 * IRQ Configuration and ISR Handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Application Callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
