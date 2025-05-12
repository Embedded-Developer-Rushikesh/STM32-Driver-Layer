/********************************************************************************
  * @file    stm32f446xx_uart_driver.h
  * @author  DELL
  * @version V1.0
  * @date    Oct 6, 2024
  * @brief   Header file for STM32F446xx UART driver.
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

#ifndef STM32F446XX_DRIVER_LAYER_STM32F446XX_UART_DRIVER_H_
#define STM32F446XX_DRIVER_LAYER_STM32F446XX_UART_DRIVER_H_

#include "stm32f446xx.h"

/**
 * @brief Configuration structure for USARTx peripheral.
 * This structure holds configuration settings for the USART peripheral.
 */
typedef struct
{
    uint8_t USART_Mode;               /*!< USART operating mode (Transmit, Receive, or Both) */
    uint32_t USART_Baud;              /*!< Baud rate for the USART communication */
    uint8_t USART_NoOfStopBits;       /*!< Number of stop bits (1, 1.5, 2) */
    uint8_t USART_WordLength;         /*!< Word length (8 or 9 bits) */
    uint8_t USART_ParityControl;      /*!< Parity control (None, Odd, Even) */
    uint8_t USART_HWFlowControl;      /*!< Hardware flow control (None, CTS, RTS, CTS/RTS) */
} USART_Config_t;

/**
 * @brief Handle structure for USARTx peripheral.
 * This structure is used to manage the state and communication buffers of the USART peripheral.
 */
typedef struct
{
    USART_RegDef_t *pUSARTx;           /*!< Pointer to USART register structure */
    USART_Config_t USART_Config;       /*!< Configuration settings for USART */
    uint8_t *pTxBuffer;                /*!< Pointer to transmit data buffer */
    uint8_t *pRxBuffer;                /*!< Pointer to receive data buffer */
    uint32_t TxLen;                    /*!< Length of the data to be transmitted */
    uint32_t RxLen;                    /*!< Length of the data to be received */
    uint8_t TxBusyState;               /*!< State flag indicating if TX is busy */
    uint8_t RxBusyState;               /*!< State flag indicating if RX is busy */
} USART_Handle_t;

/**
 * @brief USART Mode options.
 */
#define USART_MODE_ONLY_TX     0
#define USART_MODE_ONLY_RX     1
#define USART_MODE_TXRX        2

/**
 * @brief USART Baud Rate options.
 */
#define USART_STD_BAUD_1200     1200
#define USART_STD_BAUD_2400     2400
#define USART_STD_BAUD_9600     9600
#define USART_STD_BAUD_19200    19200
#define USART_STD_BAUD_38400    38400
#define USART_STD_BAUD_57600    57600
#define USART_STD_BAUD_115200   115200
#define USART_STD_BAUD_230400   230400
#define USART_STD_BAUD_460800   460800
#define USART_STD_BAUD_921600   921600
#define USART_STD_BAUD_2M       2000000
#define USART_STD_BAUD_3M       3000000

/**
 * @brief USART Parity Control options.
 */
#define USART_PARITY_EN_ODD     2
#define USART_PARITY_EN_EVEN    1
#define USART_PARITY_DISABLE     0

/**
 * @brief USART Word Length options.
 */
#define USART_WORDLEN_8BITS     0
#define USART_WORDLEN_9BITS     1

/**
 * @brief USART Number of Stop Bits options.
 */
#define USART_STOPBITS_1        0
#define USART_STOPBITS_0_5      1
#define USART_STOPBITS_2        2
#define USART_STOPBITS_1_5      3

/**
 * @brief USART Hardware Flow Control options.
 */
#define USART_HW_FLOW_CTRL_NONE     0
#define USART_HW_FLOW_CTRL_CTS      1
#define USART_HW_FLOW_CTRL_RTS      2
#define USART_HW_FLOW_CTRL_CTS_RTS  3

/**
 * @brief USART flag definitions.
 */
#define USART_FLAG_TXE          ( 1 << USART_SR_TXE)  /*!< TXE flag in USART_SR */
#define USART_FLAG_RXNE         ( 1 << USART_SR_RXNE) /*!< RXNE flag in USART_SR */
#define USART_FLAG_TC           ( 1 << USART_SR_TC)   /*!< TC flag in USART_SR */

/**
 * @brief Application states.
 */
#define USART_BUSY_IN_RX        1
#define USART_BUSY_IN_TX        2
#define USART_READY             0

/**
 * @brief USART event definitions.
 */
#define USART_EVENT_TX_CMPLT    0
#define USART_EVENT_RX_CMPLT    1
#define USART_EVENT_IDLE        2
#define USART_EVENT_CTS         3
#define USART_EVENT_PE          4
#define USART_ERR_FE            5
#define USART_ERR_NE            6
#define USART_ERR_ORE           7

/******************************************************************************************
 *                          APIs supported by this driver
 *            For more information about the APIs check the function definitions
 ******************************************************************************************/

/**
 * @brief Configures the peripheral clock for the specified USART peripheral.
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

/**
 * @brief Initializes the USART peripheral with the configuration provided in the handle.
 */
void USART_Init(USART_Handle_t *pUSARTHandle);

/**
 * @brief De-initializes the USART peripheral and resets the registers.
 */
void USART_DeInit(USART_Handle_t *pUSARTHandle);

/**
 * @brief Sends data using the USART peripheral.
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);

/**
 * @brief Receives data using the USART peripheral.
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/**
 * @brief Sends data using USART in interrupt mode.
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);

/**
 * @brief Receives data using USART in interrupt mode.
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/**
 * @brief Configures the IRQ interrupt for USART.
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);

/**
 * @brief Sets the priority for the USART IRQ.
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * @brief Handles USART IRQ interrupt and manages event callbacks.
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/**
 * @brief Gets the status of a specific flag in the USART SR register.
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);

/**
 * @brief Clears the specified USART flag.
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

/**
 * @brief Enables or disables the USART peripheral.
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

/**
 * @brief Sets the baud rate for the USART peripheral.
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/**
 * @brief Application callback function to handle USART events.
 */
extern void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApEv);

#endif /* STM32F446XX_DRIVER_LAYER_STM32F446XX_UART_DRIVER_H_ */
