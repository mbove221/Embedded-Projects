/*
 * stm32f407_spi_driver.h
 *
 *  Created on: Jul 14, 2024
 *      Author: Michael
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct{
	uint8_t SPI_DeviceMode; /* Possible values from @SPI_Device_Modes */
	uint8_t SPI_BusConfig; /* Possible values from @SPI_Bus_Config */
	uint8_t SPI_SCKSpeed;/* Possible values from @SPI_SCK_Speed */
	uint8_t SPI_DFF;	/* Possible values from @SPI_DFF */
	uint8_t SPI_CPOL; /* Possible values from @SPI_CPOL */
	uint8_t SPI_CPHA; /* Possible values from @SPI_CPHA */
	uint8_t SPI_SSM; /* Possible values from @SPI_SSM */

}SPI_Config_t;

/*
 * SPIx peripheral handle structure
 */
typedef struct{
	SPI_RegDef_t *pSPIx; /* Hold base address of SPIx peripheral */
	SPI_Config_t SPIConfig; /* Hold SPI configuration settings */
	uint8_t		 *pTxBuffer;
	uint8_t 	 *pRxBuffer;
	uint32_t	 TxLen;
	uint32_t	 RxLen;
	uint8_t 	 TxState;	/* Possible values @SPI_STATES */
	uint8_t		 RxState;

}SPI_Handle_t;

/*
 * @SPI_STATES
 * SPI possible states
 */
#define SPI_READY  0
#define SPI_BSY_RX 1
#define SPI_BSY_TX 2

/*
 * @SPI_Device_Modes
 * SPI possible modes
 */

#define SPI_DEVICE_MODE_SLAVE 0
#define SPI_DEVICE_MODE_MASTER 1

/*
 * @SPI_Bus_Config
 * SPI possible modes
 */

#define SPI_BUS_CONFIG_FD		1	/* Full Duplex */
#define SPI_BUS_CONFIG_HD		2	/* Half Duplex */
#define SPI_BUS_CONFIG_SRX		3 	/* Simplex (Rx only) */

/*
 * @SPI_SCK_Speed
 * SPI Serial Clock Speed selection
 */

#define SPI_SCK_SPEED_DIV2		0
#define SPI_SCK_SPEED_DIV4		1
#define SPI_SCK_SPEED_DIV8		2
#define SPI_SCK_SPEED_DIV16		3
#define SPI_SCK_SPEED_DIV32		4
#define SPI_SCK_SPEED_DIV64		5
#define SPI_SCK_SPEED_DIV128	6
#define SPI_SCK_SPEED_DIV256	7

/*
 * @SPI_DFF
 * SPI Data frame format (8 bit or 16 bits)
 */
#define SPI_DFF_8BITS 	0
#define SPI_DFF_16BITS 	1

/*
 * @SPI_CPOL
 * SPI Clock Polarity
 */

#define SPI_CPOL_LOW 	0
#define SPI_CPOL_HIGH 	1

/*
 * @SPI_CPHA
 * SPI Clock Phase
 */

#define SPI_CPHA_LOW 	0
#define SPI_CPHA_HIGH 	1

/*
 * @SPI_SSM
 * Software slave management
 */

#define SPI_SSM_DI		0
#define SPI_SSM_EN		1

/*
 * SPI related flag information
 * TODO: Implement macros for all SPI Status Register Flags
 */

#define SPI_TXE_FLAG	(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG	(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG	(1 << SPI_SR_BSY)

/***************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs, check function definitions
 ***************************************************************************/

/*
 * Check if SPI flag is set
 */
uint32_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName);

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t en);

/*
 * Init and De-Init
 */


void SPI_Init(SPI_Handle_t *pSPIxHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data sends and receives
 */

/*
 * Non-interrupt based
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/*
 * Interrupt based
 */
uint8_t SPI_SendDataINT(SPI_Handle_t *pSPIxHandle, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveDataINT(SPI_Handle_t *pSPIxHandle, uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ Configuration and ISR handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQEn);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(uint8_t pinNumber);

/*
 * Other peripheral control APIs
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t en);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t en);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t en);


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
