/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Aug 2, 2024
 *      Author: Michael
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

/*
 * I2C Config Structure
 */
typedef struct
{
	uint32_t I2C_SCLSpeed; /* Values found @I2C_SCLSpeed */
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl; /* Values found @I2C_AckControl */
	uint8_t  I2C_FMDutyCycle; /* Values found @I2C_FMDutyCycle */

}I2C_Config_t;

/*
 *Handle structure for I2C peripheral
 */
typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 		*pTxBuffer; /* Store the Tx buffer address */
	uint8_t 		*pRxBuffer;	/* Store the Rx buffer address */
	uint32_t 		TxLen;
	uint32_t 		RxLen;
	uint8_t 		TxRxState;	/* Store Communication state */
	uint8_t 		DevAddr;	/* Store slave/device address */
    uint32_t        RxSize;
    uint8_t         Sr;			/* !< To store repeated start value  > */
}I2C_Handle_t;

/*
 * I2C application states
 */

#define I2C_RDY 				0
#define I2C_BSY_RX 				1
#define I2C_BSY_TX 				2

/*
 * @I2C_SCLSpeed
 */

#define I2C_SCL_SPEED_SM 	100000
#define I2C_SCL_SPEED_FM4K 	400000
#define I2C_SCL_SPEED_FM2K  200000


/*
 * @I2C_AckControl
 */

#define I2C_ACK_ENABLE        1
#define I2C_ACK_DISABLE       0


/*
 * @I2C_FMDutyCycle
 */

#define I2C_FM_DUTY_2        0
#define I2C_FM_DUTY_16_9     1



/***************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs, check function definitions
 ***************************************************************************/

/*
 * Check if SPI flag is set
 */

uint32_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FlagName);

/*
 * Peripheral Clock Setup
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t en);

/*
 * Init and De-Init
 */

void I2C_Init(I2C_Handle_t *pI2CxHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data sends and receives
 */

/*
 * IRQ Configuration and ISR handling
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQEn);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Other peripheral control APIs
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t en);

/*
 * Application callback
 */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CxHandle, uint8_t AppEvent);


#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
