/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jul 5, 2024
 *      Author: Michael
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Structure to store GPIO ping configuration settings
 */
typedef struct{
	uint8_t GPIO_PinNumber; /* Possible values from @GPIO_PIN_NUMBERS */
	uint8_t GPIO_PinMode; /* Possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed; /* Possible values from @GPIO_PIN_SPEED */
	uint8_t GPIO_PinPuPdCtrl; /* Possible values from @GPIO_PUPD_CTRL */
	uint8_t GPIO_PinOPType; /* Possible values from @GPIO_PIN_OP_TYPE */
	uint8_t GPIO_PinAltFunc; /* Possible values from @GPIO_ALT_FUNCTIONS */
}GPIO_PinConfig_t;

/*
 * Handle structure for a GPIO pin
 */

typedef struct{
	GPIO_RegDef_t *pGPIOx; /* Hold base address of GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig; /* Hold GPIO pin configuration settings */

}GPIO_Handle_t;


/* @GPIO_PIN_NUMBERS
 * GPIO Pin Numbers
 */
#define GPIO_PIN_0 	0
#define GPIO_PIN_1 	1
#define GPIO_PIN_2 	2
#define GPIO_PIN_3 	3
#define GPIO_PIN_4 	4
#define GPIO_PIN_5 	5
#define GPIO_PIN_6 	6
#define GPIO_PIN_7 	7
#define GPIO_PIN_8 	8
#define GPIO_PIN_9 	9
#define GPIO_PIN_10 10
#define GPIO_PIN_11 11
#define GPIO_PIN_12 12
#define GPIO_PIN_13 13
#define GPIO_PIN_14 14
#define GPIO_PIN_15 15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */

#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG	3

#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_PIN_OP_TYPE
 * GPIO pin possible output types
 */

#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible speed modes
 */

#define GPIO_SPEED_LOW 			0
#define GPIO_SPEED_MEDIUM 		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/*
 * @GPIO_PUPD_CTRL
 * GPIO pull up/pull down configuration macros
 */

#define GPIO_PIN_NO_PUPD 			0
#define GPIO_PIN_PU			 		1
#define GPIO_PIN_PD					2

/*
 * @GPIO_ALT_FUNCTIONS
 * GPIO Alternate Function Macros
 */

#define AF0 	0
#define AF1 	1
#define AF2 	2
#define AF3 	3
#define AF4 	4
#define AF5 	5
#define AF6 	6
#define AF7 	7
#define AF8 	8
#define AF9 	9
#define AF10 	10
#define AF11 	11
#define AF12 	12
#define AF13 	13
#define AF14 	14
#define AF15 	15

/***************************************************************************
 * 						APIs supported by this driver
 * 		For more information about the APIs, check function definitions
 ***************************************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t en);

/*
 * Init and De-Init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOxHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
/*
 * Data reads and writes
 */


uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/*
 * IRQ Configuration and ISR handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQEn);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t pinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
