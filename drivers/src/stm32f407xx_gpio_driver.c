/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jul 5, 2024
 *      Author: Michael
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral clock setup
 */

/**************************************************
 * @fcn				- GPIO_PeriClockControl
 *
 * @brief			- This Function either enables or disables the peripheral clock for a given GPIO port
 *
 * @param0			- Base address of GPIO port to enable/disable clock
 * @param1			- ENABLE or DISABLE macro
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t en){
	if(en == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}
	else if(en == DISABLE){
		if(pGPIOx == GPIOA){
				GPIOA_PCLK_DI();
			}
			else if(pGPIOx == GPIOB){
				GPIOB_PCLK_DI();
			}
			else if(pGPIOx == GPIOC){
				GPIOC_PCLK_DI();
			}
			else if(pGPIOx == GPIOD){
				GPIOD_PCLK_DI();
			}
			else if(pGPIOx == GPIOE){
				GPIOE_PCLK_DI();
			}
			else if(pGPIOx == GPIOF){
				GPIOF_PCLK_DI();
			}
			else if(pGPIOx == GPIOG){
				GPIOG_PCLK_DI();
			}
			else if(pGPIOx == GPIOH){
				GPIOH_PCLK_DI();
			}
			else if(pGPIOx == GPIOI){
				GPIOI_PCLK_DI();
			}
	}

}

/**************************************************
 * @fcn				- GPIO_Init
 *
 * @brief			- This Function initializes the GPIO pin
 * @param0			- Handle of GPIO pin to configure GPIO pin settings
 *
 * @return			- none
 *
 * @Note			- none
 *
**************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOxHandle){
	uint32_t temp = 0; //temporary register

	//Enable peripheral clock
	GPIO_PeriClockControl(pGPIOxHandle->pGPIOx, ENABLE);

	//1. configure the mode of gpio pin (make sure it's a valid mode)
	if(pGPIOxHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG && pGPIOxHandle->GPIO_PinConfig.GPIO_PinMode >= GPIO_MODE_IN){
		//non-interrupt mode
		temp = pGPIOxHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOxHandle->pGPIOx->MODER &= ~(0x3 << (2*pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOxHandle->pGPIOx->MODER |= temp; //setting
	}
	//ensure a valid interrupt mode
	else if (pGPIOxHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT){
		//interrupt mode
		if(pGPIOxHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//1. configure FTSR
			EXTI->FTSR |= (1 << pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOxHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//1. configure RTSR
			EXTI->RTSR |= (1 << pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOxHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//1. configure RTSR and FTSR
			EXTI->RTSR |= (1 << pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. configure GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1, temp2, portcode;
		temp1 = pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		temp2 = pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		portcode = GPIO_BASE_ADDR_TO_CODE(pGPIOxHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3. enable EXTI interrupt delivery using IMR (interrupt mask register)
		EXTI->IMR |= (1 << pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	//2. configure the speed (slew rate)
	temp = pGPIOxHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOxHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2*pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOxHandle->pGPIOx->OSPEEDR |= temp; //setting

	temp = 0;

	//3. configure pupd settings
	temp = pGPIOxHandle->GPIO_PinConfig.GPIO_PinPuPdCtrl << (2*pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOxHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOxHandle->pGPIOx->PUPDR |= temp; //setting

	//4. configure the optype
	temp = pGPIOxHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOxHandle->pGPIOx->OTYPER &= ~(0x2 << (pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOxHandle->pGPIOx->OTYPER |= temp; //setting

	temp = 0;

	//5. configure alternate functionality

	if(pGPIOxHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint8_t temp1, temp2;
		temp1 =pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 =pGPIOxHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOxHandle->pGPIOx->AFRL[temp1] &= ~(0xF << (4*temp2)); //clearing
		pGPIOxHandle->pGPIOx->AFRL[temp1] |= pGPIOxHandle->GPIO_PinConfig.GPIO_PinAltFunc << (4*temp2); //setting
	}
}

/**************************************************
 * @fcn				- GPIO_DeInit
 *
 * @brief			- This function de-initializes the corresponding peripheral GPIO
 *
 * @param0			- Pointer of GPIO port to de-initialize the GPIO peripheral
 *
 * @return			- none
 *
 * @Note			- This can be done by writing to RCC AHB1 Reset register
 *
 *************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
			GPIOA_REG_RESET();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_REG_RESET();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_REG_RESET();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_REG_RESET();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_REG_RESET();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_REG_RESET();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_REG_RESET();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_REG_RESET();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_REG_RESET();
		}
}

/**************************************************
 * @fcn				- GPIO_ReadInputPin
 *
 * @brief			- This function reads a single bit at a specific
 *
 * @param0			- Pointer of GPIO port to get corresponding input data register
 * @param1			- Pin number we want to read
 *
 * @return			- Value at pin number (0 or 1)
 *
 * @Note			- none
 *
 */
uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
	uint8_t retVal;
	if(pinNumber >= 0 && pinNumber <= 15){
		retVal = (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x00000001);
	}
	else{
		retVal = -1;
	}
	return retVal;
}

/**************************************************
 * @fcn				- GPIO_ReadInputPort
 *
 * @brief			- This function reads a specified GPIO input port
 *
 * @param0			- Pointer of GPIO port to get corresponding input data register
 *
 * @return			- Value of entire 16-bit GPIO port
 *
 * @Note			- none
 *
 *************************************************/
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx){
		return ((uint16_t) pGPIOx->IDR);
}

/**************************************************
 * @fcn				- GPIO_WriteToOutputPin
 *
 * @brief			- This function writes value to specified pinNumber in GPIOx Port
 *
 * @param0			- Pointer to GPIO port
 * @param1			- Pin number to write to
 * @param2 			- Value to write (0 or 1)
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value){
	if(value == GPIO_PIN_SET){
		pGPIOx->ODR |= (GPIO_PIN_SET <<  pinNumber);
	}
	else if(value == GPIO_PIN_RESET){
		pGPIOx->ODR &= ~(GPIO_PIN_SET <<  pinNumber);
	}
}

/**************************************************
 * @fcn				- GPIO_WriteToOutputPort
 *
 * @brief			- This function writes value to specified GPIOx Port
 *
 * @param0			- Pointer to GPIO port
 * @param1			- Value to write
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
}

/**************************************************
 * @fcn				- GPIO_ToggleOutputPin
 *
 * @brief			- This function toggles a specific output pin of a specified GPIO port
 *
 * @param0			- Pointer to GPIO port
 * @param1			- Pin number to toggle
 *
 * @return			- none
 *
 * @Note			- Value at pin number switches from either a 1 to a 0 or
 * 					  a 0 to a 1
 *
 *************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
	pGPIOx->ODR ^= (1 << pinNumber);
}

/**************************************************
 * @fcn				- GPIO_IRQConfig
 *
 * @brief			- This function configures the IRQ for a GPIO peripheral
 *
 * @param0			- Position in NVIC for interrupt routine
 * @param1			- Enable or disable interrupt
 *
 * @return			- none
 *
 * @Note			- In the STM32F407xx based MCU, the NVIC only goes up to 81.
 * 					  This may be different for other MCUs. Additionally, this
 * 					  is specific for the Cortex-M4 processor.
 *
 *************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQEn){
	if(IRQEn == ENABLE){
		if(IRQNumber >= 0 && IRQNumber <= 31){
			//Program ISER0 Register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber >= 32 && IRQNumber <= 63){
			//Program ISER1 Register
			*NVIC_ISER1 |= (1 << (IRQNumber%32));
		}
		else if(IRQNumber >= 64 && IRQNumber <= 81){
			//Program ISER2 Register
			*NVIC_ISER2 |= (1 << (IRQNumber%64));
		}
	}
	else if(IRQEn == DISABLE){
		if(IRQNumber >= 0 && IRQNumber <= 31){
			//Program ICER0 Register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber >= 32 && IRQNumber <= 63){
			//Program ICER1 Register
			*NVIC_ICER1 |= (1 << (IRQNumber%32));
		}
		else if(IRQNumber >= 64 && IRQNumber <= 81){
			//Program ICER2 Register
			*NVIC_ICER2 |= (1 << (IRQNumber%64));
		}
	}
}

/**************************************************
 * @fcn				- GPIO_IRQPriorityConfig
 *
 * @brief			- This function configures the IRQ priority given an IRQ Number
 *
 * @param0			- Position in NVIC for interrupt routine
 * @param1			- Set the priority for the interrupt
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx_reg, iprx_sect, shift;
	//1. Get priority number register (iprx)
	iprx_reg = IRQNumber / 4;
	//2. Get section in priority number register (iprx) i.e. 0,1,2,3
	iprx_sect = IRQNumber % 4;
	//3. Calculate amount to shift by
	shift = (8 * iprx_sect) + (8 - NO_PR_BITS_IMPLEMENTED);
	//4. Apply value to NVIC priority register
	*(NVIC_PR_BASE_ADDR + iprx_reg) |= (IRQPriority << shift);
}

/**************************************************
 * @fcn				- GPIO_IRQHandling
 *
 * @brief			- This function handles the interrupt request at pin pinNumber
 *
 * @param0			- pinNumber is the pin number at which we clear the pending IRQ
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************/
void GPIO_IRQHandling(uint8_t pinNumber){
	//Clear the EXTI PR register (pending register) corresponding to pin number
	if(EXTI->PR & (1 << pinNumber)){
		//Clear by writing 1 to corresponding bit position
		EXTI->PR |= (1 << pinNumber);
	}
}
