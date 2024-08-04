/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Aug 2, 2024
 *      Author: Michael
 */

#include "stm32f407xx.h"

const uint16_t AHB_PRESCALAR[8] = {2,4,8,16,64,128,256,512};
const uint8_t APB1_PRESCALAR[4] = {2,4,8,16};

uint32_t RCC_GetPLLOPClk(void){
	return 1;
}

/**************************************************
 * @fcn				- RCC_GetPCLK1
 *
 * @brief			- This Function returns the calculated value for PCLK1
 *
 * @return			- none
 *
 * @Note			- none
 *
**************************************************/
uint32_t RCC_GetPCLK1(void){
	uint32_t pclk1, sysclk, apb1_prescalar;
	uint16_t ahb_prescalar;
	uint8_t clksrc;
	//Get only bits 2 and 3 of RCC CFGR register
	clksrc = (RCC->CFGR >> 2) & 0x3;

	//if HSI
	if(clksrc == 0){
		sysclk = 16000000;
	}
	//else if HSE
	else if(clksrc == 1){
		sysclk = 8000000;
	}
	//else if PLLL (not implemented)
	else if(clksrc == 2){
		sysclk = RCC_GetPLLOPClk();
	}

	clksrc = (RCC->CFGR >> 4)& 0xF;

	if(clksrc < 8){
		ahb_prescalar = 0;
	}
	else{
		ahb_prescalar = AHB_PRESCALAR[clksrc - 8];
	}

	clksrc = (RCC->CFGR >> 10)& 0x7;

	if(clksrc < 4){
		apb1_prescalar = 1;
	}
	else{
		apb1_prescalar = APB1_PRESCALAR[clksrc - 4];
	}

	pclk1 = (sysclk/ahb_prescalar)/apb1_prescalar;
	return pclk1;
}


/**************************************************
 * @fcn				- I2C_Init
 *
 * @brief			- This Function initializes the I2C pin
 * @param0			- Handle of I2C pin to configure I2C pin settings
 *
 * @return			- none
 *
 * @Note			- none
 *
**************************************************/

void I2C_Init(I2C_Handle_t *pI2CxHandle){
	uint32_t tempreg = 0;
	uint16_t ccr_val;

	//configure ACK control bit
	tempreg = (pI2CxHandle->I2C_Config.I2C_AckControl << 10);
	pI2CxHandle->pI2Cx->CR1 = tempreg;

	//configure FREQ field for CR2
	tempreg = 0;

	tempreg = RCC_GetPCLK1() / 1000000U;
	//Only apply to LS 6 bits
	pI2CxHandle->pI2Cx->CR2 = tempreg & 0x3F;

	//Program device own address
	tempreg = (pI2CxHandle->I2C_Config.I2C_DeviceAddress << 1);

	//Required reserved bit to be kept at 1
	tempreg |= (1 << 14);
	pI2CxHandle->pI2Cx->OAR1 = tempreg;

	//CCR Calculations
	ccr_val = 0;
	tempreg = 0;
	if(pI2CxHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//configure standard mode
		ccr_val = RCC_GetPCLK1() / (2 * pI2CxHandle->I2C_Config.I2C_SCLSpeed);
	}
	else{
		//configure fast mode
		tempreg = (1 << 15);
		tempreg |= (pI2CxHandle->I2C_Config.I2C_FMDutyCycle);
		if(pI2CxHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_val = RCC_GetPCLK1() / (3 * pI2CxHandle->I2C_Config.I2C_SCLSpeed);
		}
		else{
			ccr_val = RCC_GetPCLK1() / (25 * pI2CxHandle->I2C_Config.I2C_SCLSpeed);
		}
	}
	tempreg |= (ccr_val & 0xFFF);

	pI2CxHandle->pI2Cx->CCR = tempreg;

}

void I2C_DeInit(I2C_RegDef_t *pI2Cx);
