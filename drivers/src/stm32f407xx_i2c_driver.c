/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Aug 2, 2024
 *      Author: Michael
 */

#include "stm32f407xx.h"

const uint16_t AHB_PRESCALAR[8] = {2,4,8,16,64,128,256,512};
const uint8_t APB1_PRESCALAR[4] = {2,4,8,16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t en);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);


}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}



static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle )
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BSY_RX)
		{
			if(pI2CHandle->RxSize  == 1)
			{
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}

		}
		else
		{
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}

	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}

static void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t en)
{
	if(en == I2C_ACK_ENABLE)
	{
		//enable the ack
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
	}else if (en == I2C_ACK_DISABLE)
	{
		//disable the ack
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}

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

void I2C_DeInit(I2C_RegDef_t *pI2Cx){

}

uint32_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FlagName){
	if(pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_MasterSendData(I2C_Handle_t *pI2CxHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr){
	//1. Generate START Condition
	I2C_GenerateStartCondition(pI2CxHandle->pI2Cx);

	//2. Confirm start generation is complete by checking SB flag in SR1
	//Until SB is cleared, SCL will be stretched
	while(! I2C_GetFlagStatus(pI2CxHandle, I2C_FLAG_SB) );

	//3.Send address of slave with r/nw bit set to w(0) for total 8 bits
	I2C_ExecuteAddressPhaseRead(pI2CxHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in SR1
	while( !  I2C_GetFlagStatus(pI2CxHandle->pI2Cx,I2C_FLAG_ADDR)   );

	//5. clear the ADDR flag according to its software sequence
	//Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CxHandle);

	//6. send the data until len becomes 0

	while(len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CxHandle->pI2Cx,I2C_FLAG_TXE) ); //Wait till TXE is set
		pI2CxHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)

	while(! I2C_GetFlagStatus(pI2CxHandle->pI2Cx,I2C_FLAG_TXE) );

	while(! I2C_GetFlagStatus(pI2CxHandle->pI2Cx,I2C_FLAG_BTF) );


	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_DISABLE_SR )
		I2C_GenerateStopCondition(pI2CxHandle->pI2Cx);
}



