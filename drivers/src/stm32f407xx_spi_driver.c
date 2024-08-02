/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jul 14, 2024
 *      Author: Michael
 */

#include "stm32f407xx_spi_driver.h"

/*
 * Helper functions to handle different SPI interrupts
 */

static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIxHandle){
		//If 16-bit data frame
		if((pSPIxHandle->pSPIx->CR1>>SPI_CR1_DFF) & 1){
			pSPIxHandle->pSPIx->DR = *((uint16_t*) pSPIxHandle->pTxBuffer);
			pSPIxHandle->TxLen -= 2;
			(uint16_t*) pSPIxHandle->pTxBuffer++;
		}
		//Otherwise (8-bit data frame)
		else{
			//Load data register with information from pTxBuffer
			pSPIxHandle->pSPIx->DR = (uint32_t) pSPIxHandle->pTxBuffer;
			pSPIxHandle->TxLen--;
			pSPIxHandle->pTxBuffer++;
		}
		if(pSPIxHandle->TxLen == 0){
			//if TxLen is 0, transmission is done
			SPI_CloseTransmission(pSPIxHandle);
		}
}

static void spi_rxe_interrupt_handler(SPI_Handle_t *pSPIxHandle){
	//If 16-bit data frame
	if((pSPIxHandle->pSPIx->CR1>>SPI_CR1_DFF) & 1){
		pSPIxHandle->pSPIx->DR = *((uint16_t*) pSPIxHandle->pRxBuffer);
		pSPIxHandle->RxLen -= 2;
		(uint16_t*) pSPIxHandle->pRxBuffer++;
	}
	//Otherwise (8-bit data frame)
	else{
		//Load data register with information from pRxBuffer
		pSPIxHandle->pSPIx->DR = (uint32_t) pSPIxHandle->pRxBuffer;
		pSPIxHandle->RxLen--;
		pSPIxHandle->pRxBuffer++;
	}
	if(pSPIxHandle->RxLen == 0){
		//if RxLen is 0, transmission is done
		SPI_CloseReception(pSPIxHandle);
		SPI_ApplicationEventCallback(pSPIxHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handler(SPI_Handle_t *pSPIxHandle){
	uint8_t temp;
	//1. Clear OVR flag
	if(pSPIxHandle->TxState != SPI_BSY_TX){
		SPI_ClearOVRFlag(pSPIxHandle->pSPIx);
	}
	(void) temp;
	//2. Inform application
	SPI_ApplicationEventCallback(pSPIxHandle, SPI_EVENT_OVR_ERR_CMPLT);
}

/**************************************************
 * @fcn				- SPI_GetFlagStatus
 *
 * @brief			- This Function returns whether the flag is set (1 << FlagName) or not (0)
 *
 * @param0			- Base address of SPI peripheral
 * @param1			- Flag to check
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************/
uint32_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Peripheral clock setup
 */

/**************************************************
 * @fcn				- GPIO_PeriClockControl
 *
 * @brief			- This Function either enables or disables the peripheral clock for a given GPIO port
 *
 * @param0			- Base address of SPI peripheral to enable/disable clock
 * @param1			- ENABLE or DISABLE macro
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t en){
	if(en == ENABLE){
			if(pSPIx == SPI1){
				SPI1_PCLK_EN();
			}
			else if(pSPIx == SPI2){
				SPI2_PCLK_EN();
			}
			else if(pSPIx == SPI3){
				SPI3_PCLK_EN();
			}
		}
		else if(en == DISABLE){
			if(pSPIx == SPI1){
				SPI1_PCLK_DI();
			}
			else if(pSPIx == SPI2){
				SPI2_PCLK_DI();
			}
			else if(pSPIx == SPI3){
				SPI3_PCLK_DI();
			}
		}
}

/**************************************************
 * @fcn				- SPI_Init
 *
 * @brief			- This Function initializes the SPI peripheral
 * @param0			- Handle of SPI handle to configure SPI peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
**************************************************/
void SPI_Init(SPI_Handle_t *pSPIxHandle){
	uint32_t tempreg = 0;

	//Configure peripheral clock
	SPI_PeriClockControl(pSPIxHandle->pSPIx, ENABLE);

	//Configure SPI_CR1 Register

	//1. Configure SPI Device Mode (slave or master)
	tempreg |= (pSPIxHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	//2. Configure Bus Config (refer to @SPI_Bus_Config for possible configurations)
	if(pSPIxHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//Bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIxHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//Bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIxHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SRX){
		//Bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//Rx only bit should be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3. Configure SCK speed
	tempreg |= (pSPIxHandle->SPIConfig.SPI_SCKSpeed << SPI_CR1_BR);

	//4.Configure SPI DFF (8 bits or 16 bits)
	tempreg |= (pSPIxHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//5.Configure CPOL
	tempreg |= (pSPIxHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//6.Configure CPHA
	tempreg |= (pSPIxHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	//6.Configure SSM
	tempreg |= (pSPIxHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	pSPIxHandle->pSPIx->CR1 = tempreg;
}

/**************************************************
 * @fcn				- SPI_DeInit
 *
 * @brief			- This function de-initializes the SPI peripheral
 * @param0			- Handle of SPI handle to configure SPI peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
**************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}
}

/**************************************************
 * @fcn				- SPI_SendData
 *
 * @brief			- This function sends data through the SPI peripheral
 * @param0			- Pointer to SPIx peripheral memory address
 * @param1			- Pointer to Tx Buffer
 * @param2			- Pointer to 32-bit integer to specify number of bytes to send
 *
 * @return			- none
 *
 * @Note			- This is a blocking function (non-interrupt based)
 * 					- We are using polling
 *
**************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len){
	while(len > 0){
		//Wait for Transmit buffer to be empty
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//If 16-bit data frame
		if((pSPIx->CR1>>SPI_CR1_DFF) & 1){
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			len -= 2;
			(uint16_t*) pTxBuffer++;
		}
		//Otherwise (8-bit data frame)
		else{
			//Load data register with information from pTxBuffer
			pSPIx->DR = *pTxBuffer;
			pTxBuffer++;
		}
		len--;
	}
}

/**************************************************
 * @fcn				- SPI_ReceieveData
 *
 * @brief			- This function receieves data through the SPI peripheral
 * @param0			- Pointer to SPIx peripheral memory address
 * @param1			- Pointer to Rx Buffer
 * @param2			- Pointer to 32-bit integer to specify number of bytes to send
 *
 * @return			- none
 *
 * @Note			- This is a blocking function (non-interrupt based)
 * 					- We are using polling
 *
**************************************************/
void SPI_ReceieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len){
	while(len > 0){
		//Wait for RXNE to be set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//If 16-bit data frame
		if((pSPIx->CR1>>SPI_CR1_DFF) & 1){
			//Read data from SPI DR into *pRxBuffer
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			len -= 2;
			(uint16_t*) pRxBuffer++;
		}
		//Otherwise (8-bit data frame)
		else{
			//Read data from SPI DR into *pRxBuffer
			*pRxBuffer = pSPIx->DR;
			pRxBuffer++;
		}
		len--;
	}
}

/**************************************************
 * @fcn				- SPI_PeripheralControl
 *
 * @brief			- This function enables or disable SPI peripheral
 * @param0			- Pointer to SPIx peripheral memory address
 * @param1			- Enable or disable SPIx peripheral
 * @param2			- Pointer to 32-bit integer to specify number of bytes to send
 *
 * @return			- none
 *
 * @Note			- Only enable SPI peripheral once initialization is complete
 * 					- Otherwise, writes to SPI CRs will be invalid
 *
**************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t en){
	if(en == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else if(en == DISABLE){
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}

}

/**************************************************
 * @fcn				- SPI_SSIConfig
 *
 * @brief			- This function enables or disable SPI SSI (used for software NSS)
 * @param0			- Pointer to SPIx peripheral memory address
 * @param1			- Set SSI low or high
 * @param2			- Pointer to 32-bit integer to specify number of bytes to send
 *
 * @return			- none
 *
 * @Note			- Only enable SPI peripheral once initialization is complete
 * 					- Otherwise, writes to SPI CRs will be invalid
 *
**************************************************/

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t en){
	if(en == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else if(en == DISABLE){
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/**************************************************
 * @fcn				- SPI_SSOEConfig
 *
 * @brief			- This function enables or disable SPI SSOE (used for software NSS)
 * @param0			- Pointer to SPIx peripheral memory address
 * @param1			- Set SSOE low or high
 * @param2			- Pointer to 32-bit integer to specify number of bytes to send
 *
 * @return			- none
 *
 * @Note			- Only enable SPI peripheral once initialization is complete
 * 					- Otherwise, writes to SPI CRs will be invalid
 *
**************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t en){
	if(en == ENABLE){
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else if(en == DISABLE){
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


/**************************************************
 * @fcn				- SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
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
 * @fcn				- SPI_SendDataINT
 *
 * @brief			- This function handles INT config. for sending data
 *
 * @param0			- pSPIxHandle is the pointer to the SPI handle
 * @param1			- pTxBuffer is the pointer to the buffer of data to send
 * @param2			- len is the length of data we want to send
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************/
uint8_t SPI_SendDataINT(SPI_Handle_t *pSPIxHandle, uint8_t *pTxBuffer, uint32_t len){
	if(pSPIxHandle->TxState != SPI_BSY_RX){
		//Save Tx buffer address in handle structure
		pSPIxHandle->pTxBuffer = pTxBuffer;
		pSPIxHandle->TxLen = len;

		//Mark Tx state as busy
		pSPIxHandle->TxState = SPI_BSY_TX;

		//Enable TXEIE to enable interrupt when TXE flag is set
		pSPIxHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE);

		//Data transmission handled by ISR
	}
	return pSPIxHandle->TxState;
}

/**************************************************
 * @fcn				- SPI_ReceieveDataINT
 *
 * @brief			- This function handles INT config. for receiving data
 *
 * @param0			- pSPIxHandle is the pointer to the SPI handle
 * @param1			- pTxBuffer is the pointer to the buffer of data to send
 * @param2			- len is the length of data we want to send
 *
 * @return			- none
 *
 * @Note			- none
 *
 *************************************************/
uint8_t SPI_ReceieveDataINT(SPI_Handle_t *pSPIxHandle, uint8_t *pRxBuffer, uint32_t len){
	if(pSPIxHandle->RxState != SPI_BSY_RX){
		//Save Tx buffer address in handle structure
		pSPIxHandle->pRxBuffer = pRxBuffer;
		pSPIxHandle->RxLen = len;

		//Mark Tx state as busy
		pSPIxHandle->TxState = SPI_BSY_RX;

		//Enable TXEIE to enable interrupt when TXE flag is set
		pSPIxHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE);

		//Data transmission handled by ISR
	}
	return pSPIxHandle->RxState;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIxHandle){
	uint8_t temp1, temp2;
	//Check for TXE from interrupt
	temp1 = ((pSPIxHandle->pSPIx->SR >> SPI_SR_TXE) & 0x1);
	temp2 = ((pSPIxHandle->pSPIx->CR2 >> SPI_CR2_TXEIE) & 0x1);

	if(temp1 && temp2){
		//handle TXE from interrupt
		spi_txe_interrupt_handler(pSPIxHandle);
	}

	//Check for RXNE from interrupt
	temp1 = ((pSPIxHandle->pSPIx->SR >> SPI_SR_RXNE) & 0x1);
	temp2 = ((pSPIxHandle->pSPIx->CR2 >> SPI_CR2_RXNEIE) & 0x1);

	if(temp1 && temp2){
		//handle RXNE from interrupt
		spi_rxe_interrupt_handler(pSPIxHandle);
	}

	//Check for RXNE from interrupt
	temp1 = ((pSPIxHandle->pSPIx->SR >> SPI_SR_OVR) & 0x1);
	temp2 = ((pSPIxHandle->pSPIx->CR2 >> SPI_CR2_ERRIE) & 0x1);

	if(temp1 && temp2){
		//handle OVRERR from interrupt
		spi_ovr_err_interrupt_handler(pSPIxHandle);
	}


	/*
	 * @TODO: Implement for other ERRIE
	 */

}



void SPI_CloseTransmission(SPI_Handle_t *pSPIxHandle){
	//clears TXEIE flag
	pSPIxHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	//Set pTxBuffer to NULL (no more TxBuffer because transmission complete)
	pSPIxHandle->pTxBuffer = NULL;
	pSPIxHandle->TxLen = 0;
	pSPIxHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIxHandle){
	//clears RXNEIE flag
	pSPIxHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	//Set pRxBuffer to NULL (no more RxBuffer because Reception complete)
	pSPIxHandle->pRxBuffer = NULL;
	pSPIxHandle->RxLen = 0;
	pSPIxHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	//clear flags
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void) temp;

}


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIxHandle, uint8_t AppEvent){
	//Weak implementation. The application may override this function
}
