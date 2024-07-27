/*
 * 006_spi_tx_only.c
 *
 *  Created on: Jul 27, 2024
 *      Author: Michael
 */

#include <string.h>
#include "stm32f407xx.h"

/*
 * ******SPI2******
 * MOSI : PB15
 * SCK  : PB13
 * MISO : PB14
 * NSS  : PB12
 * Alt function mode : 5
 ******************/

void delay(void){
	uint32_t i;
	for(i = 0; i < 500000/2; ++i);
}

static void SPI2_GPIO_Init(void){
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunc = AF5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//Configure SPI pins (MOSI and SCK)
	//Configure MOSI pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);

	//Configure SCK pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	//Configure MISO pin
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	//GPIO_Init(&SPIPins);

	//Configure NSS pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);
}

void GPIO_Button_Init(void){
	GPIO_Handle_t GpioBtn;
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);
}

void SPI2_Inits(void){
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	//Note: max speed is fPCLK/2
	SPI2handle.SPIConfig.SPI_SCKSpeed = SPI_SCK_SPEED_DIV8; //Generate 2 MHz SCK
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //Hardware slave management

	SPI_Init(&SPI2handle);
}

int main(void){
	char data[] = "Hello World AYAYAYAYAASAD!";
	uint8_t data_length = strlen(data);

	SPI2_GPIO_Init();

	SPI2_Inits();

	//Configure SSI for Master to Vcc (avoid MODF fault)
	SPI_SSIConfig(SPI2, DISABLE);

	//Configure SSI for Master to Vcc (avoid MODF fault)
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1){

	while(!(GPIO_ReadInputPin(GPIOA, GPIO_PIN_0))); //Wait for button press

		//Necessary for button de-bouncing
		delay();
		//Enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//Send data
		//Send how many bytes
		SPI_SendData(SPI2, &data_length, 1);
		SPI_SendData(SPI2, (uint8_t*)data, strlen(data));

		//Wait for SPI flag to indicate not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//Disable SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}
	return 0;
}
