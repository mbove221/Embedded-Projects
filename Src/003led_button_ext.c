/*
 * 001LEDToggle.c
 *
 *  Created on: Jul 7, 2024
 *      Author: Michael
 */

#include "stm32f407xx_gpio_driver.h"

#define LOW 0
#define HIGH 1

#define BTN_PRESSED

void delay(void){
	uint32_t i;
	for(i = 0; i < 500000/2; ++i);
}

int main(void){
	GPIO_Handle_t GpioLed, GpioBtn;
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOB;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioBtn);

	while(1){
		if(GPIO_ReadInputPin(GpioBtn.pGPIOx, GpioBtn.GPIO_PinConfig.GPIO_PinNumber)){
			delay();
			GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
		}

	}

	return 0;
}
