/*
 * 004btn_interrupt.c
 *
 *  Created on: Jul 7, 2024
 *      Author: Michael
 */

#include "stm32f407xx.h"
#include <string.h>

#define LOW 0
#define HIGH 1

#define BTN_PRESSED

void delay(void){
	uint32_t i;
	//This will introduce a delay of ~200 ms when SYSCLK is 16 MHz
	for(i = 0; i < 500000/2; ++i);
}

int main(void){
	GPIO_Handle_t GpioLed, GpioBtn;
	memset(&GpioLed, 0, sizeof(GPIO_Handle_t));
	memset(&GpioBtn, 0, sizeof(GPIO_Handle_t));

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_NO_PUPD;



	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_PIN_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);

	//IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PR15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	while(1){

	}

	return 0;
}

void EXTI0_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_0);
	delay();
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
}
