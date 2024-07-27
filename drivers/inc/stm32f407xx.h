/*
 * stm32f406xx.h
 *
 *  Created on: Jul 5, 2024
 *      Author: Michael Bove
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

/********************START: Processor Specific Details************************/
/*
 * ARM Cortex-Mx Processor NVIC ISERx Register Addresses
 */

#define NVIC_ISER0					((__vo uint32_t*)0xE000E100UL)
#define NVIC_ISER1					((__vo uint32_t*)0xE000E104UL)
#define NVIC_ISER2					((__vo uint32_t*)0xE000E108UL)
#define NVIC_ISER3					((__vo uint32_t*)0xE000E10CUL)

/*
 * ARM Cortex-Mx Processor NVIC ICERx Register Addresses
 */

#define NVIC_ICER0					((__vo uint32_t*)0xE000E180UL)
#define NVIC_ICER1					((__vo uint32_t*)0xE000E184UL)
#define NVIC_ICER2					((__vo uint32_t*)0xE000E188UL)
#define NVIC_ICER3					((__vo uint32_t*)0xE000E18CUL)

#define NVIC_PR_BASE_ADDR			((__vo uint32_t*) 0xE000E400UL)

#define NO_PR_BITS_IMPLEMENTED		4

/**********************END: Processor Specific Details************************/

/*
 * Base address of Flash Memory and SRAM memory
 */

#define FLASH_BASE_ADDR				0x08000000UL /* Base address of Flash Memory */
#define SRAM1_BASE_ADDR				0x20000000UL /* Base address of SRAM1 */
#define SRAM2_BASE_ADDR				0x2001C000UL /* Base address of SRAM2 */
#define ROM							0x1FFF0000UL /* Base address of ROM (system memory */
#define SRAM 						SRAM1_BASE_ADDR /* Base address of SRAM */


/*
 * AHBx and APBx Bus peripheral base addresses
 */

#define PERIPH_BASE_ADDR			0x40000000UL /* Peripheral Base address */
#define APB1_PERIPH_BASE_ADDR		PERIPH_BASE_ADDR /* APB1 bus base address */
#define APB2_PERIPH_BASE_ADDR		0x40010000UL /* APB2 bus base address */
#define AHB1_PERIPH_BASE_ADDR		0x40020000UL /* AHB1 bus base address */
#define AHB2_PEIPH_BASE_ADDR		0x50050000UL /* AHB2 bus base address */

/*
 * Base addresses for all peripherals hanging on AHB1 bus
 * TODO : Complete for all other peripherals on AHB1 bus
 */

#define AHB1_AREA_SIZE 				0x0400U /* Common size for peripherals on AHB1 Bus */

#define GPIOA_BASE_ADDR				(AHB1_PERIPH_BASE_ADDR) /* GPIO A Base address */
#define GPIOB_BASE_ADDR				(GPIOA_BASE_ADDR + AHB1_AREA_SIZE) /* GPIO B Base address */
#define GPIOC_BASE_ADDR				(GPIOB_BASE_ADDR + AHB1_AREA_SIZE) /* GPIO C Base address */
#define GPIOD_BASE_ADDR				(GPIOC_BASE_ADDR + AHB1_AREA_SIZE) /* GPIO D Base address */
#define GPIOE_BASE_ADDR				(GPIOD_BASE_ADDR + AHB1_AREA_SIZE) /* GPIO E Base address */
#define GPIOF_BASE_ADDR				(GPIOE_BASE_ADDR + AHB1_AREA_SIZE) /* GPIO F Base address */
#define GPIOG_BASE_ADDR				(GPIOF_BASE_ADDR + AHB1_AREA_SIZE) /* GPIO G Base address */
#define GPIOH_BASE_ADDR				(GPIOG_BASE_ADDR + AHB1_AREA_SIZE) /* GPIO H Base address */
#define GPIOI_BASE_ADDR				(GPIOH_BASE_ADDR + AHB1_AREA_SIZE) /* GPIO I Base address */

#define RCC_BASE_ADDR				(AHB1_PERIPH_BASE_ADDR + 0x3800) /* Reset and clock control base address */

/*
 * Base addresses for all peripherals hanging on APB1 bus
 * TODO: Complete for all other peripherals on APB1 bus
 */

#define APB1_AREA_SIZE 				0x0400U /* Common size for peripherals on APB1 Bus */

#define I2C1_BASE_ADDR				(APB1__PERIPH_BASE_ADDR + 0x5400U) /* I2C1 Base address */
#define I2C2_BASE_ADDR				(I2C1_BASE_ADDR + APB1_AREA_SIZE) /* I2C2 Base address */
#define I2C3_BASE_ADDR				(I2C2_BASE_ADDR + APB1_AREA_SIZE) /* I2C3 Base address */

#define SPI2_BASE_ADDR				(APB1_PERIPH_BASE_ADDR + 0x3800U) /* SPI2 Base address */
#define SPI3_BASE_ADDR				(SPI2_BASE_ADDR + APB1_AREA_SIZE) /* SPI2 Base address */

#define USART2_BASE_ADDR			(APB1__PERIPH_BASE_ADDR + 0x4400U) /* USART2 Base address */
#define USART3_BASE_ADDR			(USART2_BASE_ADDR + APB1_AREA_SIZE) /* USART3 Base address */
#define UART4_BASE_ADDR				(USART3_BASE_ADDR + APB1_AREA_SIZE) /* UART4 Base address */
#define UART5_BASE_ADDR				(UART4_BASE_ADDR + APB1_AREA_SIZE) /* UART5 Base address */

/*
 * Base addresses for all peripherals hanging on APB2 bus
 * TODO: Complete for all other peripherals on APB2 bus
 */

#define APB2_AREA_SIZE				0x0400U /* Common size for peripherals on APB2 Bus */

#define EXTI_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x3C00U) /* External interrupt base address */

#define SPI1_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x3000U) /* SPI1 Base address */

#define SYSCFG_BASE_ADDR			(APB2_PERIPH_BASE_ADDR + 0x3800U) /* System config base address */

#define USART1_BASE_ADDR			(APB2_PERIPH_BASE_ADDR + 0x1000U) /* USART1 Base address */
#define USART6_BASE_ADDR			(USART1_BASE_ADDR + APB2_AREA_SIZE) /* USART1 Base address */


/***********************************************************************/

/*
 * Note: Registers of a peripheral are specific to  MCU
 * This structure is defined for the STM32F407xx family
 * Please check your device RM
 */

typedef struct{
	__vo uint32_t MODER; /* GPIO Port Mode Register 					Address offset: 0x00 */
	__vo uint32_t OTYPER; /* GPIO Port Output Type Register 			Address offset: 0x04 */
	__vo uint32_t OSPEEDR; /* GPIO Port Output Speed Register 			Address offset: 0x08 */
	__vo uint32_t PUPDR; /* GPIO Port PU/PD Resistor Register 			Address offset: 0x0C */
	__vo uint32_t IDR; /* GPIO Port Input Data Register 				Address offset: 0x10 */
	__vo uint32_t ODR; /* GPIO Port Output Data Register 				Address offset: 0x14 */
	__vo uint32_t BSRR; /* GPIO Port Bit Set/Reset Register 			Address offset: 0x18 */
	__vo uint32_t LCKR; /* GPIO Port Configuration Lock Register 		Address offset: 0x1C */
	__vo uint32_t AFRL[2]; /* GPIO Port Alternate Function Register 	Address offset: 0x20 */

}GPIO_RegDef_t;

/*
 * Peripheral register structure for RCC
 */

typedef struct{
	__vo uint32_t CR; 		/* Clock control register 										Address offset: 0x00 */
	__vo uint32_t PLLCFGR; 	/* PLL configuration register 									Address offset: 0x04  */
	__vo uint32_t CFGR; 	/* Clock configuration register 								Address offset: 0x08 */
	__vo uint32_t CIR;		/* clock interrupt register 									Address offset: 0x0C */
	__vo uint32_t AHB1RSTR; /* AHB1 peripheral reset register 								Address offset: 0x10 */
	__vo uint32_t AHB2RSTR; /* AHB2 peripheral reset register 								Address offset: 0x14 */
	__vo uint32_t AHB3RSTR; /* AHB3 peripheral reset register 								Address offset: 0x18 */
	uint32_t RESERVED0;		/* Reserved 0x1C												Address offset: 0x1C */
	__vo uint32_t APB1RSTR; /* APB1 peripheral reset register 								Address offset: 0x20 */
	__vo uint32_t APB2RSTR; /* APB2 peripheral reset register 								Address offset: 0x24 */
	uint32_t RESERVED1[2];  /* Reserved 0x28-0x2C											Address offset: 0x28 */
	__vo uint32_t AHB1ENR;  /* AHB1 peripheral clock register 								Address offset: 0x30 */
	__vo uint32_t AHB2ENR;  /* AHB1 peripheral clock register 								Address offset: 0x34 */
	__vo uint32_t AHB3ENR;  /* AHB3 peripheral clock register 								Address offset: 0x38 */
	uint32_t RESERVED2; 	/* Reserved 0x3C 												Address offset: 0x3C */
	__vo uint32_t APB1ENR;  /* APB1 peripheral clock register 								Address offset: 0x40 */
	__vo uint32_t APB2ENR;  /* AHB1 peripheral clock register 								Address offset: 0x44 */
	uint32_t RESERVED3[2];  /* Reserved 0x48-0x4C											Address offset: 0x48 */
	__vo uint32_t AHB1LPENR;/* AHB1 peripheral clock enable in low power mode register 		Address offset: 0x50 */
	__vo uint32_t AHB2LPENR;/* AHB2 peripheral clock enable in low power mode register 		Address offset: 0x54 */
	__vo uint32_t AHB3LPENR;/* AHB3 peripheral clock enable in low power mode register		Address offset: 0x58 */
	uint32_t RESEREVED4; 	/* Reserved 0x5C												Address offset: 0x5C */
	__vo uint32_t APB1LPENR;/* APB1 peripheral clock enable in low power mode register 		Address offset: 0x60 */
	__vo uint32_t APB2LPENR;/* APB2 peripheral clock enable in low power mode register 		Address offset: 0x64 */
	uint32_t RESERVED5[2];  /* Reserved 0x68- 0x6C 											Address offset: 0x68 */
	__vo uint32_t BDCR; 	/*  Backup domain control register 								Address offset: 0x70 */
	__vo uint32_t CSR; 		/* clock control & status register 								Address offset: 0x74 */
	uint32_t RESERVED6[2];  /* Reserved 0x78-0x7C 											Address offset: 0x78 */
	__vo uint32_t SSCGR; 	/* spread spectrum clock generation register 					Address offset: 0x80 */
	__vo uint32_t PLLI2SCFGR;/* PLLI2S configuration register								Address offset: 0x84 */
	__vo uint32_t PLLSAICFGR;/*  PLLSAI configuration register 								Address offset: 0x88 */
	__vo uint32_t DCKCFGR;	 /* Dedicated Clock Configuration Register 						Address offset: 0x8C */
}RCC_RegDef_t;

/*
 * Peripheral register structure for EXTI
 */

typedef struct{
	__vo uint32_t IMR;	/* Interrupt Mask Register 				Address Offset: 0x00 */
	__vo uint32_t EMR; 	/* Event Mask Register					Address Offset: 0x04 */
	__vo uint32_t RTSR;	/* Rising Trigger Selection Register	Address Offset: 0x08 */
	__vo uint32_t FTSR; /* Falling Trigger Selection Register 	Address Offset: 0x0C */
	__vo uint32_t SWIER;/* Software Interrupt Event Register	Address Offset: 0x10 */
	__vo uint32_t PR;	/* Pending Register						Address Offset: 0x14 */
}EXTI_RegDef_t;

/*
 * Peripheral register structure for SPI
 */

typedef struct{
	__vo uint32_t CR1; /* SPI Control Register 1				Address offset: 0x00 */
	__vo uint32_t CR2; /* SPI Control Register 2				Address offset: 0x04 */
	__vo uint32_t SR; /* SPI Status Register 					Address offset: 0x08 */
	__vo uint32_t DR; /* SPI Data Register 						Address offset: 0x0C */
	__vo uint32_t CRCPR; /* SPI CRC Polynomial Register			Address offset: 0x10 */
	__vo uint32_t RXCRCR; /* SPI RX CRC Register				Address offset: 0x14 */
	__vo uint32_t TXCRCR; /* SPI TX CRC Register				Address offset: 0x18 */
	__vo uint32_t I2SCFGR; /* SPI I2S Configuration Register	Address offset: 0x00 */
	__vo uint32_t I2SPR; /* SPI I2S Prescalar Register			Address offset: 0x00 */

}SPI_RegDef_t;


/*
 * Peripheral register structure for SYSCFG
 */

typedef struct{
	__vo uint32_t MEMRMP;	/* Memory Remap Register 							Address Offset: 0x00 */
	__vo uint32_t PMC; 		/* Peripheral Mode Configuration Register			Address Offset: 0x04 */
	__vo uint32_t EXTICR[4];/* External Interrupt Configuration Registers		Address Offset: 0x08 */
	uint32_t RESERVED[2];	/* Reserved 0x18-0x1C 								Address Offset: 0x18 */
	__vo uint32_t CMPCR; 	/* Compensation Cell Control Register 				Address Offset: 0x20 */
}SYSCFG_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted)
 */

#define GPIOA 	((GPIO_RegDef_t*) GPIOA_BASE_ADDR)
#define GPIOB 	((GPIO_RegDef_t*) GPIOB_BASE_ADDR)
#define GPIOC 	((GPIO_RegDef_t*) GPIOC_BASE_ADDR)
#define GPIOD 	((GPIO_RegDef_t*) GPIOD_BASE_ADDR)
#define GPIOE 	((GPIO_RegDef_t*) GPIOE_BASE_ADDR)
#define GPIOF 	((GPIO_RegDef_t*) GPIOF_BASE_ADDR)
#define GPIOG 	((GPIO_RegDef_t*) GPIOG_BASE_ADDR)
#define GPIOH 	((GPIO_RegDef_t*) GPIOH_BASE_ADDR)
#define GPIOI 	((GPIO_RegDef_t*) GPIOI_BASE_ADDR)

#define RCC 	((RCC_RegDef_t*) RCC_BASE_ADDR)

#define EXTI 	((EXTI_RegDef_t*) EXTI_BASE_ADDR)

#define SYSCFG	((SYSCFG_RegDef_t*) SYSCFG_BASE_ADDR)

#define SPI1 	((SPI_RegDef_t*) SPI1_BASE_ADDR)
#define SPI2	((SPI_RegDef_t*) SPI2_BASE_ADDR)
#define SPI3	((SPI_RegDef_t*) SPI3_BASE_ADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |= (1 << 8))

/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 23))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))

/*
 * Clock enable macros for USARTx peripherals
 */

#define USART1_PCLK_EN() 	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN() 	(RCC->APB1ENR |= (1 << 20))
#define UART5_PCLK_EN() 	(RCC->APB1ENR |= (1 << 21))
#define USART6_PCLK_EN() 	(RCC->APB2ENR |= (1 << 5))

/*
 * Clock enable macro for SYSCFG peripheral
 */

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock disable macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock disable macros for SPIx peripherals
 */

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 23))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))

/*
 * Clock disable macros for USARTx peripherals
 */

#define USART1_PCLK_DI() 	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() 	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() 	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI() 	(RCC->APB1ENR &= ~(1 << 20))
#define UART5_PCLK_DI() 	(RCC->APB1ENR &= ~(1 << 21))
#define USART6_PCLK_DI()	 (RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock disable macro for SYSCFG peripheral
 */

#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET() 	do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET() 	do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET() 	do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET() 	do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET() 	do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET() 	do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET() 	do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET() 	do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET() 	do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

/*
 * Macros to reset GPIOx peripherals
 */

#define SPI1_REG_RESET()	do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)

/*
 * @return port code for given GPIOx base address
 */
#define GPIO_BASE_ADDR_TO_CODE(x)	((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 :\
									(x == GPIOI) ? 8 :-1)


/*
 * IRQ (Interrupt Request) Numbers of STM32F407xx MCU
 * NOTE: These macros apply to the STM32F407xx specific family.
 * 		 This is used for position used in NVIC
 * TODO: Complete list for all other peripherals
 */

#define IRQ_NO_EXTI0 	6
#define IRQ_NO_EXTI1 	7
#define IRQ_NO_EXTI2 	8
#define IRQ_NO_EXTI3 	9
#define IRQ_NO_EXTI4 	10
#define IRQ_NO_EXTI9_5	23
#define IRQ_NO_EXTI5_10	40

/*
 * Macros for priority level
 */
#define NVIC_IRQ_PR0	0
#define NVIC_IRQ_PR1	1
#define NVIC_IRQ_PR2	2
#define NVIC_IRQ_PR3	3
#define NVIC_IRQ_PR4	4
#define NVIC_IRQ_PR5	5
#define NVIC_IRQ_PR6	6
#define NVIC_IRQ_PR7	7
#define NVIC_IRQ_PR8	8
#define NVIC_IRQ_PR9	9
#define NVIC_IRQ_PR10	10
#define NVIC_IRQ_PR11	11
#define NVIC_IRQ_PR12	12
#define NVIC_IRQ_PR13	13
#define NVIC_IRQ_PR14	14
#define NVIC_IRQ_PR15	15


/*
 * Generic macros
 */

#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET 	RESET
#define FLAG_SET 		SET
#define FLAG_RESET		RESET

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

/***************************************************************************
 * 				    Bit shift positions for SPI peripheral
 ***************************************************************************/

/*
 * Bit shift positions for SPI_CR1 Register
 */

#define SPI_CR1_CPHA 		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 * Bit shift positions for SPI_CR2 Register
 */

#define SPI_CR2_RXDMAEN 0
#define SPI_CR2_TXDMAEN 1
#define SPI_CR2_SSOE	2
#define SPI_CR2_FRF		4
#define SPI_CR2_ERRIE	5
#define SPI_CR2_RXNEIE	6
#define SPI_CR2_TXEIE	7

/*
 * Bit shift positions for SPI_SR Register
 */

#define SPI_SR_RXNE		0
#define SPI_SR_TXE		1
#define SPI_SR_CHSIDE	2
#define SPI_SR_UDR		3
#define SPI_SR_CRCERR	4
#define SPI_SR_MODF		5
#define SPI_SR_OVR		6
#define SPI_SR_BSY		7
#define FRE				8

#endif /* INC_STM32F407XX_H_ */
