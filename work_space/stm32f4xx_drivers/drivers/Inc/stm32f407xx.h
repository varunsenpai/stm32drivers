/*
 * stm32f407xx.h
 *
 *  Created on: May 7, 2020
 *      Author: Varun
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile

/*************************************START:Processor specific details *************************************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx Register Addresses
 */
#define NVIC_ISER0									( (__vo uint32_t *)0xE000E100 ) /*NVIC ISER0 Base address */
#define NVIC_ISER1									( (__vo uint32_t *)0xE000E104 ) /*NVIC ISER1 Base address */
#define NVIC_ISER2									( (__vo uint32_t *)0xE000E108 ) /*NVIC ISER2 Base address */
#define NVIC_ISER3									( (__vo uint32_t *)0xE000E10C ) /*NVIC ISER3 Base address */

/*
 * ARM Cortex Mx Processor NVIC ICERx Register Addresses
 */

#define NVIC_ICER0									((__vo uint32_t *)0XE000E180 ) /*NVIC ICER0 Base address */
#define NVIC_ICER1									((__vo uint32_t *)0xE000E184 ) /*NVIC ICER1 Base address */
#define NVIC_ICER2									((__vo uint32_t *)0xE000E188 ) /*NVIC ICER2 Base address */
#define NVIC_ICER3									((__vo uint32_t *)0xE000E18C ) /*NVIC ICER3 Base address */

/*
 * ARM Cortex Mx Processor Priority Register Addresses
 */
#define NVIC_PR_BASE_ADDR							((__vo uint32_t *)0xE000E400 )

/*
 *  base addressed of flash and SRAM memories
 */

#define FLASH_BASEADDR								0x08000000U	/*Flash base address */
#define SRAM1_BASEADDR								0x20000000U	/*SRAM1 base address or the main RAM */
#define SRAM2_BASEADDR								0x2001C000U	/*SRAM2 base address */
#define ROM											0x1FFF0000U	/*ROM or System memory base address */
#define SRAM 										SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR								0X40000000U	/*Peripheral address starting point*/
#define APB1PERIPH_BASEADDR							PERIPH_BASEADDR /* APB1 peripheral starting address */
#define APB2PERIPH_BASEADDR							0x40010000U	/* APB2 peripheral starting address */
#define AHB1PERIPH_BASEADDR							0x40020000U	/* AHB1 peripheral starting address */
#define AHB2PERIPH_BASEADDR							0x50000000U	/* AHB2 peripheral starting address */




/* Devices on AHB1 bus */

#define GPIOA_BASEADDR								(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR								(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR								(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR								(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR								(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR								(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR								(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR								(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR								(AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR                     			(AHB1PERIPH_BASEADDR + 0x3800)

/* Devices on APB1 bus */

#define I2C1_BASEADDR								(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR								(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR								(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR								(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR								(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR								(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR								(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR								(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR								(APB1PERIPH_BASEADDR + 0x5000)

/*Devices on APB2 bus */

#define EXTI_BASEADDR								(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR								(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR        						(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR								(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR								(APB2PERIPH_BASEADDR + 0x1400)
#define UART7_BASEADDR								(APB1PERIPH_BASEADDR + 0x7800)
#define UART8_BASEADDR								(APB1PERIPH_BASEADDR + 0x7C00)

/**********************************peripheral register definition structures **********************************/

typedef struct
{
	__vo uint32_t MODER;                        /*!< GPIO port mode register,                    	Address offset: 0x00      */
	__vo uint32_t OTYPER;                       /*!< GPIO Otype register,     						Address offset: 0x04      */
	__vo uint32_t OSPEEDR;						/* GPIO O speed register,							Address offset: 0x08	  */
	__vo uint32_t PUPDR;						/*!< GPIOA_PUPDR     								Address offset: 0x0C      */
	__vo uint32_t IDR;							/*GPIO input data register,							Address offset: 0x10 	  */
	__vo uint32_t ODR;							/* GPIO port output data register					Address offset: 0x14	  */
	__vo uint32_t BSRR;							/* GPIO port bit set/reset register					Address offset: 0x18	  */
	__vo uint32_t LCKR;							/* GPIO port configuration lock register			Address offset: 0x2C	  */
	__vo uint32_t AFR[2];						/*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */

typedef struct
{
  __vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
  __vo uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
  __vo uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
  __vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
  __vo uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
  __vo uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
  __vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
  __vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
  __vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
  __vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
  __vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
  __vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
  __vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
  __vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
  __vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
  __vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
  __vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
  __vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
  __vo uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */

}RCC_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;    /*!< Give a short description,          	  	    		Address offset: 0x00 */
	__vo uint32_t EMR;    /*!< TODO,                								Address offset: 0x04 */
	__vo uint32_t RTSR;   /*!< TODO,  									    		Address offset: 0x08 */
	__vo uint32_t FTSR;   /*!< TODO, 												Address offset: 0x0C */
	__vo uint32_t SWIER;  /*!< TODO,  									    		Address offset: 0x10 */
	__vo uint32_t PR;     /*!< TODO,                   					   			Address offset: 0x14 */
}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t 	  RESERVED1[2];
	__vo uint32_t CMPCR;
	uint32_t 	  RESERVED2[2];
	__vo uint32_t CFGR;
}SYSCFG_RegDef_t;

/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CRCPR;      /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t RXCRCR;     /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t TXCRCR;     /*!< TODO,     										Address offset: 0x18 */
	__vo uint32_t I2SCFGR;    /*!< TODO,     										Address offset: 0x1C */
	__vo uint32_t I2SPR;      /*!< TODO,     										Address offset: 0x20 */
} SPI_RegDef_t;

/*
 * UART reg def
 */

typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
} UART_RegDef_t;

/*
 * peripheral definitions ( Peripheral base addresses type-casted to xxx_RegDef_t)
 */

#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define USART1				((UART_Reg_Def_t*)USART1_BASEADDR)
#define USART6				((UART_Reg_Def_t*)USART6_BASEADDR)
#define UART7				((UART_Reg_Def_t*)UART7_BASEADDR)
#define UART8				((UART_Reg_Def_t*)USART8_BASEADDR)
#define UART4				((UART_Reg_Def_t*)UART4_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1  				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2  				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3  				((SPI_RegDef_t*)SPI3_BASEADDR)

#define GPIO_BASEADDR_TO_CODE(x)  (	(x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 :\
									(x == GPIOI) ? 8 : 0 )

/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1<<8))

/*
 * Clock enable macros for I2C peripherals
 */

#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1<<23))

/*
 * Clock enable macros for SPI peripherals
 */

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() 		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() 		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() 		(RCC->APB2ENR |= (1 << 13))

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN() 	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() 	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() 	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()  	(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()  	(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN() 	(RCC->APB1ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_EN() 	(RCC->APB2ENR |= (1 << 14))


/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1<<8))

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() 		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() 		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI() 		(RCC->APB2ENR &= ~(1 << 13))

/*
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()	do{ (RCC->AHB1RSTR |= 1 << 0);	(RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()	do{ (RCC->AHB1RSTR |= 1 << 1);	(RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()	do{ (RCC->AHB1RSTR |= 1 << 2);	(RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()	do{ (RCC->AHB1RSTR |= 1 << 3);	(RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()	do{ (RCC->AHB1RSTR |= 1 << 4);	(RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()	do{ (RCC->AHB1RSTR |= 1 << 5);	(RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()	do{ (RCC->AHB1RSTR |= 1 << 6);	(RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()	do{ (RCC->AHB1RSTR |= 1 << 7);	(RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()	do{ (RCC->AHB1RSTR |= 1 << 8);	(RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

/*
 * Macros to reset SPIx peripherals
 */
#define SPI2_REG_RESET()	do{ (RCC->APB1RSTR |= 1 << 14); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()	do{ (RCC->APB1RSTR |= 1 << 15); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI1_REG_RESET()	do{ (RCC->APB2RSTR |= 1 << 12); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI() 	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() 	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() 	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()  	(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()  	(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI() 	(RCC->APB1ENR &= ~(1 << 5))


/*
 * Clock Disable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI() 	(RCC->APB2ENR &= ~(1 << 14))

// Some generic macros
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET 		ENABLE
#define GPIO_PIN_RESET 		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51

#define NVIC_IRQ_PRI0    	0
#define NVIC_IRQ_PRI15    	15

#define NO_PR_BITS_IMPLEMENTED 		4

/*
 * bit field definitions for SPI CR
 */
#define SPI_CR1_CPHA			0
#define SPI_CR1_MSTR			1
#define SPI_CR1_CPOL			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/*
 * Possible SPI Applications Events
 */
#define SPI_EVENT_TX_CMPLT				1
#define SPI_EVENT_RX_CMPLT				2
#define SPI_EVENT_OVR_CMPLT				3
#define SPI_EVENT_CRC_CMPLT				4

#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_uart_driver.h"

#endif /* INC_STM32F407XX_H_ */
