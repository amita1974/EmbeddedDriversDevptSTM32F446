/*
 * stm32f446xx.h
 *
 *  Created on: Aug 20, 2020
 *      Author: Amit Alon
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include "utils.h"
#include <stdint.h>

/* ********************************** Processor Specific Details ******************************** */
/* ARM Cortex 4 NVIC ISER (Interrupt Set-enable Registers) Registers Addresses */
#define NVIC_ISER0		((volatile uint32_t*) 0xE000E100)
#define NVIC_ISER1		((volatile uint32_t*) 0xE000E104)
#define NVIC_ISER2		((volatile uint32_t*) 0xE000E108)
#define NVIC_ISER3		((volatile uint32_t*) 0xE000E10C)
#define NVIC_ISER_BASE	0xE000E100

/* ARM Cortex 4 NVIC ICER (Interrupt Clear-enable Registers) Registers Addresses */
#define NVIC_ICER0		((volatile uint32_t*) 0xE000E180)
#define NVIC_ICER1		((volatile uint32_t*) 0xE000E184)
#define NVIC_ICER2		((volatile uint32_t*) 0xE000E188)
#define NVIC_ICER3		((volatile uint32_t*) 0xE000E18C)
#define NVIC_ICER_BASE	0xE000E180

/* ARM Cortex 4 NVIC ISPR (Interrupt Set-pending Registers) Registers Addresses */
#define NVIC_ISPR0		((volatile uint32_t*) 0xE000E200)
#define NVIC_ISPR1		((volatile uint32_t*) 0xE000E204)
#define NVIC_ISPR2		((volatile uint32_t*) 0xE000E208)
#define NVIC_ISPR3		((volatile uint32_t*) 0xE000E20C)
#define NVIC_ISPR_BASE	0xE000E200

/* ARM Cortex 4 NVIC ISPR (Interrupt Clear-pending Registers) Registers Addresses */
#define NVIC_ICPR0		((volatile uint32_t*) 0xE000E280)
#define NVIC_ICPR1		((volatile uint32_t*) 0xE000E284)
#define NVIC_ICPR2		((volatile uint32_t*) 0xE000E288)
#define NVIC_ICPR3		((volatile uint32_t*) 0xE000E28C)
#define NVIC_ICPR_BASE	0xE000E280

/* ARM Cortex 4 NVIC IABR (Interrupt Active Bit Registers) Registers Addresses */
#define NVIC_IABR0		((volatile uint32_t*) 0xE000E300)
#define NVIC_IABR1		((volatile uint32_t*) 0xE000E304)
#define NVIC_IABR2		((volatile uint32_t*) 0xE000E308)
#define NVIC_IABR3		((volatile uint32_t*) 0xE000E30C)
#define NVIC_IABR_BASE	0xE000E300

/* ARM Cortex 4 NVIC IPR (Interrupt Priority Registers) Registers Addresses */
#define NVIC_IPR0	((volatile uint32_t*) 0xE000E400)
#define NVIC_IPR_BASE	0xE000E400

#define NVIC_STIR		((volatile uint32_t*) 0xE000EF00)

/* Bus addresses of Flash and SRAM Memories */
#define FLASH_BASEADDR							0x08000000U
#define ROM_BASEADDR							0x1FFF0000U
#define SRAM1_BASEADDR							0x20000000U	// 112KB
#define SRAM2_BASEADDR							0x2001C000U	// 16KB
#define SRAM									SRAM1_BASEADDR


/* Bus AHBx and APBx bus Peripheral base addresses */
#define PERIPH_BASEADDR							0x40000000U
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR						0x40010000U
#define AHB1PERIPH_BASEADDR						0x40020000U
#define AHB2PERIPH_BASEADDR						0x50000000U

/* Base addresses for peripherals that hang on to AHB1 bus
   TODO: Complete for all other peripherals on AHB1 bus */
#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000U)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400U)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800U)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00U)
#define CRC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3000U)
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800U)
#define FLASH_INTERFACE_BASEADDR	(AHB1PERIPH_BASEADDR + 0x3C00U)
#define BKPSRAM_BASEADDR			(AHB1PERIPH_BASEADDR + 0x4000U)
#define DMA1_BASEADDR				(AHB1PERIPH_BASEADDR + 0x6000U)
#define DMA2_BASEADDR				(AHB1PERIPH_BASEADDR + 0x6400U)
#define USB_OTG_HS_BASEADDR			(AHB1PERIPH_BASEADDR + 0x20000U)


/* Base addresses for peripherals that hang on to APB1 bus */
#define TIM2_BASEADDR			(APB1PERIPH_BASEADDR + 0x0000U)
#define TIM3_BASEADDR			(APB1PERIPH_BASEADDR + 0x0400U)
#define TIM4_BASEADDR			(APB1PERIPH_BASEADDR + 0x0800U)
#define TIM5_BASEADDR			(APB1PERIPH_BASEADDR + 0x0C00U)
#define TIM6_BASEADDR			(APB1PERIPH_BASEADDR + 0x1000U)
#define TIM7_BASEADDR			(APB1PERIPH_BASEADDR + 0x1400U)
#define TIM12_BASEADDR			(APB1PERIPH_BASEADDR + 0x1800U)
#define TIM13_BASEADDR			(APB1PERIPH_BASEADDR + 0x1C00U)
#define TIM14_BASEADDR			(APB1PERIPH_BASEADDR + 0x2000U)
#define RTC_AND_BKP_BASEADDR	(APB1PERIPH_BASEADDR + 0x2800U)
#define WWDG_BASEADDR			(APB1PERIPH_BASEADDR + 0x2C00U)
#define IWDG_BASEADDR			(APB1PERIPH_BASEADDR + 0x3000U)
#define SPI2_OR_I2S2_BASEADDR	(APB1PERIPH_BASEADDR + 0x3800U)
#define SPI3_OR_I2S3_BASEADDR	(APB1PERIPH_BASEADDR + 0x3C00U)
#define SPI2_BASEADDR			(SPI2_OR_I2S2_BASEADDR)
#define SPI3_BASEADDR			(SPI3_OR_I2S3_BASEADDR)
#define I2S2_BASEADDR			(SPI2_OR_I2S2_BASEADDR)
#define I2S3_BASEADDR			(SPI3_OR_I2S3_BASEADDR)
#define SPDIF_RX_BASEADDR		(APB1PERIPH_BASEADDR + 0x4000U)
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400U)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800U)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00U)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000U)
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800U)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00U)
#define CAN1_BASEADDR			(APB1PERIPH_BASEADDR + 0x6400U)
#define CAN2_BASEADDR			(APB1PERIPH_BASEADDR + 0x6800U)
#define HDMI_CEC_BASEADDR		(APB1PERIPH_BASEADDR + 0x6C00U)
#define PWR_BASEADDR			(APB1PERIPH_BASEADDR + 0x7000U)
#define DAC_BASEADDR			(APB1PERIPH_BASEADDR + 0x7400U)

/* Base addresses for peripherals that hang on to APB2 bus */
#define TIM1_BASEADDR			(APB2PERIPH_BASEADDR + 0x0000U)
#define TIM8_BASEADDR			(APB2PERIPH_BASEADDR + 0x0400U)
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000U)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400U)
#define ADC1_2_3_BASEADDR		(APB2PERIPH_BASEADDR + 0x2000U)
#define SDMMC_BASEADDR			(APB2PERIPH_BASEADDR + 0x2C00U)
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000U)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400U)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800U)
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00U)
#define TIM9_BASEADDR			(APB2PERIPH_BASEADDR + 0x4000U)
#define TIM10_BASEADDR			(APB2PERIPH_BASEADDR + 0x4400U)
#define TIM11_BASEADDR			(APB2PERIPH_BASEADDR + 0x4800U)
#define SAI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x5800U)
#define SAI2_BASEADDR			(APB2PERIPH_BASEADDR + 0x5C00U)



/* *************************** Peripheral registers definition structures *************************** */
/*
 * GPIO registers structure
 */
typedef struct {
	volatile uint32_t MODER;	/*!< GPIO port mode register, offset 0 */
	volatile uint32_t OTYPR;	/*!< GPIO port output type register, offset 4 */
	volatile uint32_t OSPEEDR;	/*!< GPIO port output speed register, offset 8 */
	volatile uint32_t PUPDR;	/*!< GPIO port pull-up/pull-down register, offset 0xc */
	volatile uint32_t IDR;		/*!< GPIO port input data register, offset 0x10 */
	volatile uint32_t ODR;		/*!< GPIO port output data register, offset 0x14 */
	volatile uint32_t BSRR;		/*!< GPIO port bit set/reset register, offset 0x18 */
	volatile uint32_t LCKR;		/*!< GPIO port configuration lock register, offset 0x1C */
	volatile uint32_t AFR[2];	/*!< GPIO alternate function low registers, offset 0x20, 0x24 for AFRL and AFRH accordingly */
} GPIO_RegDef_t;

/*
 * SPI and I2S registers
 */
typedef struct {
	volatile uint32_t CR1;		/* !< SPI control register 1, offset 0x00 */
	volatile uint32_t CR2;		/* !< SPI control register 2, offset 0x04 */
	volatile uint32_t SR;		/* !< SPI status register, offset 0x08 */
	volatile uint32_t DR;		/* !< SPI data register, offset 0x0C */
	volatile uint32_t CRCPR;	/* !< SPI CRC polynomial register (not used in I2S mode), offset 0x10 */
	volatile uint32_t RXCRCR;	/* !< SPI RX CRC register (not used in I2S mode), offset 0x14 */
	volatile uint32_t TXCRCR;	/* !< SPI TX CRC register (not used in I2S mode), offset 0x18 */
	volatile uint32_t I2SCFGR;	/* !< SPI_I2S configuration register, offset 0x1C */
	volatile uint32_t I2SPR;	/* !< SPI_I2S prescaler register, offset 0x20 */
} SPIAndI2s_RegDef_t;

/*
 * RCC registers structure
 */
typedef struct {
	volatile uint32_t CR;			/*!< clock control register, offset 0 */
	volatile uint32_t PLL_CFGR;		/*!< PLL configuration register, offset 4 */
	volatile uint32_t CFGR;			/*!< clock configuration register, offset 8 */
	volatile uint32_t CIR;			/*!< clock interrupt register, offset 0xc */
	volatile uint32_t AHB1RSTR;		/*!< AHB1 peripheral reset register, offset 0x10 */
	volatile uint32_t AHB2RSTR;		/*!< AHB2 peripheral reset register, offset 0x14 */
	volatile uint32_t AHB3RSTR;		/*!< AHB3 peripheral reset register, offset 0x18 */
	volatile uint32_t RESERVED1;	/*!< Reserved, offset 0x1C */
	volatile uint32_t APB1RSTR;		/*!< APB1 peripheral reset register, offset 0x20 */
	volatile uint32_t APB2RSTR;		/*!< APB2 peripheral reset register, offset 0x24 */
	         uint32_t RESERVED2;	/*!< Reserved, offset 0x28 */
	         uint32_t RESERVED3;	/*!< Reserved, offset 0x2C */
	volatile uint32_t AHB1ENR;		/*!< AHB1 peripheral clock enable register, offset 0x30 */
	volatile uint32_t AHB2ENR;		/*!< AHB2 peripheral clock enable register, offset 0x34 */
	volatile uint32_t AHB3ENR;		/*!< AHB3 peripheral clock enable register, offset 0x38 */
			 uint32_t RESERVED4;	/*!< Reserved, offset 0x3C */
	volatile uint32_t APB1ENR;		/*!< APB1 peripheral clock enable register, offset 0x40 */
	volatile uint32_t APB2ENR;		/*!< APB2 peripheral clock enable register, offset 0x44 */
			 uint32_t RESERVED5;	/*!< Reserved, offset 0x48 */
			 uint32_t RESERVED6;	/*!< Reserved, offset 0x4C */
	volatile uint32_t AHB1LPENR;	/*!< AHB1 peripheral clock enable in low power mode register, offset 0x50 */
	volatile uint32_t AHB2LPENR;	/*!< AHB2 peripheral clock enable in low power mode register, offset 0x54 */
	volatile uint32_t AHB3LPENR;	/*!< AHB3 peripheral clock enable in low power mode register, offset 0x58 */
			 uint32_t RESERVED7;	/*!< Reserved, offset 0x5C */
	volatile uint32_t LPB1APENR;	/*!< APB1 peripheral clock enable in low power mode register, offset 0x60 */
	volatile uint32_t LPB2APENR;	/*!< APB2 peripheral clock enable in low power mode register, offset 0x64 */
			 uint32_t RESERVED8;	/*!< Reserved, offset 0x68 */
			 uint32_t RESERVED9;	/*!< Reserved, offset 0x6C */
	volatile uint32_t BDCR;			/*!< Backup domain control register, offset 0x70 */
	volatile uint32_t SCR;			/*!< clock control & status register, offset 0x74 */
			 uint32_t RESERVED10;	/*!< Reserved, offset 0x78 */
			 uint32_t RESERVED11;	/*!< Reserved, offset 0x7C */
	volatile uint32_t SSCGR;		/*!< spread spectrum clock generation register, offset 0x80 */
	volatile uint32_t PLLI2_SCFGR;	/*!< PLLI2S configuration register, offset 0x84 */
	volatile uint32_t PLL_SAICFGR;	/*!< PLL configuration register, offset 0x88 */
	volatile uint32_t DCKCFGR;		/*!< Dedicated Clock Configuration Register, offset 0x8C */
	volatile uint32_t CKGATENR;		/*!< clocks gated enable register, offset 0x90 */
	volatile uint32_t DCKCFGR2;		/*!< dedicated clocks configuration register 2, offset 0x94 */
} RCC_RegDef_t;

/*
 * EXTI registers structure
 */
typedef struct {
	volatile uint32_t IMR;		/* <! Interrupt mask register, 0ffset 0x00 > */
	volatile uint32_t EMR;		/* <! Event mask register, offset 0x04 > */
	volatile uint32_t RTSR;		/* <! Rising trigger selection register. offset 0x08 > */
	volatile uint32_t FTSR;		/* <! Falling trigger selection register, offset 0x0C > */
	volatile uint32_t SWIER;	/* <! Software interrupt event register, offset 0x10 > */
	volatile uint32_t PR;		/* <! Pending register, offset 0x14 > */
} EXTI_RegDef_t;

typedef struct {
	volatile uint32_t	MEMRMP;			/* <! memory remap register, offset 0x00 > */
	volatile uint32_t	PMC;			/* <! peripheral mode configuration register, offset 0x04 > */
	volatile uint32_t	EXTICR[4];		/* <! external interrupt configuration registers1..4, offset 0x08..14 > */
			 uint32_t	RESERVED1;		/* <! reserved - unused, offset 0x18 > */
	volatile uint32_t	CMPCR;			/* <! Compensation cell control register, offset 0x20 > */
			 uint32_t	RESERVED2[2];	/* <! reserved - unused, offset 0x24,0x28 > */
	volatile uint32_t	CFGR;			/* <! configuration register, offset 0x2C > */
} SYSCFG_RegDef_t;

/* Peripheral definitions - (Peripheral base addresses typecasted to xxxx_RegDef_t */
// Use example:
// GPIOA->MODER = ...
#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF	((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG	((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH	((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define SPI1	((SPIAndI2s_RegDef_t*)SPI1_BASEADDR)
#define SPI2	((SPIAndI2s_RegDef_t*)SPI2_BASEADDR)
#define SPI3	((SPIAndI2s_RegDef_t*)SPI3_BASEADDR)
#define SPI4	((SPIAndI2s_RegDef_t*)SPI4_BASEADDR)
#define I2S2	((SPIAndI2s_RegDef_t*)I2S2_BASEADDR)
#define I2S3	((SPIAndI2s_RegDef_t*)I2S3_BASEADDR)

#define RCC		((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI	((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define GPIO_BASE_ADDR_TO_PORT_CODE(pGPIOHanlde) (uint8_t)(((uint32_t)pGPIOHanlde->pGPIOx - AHB1PERIPH_BASEADDR) / 0x400)

/* Clock enable macros for GPIOx Peripherals */
#define GPIOA_PCLK_EN()	( RCC->AHB1ENR |= (0x1 << 0) )
#define GPIOB_PCLK_EN()	( RCC->AHB1ENR |= (0x1 << 1) )
#define GPIOC_PCLK_EN()	( RCC->AHB1ENR |= (0x1 << 2) )
#define GPIOD_PCLK_EN()	( RCC->AHB1ENR |= (0x1 << 3) )
#define GPIOE_PCLK_EN()	( RCC->AHB1ENR |= (0x1 << 4) )
#define GPIOF_PCLK_EN()	( RCC->AHB1ENR |= (0x1 << 5) )
#define GPIOG_PCLK_EN()	( RCC->AHB1ENR |= (0x1 << 6) )
#define GPIOH_PCLK_EN()	( RCC->AHB1ENR |= (0x1 << 7) )
/* Clock disable macros for GPIOx Peripherals */
#define GPIOA_PCLK_DIS()	( RCC->AHB1ENR &= ~(0x1 << 0) )
#define GPIOB_PCLK_DIS()	( RCC->AHB1ENR &= ~(0x1 << 1) )
#define GPIOC_PCLK_DIS()	( RCC->AHB1ENR &= ~(0x1 << 2) )
#define GPIOD_PCLK_DIS()	( RCC->AHB1ENR &= ~(0x1 << 3) )
#define GPIOE_PCLK_DIS()	( RCC->AHB1ENR &= ~(0x1 << 4) )
#define GPIOF_PCLK_DIS()	( RCC->AHB1ENR &= ~(0x1 << 5) )
#define GPIOG_PCLK_DIS()	( RCC->AHB1ENR &= ~(0x1 << 6) )
#define GPIOH_PCLK_DIS()	( RCC->AHB1ENR &= ~(0x1 << 7) )

#define GPIOA_REGS_RESET()	do { ( RCC->AHB1RSTR |= (0x1 << 0) ); ( RCC->AHB1RSTR &= ~(0x1 << 0) ); } while (0)
#define GPIOB_REGS_RESET()	do { ( RCC->AHB1RSTR |= (0x1 << 1) ); ( RCC->AHB1RSTR &= ~(0x1 << 1) ); } while (0)
#define GPIOC_REGS_RESET()	do { ( RCC->AHB1RSTR |= (0x1 << 2) ); ( RCC->AHB1RSTR &= ~(0x1 << 2) ); } while (0)
#define GPIOD_REGS_RESET()	do { ( RCC->AHB1RSTR |= (0x1 << 3) ); ( RCC->AHB1RSTR &= ~(0x1 << 3) ); } while (0)
#define GPIOE_REGS_RESET()	do { ( RCC->AHB1RSTR |= (0x1 << 4) ); ( RCC->AHB1RSTR &= ~(0x1 << 4) ); } while (0)
#define GPIOF_REGS_RESET()	do { ( RCC->AHB1RSTR |= (0x1 << 5) ); ( RCC->AHB1RSTR &= ~(0x1 << 5) ); } while (0)
#define GPIOG_REGS_RESET()	do { ( RCC->AHB1RSTR |= (0x1 << 6) ); ( RCC->AHB1RSTR &= ~(0x1 << 6) ); } while (0)
#define GPIOH_REGS_RESET()	do { ( RCC->AHB1RSTR |= (0x1 << 7) ); ( RCC->AHB1RSTR &= ~(0x1 << 7) ); } while (0)

#define SPI1_REGS_RESET()	do { ( RCC->APB2RSTR |= (0x1 << 12) ); ( RCC->APB2RSTR &= ~(0x1 << 12) ); } while (0)
#define SPI2_REGS_RESET()	do { ( RCC->APB1RSTR |= (0x1 << 14) ); ( RCC->APB1RSTR &= ~(0x1 << 14) ); } while (0)
#define SPI3_REGS_RESET()	do { ( RCC->APB1RSTR |= (0x1 << 15) ); ( RCC->APB1RSTR &= ~(0x1 << 15) ); } while (0)
#define SPI4_REGS_RESET()	do { ( RCC->APB2RSTR |= (0x1 << 13) ); ( RCC->APB2RSTR &= ~(0x1 << 13) ); } while (0)

/* Clock enable macros for I2Cx Peripherals */
#define I2C1_PCLK_EN()	( RCC->APB1ENR |= (0x1 << 21) )
#define I2C2_PCLK_EN()	( RCC->APB1ENR |= (0x1 << 22) )
#define I2C3_PCLK_EN()	( RCC->APB1ENR |= (0x1 << 23) )
/* Clock disable macros for I2Cx Peripherals */
#define I2C1_PCLK_DIS()	( RCC->APB1ENR &= ~(0x1 << 21) )
#define I2C2_PCLK_DIS()	( RCC->APB1ENR &= ~(0x1 << 22) )
#define I2C3_PCLK_DIS()	( RCC->APB1ENR &= ~(0x1 << 23) )

/* Clock enable macros for SPIx Peripherals */
#define SPI1_PCLK_EN()	( RCC->APB2ENR |= (0x1 << 12) )
#define SPI2_PCLK_EN()	( RCC->APB1ENR |= (0x1 << 14) )
#define SPI3_PCLK_EN()	( RCC->APB1ENR |= (0x1 << 15) )
#define SPI4_PCLK_EN()	( RCC->APB2ENR |= (0x1 << 13) )
/* Clock disable macros for SPIx Peripherals */
#define SPI1_PCLK_DIS()	( RCC->APB2ENR &= ~(0x1 << 12) )
#define SPI2_PCLK_DIS()	( RCC->APB1ENR &= ~(0x1 << 14) )
#define SPI3_PCLK_DIS()	( RCC->APB1ENR &= ~(0x1 << 15) )
#define SPI4_PCLK_DIS()	( RCC->APB2ENR &= ~(0x1 << 13) )

/* Clock enable macros for USARTx / UARTX Peripherals */
#define USART1_PCLK_EN()	( RCC->APB2ENR |= (0x1 << 4) )
#define USART2_PCLK_EN()	( RCC->APB1ENR |= (0x1 << 17) )
#define USART3_PCLK_EN()	( RCC->APB1ENR |= (0x1 << 18) )
#define UART4_PCLK_EN()		( RCC->APB1ENR |= (0x1 << 19) )
#define UART5_PCLK_EN()		( RCC->APB1ENR |= (0x1 << 20) )
#define USART6_PCLK_EN()	( RCC->APB2ENR |= (0x1 << 5) )
/* Clock disable macros for USARTx / UARTX Peripherals */
#define USART1_PCLK_DIS()	( RCC->APB2ENR &= ~(0x1 << 4) )
#define USART2_PCLK_DIS()	( RCC->APB1ENR &= ~(0x1 << 17) )
#define USART3_PCLK_DIS()	( RCC->APB1ENR &= ~(0x1 << 18) )
#define UART4_PCLK_DIS()	( RCC->APB1ENR &= ~(0x1 << 19) )
#define UART5_PCLK_DIS()	( RCC->APB1ENR &= ~(0x1 << 20) )
#define USART6_PCLK_DIS()	( RCC->APB2ENR &= ~(0x1 << 5) )

/* Clock enable macros for SYSCFG Peripherals */
#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= (0x1 << 14) )
/* Clock disable macros for SYSCFG Peripherals */
#define SYSCFG_PCLK_DIS()	( RCC->APB2ENR &= ~(0x1 << 14) )


/*
 * NVIC IRQ mapping for all peripherals and IRQ sources, interrupt priorities, interrupt handler function pointer address.
 * Source: STM34446XX RM, Table 38. Vector table for STM32F446xx
 */
// Non-IRQ-vector-Table_entries - Priority - Reserved 0x0000 0000
// Non-IRQ-vector-Table_entries - Priority (-3 fixed Priority) Reset Reset 0x0000 0004
// Non-IRQ-vector-Table_entries - Priority (-2 fixed Priority) NMI Non maskable interrupt, Clock Security System 0x0000 0008
// Non-IRQ-vector-Table_entries - Priority (-1 fixed Priority) HardFault All class of fault 0x0000 000C
// Non-IRQ-vector-Table_entries - Priority 0 MemManage Memory management 0x0000 0010
// Non-IRQ-vector-Table_entries - Priority 1 BusFault Pre-fetch fault, memory access fault 0x0000 0014
// Non-IRQ-vector-Table_entries - Priority 2 UsageFault Undefined instruction or illegal state 0x0000 0018
// Non-IRQ-vector-Table_entries - Priority - Reserved 0x0000 001C - 0x0000 002B
// Non-IRQ-vector-Table_entries - Priority 3 SVCall System Service call via SWI instruction 0x0000 002C
// Non-IRQ-vector-Table_entries - Priority 4 Debug Monitor Debug Monitor 0x0000 0030
// Non-IRQ-vector-Table_entries - Priority - Reserved 0x0000 0034
// Non-IRQ-vector-Table_entries - Priority 5 PendSV Pendable request for system service 0x0000 0038
// Non-IRQ-vector-Table_entries - Priority 6 Systick System tick timer 0x0000 003C
#define IRQ_NUM_WWDG				0  // default priority 7   Window Watchdog interrupt 0x0000 0040
#define IRQ_NUM_PVD					1  // default priority 8   PVD through EXTI line detection interrupt 0x0000 0044
#define IRQ_NUM_TAMP_STAMP			2  // default priority 9   Tamper and TimeStamp interrupts through the EXTI line 0x0000 0048
#define IRQ_NUM_RTC_WKUP			3  // default priority 10  RTC Wakeup interrupt through the EXTI line 0x0000 004C
#define IRQ_NUM_FLASH				4  // default priority 11  Flash global interrupt 0x0000 0050
#define IRQ_NUM_RCC					5  // default priority 12  RCC global interrupt 0x0000 0054
#define IRQ_NUM_EXTI0				6  // default priority 13  EXTI Line0 interrupt 0x0000 0058
#define IRQ_NUM_EXTI1				7  // default priority 14  EXTI Line1 interrupt 0x0000 005C
#define IRQ_NUM_EXTI2				8  // default priority 15  EXTI Line2 interrupt 0x0000 0060
#define IRQ_NUM_EXTI3				9  // default priority 16  EXTI Line3 interrupt 0x0000 0064
#define IRQ_NUM_EXTI4				10 // default priority 17  EXTI Line4 interrupt 0x0000 0068
#define IRQ_NUM_DMA1_STREAM0		11 // default priority 18  DMA1 Stream0 global interrupt 0x0000 006C
#define IRQ_NUM_DMA1_STREAM1		12 // default priority 19  DMA1 Stream1 global interrupt 0x0000 0070
#define IRQ_NUM_DMA1_STREAM2		13 // default priority 20  DMA1 Stream2 global interrupt 0x0000 0074
#define IRQ_NUM_DMA1_STREAM3		14 // default priority 21  DMA1 Stream3 global interrupt 0x0000 0078
#define IRQ_NUM_DMA1_STREAM4		15 // default priority 22  DMA1 Stream4 global interrupt 0x0000 007C
#define IRQ_NUM_DMA1_STREAM5		16 // default priority 23  DMA1 Stream5 global interrupt 0x0000 0080
#define IRQ_NUM_DMA1_STREAM6		17 // default priority 24  DMA1 Stream6 global interrupt 0x0000 0084
#define IRQ_NUM_ADC					18 // default priority 25  ADC1, ADC2 and ADC3 global interrupts 0x0000 0088
#define IRQ_NUM_CAN1_TX				19 // default priority 26  CAN1 TX interrupts 0x0000 008C
#define IRQ_NUM_CAN1_RX0			20 // default priority 27  CAN1 RX0 interrupts 0x0000 0090
#define IRQ_NUM_CAN1_RX1			21 // default priority 28  CAN1 RX1 interrupt 0x0000 0094
#define IRQ_NUM_CAN1_SCE			22 // default priority 29  CAN1 SCE interrupt 0x0000 0098
#define IRQ_NUM_EXTI9_5				23 // default priority 30  EXTI Line[9:5] interrupts 0x0000 009C
#define IRQ_NUM_TIM1_BRK_TIM9		24 // default priority 31  TIM1 Break interrupt and TIM9 global interrupt 0x0000 00A0
#define IRQ_NUM_TIM1_UP_TIM10		25 // default priority 32  TIM1 Update interrupt and TIM10 global interrupt 0x0000 00A4
#define IRQ_NUM_TIM1_TRG_COM_TIM11	26 // default priority 33  TIM1 Trigger and Commutation interrupts and TIM11 global interrupt 0x0000 00A8
#define IRQ_NUM_TIM1_CC				27 // default priority 34  TIM1 Capture Compare interrupt 0x0000 00AC
#define IRQ_NUM_TIM2				28 // default priority 35  TIM2 global interrupt 0x0000 00B0
#define IRQ_NUM_TIM3				29 // default priority 36  TIM3 global interrupt 0x0000 00B4
#define IRQ_NUM_TIM4				30 // default priority 37  TIM4 global interrupt 0x0000 00B8
#define IRQ_NUM_I2C1_EV				31 // default priority 38  I2C1 event interrupt 0x0000 00BC
#define IRQ_NUM_I2C1_ER				32 // default priority 39  I2C1 error interrupt 0x0000 00C0
#define IRQ_NUM_I2C2_EV				33 // default priority 40  I2C2 event interrupt 0x0000 00C4
#define IRQ_NUM_I2C2_ER				34 // default priority 41  I2C2 error interrupt 0x0000 00C8
#define IRQ_NUM_SPI1				35 // default priority 42  SPI1 global interrupt 0x0000 00CC
#define IRQ_NUM_SPI2				36 // default priority 43  SPI2 global interrupt 0x0000 00D0
#define IRQ_NUM_USART1				37 // default priority 44  USART1 global interrupt 0x0000 00D4
#define IRQ_NUM_USART2				38 // default priority 45  USART2 global interrupt 0x0000 00D8
#define IRQ_NUM_USART3				39 // default priority 46  USART3 global interrupt 0x0000 00DC
#define IRQ_NUM_EXTI15_10			40 // default priority 47  EXTI Line[15:10] interrupts 0x0000 00E0
#define IRQ_NUM_RTC_ALARM			41 // default priority 48  RTC Alarms (A and B) through EXTI line interrupt 0x0000 00E4
#define IRQ_NUM_OTG_FS_WKUP			42 // default priority 49  USB On-The-Go FS Wakeup through EXTI line interrupt 0x0000 00E8
#define IRQ_NUM_TIM8_BRK_TIM12		43 // default priority 50  TIM8 Break interrupt and TIM12 global interrupt 0x0000 00EC
#define IRQ_NUM_TIM8_UP_TIM13		44 // default priority 51  TIM8 Update interrupt and TIM13 global interrupt 0x0000 00F0
#define IRQ_NUM_TIM8_TRG_COM_TIM14	45 // default priority 52  TIM8 Trigger and Commutation interrupts and TIM14 global interrupt 0x0000 00F4
#define IRQ_NUM_TIM8_CC				46 // default priority 53  TIM8 Capture Compare interrupt 0x0000 00F8
#define IRQ_NUM_DMA1_STREAM7		47 // default priority 54  DMA1 Stream7 global interrupt 0x0000 00FC
#define IRQ_NUM_FMC					48 // default priority 55  FMC global interrupt 0x0000 0100
#define IRQ_NUM_SDIO				49 // default priority 56  SDIO global interrupt 0x0000 0104
#define IRQ_NUM_TIM5				50 // default priority 57  TIM5 global interrupt 0x0000 0108
#define IRQ_NUM_SPI3				51 // default priority 58  SPI3 global interrupt 0x0000 010C
#define IRQ_NUM_UART4				52 // default priority 59  UART4 global interrupt 0x0000 0110
#define IRQ_NUM_UART5				53 // default priority 60  UART5 global interrupt 0x0000 0114
#define IRQ_NUM_TIM6_DAC			54 // default priority 61  TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts 0x0000 0118
#define IRQ_NUM_TIM7				55 // default priority 62  TIM7 global interrupt 0x0000 011C
#define IRQ_NUM_DMA2_STREAM0		56 // default priority 63  DMA2 Stream0 global interrupt 0x0000 0120
#define IRQ_NUM_DMA2_STREAM1		57 // default priority 64  DMA2 Stream1 global interrupt 0x0000 0124
#define IRQ_NUM_DMA2_STREAM2		58 // default priority 65  DMA2 Stream2 global interrupt 0x0000 0128
#define IRQ_NUM_DMA2_STREAM3		59 // default priority 66  DMA2 Stream3 global interrupt 0x0000 012C
#define IRQ_NUM_DMA2_STREAM4		60 // default priority 67  DMA2 Stream4 global interrupt 0x0000 0130
#define IRQ_NUM_RESERVED1			61 // default priority 68  - Reserved 0x0000 0134
#define IRQ_NUM_RESERVED2			62 // default priority 69  - Reserved 0x0000 0138
#define IRQ_NUM_CAN2_TX				63 // default priority 70  CAN2 TX interrupts 0x0000 013C
#define IRQ_NUM_CAN2_RX0			64 // default priority 71  CAN2 RX0 interrupts 0x0000 0140
#define IRQ_NUM_CAN2_RX1			65 // default priority 72  CAN2 RX1 interrupt 0x0000 0144
#define IRQ_NUM_CAN2_SCE			66 // default priority 73  CAN2 SCE interrupt 0x0000 0148
#define IRQ_NUM_OTG_FS				67 // default priority 74  USB On The Go FS global interrupt 0x0000 014C
#define IRQ_NUM_DMA2_STREAM5		68 // default priority 75  DMA2 Stream5 global interrupt 0x0000 0150
#define IRQ_NUM_DMA2_STREAM6		69 // default priority 76  DMA2 Stream6 global interrupt 0x0000 0154
#define IRQ_NUM_DMA2_STREAM7		70 // default priority 77  DMA2 Stream7 global interrupt 0x0000 0158
#define IRQ_NUM_USART6				71 // default priority 78  USART6 global interrupt 0x0000 015C
#define IRQ_NUM_I2C3_EV				72 // default priority 79  I2C3 event interrupt 0x0000 0160
#define IRQ_NUM_I2C3_ER				73 // default priority 80  I2C3 error interrupt 0x0000 0164
#define IRQ_NUM_OTG_HS_EP1_OUT		74 // default priority 81  USB On The Go HS End Point 1 Out global interrupt 0x0000 0168
#define IRQ_NUM_OTG_HS_EP1_IN		75 // default priority 82  USB On The Go HS End Point 1 In global interrupt 0x0000 016C
#define IRQ_NUM_OTG_HS_WKUP			76 // default priority 83  USB On The Go HS Wakeup through EXTI interrupt 0x0000 0170
#define IRQ_NUM_OTG_HS				77 // default priority 84  USB On The Go HS global interrupt 0x0000 0174
#define IRQ_NUM_DCMI				78 // default priority 85  DCMI global interrupt 0x0000 0178
#define IRQ_NUM_RESERVED3			79 // default priority 86  - Reserved 0x0000 017C
#define IRQ_NUM_RESERVED4			80 // default priority 87  - Reserved 0x0000 0180
#define IRQ_NUM_FPU					81 // default priority 88  FPU global interrupt 0x0000 0184
#define IRQ_NUM_RESERVED5			82 // default priority 89  - Reserved 0x0000 0188
#define IRQ_NUM_RESERVED6			83 // default priority 90  - Reserved 0x0000 018C
#define IRQ_NUM_SPI4				84 // default priority 91  SPI 4 global interrupt 0x0000 0190
#define IRQ_NUM_RESERVED7			85 // default priority 92  - Reserved 0x0000 0194
#define IRQ_NUM_RESERVED8			86 // default priority 93  - Reserved 0x0000 0198
#define IRQ_NUM_SAI1				87 // default priority 94  SAI1 global interrupt 0x0000 019C
#define IRQ_NUM_RESERVED9			88 // default priority 95  - Reserved 0x0000 01A0
#define IRQ_NUM_RESERVED10			89 // default priority 96  - Reserved 0x0000 01A4
#define IRQ_NUM_RESERVED11			90 // default priority 97  - Reserved 0x0000 01A8
#define IRQ_NUM_SAI2				91 // default priority 98  SAI2 global interrupt 0x0000 01AC
#define IRQ_NUM_QUADSPI				92 // default priority 99  QuadSPI global interrupt 0x0000 01B0
#define IRQ_NUM_HDMI				93 // default priority 100 -CEC HDMI-CEC global interrupt 0x0000 01B4
#define IRQ_NUM_SPDIF				94 // default priority 101 -Rx SPDIF-Rx global interrupt 0x0000 01B8
#define IRQ_NUM_FMPI2C1				95 // default priority 102  FMPI2C1 event interrupt 0x0000 01BC
#define IRQ_NUM_FMPI2C1_ERR			96 // default priority 103  error FMPI2C1 error interrup 0x0000 01C0

/*
 * Some Generic Macros
 */
#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

/* **********************************************
 * Bit positions for SPI Peripheral registers
 ************************************************ */
#define SPI_CR1_CPHA_BIT_POS		0
#define SPI_CR1_CPOL_BIT_POS		1
#define SPI_CR1_MSTR_BIT_POS		2
#define SPI_CR1_BR_BIT_POS			3
#define SPI_CR1_SPE_BIT_POS			6
#define SPI_CR1_LSB_FIRST_BIT_POS	7
#define SPI_CR1_SSI_BIT_POS			8
#define SPI_CR1_SSM_BIT_POS			9
#define SPI_CR1_RX_ONLY_BIT_POS		10
#define SPI_CR1_DFF_BIT_POS			11
#define SPI_CR1_CRC_NEXT_BIT_POS	12
#define SPI_CR1_CRC_EN_BIT_POS		13
#define SPI_CR1_BIDI_OE_BIT_POS		14
#define SPI_CR1_BIDI_MODE_BIT_POS	15

#define SPI_CR2_RXDMAEN_BIT_POS		0
#define SPI_CR2_TXDMAEN_BIT_POS		1
#define SPI_CR2_SSOE_BIT_POS		2
#define SPI_CR2_FRF_BIT_POS			4
#define SPI_CR2_ERRIE_BIT_POS		5
#define SPI_CR2_RXNEIE_BIT_POS		6
#define SPI_CR2_TXEIE_BIT_POS		7

#define SPI_SR_RXNE_BIT_POS			0
#define SPI_SR_TXE_BIT_POS			1
#define SPI_SR_CHSIDE_BIT_POS		2
#define SPI_SR_UDR_BIT_POS			3
#define SPI_SR_CRC_ERR_BIT_POS		4
#define SPI_SR_MODF_BIT_POS			5
#define SPI_SR_OVR_BIT_POS			6
#define SPI_SR_BSY_BIT_POS			7
#define SPI_SR_FRE_BIT_POS			8

/* *********************************************** */

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"

#endif /* INC_STM32F446XX_H_ */
