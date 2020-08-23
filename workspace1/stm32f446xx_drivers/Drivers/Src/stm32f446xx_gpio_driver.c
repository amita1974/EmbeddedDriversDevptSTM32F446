/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: 20 באוג׳ 2020
 *      Author: amita
 */

#include "stm32f446xx_gpio_driver.h"

//TODO: Add to the driver's functions values checking and logic to check if some configuration values does not make sence,
//      e.g. setting PU/PD on input port, set output type on input port, etc...

/*
 * Peripheral clock setup
 */
/**************************************************************
 * @fn			- GPIO_PeriClockCOntrol
 *
 * @brief		- This function enables or disables Peripheral clock for given GPIO port.
 *
 * @param[in]	- Base address of the GPIO Port Peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 *
 **************************************************************/
void GPIO_PerClockControl (GPIO_RegDef_t * pGPIOx, uint8_t EnOrDis)
{
	if (EnOrDis == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DIS();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DIS();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DIS();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DIS();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DIS();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DIS();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DIS();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DIS();
		}
	}
}

/*
 * Init and De-init
 */
/**************************************************************
 * @fn			- GPIO_Init
 *
 * @brief		- This function initializes a GPIO pin.
 *
 * @param[in]	- Handle of the GPIO port which holds the configuration data
 *
 * @return		- none
 *
 * @Note		- none
 *
 **************************************************************/
void GPIO_Init(GPIO_Handle_t* pGPIOHanlde)
{
	/* Configure the mode of the GPIO pin */
	uint32_t temp = 0;
	uint8_t reg_bit_offset;
	reg_bit_offset = (2 * pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber); // offset for 2 bits configuration value per port
	if (pGPIOHanlde->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		temp = (pGPIOHanlde->GPIO_PinConfig.GPIO_PinMode << reg_bit_offset);
		pGPIOHanlde->pGPIOx->MODER &= ~(0x3 << reg_bit_offset);
		pGPIOHanlde->pGPIOx->MODER |= temp;
	} else {
		//TODO: is the EXTI peripheral clock enabled??? how? when was it enabled? without enabling it, its registers can't be configured...

		/* Peripheral related configurations */
		/* GPIO peripheral interrupt related modes. */
		/* 1. Set the interrupt detection as Rising / Falling / Rising+Falling edge detection. */
		if (pGPIOHanlde->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_FEI) {
			/* Configure the FTSR to detect Falling edge and clear the Rising edged detection in case it was already set */
			EXTI->FTSR |= (1 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHanlde->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_REI) {
			/* Configure the RTSR to detect Rising edge and clear the Falling edged detection in case it was already set */
			EXTI->RTSR |= (1 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHanlde->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_RFEI) {
			/* Configure both FTSR and RTSR to detect both rising and falling edge*/
			EXTI->RTSR |= (1 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber);
		}
		/* SYSCFG peripheral related configurations */
		/* 2. Configure the GPIO port selection in SYSCFG_EXTICR, so that the input pin of the correct port will cause the interrupt */
		SYSCFG_PCLK_EN();
		uint8_t syscfg_exticr_reg_idx = pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t	syscfg_exticr_bit_offset = (pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber % 4) * 4;
		SYSCFG->EXTICR[syscfg_exticr_reg_idx] &= ~(0x0F << syscfg_exticr_bit_offset);
		SYSCFG->EXTICR[syscfg_exticr_reg_idx] |= (GPIO_BASE_ADDR_TO_PORT_CODE(pGPIOHanlde) << syscfg_exticr_bit_offset);

		/* EXTI peripheral related configuration */
		/* 3. Enable interrupt delivery from peripheral to the processor using the IMR (Interrupt Mask register in the EXTI module). */
		EXTI->IMR |= (1 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber);
	}

	/* Configure the PU/PD resistor */
	temp = (pGPIOHanlde->GPIO_PinConfig.GPIO_PinPuPdControl << reg_bit_offset);
	pGPIOHanlde->pGPIOx->PUPDR &= ~(0x3 << reg_bit_offset);
	pGPIOHanlde->pGPIOx->PUPDR |= temp;

	// Configurations relevant ONLY for output pins
	if (pGPIOHanlde->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT) {
		/* Configure the Speed */
		temp = (pGPIOHanlde->GPIO_PinConfig.GPIO_PinSpeed << reg_bit_offset);
		pGPIOHanlde->pGPIOx->OSPEEDR &= ~(0x3 << reg_bit_offset);
		pGPIOHanlde->pGPIOx->OSPEEDR |= temp;

		/* Configure the OTYPE (output type) */
		if (pGPIOHanlde->GPIO_PinConfig.GPIO_PinOPType) {
			pGPIOHanlde->pGPIOx->OTYPR |= (1 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber);
		} else {
			pGPIOHanlde->pGPIOx->OTYPR &= ~(1 << pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber);
		}
	}

	/* Configure the alternate functionality */
	if (pGPIOHanlde->GPIO_PinConfig.GPIO_PinMode < GPIO_MODE_ALTFN) {
		uint8_t GPIO_AFR_reg_h_l = pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber / 8; // 0: 0..7, 1: 8..15.
		reg_bit_offset = ((pGPIOHanlde->GPIO_PinConfig.GPIO_PinNumber % 8) * 4);
		uint32_t GPIO_AFR_reg_pin_val = (pGPIOHanlde->GPIO_PinConfig.GPIO_PinAltFunMode << reg_bit_offset);
		pGPIOHanlde->pGPIOx->AFR[GPIO_AFR_reg_h_l] &= ~(0x0f << reg_bit_offset);
		pGPIOHanlde->pGPIOx->AFR[GPIO_AFR_reg_h_l] |= GPIO_AFR_reg_pin_val;
	}

	// TODO: Add support for Analog ports
}

/**************************************************************
 * @fn			- GPIO_DeInit
 *
 * @brief		- This function De-initializes a GPIO port by setting its registers back to the reset values.
 *
 * @param[in]	- Base address of the GPIO Port Peripheral
 *
 * @return		- none
 *
 * @Note		- This will reset ALL the registers of specific GPIO port, not just a single pin of that port.
 *
 **************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA) {
		GPIOA_REGS_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REGS_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REGS_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REGS_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REGS_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REGS_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REGS_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REGS_RESET();
	}
}

/*
 * Data read and write
 */
/**************************************************************
 * @fn			- GPIO_ReadFromInputPin
 *
 * @brief		- This function reads a pin state and returns the read value. This may be applied on pin of any type.
 *
 * @param[in]	- Base address of the GPIO Port Peripheral
 * @param[in]	- The pin number to read
 *
 * @return		- port value: 1 or 0.
 *
 * @Note		- none
 *
 **************************************************************/
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum)
{
	uint8_t value = (uint8_t)(pGPIOx->IDR >> PinNum) & 0x1;
	return value;
}

/**************************************************************
 * @fn			- GPIO_ReadFromInputPort
 *
 * @brief		- This function reads a whole port and returns the read value. This may be applied on pin of any type.
 *
 * @param[in]	- Base address of the GPIO Port Peripheral
 *
 * @return		- 16 bit port value.
 *
 * @Note		- none
 *
 **************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)(pGPIOx->IDR & 0xffff);
}


/**************************************************************
 * @fn			- GPIO_WriteToOutputPin
 *
 * @brief		- This function sets pin value on specific port. This may be applied on an output pin only.
 *
 * @param[in]	- Base address of the GPIO Port Peripheral
 * @param[in]	- The pin number to write
 * @param[in]	- The value to set (use macros GPIO_PIN_SET or GPIO_PIN_RESET)
 *
 * @return		- none. // TODO: in case that this is applied on an input port, this function should abort and return error.
 *
 * @Note		- none
 *
 **************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t Val)
{
	if (Val == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNum);
	} else {
		pGPIOx->ODR &= ~(1 << PinNum);
	}
}



/**************************************************************
 * @fn			- GPIO_WriteToOutputPort
 *
 * @brief		- This function writes a 16 bit value to specific port. This may be applied on an output port only.
 *
 * @param[in]	- Base address of the GPIO Port Peripheral
 * @param[in]	- The 16 bit value to write to the port
 *
 * @return		- none. // TODO: in case that this is applied on an input port, this function should abort and return error.
 *
 * @Note		- none
 *
 **************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Val)
{
	pGPIOx->ODR = Val;
}


/**************************************************************
 * @fn			- GPIO_ToggleOutputPin
 *
 * @brief		- This function toggles a GPIO pin value. This may be applied on an output pin only.
 *
 * @param[in]	- Base address of the GPIO Port Peripheral
 * @param[in]	- The pin number to toggle
 *
 * @return		- none. // TODO: in case that this is applied on an input port, this function should abort and return error.
 *
 * @Note		- none
 *
 **************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum)
{
	pGPIOx->ODR ^= (1 << PinNum);
}



/*
 * IRQ configuration and ISR handling
 */

/**************************************************************
 * @fn			- GPIO_IRQInterrupEnDisCfg
 *
 * @brief		- This function set interrupt enable or disable for specific IRQ number.
 *                It configures the ISER / ICER registers of the MCU's NVIC.
 *
 * @param[in]	- IRQ Number
 * @param[in]	- Enable/Disable this interrupt.
 *
 * @return		- none.
 *
 * @Note		- none
 *
 **************************************************************/
void GPIO_IRQInterrupEnDisCfg(uint8_t IRQNum, uint8_t EnOrDis)
{
	uint8_t reg_offset_from_base = (IRQNum / 32) * 4;
	uint8_t bit_offset = IRQNum % 32;
	if (EnOrDis) {
		*((volatile uint32_t*) (NVIC_ISER_BASE + reg_offset_from_base)) = (1 << bit_offset);
	} else {
		*((volatile uint32_t*) (NVIC_ICER_BASE + reg_offset_from_base)) = (1 << bit_offset);
	}
}

/**************************************************************
 * @fn			- GPIO_IRQInterrupPriCfg
 *
 * @brief		- This function set interrupt priority for specific IRQ number.
 *                It configures the PRI register of the MCU's NVIC.
 *
 * @param[in]	- IRQ Number
 * @param[in]	- Priority for this interrupt
 *
 * @return		- none.
 *
 * @Note		- none
 *
 **************************************************************/
void GPIO_IRQInterrupPriCfg(uint8_t IRQNum, uint8_t IRQPrio)
{
	uint8_t reg_offset_from_base = (IRQNum / 4) * 4;
	uint8_t bit_offset = ((IRQNum % 4) * 8) + NUM_PRIO_BITS_UNUSED_STM32MCU;
	*((volatile uint32_t*) (NVIC_IPR_BASE + reg_offset_from_base)) |= (IRQPrio << (bit_offset));
}


/**************************************************************
 * @fn			- GPIO_IRQHandle
 *
 * @brief		- This function configures IRQ handler for a selected GPIO pin.
 *
 * @param[in]	- The Pin number 0..15
 *
 * @return		- none.
 *
 * @Note		- none
 *
 **************************************************************/
void GPIO_IRQHandle(uint8_t PinNum)
{
// The commented code in this function checks if there is bouncing problem with the switch or if the HW in the devpt board is giving a good debounce solution.
// it appears that the board is solving the switch bouncing issue in the HW.
//	static count = 0;
	if (EXTI->PR & (1 << PinNum)) {
		EXTI->PR = (1 << PinNum);
//		count ++;
//		if (count > 100)
//			while(1) {
//				;
//			}
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
		delay(4);
	}
}


