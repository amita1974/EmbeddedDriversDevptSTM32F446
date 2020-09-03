/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Aug 29 2020
 *      Author: AA
 */

#include <stdio.h>
#include "stm32f446xx_spi_driver.h"

/*
 * Peripheral clock setup
 */
/**************************************************************
 * @fn			- SPIandI2s_PeriClockControl
 *
 * @brief		- This function enables or disables Peripheral clock for SPIx/I2Sx peripheral (x = 1/2/3/4).
 *
 * @param[in]	- Base address of the SPI or I2S peripheral
 * @param[in]	- ENABLE or DISABLE
 *
 * @return		- none
 *
 * @Note		- none
 *
 **************************************************************/
void SPI_PerClockControl (SPIAndI2s_RegDef_t * pSPIx, uint8_t EnOrDis)
{
	if (EnOrDis == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DIS();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DIS();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DIS();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_DIS();
		}
	}
}


/*
 * Init and De-init
 */
/**************************************************************
 * @fn			- SPI_Init
 *
 * @brief		- Configure the relevant SPI peripheral with the values set in the SPI_Handle configuration struct
 *
 * @param[in]	- The SPI peripheral configuration struct and pinter to the relevant SP device to be configured.
 *
 * @return		- none
 *
 * @Note		- none
 *
 **************************************************************/
void SPI_Init(SPI_Handle_t* pSPIHanlde) {
	// Peripheral Clock enable
	SPI_PerClockControl(pSPIHanlde->pSPIx, ENABLE);

	uint32_t spi_cr1_reg = 0;
	// 1. Configure the device mode
	spi_cr1_reg |= pSPIHanlde->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR_BIT_POS;
	// 2. Configure the bus config
	if (SPI_BUS_CONFIG__FULL_DUPLEX == pSPIHanlde->SPIConfig.SPI_BusConfig) {
		// BIDI mode should be cleared
		spi_cr1_reg &= ~(1 << SPI_CR1_BIDI_MODE_BIT_POS);
	} else if (SPI_BUS_CONFIG__HALF_DUPLEX== pSPIHanlde->SPIConfig.SPI_BusConfig) {
		// BIDI mode should be set
		spi_cr1_reg |= (1 << SPI_CR1_BIDI_MODE_BIT_POS);
	} else if (SPI_BUS_CONFIG__SYMPLEX_RX_ONLY == pSPIHanlde->SPIConfig.SPI_BusConfig) {
		// BIDI mode should be cleared
		spi_cr1_reg &= ~(1 << SPI_CR1_BIDI_MODE_BIT_POS);
		// RXONLY mode should be set
		spi_cr1_reg |= (1 << SPI_CR1_RX_ONLY_BIT_POS);
	} else if (SPI_BUS_CONFIG__SYMPLEX_TX_ONLY == pSPIHanlde->SPIConfig.SPI_BusConfig) {
		// BIDI mode should be cleared
		spi_cr1_reg &= ~(1 << SPI_CR1_BIDI_MODE_BIT_POS);
		// TODO: Finish this code, check if need to configure bit 14 BIIOE in any of the cases of the bus config.
	}

	// 3. Configure the Serial Clock Speed
	spi_cr1_reg |= (pSPIHanlde->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR_BIT_POS);
	// 4. Configure the Data Frame format
	spi_cr1_reg |= (pSPIHanlde->SPIConfig.SPI_DFF << SPI_CR1_DFF_BIT_POS);

	// 5. Configure the Clock Polarity and Phase
	spi_cr1_reg |= (pSPIHanlde->SPIConfig.SPI_CPOL << SPI_CR1_CPOL_BIT_POS);
	spi_cr1_reg |= (pSPIHanlde->SPIConfig.SPI_CPHA << SPI_CR1_CPHA_BIT_POS);
	// 6. Configure the Software Slave Management
	spi_cr1_reg |= (pSPIHanlde->SPIConfig.SPI_SSM << SPI_CR1_SSM_BIT_POS);
	// In Software Slave Management mode, set the SSI to 1, in order to pull using the SW the NSS pin to High. This will avoid MODF error.
	if (pSPIHanlde->SPIConfig.SPI_SSM == SPI_SSM__SW) {
		spi_cr1_reg |= (1 << SPI_CR1_SSI_BIT_POS);
	}

	// Write the calculated configuration value to the register(s) of the relevant device.
	pSPIHanlde->pSPIx->CR1 = spi_cr1_reg;
}

/**************************************************************
 * @fn			- SPI_DeInit
 *
 * @brief		- Set all the registers of specified SPI peripheral to their reset values
 *
 * @param[in]	- Base address of the SPI or I2S peripheral
 *
 * @return		- none
 *
 * @Note		- none
 *
 **************************************************************/
void SPI_DeInit(SPIAndI2s_RegDef_t *pSPIx) { // this will reset ALL the registers of specific SPI port.
	if (pSPIx == SPI1) {
		SPI1_REGS_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REGS_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REGS_RESET();
	} else if (pSPIx == SPI4) {
		SPI4_REGS_RESET();
	}
}

/**************************************************************
 * @fn			- SPI_GetFlagStatus
 *
 * @brief		- returns the status of a single bit flag in Status Register (SR) of the relevant SPI peripheral
 *
 * @param[in]	- Base address of the SPI or I2S peripheral
 *
 * @param[in]	- The flag mask
 *
 * @return		- FLAG_SET / FLAG_RESET macro.
 *
 * @Note		- none
 *
 **************************************************************/
uint8_t SPI_GetFlagStatus (SPIAndI2s_RegDef_t *pSPIx, uint32_t FlagBitMask) {
	if (pSPIx->SR & FlagBitMask) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Data TX and RX
 * Available tx/rx options: polling based (blocking), interrupt based (non-blocking), using DMA (currently not implemented)
 */

/**************************************************************
 * @fn			- SPI_DataTx
 *
 * @brief		-
 *
 * @param[in]	- Base address of the SPI or I2S peripheral
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- This is a blocking function
 *
 **************************************************************/
void SPI_DataTx(SPIAndI2s_RegDef_t *pSPIx, uint8_t* pTxBuff, uint32_t numBytesToSend) {
	while (numBytesToSend > 0) {
		// 1. Wait until TXE is empty
		// TODO: update the code to use interrupt instead of using busy wait.
		//while (!(pSPIx->SR & (1 << SPI_SR_TXE_BIT_POS))); // Busy wait to send the data byte.
		while (SPI_GetFlagStatus(pSPIx, SPI_SR_TXE_BIT_MASK) == FLAG_RESET);
		for (int j = 0; j < 100; j++) {
			;// TODO: This delay is needed in order to allow the code to work - not clear why !
		}

		// 2. Write new data byte(s) (from RX Buffer to DR) according to the frame format, and decrement the number of bytes to send accordingly.
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF_BIT_POS)) {
			// 16 bit Data Frame Format
			if (numBytesToSend > 1) {
				pSPIx->DR = *((uint16_t*)pTxBuff);
				numBytesToSend -= 2;
				(uint16_t*)pTxBuff++;
			} else {
				// Last byte - write only 1 data byte into the 16 bit register leaving the value of the other 8 bits...
				// ..as zero, in order to prevent writing garbage data in the last byte.
				pSPIx->DR = *pTxBuff;
				numBytesToSend--;
				pTxBuff++;
			}
		} else {
			// 8 bit Data Frame Format
			//printf("sending on SPI: %c\n", (uint8_t)*pTxBuff);
			pSPIx->DR = *pTxBuff;
			numBytesToSend--;
			pTxBuff++;
		}
	}
}

/**************************************************************
 * @fn			- SPI_DataRx
 *
 * @brief		-
 *
 * @param[in]	- Base address of the SPI or I2S peripheral
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 *
 **************************************************************/
void SPI_DataRx(SPIAndI2s_RegDef_t *pSPIx, uint8_t* pRxBuff, uint32_t numBytesToReceive) {
	while (numBytesToReceive > 0) {
		// 1. Wait until a byte will arrive (RXNE is not empty)
		// TODO: update the code to use interrupt instead of using busy wait.
		while (SPI_GetFlagStatus(pSPIx, SPI_SR_RXNE_BIT_MASK) == FLAG_RESET);

		// 2. Read data byte(s) (from DR to RX Buffer) according to the frame format, and decrement the number of bytes to receive accordingly.
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF_BIT_POS)) {
			// 16 bit Data Frame Format
			if (numBytesToReceive > 1) {
				*((uint16_t*)pRxBuff) = pSPIx->DR;
				numBytesToReceive -= 2;
				(uint16_t*)pRxBuff++;
			} else {
				// Note: when using 16 bits frame format, the value in numBytesToReceive must be even, else the code will get stuck.
				// to prevent this - if this happens - get out of the function at the last byte, since sending odd number of bytes will not make the last byte complete and will not cause the interrupt to occure.
				numBytesToReceive--;
			}
		} else {
			// 8 bit Data Frame Format
			*pRxBuff = pSPIx->DR;
			numBytesToReceive--;
			pRxBuff++;
		}
	}
}


/**************************************************************
 * @fn			- SPI_SSIConfig
 *
 * @brief		- Set the SSI to High or Low - to select the NSS (Negative Slave Select) pin internally by SW, when using SSM=1 (Software Slave Management).
 *
 * @param[in]	- Pointer to the SPI peripheral's registers
 * @param[in]	- GPIO_PIN_SET or GPIO_PIN_RESET
 *
 * @return		- none
 *
 * @Note		- When using SSM = 1 and acting as Master the pin must be set to high in order to avoid SPI MODF Error
 *
 **************************************************************/
void SPI_SSIConfig(SPIAndI2s_RegDef_t *pSPIx, uint8_t NSSPinSetOrReset) {
	if (NSSPinSetOrReset == GPIO_PIN_SET) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI_BIT_POS);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI_BIT_POS);
	}
}


/**************************************************************
 * @fn			- SPI_SSOEConfig
 *
 * @brief		- Set the SSOE (Slave Select Output Enable) to fit the configuration of Master with SSM disabled (HW control).
 * 				  This will cause the HW to automatically set the NSS pin to low as soon as the SPI peripheral will be enabled, and back to high when the SPI will be disabled.
 * 				  (i.e. for SSM = 0, when SPE=1 NSS will be pulled to low and NSS will be high when SPE = 0)
 * 				  For more details see "STM32F446xx Reference manual.pdf" at chapter "26.3.5 Slave select (NSS) pin management"
 *
 * @param[in]	- Pointer to the SPI peripheral's registers
 * @param[in]	- GPIO_PIN_SET or GPIO_PIN_RESET
 *
 * @return		- none
 *
 * @Note		-
 *
 **************************************************************/
void SPI_SSOEConfig(SPIAndI2s_RegDef_t *pSPIx, uint8_t SSOESetOrReset) {
	if (SSOESetOrReset == SET) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE_BIT_POS);
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE_BIT_POS);
	}
}



/**************************************************************
 * @fn			- SPI_PerControl
 *
 * @brief		-
 *
 * @param[in]	- Pointer to the SPI peripheral's registers
 * @param[in]	- ENABLE or DISABL
 *
 * @return		- none
 *
 * @Note		- The SPI Peripheral configuration should be done BEFORE calling to this function !
 *
 **************************************************************/
void SPI_PerControl (SPIAndI2s_RegDef_t * pSPIx, uint8_t EnOrDis) {
	if (EnOrDis == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE_BIT_POS);
	} else {
		// TODO: See the important note about disable SPI procedure in the RM - Section "26.3.10 Procedure for disabling the SPI"
		/*
		 * from the RM:
		 * The correct disable procedure is (except when receive-only mode is used):
		 * 1. Wait until RXNE=1 to receive the last data.
		 * 2. Wait until TXE=1 and then wait until BSY=0 before disabling the SPI.
		 * 3. Read received data.
		 */
		// Currently only working with RX so implementing partial solution without the RXNE and data reading part. still left as TODO.

		// wait until the TXE and BSY flags of the SPI will be cleared before turning of the SPI peripheral.
		while (SPI_GetFlagStatus(pSPIx, SPI_SR_TXE_BIT_MASK) == FLAG_RESET);
		while (SPI_GetFlagStatus(pSPIx, SPI_SR_BSY_BIT_MASK) == FLAG_SET);
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE_BIT_POS);
		// and update the code to follow the described rules.
	}
}

/*
 * IRQ configuration and ISR handling
 */
/**************************************************************
 * @fn			- SPI_IRQInterrupEnDisCfg
 *
 * @brief		-
 *
 * @param[in]	- Base address of the SPI or I2S peripheral
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 *
 **************************************************************/
void SPI_IRQInterrupEnDisCfg(uint8_t IRQNum, uint8_t EnOrDis) {

}

/**************************************************************
 * @fn			- SPI_IRQInterrupPriCfg
 *
 * @brief		-
 *
 * @param[in]	- Base address of the SPI or I2S peripheral
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 *
 **************************************************************/
void SPI_IRQInterrupPriCfg(uint8_t IRQNum, uint8_t IRQPrio) {

}

/**************************************************************
 * @fn			- SPI_IRQHandle
 *
 * @brief		-
 *
 * @param[in]	- Base address of the SPI or I2S peripheral
 * @param[in]	-
 *
 * @return		- none
 *
 * @Note		- none
 *
 **************************************************************/
void SPI_IRQHandle(SPI_Handle_t *pHandle) {

}

