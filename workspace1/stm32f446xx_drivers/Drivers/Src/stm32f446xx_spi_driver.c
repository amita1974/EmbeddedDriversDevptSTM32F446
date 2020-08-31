/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Aug 29 2020
 *      Author: AA
 */

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
	// 6. Configure the Software Slave Management
	spi_cr1_reg |= (pSPIHanlde->SPIConfig.SPI_CPHA << SPI_CR1_CPHA_BIT_POS);

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
 * @Note		- none
 *
 **************************************************************/
void SPI_DataTx(SPIAndI2s_RegDef_t *pSPIx, uint8_t* pTxBuff, uint32_t numBytesToSend) {

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

