/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Aug 29 2020
 *      Author: AA
 */

#include <stdio.h>
#include "stm32f446xx_spi_driver.h"

/*
 * These functions are private for use in this driver file only and that
 * are not exposed accessible to be called from other locations. (marked using the static keyword)
 */
static void SPI_InterruptHandleTxe(SPI_Handle_t* pSPIHandle);
static void SPI_InterruptHandleRxne(SPI_Handle_t* pSPIHandle);
static void SPI_InterruptHandleOvr(SPI_Handle_t* pSPIHandle);
static void SPI_InterruptHandleModf(SPI_Handle_t* pSPIHandle);
static void SPI_InterruptHandleCrcErr(SPI_Handle_t* pSPIHandle);
static void SPI_InterruptHandleFre(SPI_Handle_t* pSPIHandle);



// TODO:
// * Add code in Software Slave Select to control the chosen SS pin when several GPIOs are in use
// * Mark the interrupt as handled when handling it
// * When is the interrupt enabled?
// * Add logic to enable the Error interrupts only if at least one of the sub sources is set, disable it if none is set
//   Specifically add code to enable and disable the OVR interrupts and the SPI_CR2_ERRIE_BIT_POS
// * add timeouts to the SPI RX function in order to prevent from getting stuck for a byte that will never arrive.
// * add support to work in other modes, like slave, master to many slaves (need several NSS pins), etc.
// * The OVR Error SPI Interrupt driver should be updated to include API to enable / disable this interrupt source.
// * Consider adding the option to mask all interrupt while the configuration is done and restore the interrupts global
//   mask state once done with the configuration (See PRIMASK register on the ARM Cortex MCU UG - "Core registers" chapter.
// * Consider adding a macro/function to allow enable/disable all interrupts in order to allow configuration of several
//   GPIOs and only once done enable the general IRQ

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
void SPI_PerClockControl (SPIAndI2s_RegDef_t* pSPIx, uint8_t EnOrDis)
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
void SPI_Init(SPI_Handle_t* pSPIHanlde)
{
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
void SPI_DeInit(SPIAndI2s_RegDef_t* pSPIx)
{
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
uint8_t SPI_GetFlagStatus (SPIAndI2s_RegDef_t* pSPIx, uint32_t FlagBitMask)
{
	if (pSPIx->SR & FlagBitMask) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*
 * Data TX and RX
 * Available TX/RX options: polling based (blocking), interrupt based (non-blocking), using DMA (currently not implemented)
 */

/**************************************************************
 * @fn			- SPI_DataTx
 *
 * @brief		- Blocking version of SPI Data TX (using busy wait polling)
 *
 * @param[in]	- Base address of the SPI or I2S peripheral
 * @param[in]	- Pointer to a buffer of the data to transmit
 * @param[in]	- The number of bytes to send
 *
 * @return		- none
 *
 * @Note		- This is a blocking function
 *
 **************************************************************/
void SPI_DataTx(SPIAndI2s_RegDef_t* pSPIx, uint8_t* pTxBuff, uint32_t numBytesToSend)
{
	while (numBytesToSend > 0) {
		// 1. Wait until TXE is empty
		// TODO: update the code to use interrupt instead of using busy wait.
		//while (!(pSPIx->SR & (1 << SPI_SR_TXE_BIT_POS))); // Busy wait to send the data byte.
		while (SPI_GetFlagStatus(pSPIx, SPI_SR_TXE_BIT_MASK) == FLAG_RESET);

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
 * @brief		- Blocking version of SPI Data RX (using busy wait polling)
 *
 * @param[in]	- Base address of the SPI or I2S peripheral
 * @param[in]	- Pointer to a buffer of the data to receive
 * @param[in]	- The number of bytes to receive
 *
 * @return		- none
 *
 * @Note		- none
 *
 **************************************************************/
void SPI_DataRx(SPIAndI2s_RegDef_t* pSPIx, uint8_t* pRxBuff, uint32_t numBytesToReceive)
{
	// TODO: Add code that return back to main within timeout of not receiving the expected numBytesToReceive on the SPI interface.
	while (numBytesToReceive > 0) {
		// 1. Wait until a byte will arrive (RXNE bit will be set to mark that RX buffer is Not Empty - new byte has arrived.)
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


/* Interrupt based TX, RX and errors handling functions (Non Blocking) */
/**************************************************************
 * @fn			- SPI_DataTxInt
 *
 * @brief		- Non blocking interrupt based version of SPI Data TX
 *
 * @param[in]	- SPI Handle of the relevant SPI peripheral
 * @param[in]	- Pointer to a buffer of the data to transmit
 * @param[in]	- The number of bytes to transmit
 *
 * @return		- SPI_TXRX_STATE__ACTIVE if the peripheral was already busy in other data transmission (the function call was ignored)
 * 				  or
 *                SPI_TXRX_STATE__INACTIVE if the peripheral was available to transmit the data.

 * @return		- The TX state that was during the call to the function.
 *                If returned SPI_TXRX_STATE__ACTIVE this means that the execution did not start since the
 *                device was already in TX state of other data transmission.
 *
 * @Note		- This is a Non-blocking function
 *
 **************************************************************/
SPI_TxRxState_t SPI_DataTxInt(SPI_Handle_t* pSPIHandle, uint8_t* pTxBuff, uint32_t numBytesToSend)
{
	SPI_TxRxState_t txState = pSPIHandle->TxState;
	if (txState == SPI_TXRX_STATE__INACTIVE) {
		//1. Save the Tx buffer address and Len information in the relevant global variables
		pSPIHandle->pTxBuff = pTxBuff;
		pSPIHandle->TxLen = numBytesToSend;

		//2. Mark the SPI State as busy in transmission so that no other code will start using...
		//   ..this SPI peripheral until the transmission will be over.
		pSPIHandle->TxState = SPI_TXRX_STATE__ACTIVE;

		//3. Enable the TXEIE control bit to get interrupt when TX of a byte of this SPI peripheral is ended.
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE_BIT_POS);
		//4. Data transmission will be handled in the ISR code.
	} else {
		printf("Trying to transmit data while the SPI peripheral is in the middle of other TX operation\n");
	}
	return txState;
}


/**************************************************************
 * @fn			- SPI_DataRxInt
 *
 * @brief		- Non blocking interrupt based version of SPI Data RX
 *
 * @param[in]	- SPI Handle of the relevant SPI peripheral
 * @param[in]	- Pointer to a buffer of the data to receive
 * @param[in]	- The number of bytes to receive
 *
 * @return		- SPI_TXRX_STATE__ACTIVE if the peripheral was already busy in other data reception (the function call was ignored)
 * 				  or
 *                SPI_TXRX_STATE__INACTIVE if the peripheral was available to receive the data.
 *
 * @Note		- none
 *
 **************************************************************/
SPI_TxRxState_t SPI_DataRxInt(SPI_Handle_t* pSPIHandle, uint8_t* pRxBuff, uint32_t numBytesToReceive)
{
	SPI_TxRxState_t rxState = pSPIHandle->RxState;
	if (rxState == SPI_TXRX_STATE__INACTIVE) {
		//1. Save the Rx buffer address and Len information in the relevant global variables
		pSPIHandle->pRxBuff = pRxBuff;
		pSPIHandle->RxLen = numBytesToReceive;

		//2. Mark the SPI State as busy in reception so that no other code will start using...
		//   ..this SPI peripheral until the required bytes reception will be over.
		pSPIHandle->RxState = SPI_TXRX_STATE__ACTIVE;

		//3. Enable the RXNEIE control bit to get interrupt when RX of a byte of this SPI peripheral is ended.
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE_BIT_POS);
		//4. Data reception will be handled in the ISR code.
	} else {
		printf("Trying to receive data while the SPI peripheral is in the middle of other RX operation\n");
	}
	return rxState;
}


/**************************************************************
 * @fn			- SPI_SSIConfig
 *
 * @brief		- Set the SSI to High or Low - to select the NSS (Negative Slave Select) pin internally by SW, when using SSM=1 (Software Slave Management).
 *
 * @param[in]	- Pointer to the SPI peripheral's registers
 * @param[in]	- GPIO_PIN_SET or GPIO_PIN_RESET
 *
 * @return		- TXRX_STATE__ACTIVE if the peripheral was already busy in reception and in use (action not taken care of) or
 *                TXRX_STATE__INACTIVE if the peripheral was available to transmit the data.
 *
 * @Note		- When using SSM = 1 and acting as Master the pin must be set to high in order to avoid SPI MODF Error
 *
 **************************************************************/
void SPI_SSIConfig(SPIAndI2s_RegDef_t* pSPIx, uint8_t NSSPinSetOrReset)
{
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
void SPI_SSOEConfig(SPIAndI2s_RegDef_t* pSPIx, uint8_t SSOESetOrReset)
{
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
void SPI_PerControl(SPIAndI2s_RegDef_t* pSPIx, uint8_t EnOrDis)
{
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
 * @param[in]	- IRQ Number
 * @param[in]	- Enable/Disable this interrupt.
 *
 * @return		- none.
 *
 * @Note		- none
 *
 **************************************************************/
// TODO: Actually this function is the same for all interrupts - not only for this peripheral.
//       Consider moving it to a more centralized place and call this code from here as well as from the GPIO_IRQInterrupEnDisCfg() function
//       in order to avoid code duplication.
void SPI_IRQInterrupEnDisCfg(uint8_t IRQNum, uint8_t EnOrDis)
{
	uint8_t reg_offset_from_base = (IRQNum / 32) * 4;
	uint8_t bit_offset = IRQNum % 32;
	if (EnOrDis == ENABLE) {
		*((volatile uint32_t*) (NVIC_ISER_BASE + reg_offset_from_base)) = (1 << bit_offset);
	} else {
		*((volatile uint32_t*) (NVIC_ICER_BASE + reg_offset_from_base)) = (1 << bit_offset);
	}
}


/**************************************************************
 * @fn			- SPI_IRQInterrupPriCfg
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
void SPI_IRQInterrupPriCfg(uint8_t IRQNum, uint8_t IRQPrio)
{
	uint8_t reg_offset_from_base = (IRQNum / 4) * 4;
	uint8_t bit_offset = ((IRQNum % 4) * 8) + NUM_PRIO_BITS_UNUSED_STM32MCU;
	*((volatile uint32_t*) (NVIC_IPR_BASE + reg_offset_from_base)) |= (IRQPrio << (bit_offset));
}


/**************************************************************
 * @fn			- SPI_GetRxState
 *
 * @brief		- Returns the Rx State of the relevant SPI Peripheral.
 * 				  In state SPI_TXRX_STATE__ACTIVE when the device is in process of collecting expected bytes using the interrupt Rx function,
 * 				  else in SPI_TXRX_STATE__INACTIVE state.
 *
 * @param[in]	- Pointer to the SPI peripheral's registers
 *
 * @return		- RX State.
 *
 * @Note		- none
 *
 **************************************************************/
SPI_TxRxState_t SPI_GetRxState(SPI_Handle_t* pSPIHandle)
{
	return pSPIHandle->RxState;
}


/**************************************************************
 * @fn			- SPI_GetTxState
 *
 * @brief		- Returns the Tx State of the relevant SPI Peripheral.
 * 				  In state SPI_TXRX_STATE__ACTIVE when the device is in process of Sending bytes using the interrupt Tx function,
 * 				  else in SPI_TXRX_STATE__INACTIVE state.
 *
 * @param[in]	- Pointer to the SPI peripheral's registers
 *
 * @return		- TX State.
 *
 * @Note		- none
 *
 **************************************************************/
SPI_TxRxState_t SPI_GetTxState(SPI_Handle_t* pSPIHandle)
{
	return pSPIHandle->TxState;
}


/**************************************************************
 * @fn			- SPI_ClearOVRFlag
 *
 * @brief		- This function performs the sequence of clearing the OVR interrupt flag.
 *                It reads the DR and the SR.
 *                As a result of calling it - the last received byte, located in the Data register will be lost,
 *                so if the content of the last received byte is important to the application layer,
 *                the programmer should perform the last read before calling this function.
 *                If Overflow occurred on the RX channel of the SPI peripheral and the device was NOT in RX mode,
 *                the the driver will clean this flag.
 *                However if the spi peripheral was in the middle of data bytes reception process (using the SPI RX interrupt function),
 *                the driver did not clear this error bit and left this
 *                to be done by the application.
 *                The programmer may check if the SPI peripheral is in the middle of RX or not using
 *                in order to decide on his actions by calling to SPI_GetRxState
 *
 *
 * @param[in]	- Pointer to the SPI peripheral's registers
 *
 * @return		- none.
 *
 * @Note		- TODO: Add more functions to to support this procedure by the application: Is there an unread byte?
 *                We don't want to get stuck waiting forever for a byte to arrive - we only want to read the byte if it was already received.
 *                One options is to add function get RXNE flag status to allow the application to read the DR only if RXNE is already set...
 *
 **************************************************************/
void SPI_ClearOVRFlag(SPIAndI2s_RegDef_t* pSPIx)
{
	// Clearing the OVR bit is done by a read access to the SPI_DR register followed by a read access to the SPI_SR register.
	uint16_t dummyRead;
	dummyRead = pSPIx->DR;
	dummyRead = pSPIx->SR;
	(void)dummyRead; // in order to prevent warning about unused variable.
}


/**************************************************************
 * @fn			- SPI_CloseTransmission
 *
 * @brief		- Used to mark that the device is not transmitting anymore (once the transmission of the requested number of bytes
 *                using the TX interrupt function was over). The function sets all the peripheral's data structure in TX Inactive state.
 *                In addition it can be used to stop the transmission via the interrupt TX function by the application layer,
 *                even if TX of all bytes was not finished.
 *
 *
 * @param[in]	- Pointer to the SPI peripheral's registers
 *
 * @return		- none.
 *
 * @Note		- none.
 *
 **************************************************************/
void SPI_CloseTransmission(SPI_Handle_t* pSPIHandle)
{
	// Disable new TXE interrupts
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE_BIT_POS);
	pSPIHandle->pTxBuff = NULL;
	pSPIHandle->TxState = SPI_TXRX_STATE__INACTIVE;
	pSPIHandle->TxLen = 0; // used in case of call to this function from the Application layer, before the data was completed to be transmitted.
}


/**************************************************************
 * @fn			- SPI_CloseReception
 *
 * @brief		- Used to mark that the device is not receiving bytes anymore (once the reception of the requested number of bytes
 *                using the RX interrupt function was over). The function sets all the peripheral's data structure in RX Inactive state.
 *                In addition it can be used to stop the Reception via the interrupt RX function by the application layer,
 *                even if RX of all bytes was not finished.
 *
 *
 * @param[in]	- Pointer to the SPI peripheral's registers
 *
 * @return		- none.
 *
 * @Note		- TODO: maybe we should also check the currently the data is not being transmitted and only once it is done read the
 *                last data byte from the DR and put it in the data buffer. only need to verify that RXNE is set before doing this
 *                else this will be the last byte that was already read. this check is needed only when this function is called from
 *                the application, since from the driver the call to the function will always be at the end of the data transmission.
 *
 **************************************************************/
void SPI_CloseReception(SPI_Handle_t* pSPIHandle)
{
	// prevent more interrupts from RXNE source.
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE_BIT_POS);
	//pSPIHandle->pRxBuff = NULL; // TODO: Verify that this not needed
	pSPIHandle->RxState = SPI_TXRX_STATE__INACTIVE;
	pSPIHandle->RxLen = 0; // used in case of call to this function from the Application layer, before the data was completed to be received.
}


/**************************************************************
 * @fn			- SPI_InterruptHandleTxe
 *
 * @brief		- This function is called from the ISR function to handle SPI TXE event (Marks end of TXed byte)
 *
 * @param[in]	- The handle to the SPI Peripheral
 *
 * @return		- none.
 *
 * @Note		- none
 *
 **************************************************************/
static void SPI_InterruptHandleTxe(SPI_Handle_t* pSPIHandle)
{
	// 1. Write new data byte(s) (from RX Buffer to DR) according to the frame format, and decrement the number of bytes to send accordingly.
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF_BIT_POS)) {
		// 16 bit Data Frame Format
		if (pSPIHandle->TxLen > 1) {
			pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuff);
			pSPIHandle->TxLen -= 2;
			(uint16_t*)pSPIHandle->pTxBuff++;
		} else {
			// Last byte - write only 1 data byte into the 16 bit register leaving the value of the other 8 bits...
			// ..as zero, in order to prevent writing garbage data in the last byte.
			pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuff;
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuff++;
		}
	} else {
		// 8 bit Data Frame Format
		//printf("sending on SPI: %c\n", (uint8_t)*pTxBuff);
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuff;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuff++;
	}
	// 2. If no more bytes left to transmit than mark the peripheral as not transmitting anymore.
	if (pSPIHandle->TxLen == 0) {
		// Close the transmission
		SPI_CloseTransmission(pSPIHandle);
		// Report to the application on completion of transmission of the requested bytes
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_APP_CALLBACK__TX_CMPLT);
	}
}


/**************************************************************
 * @fn			- SPI_InterruptHandleRxne
 *
 * @brief		- This function is called from the ISR function to handle SPI RXNE event (marks RXed byte)
 *
 * @param[in]	- The handle to the SPI Peripheral
 *
 * @return		- none.
 *
 * @Note		- none
 *
 **************************************************************/
static void SPI_InterruptHandleRxne(SPI_Handle_t* pSPIHandle)
{
	// 1. Read new data byte(s) (from the DR to RX Buffer) according to the frame format, and decrement the number of bytes to read accordingly.
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF_BIT_POS)) {
		// 16 bit Data Frame Format
		if (pSPIHandle->RxLen > 1) {
			*((uint16_t*)pSPIHandle->pRxBuff) = pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen -= 2;
			(uint16_t*)pSPIHandle->pRxBuff++;
		} else {
			// Last byte - read only 1 data byte into the 16 bit register leaving the value of the other 8 bits...
			// ..as zero, in order to prevent writing garbage data in the last byte.
			*pSPIHandle->pRxBuff = pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuff++;
		}
	} else {
		*pSPIHandle->pRxBuff = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuff++;
	}
	// 2. If no more bytes left to receive than mark the peripheral as not receiving anymore.
	if (pSPIHandle->RxLen == 0) {
		// Close the reception
		SPI_CloseReception(pSPIHandle);
		// Report to the application on completion of receiving the expected number of bytes.
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_APP_CALLBACK__RX_CMPLT);
	}
}


/**************************************************************
 * @fn			- SPI_InterruptHandleOvr
 *
 * @brief		- This function is called from the ISR function to handle SPI Overrun Error.
 * 				  It marks RXed Byte(s) before the DR was read - lost of data byte or more.
 * 				  The function  is called from the ISR Only if this interrupt source is enabled.
 *
 * @param[in]	- The handle to the SPI Peripheral
 *
 * @return		- none.
 *
 * @Note		- none
 *
 **************************************************************/
static void SPI_InterruptHandleOvr(SPI_Handle_t* pSPIHandle)
{
 /*
  * Overrun Error flag (OVR)
  * An overrun condition occurs when the master or the slave completes the reception of the
  * next data frame while the read operation of the previous frame from the Rx buffer has not
  * completed (case RXNE flag is set).
  * In this case, the content of the Rx buffer is not updated with the new data received. A read
  * operation from the SPI_DR register returns the frame previously received. All other
  * subsequently transmitted data are lost.
  * Clearing the OVR bit is done by a read access to the SPI_DR register followed by a read
  * access to the SPI_SR register.
  */
	/*
	 * 1. Clear the OVR Flag
	 * The clearing is done by reading the DR and then reading the SR.
	 * In case that the communication is in the middle of Reception, the OVR flag will not be
	 *  cleared from the interrupt level, in order not to clear the content of the DR by the application.
	 */
	if (pSPIHandle->RxState != SPI_TXRX_STATE__ACTIVE) {
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}
	// 2. Inform the application about the Overrun error
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_APP_CALLBACK__MODF_ERROR);
}


/**************************************************************
 * @fn			- SPI_InterruptHandleModf
 *
 * @brief		- This function is called from the ISR function to handle Master Mode Fault Error.
 * 				  The function  is called from the ISR Only if this interrupt source is enabled.
 *
 * @param[in]	- The handle to the SPI Peripheral
 *
 * @return		- none.
 *
 * @Note		- Currently NOT implemented. Do not enable this interrupt source until the function will be implemented.
 *
 **************************************************************/
static void SPI_InterruptHandleModf(SPI_Handle_t* pSPIHandle)
{
	// TODO: Add support to this interrupt source if it will be needed and update the above comment at the note field
}


/**************************************************************
 * @fn			- SPI_InterruptHandleCrcErr
 *
 * @brief		- This function is called from the ISR function to handle CRC Error.
 * 				  The function  is called from the ISR Only if this interrupt source is enabled.
 *
 * @param[in]	- The handle to the SPI Peripheral
 *
 * @return		- none.
 *
 * @Note		- Currently NOT implemented. Do not enable this interrupt source until the function will be implemented.
 *
 **************************************************************/
static void SPI_InterruptHandleCrcErr(SPI_Handle_t* pSPIHandle)
{
	// TODO: Add support to this interrupt source if it will be needed and update the above comment at the note field
}


/**************************************************************
 * @fn			- SPI_InterruptHandleFre
 *
 * @brief		- This function is called from the ISR function to handle TI Frame Format Error.
 * 				  The function  is called from the ISR Only if this interrupt source is enabled.
 *
 * @param[in]	- The handle to the SPI Peripheral
 *
 * @return		- none.
 *
 * @Note		- Currently NOT implemented. Do not enable this interrupt source until the function will be implemented.
 *
 **************************************************************/
static void SPI_InterruptHandleFre(SPI_Handle_t* pSPIHandle)
{
	// TODO: Add support to this interrupt source if it will be needed and update the above comment at the note field
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
void SPI_IRQHandle(SPI_Handle_t* pSPIHandle)
{
	/* Check the source of the SPI interrupt */
	/* SPI TX/RX related interrupts */
	if ((pSPIHandle->pSPIx->SR  & (SPI_SR_TXE_BIT_MASK)) &&
		(pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE_BIT_POS))) {
		// TXE and this interrupt source is enabled
		SPI_InterruptHandleTxe(pSPIHandle);
	}

	if ((pSPIHandle->pSPIx->SR  & (SPI_SR_RXNE_BIT_MASK)) &&
		(pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE_BIT_POS))) {
		// RXNE and this interrupt source is enabled
		SPI_InterruptHandleRxne(pSPIHandle);
	}

	/* SPI Errors related interrupts */
	if ((pSPIHandle->pSPIx->SR  & (SPI_SR_OVR_BIT_MASK)) &&
		(pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE_BIT_POS))) {
		// Overrun Error and this interrupt source is enabled
		SPI_InterruptHandleOvr(pSPIHandle);
	}
	if ((pSPIHandle->pSPIx->SR  & (SPI_SR_MODF_BIT_POS)) &&
		(pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE_BIT_POS))) {
		// Master Mode Fault Error and this interrupt source is enabled
		SPI_InterruptHandleModf(pSPIHandle);
	}
	if ((pSPIHandle->pSPIx->SR  & (SPI_SR_CRC_ERR_BIT_MASK)) &&
		(pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE_BIT_POS))) {
		// CRC Error and this interrupt source is enabled
		SPI_InterruptHandleCrcErr(pSPIHandle);
	}
	if ((pSPIHandle->pSPIx->SR  & (SPI_SR_FRE_BIT_MASK)) &&
		(pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE_BIT_POS))) {
		// TI Frame Format Error and this interrupt source is enabled
		SPI_InterruptHandleFre(pSPIHandle);
	}
}


/**************************************************************
 * @fn			- SPI_ApplicationEventCallback
 *
 * @brief		- This function should be implemented in the application layer.
 *                The interrupt layer will report SPI related events using this function to the application.
 *                Relevant events are listed in SPI_EventAppCallback_t
 *
 * @param[in]	- The handle to the SPI Peripheral
 * @param[in]	- SPI related event to be reported by the interrupt layer.
 *
 * @return		- none
 *
 * @Note		- This function should be implemented in the application layer
 *
 **************************************************************/
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t* pSPIHandle, SPI_EventAppCallback_t SPI_EventApp)
{
	/*
	 * This is a weak function - in order to allow the user application to implement this function
	 * and override the content of this empty function.
	 * A skeleton of the main application is suggested in comments below.
	 */
	printf("SPI_EventApp: %d ==> ", SPI_EventApp);
	switch (SPI_EventApp) {
		case SPI_EVENT_APP_CALLBACK__TX_CMPLT: {
			// The requested buffer transmission was just completed.
			printf("weak implementation from the driver: SPI_EVENT_APP_CALLBACK__TX_CMPLT (%d)\n", SPI_EventApp);
			break;
		}
		case SPI_EVENT_APP_CALLBACK__RX_CMPLT: {
			// The requested number of bytes reception was just completed. the data is pending in the rxBuffer
			printf("weak implementation from the driver: SPI_EVENT_APP_CALLBACK__RX_CMPLT (%d)\n", SPI_EventApp);
			break;
		}
		case SPI_EVENT_APP_CALLBACK__OVR_ERROR: {
			// OVR Error occurred in the RX channel
			printf("weak implementation from the driver: SPI_EVENT_APP_CALLBACK__OVR_ERROR (%d)\n", SPI_EventApp);
			break;
		}
		case SPI_EVENT_APP_CALLBACK__CRC_ERROR: {
			// CRC Error identified in the SPI communication
			printf("weak implementation from the driver: SPI_EVENT_APP_CALLBACK__CRC_ERROR (%d)\n", SPI_EventApp);
			break;
		}
		case SPI_EVENT_APP_CALLBACK__MODF_ERROR: {
			// MODF Error identified in the SPI communication
			printf("weak implementation from the driver: SPI_EVENT_APP_CALLBACK__MODF_ERROR (%d)\n", SPI_EventApp);
			break;
		}
		case SPI_EVENT_APP_CALLBACK__FRE_ERROR: {
			// FRE Error identified in the SPI communication
			printf("weak implementation from the driver: SPI_EVENT_APP_CALLBACK__FRE_ERROR (%d)\n", SPI_EventApp);
			break;
		}
		default: {
		}
	}
}

