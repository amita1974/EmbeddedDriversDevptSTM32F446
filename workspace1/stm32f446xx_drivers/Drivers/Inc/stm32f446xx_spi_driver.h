/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Aug 29 2020
 *      Author: AA
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"


/* Configuration structure for a SPI pin */
typedef enum {
	SPI_DVICE_MODE__SLAVE = 0,
	SPI_DVICE_MODE__MASTER,
} SPI_DeviceMode_t;

typedef enum {
	SPI_BUS_CONFIG__FULL_DUPLEX = 1,
	SPI_BUS_CONFIG__HALF_DUPLEX,
	SPI_BUS_CONFIG__SYMPLEX_TX_ONLY,
	SPI_BUS_CONFIG__SYMPLEX_RX_ONLY,
} SPI_BusConfig_t;

typedef enum {
	SPI_BUS_SPEED__DIV2 = 0,
	SPI_BUS_SPEED__DIV4,
	SPI_BUS_SPEED__DIV8,
	SPI_BUS_SPEED__DIV16,
	SPI_BUS_SPEED__DIV32,
	SPI_BUS_SPEED__DIV64,
	SPI_BUS_SPEED__DIV128,
	SPI_BUS_SPEED__DIV256,
} SPI_SclkSpeed_t;

typedef enum {
	SPI_DFF__8_BITS = 0,
	SPI_DFF__16_BITS,
} SPI_Dff_t;

typedef enum {
	SPI_CPOL__CK_IDLE_LOW = 0,
	SPI_CPOL__CK_IDLE_HIGH,
} SPI_Cpol_t;

typedef enum {
	SPI_CPHA__1ST_CLK_SAMPLE = 0,
	SPI_CPHA__2ND_CLK_SAMPLE,
} SPI_Cpha_t;

typedef enum {
	SPI_SSM__HW = 0,
	SPI_SSM__SW,
} SPI_Ssm_t;

typedef enum {
	SPI_TXRX_STATE__INACTIVE = 0,
	SPI_TXRX_STATE__ACTIVE,
} SPI_TxRxState_t;

typedef enum {
	SPI_EVENT_APP_CALLBACK__TX_CMPLT = 1,
	SPI_EVENT_APP_CALLBACK__RX_CMPLT,
	SPI_EVENT_APP_CALLBACK__OVR_ERROR,
	SPI_EVENT_APP_CALLBACK__CRC_ERROR,
	SPI_EVENT_APP_CALLBACK__MODF_ERROR,
	SPI_EVENT_APP_CALLBACK__FRE_ERROR,
} SPI_EventAppCallback_t;

typedef struct {
	SPI_DeviceMode_t SPI_DeviceMode;	/*!<  >*/
	SPI_BusConfig_t	SPI_BusConfig;		/*!<  >*/
	SPI_SclkSpeed_t	SPI_SclkSpeed;		/*!<  >*/
	SPI_Dff_t		SPI_DFF;			/*!<  >*/
	SPI_Cpol_t		SPI_CPOL;			/*!<  >*/
	SPI_Cpha_t		SPI_CPHA;			/*!<  >*/
	SPI_Ssm_t		SPI_SSM;			/*!<  >*/
} SPI_Config_t;


typedef struct {
	SPIAndI2s_RegDef_t*	pSPIx; 	/* !< The base address of SPIx peripheral (x = 1/2/3/4) */
	SPI_Config_t 		SPIConfig;	/* !< SPI configuration struct passed from the application to the driver */
	uint8_t*			pTxBuff;	/* !< Pointer to the TX Buffer used by the App for this SPI Peripheral */
	uint8_t*			pRxBuff;	/* !< Pointer to the RX Buffer used by the App for this SPI Peripheral */
	uint32_t			TxLen;		/* !< */
	uint32_t			RxLen;		/* !< */
	SPI_TxRxState_t		TxState;	/* !< marks if the device is in the middle of TX or if it is free to start a new transmission */
	SPI_TxRxState_t		RxState;	/* !< marks if the device is in the middle of TX or if it is free to start a new reception*/
} SPI_Handle_t;

#define SPI_SR_RXNE_BIT_MASK		(1 << SPI_SR_RXNE_BIT_POS)
#define SPI_SR_TXE_BIT_MASK			(1 << SPI_SR_TXE_BIT_POS)
#define SPI_SR_CHSIDE_BIT_MASK		(1 << SPI_SR_CHSIDE_BIT_POS)
#define SPI_SR_UDR_BIT_MASK			(1 << SPI_SR_UDR_BIT_POS)
#define SPI_SR_CRC_ERR_BIT_MASK		(1 << SPI_SR_CRC_ERR_BIT_POS)
#define SPI_SR_MODF_BIT_MASK		(1 << SPI_SR_MODF_BIT_POS)
#define SPI_SR_OVR_BIT_MASK			(1 << SPI_SR_OVR_BIT_POS)
#define SPI_SR_BSY_BIT_MASK			(1 << SPI_SR_BSY_BIT_POS)
#define SPI_SR_FRE_BIT_MASK			(1 << SPI_SR_FRE_BIT_POS)


/* ***************************** */
/* APIs supported by this driver */
/* ***************************** */

/*
 * Peripheral clock setup
 */
void SPI_PerClockControl (SPIAndI2s_RegDef_t* pSPIx, uint8_t EnOrDis);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t* pSPIHanlde);
void SPI_DeInit(SPIAndI2s_RegDef_t* pSPIx); // this will reset ALL the registers of specific SPI port.

/*
 * Data TX and RX
 * Available SPI TX/RX options: polling based (blocking), interrupt based (non-blocking), using DMA (currently not implemented)
 */
/* Polling based (using busy wait, blocking) */
void SPI_DataTx(SPIAndI2s_RegDef_t* pSPIx, uint8_t* pTxBuff, uint32_t numBytesToSend);
void SPI_DataRx(SPIAndI2s_RegDef_t* pSPIx, uint8_t* pRxBuff, uint32_t numBytesToReceive);
/* Interrupt based (Non Blocking) */
SPI_TxRxState_t SPI_DataTxInt(SPI_Handle_t* pSPIHanle, uint8_t* pTxBuff, uint32_t numBytesToSend);
SPI_TxRxState_t SPI_DataRxInt(SPI_Handle_t* pSPIHanle, uint8_t* pRxBuff, uint32_t numBytesToReceive);

/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQInterrupEnDisCfg(uint8_t IRQNum, uint8_t EnOrDis);
void SPI_IRQInterrupPriCfg(uint8_t IRQNum, uint8_t IRQPrio);

void SPI_IRQHandle(SPI_Handle_t* pSPIHandle);

/*
 * Other Peripheral Control APIs
 */


/*
 * Other SPI Peripheral control APIs
 */
void SPI_PerControl (SPIAndI2s_RegDef_t* pSPIx, uint8_t EnOrDis);
void SPI_SSIConfig(SPIAndI2s_RegDef_t* pSPIx, uint8_t NSSPinSetOrReset);
void SPI_SSOEConfig(SPIAndI2s_RegDef_t* pSPIx, uint8_t SSOESetOrReset);
// TODO: fill this with more functions as needed.
SPI_TxRxState_t SPI_GetRxState(SPI_Handle_t* pSPIHandle);
SPI_TxRxState_t SPI_GetTxState(SPI_Handle_t* pSPIHandle);
void SPI_ClearOVRFlag(SPIAndI2s_RegDef_t* pSPIx);
void SPI_CloseTransmission(SPI_Handle_t* pSPIHandle);
void SPI_CloseReception(SPI_Handle_t* pSPIHandle);

/*
 * Application Callback function
 */
void SPI_ApplicationEventCallback(SPI_Handle_t* pSPIHandle, SPI_EventAppCallback_t SPI_EventApp);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
