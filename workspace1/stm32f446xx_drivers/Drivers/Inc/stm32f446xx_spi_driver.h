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
	SPI_SSM__SW = 0,
	SPI_SSM__HW,
} SPI_Ssm_t;

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
	SPIAndI2s_RegDef_t	*pSPIx; /*!< The base address of SPIx peripheral (x = 1/2/3/4) */
	SPI_Config_t SPIConfig;
} SPI_Handle_t;



/* ***************************** */
/* APIs supported by this driver */
/* ***************************** */

/*
 * Peripheral clock setup
 */
void SPI_PerClockControl (SPIAndI2s_RegDef_t * pSPIx, uint8_t EnOrDis);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t* pSPIHanlde);
void SPI_DeInit(SPIAndI2s_RegDef_t *pSPIx); // this will reset ALL the registers of specific SPI port.

/*
 * Data TX and RX
 * Available tx/rx options: polling based (blocking), interrupt based (non-blocking), using DMA (currently not implemented)
 */
void SPI_DataTx(SPIAndI2s_RegDef_t *pSPIx, uint8_t* pTxBuff, uint32_t numBytesToSend);
void SPI_DataRx(SPIAndI2s_RegDef_t *pSPIx, uint8_t* pRxBuff, uint32_t numBytesToReceive);

/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQInterrupEnDisCfg(uint8_t IRQNum, uint8_t EnOrDis);
void SPI_IRQInterrupPriCfg(uint8_t IRQNum, uint8_t IRQPrio);

void SPI_IRQHandle(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
// TODO: fill this with more functions as needed.


#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
