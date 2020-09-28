/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Aug 20 2020
 *      Author: Amit Alon
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"
#include <stdint.h>

#ifdef DEBUG
#include <assert.h>
#endif

typedef enum {
	GPIO_PINMODE__IN = 0,
	GPIO_PINMODE__OUT,
	GPIO_PINMODE__ALTFN,		// alternate function
	GPIO_PINMODE__ANALOG,
	GPIO_PINMODE__INT_FEI,	// Falling Edge Interrupt
	GPIO_PINMODE__INT_REI,	// Rising Edge Interrupt
	GPIO_PINMODE__INT_RFEI,	// Rising and Falling Edge Interrupt
} GPIO_PinMode_t;

typedef enum {
	GPIO_OP_TYPE__PP = 0,	// Push Pull
	GPIO_OP_TYPE__OD,		// Open Drain
} GPIO_OutputType_t;

typedef enum {
	GPIO_OP_SPEED__LOW = 0,
	GPIO_OP_SPEED__MED,
	GPIO_OP_SPEED__FAST,
	GPIO_OP_SPEED__HIGH,
} GPIO_OutputSpeed_t;

typedef enum {
	GPIO_PU_PD__NONE = 0,
	GPIO_PU_PD__PU,
	GPIO_PU_PD__PD,
} GPIO_PullupPulldown_t;

typedef enum {
	GPIO_PIN_NUM__0 = 0,
	GPIO_PIN_NUM__1 = 1,
	GPIO_PIN_NUM__2 = 2,
	GPIO_PIN_NUM__3 = 3,
	GPIO_PIN_NUM__4 = 4,
	GPIO_PIN_NUM__5 = 5,
	GPIO_PIN_NUM__6 = 6,
	GPIO_PIN_NUM__7 = 7,
	GPIO_PIN_NUM__8 = 8,
	GPIO_PIN_NUM__9 = 9,
	GPIO_PIN_NUM__10 = 10,
	GPIO_PIN_NUM__11 = 11,
	GPIO_PIN_NUM__12 = 12,
	GPIO_PIN_NUM__13 = 13,
	GPIO_PIN_NUM__14 = 14,
	GPIO_PIN_NUM__15 = 15,
} GPIO_PinNum_t;

typedef enum {
	GPIO_AF__0 = 0,
	GPIO_AF__1 = 1,
	GPIO_AF__2 = 2,
	GPIO_AF__3 = 3,
	GPIO_AF__4 = 4,
	GPIO_AF__5 = 5,
	GPIO_AF__6 = 6,
	GPIO_AF__7 = 7,
	GPIO_AF__8 = 8,
	GPIO_AF__9 = 9,
	GPIO_AF__10 = 10,
	GPIO_AF__11 = 11,
	GPIO_AF__12 = 12,
	GPIO_AF__13 = 13,
	GPIO_AF__14 = 14,
	GPIO_AF__15 = 15,
} GPIO_AltFuncMode_t;

/* Configuration structure for a GPIO pin */
typedef struct {
	GPIO_PinNum_t			GPIO_PinNum;
	GPIO_PinMode_t			GPIO_PinMode;
	GPIO_OutputSpeed_t		GPIO_OutputSpeed;
	GPIO_PullupPulldown_t	GPIO_PuPdControl;
	GPIO_OutputType_t		GPIO_OPType;
	GPIO_AltFuncMode_t		GPIO_AltFunMode; /*!< Only applicable when GPIO_PinMode is set to GPIO_PINMODE_ALTFN, else the value here is ignored. >*/
} GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t		*pGPIOx; /*!< The base address of the port of the GPIO */
	GPIO_PinConfig_t 	GPIO_PinConfig;
} GPIO_Handle_t;

#define NUM_PRIO_BITS_UNUSED_STM32MCU	4 // TODO: understand from which spec this info is coming from - I took it from lecture 113 but could not find any reference that says 4 LSbytes of the priority fields are NA out of each 8...


/* ***************************** */
/* APIs supported by this driver */
/* ***************************** */

/*
 * Peripheral clock setup
 */
void GPIO_PerClockControl (GPIO_RegDef_t * pGPIOx, uint8_t EnOrDis);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHanlde);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx); // this will reset ALL the registers of specific GPIO port.

/*
 * Data read and write
 */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t Val);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Val);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterrupEnDisCfg(uint8_t IRQNum, uint8_t EnOrDis);
void GPIO_IRQInterrupPriCfg(uint8_t IRQNum, uint8_t IRQPrio);

void GPIO_IRQHandle(uint8_t PinNum);

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
