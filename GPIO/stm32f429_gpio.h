/*
 * stm32f429_gpio.h
 *
 *  Created on: Jul 3, 2022
 *      Author: soulaimane Oulad Belayachi
 */

#ifndef INC_STM32F429_GPIO_H_
#define INC_STM32F429_GPIO_H_

#include "stm32f4xx.h"


/*
 * Handle structure for a gpio
 */

typedef struct{
	uint8_t PinNumber;
	uint8_t PinMode;
	uint8_t PinOutType;
	uint8_t PinOutSpeed;
	uint8_t PinPuPd;
	uint8_t PinAltFunMode;
}GPIO_PinConfig_TypeDef;

typedef struct{
	GPIO_TypeDef *pGPIOx;//hold the base address of gpio
	GPIO_PinConfig_TypeDef GPIO_PinConfig;//hold the gpio pin settings
}GPIO_Handle_TypeDef;


//GPIO pin number
#define GPIO_PIN_0					0
#define GPIO_PIN_1					1
#define GPIO_PIN_2					2
#define GPIO_PIN_3					3
#define GPIO_PIN_4					4
#define GPIO_PIN_5					5
#define GPIO_PIN_6					6
#define GPIO_PIN_7					7
#define GPIO_PIN_8					8
#define GPIO_PIN_9					9
#define GPIO_PIN_10					10
#define GPIO_PIN_11					11
#define GPIO_PIN_12					12
#define GPIO_PIN_13					13
#define GPIO_PIN_14					14
#define GPIO_PIN_15					15

//GPIO  modes
#define GPIO_MODE_INPUT				0
#define GPIO_MODE_OUTPUT			1
#define GPIO_MODE_AF				2
#define GPIO_MODE_ANALOG			3
#define GPIO_MODE_IT_RT				4
#define GPIO_MODE_IT_FT				5
#define GPIO_MODE_IT_RFT			6

//GPIO	Output Types
#define GPIO_OUT_TYPE_PP			0//Push Pull
#define GPIO_OUT_TYPE_OD			1//Open Drain

//GPIO	Output Speed
#define GPIO_SPEED_LOW				0
#define GPIO_SPEED_MEDIUM			1
#define GPIO_SPEED_FAST				2
#define GPIO_SPEED_HIGH				3

//GPIO  pull-up/pull-down
#define GPIO_PIN_NoPUPD				0
#define GPIO_PIN_PU					1
#define GPIO_PIN_PD					2


//GPIO pin AF
#define GPIO_PIN_AF0				0
#define GPIO_PIN_AF1				1
#define GPIO_PIN_AF2				2
#define GPIO_PIN_AF3				3
#define GPIO_PIN_AF4				4
#define GPIO_PIN_AF5				5
#define GPIO_PIN_AF6				6
#define GPIO_PIN_AF7				7
#define GPIO_PIN_AF8				8
#define GPIO_PIN_AF9				9
#define GPIO_PIN_AF10				10
#define GPIO_PIN_AF11				11
#define GPIO_PIN_AF12				12
#define GPIO_PIN_AF13				13
#define GPIO_PIN_AF14				14
#define GPIO_PIN_AF15				15

/*********************************************************/
/*				 GPIO Driver Prototypes :				 */



/*Peripheral Clock setup*/
void GPIO_PeriClockControl(GPIO_TypeDef *pGPIO,uint8_t EnDi);

/*init and deinit */
void GPIO_Init(GPIO_Handle_TypeDef *pGPIO_Handle);
void GPIO_DeInit(GPIO_TypeDef *pGPIOx);


/*Read and Write */
uint8_t GPIO_ReadInputPin(GPIO_TypeDef *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadInputPort(GPIO_TypeDef *pGPIOx);
void GPIO_WriteOutputPin(GPIO_TypeDef *pGPIOx,uint8_t PinNumber,uint8_t SET_RESET);
void GPIO_WriteOutputPort(GPIO_TypeDef *pGPIOx,uint8_t SET_RESET);
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx,uint8_t PinNumber);

/*IRQ*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnDi);
void GPIO_IRQPriorityConfig(uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


/*Delay
 */
void SysTickDelayMs(uint32_t n);

#endif /* INC_STM32F429_GPIO_H_ */
