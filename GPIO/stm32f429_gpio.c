/*
 * stm32f429_gpio.c
 *
 *  Created on: Jul 3, 2022
 *      Author: soula
 */

#include "stm32f429_gpio.h"

/*
 * GPIO Driver Prototypes :
 */


/*Peripheral Clock setup*/
void GPIO_PeriClockControl(GPIO_TypeDef *pGPIO,uint8_t EnDi){
	if(EnDi == ENABLE){
		if(pGPIO == GPIOA){
			GPIOA_CLK_EN();
		}else if(pGPIO == GPIOB){
			GPIOB_CLK_EN();
		}else if(pGPIO == GPIOC){
			GPIOC_CLK_EN();
		}else if(pGPIO == GPIOD){
			GPIOD_CLK_EN();
		}else if(pGPIO == GPIOE){
			GPIOE_CLK_EN();
		}else if(pGPIO == GPIOF){
			GPIOF_CLK_EN();
		}else if(pGPIO == GPIOG){
			GPIOG_CLK_EN();
		}else if(pGPIO == GPIOH){
			GPIOH_CLK_EN();
		}else if(pGPIO == GPIOI){
			GPIOI_CLK_EN();
		}else if(pGPIO == GPIOJ){
			GPIOJ_CLK_EN();
		}else if(pGPIO == GPIOK){
			GPIOK_CLK_EN();
		}
	}else if(EnDi == DISABLE){
		if(pGPIO == GPIOA){
			GPIOA_CLK_DIS();
		}else if(pGPIO == GPIOB){
			GPIOB_CLK_DIS();
		}else if(pGPIO == GPIOC){
			GPIOC_CLK_DIS();
		}else if(pGPIO == GPIOD){
			GPIOD_CLK_DIS();
		}else if(pGPIO == GPIOE){
			GPIOE_CLK_DIS();
		}else if(pGPIO == GPIOF){
			GPIOF_CLK_DIS();
		}else if(pGPIO == GPIOG){
			GPIOG_CLK_DIS();
		}else if(pGPIO == GPIOH){
			GPIOH_CLK_DIS();
		}else if(pGPIO == GPIOI){
			GPIOI_CLK_DIS();
		}else if(pGPIO == GPIOJ){
			GPIOJ_CLK_DIS();
		}else if(pGPIO == GPIOK){
			GPIOK_CLK_DIS();
		}
	}
}

/*init and deinit */
void GPIO_Init(GPIO_Handle_TypeDef *pGPIO_Handle){

	GPIO_PeriClockControl(pGPIO_Handle->pGPIOx,ENABLE);

	uint32_t temp = 0;

	if(pGPIO_Handle->GPIO_PinConfig.PinMode < GPIO_MODE_ANALOG){

		// Configure GPIO Mode
		temp = pGPIO_Handle->GPIO_PinConfig.PinMode << (2*pGPIO_Handle->GPIO_PinConfig.PinNumber);
		pGPIO_Handle->pGPIOx->MODER &= ~(temp);
		pGPIO_Handle->pGPIOx->MODER |=  (temp);
		temp = 0;

	}else{
		//interrupt mode
		if(pGPIO_Handle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_FT){
			//configure the FTSR
			EXTI->FTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.PinNumber );
			EXTI->RTSR &= ~(1 << pGPIO_Handle->GPIO_PinConfig.PinNumber );
		}else if(pGPIO_Handle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_RT){
			//configure the RTSR
			EXTI->FTSR &= ~(1 << pGPIO_Handle->GPIO_PinConfig.PinNumber );
			EXTI->RTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.PinNumber );
		}else if(pGPIO_Handle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_FT){
			//configure both the FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.PinNumber );
			EXTI->RTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.PinNumber );
		}

		//configure GPIO PORT in SYSCFG_EXTICR
		uint8_t temp1 = pGPIO_Handle->GPIO_PinConfig.PinNumber / 4;
		uint8_t temp2 = pGPIO_Handle->GPIO_PinConfig.PinNumber % 4;
		uint8_t temp3 = GPIO_Port_Number(pGPIO_Handle->pGPIOx);

		SYSCFGR_CLK_EN();
		SYSCFG->EXTICR[temp1] &= ~(0xf << temp2*4);
		SYSCFG->EXTICR[temp1] |= (temp3 << temp2*4);


		// configure IMR
		EXTI->IMR |= (1 << pGPIO_Handle->GPIO_PinConfig.PinNumber );


	}

	//Configure Output Type
	pGPIO_Handle->pGPIOx->OTYPER &= ~(1 << pGPIO_Handle->GPIO_PinConfig.PinNumber);
	pGPIO_Handle->pGPIOx->OTYPER |= (pGPIO_Handle->GPIO_PinConfig.PinOutType << pGPIO_Handle->GPIO_PinConfig.PinNumber);

	//Configure Output Speed
	temp = pGPIO_Handle->GPIO_PinConfig.PinOutSpeed << (2*pGPIO_Handle->GPIO_PinConfig.PinNumber);
	pGPIO_Handle->pGPIOx->OSPEEDR &= ~(temp);
	pGPIO_Handle->pGPIOx->OSPEEDR |=  (temp);
	temp = 0;

	//Configure PullUp PullDown
	temp = pGPIO_Handle->GPIO_PinConfig.PinPuPd << (2*pGPIO_Handle->GPIO_PinConfig.PinNumber);
	pGPIO_Handle->pGPIOx->PUPDR &= ~(temp);
	pGPIO_Handle->pGPIOx->PUPDR |=  (temp);
	temp = 0;


	//configure Alternate Function
	if(pGPIO_Handle->GPIO_PinConfig.PinMode == GPIO_MODE_AF){
		uint32_t temp1 = pGPIO_Handle->GPIO_PinConfig.PinNumber / 8;
		uint32_t temp2 = pGPIO_Handle->GPIO_PinConfig.PinNumber % 8;

		uint32_t temp = pGPIO_Handle->GPIO_PinConfig.PinAltFunMode << (4*temp2);

		pGPIO_Handle->pGPIOx->AFR[temp1] &= ~(temp);
		pGPIO_Handle->pGPIOx->AFR[temp1] |=  (temp);

	}

}



void GPIO_DeInit(GPIO_TypeDef *pGPIOx){
	GPIO_PeriClockControl(pGPIOx,DISABLE);
}


/*Read and Write */
uint8_t GPIO_ReadInputPin(GPIO_TypeDef *pGPIOx,uint8_t PinNumber){
	uint8_t temp = (pGPIOx->IDR >> PinNumber) & 0x1;
	return temp;
}


uint16_t GPIO_ReadInputPort(GPIO_TypeDef *pGPIOx){
	uint16_t temp = pGPIOx->IDR;
	return temp;
}

void GPIO_WriteOutputPin(GPIO_TypeDef *pGPIOx,uint8_t PinNumber,uint8_t SET_RESET){
	if(SET_RESET == SET){
		pGPIOx->ODR |= (SET << PinNumber);
	}else if(SET_RESET == RESET){
		pGPIOx->ODR &= ~(SET << PinNumber);
	}
}
void GPIO_WriteOutputPort(GPIO_TypeDef *pGPIOx,uint8_t SET_RESET){
	if(SET_RESET == SET){
		pGPIOx->ODR |= (0x0000FFFF);
	}else if(SET_RESET == RESET){
		pGPIOx->ODR &= ~(0x0000FFFF);
	}
}
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnDi){

	if(EnDi == ENABLE){
		if(IRQNumber < 32){
			//Configure ISER0
			*NVIC_ISER0 |= (1 << (IRQNumber));
		}else if(IRQNumber >= 32 && IRQNumber < 64){
			//Configure ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			//Configure ISER2
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else{
		if(IRQNumber < 32){
			//Configure ISER0
			*NVIC_ICER0 |= (1 << (IRQNumber));
		}else if(IRQNumber >= 32 && IRQNumber < 64){
			//Configure ISER1
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			//Configure ISER2
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQPriority){

	uint8_t iprx = IRQPriority / 4;
	uint8_t ipr_section = IRQPriority % 4;
	uint8_t shift_amount = (ipr_section*8) + (8-NO_BITS_IMPLEMENTED);
	*(NVIC_IPR + iprx*4) &=  ~(IRQPriority << shift_amount);
	*(NVIC_IPR + iprx*4) |=  (IRQPriority << shift_amount);

}


void GPIO_IRQHandling(uint8_t pinNumber){
	//clear EXTI pr register
	if(EXTI->PR & (1 << pinNumber)){
		//to clear pr , set pr to 1
		EXTI->PR |= (1 << pinNumber);
	}
}



void SysTickDelayMs(uint32_t n){

	SysTick->STRVR = 16000 - 1;
	SysTick->STCVR = 0;
	SysTick->STCSR = 0x5;

	uint32_t i = 0;

	for(i = 0;i < n;i++){
		while(!(SysTick->STCSR & ( 1<<16 ))){}
	}

	SysTick->STCSR = 0;

}
