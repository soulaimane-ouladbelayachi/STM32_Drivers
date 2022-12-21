/*
 * stm32f429_i2c.c
 *
 *  Created on: Oct 9, 2022
 *      Author: soula
 */


#include "stm32f429_i2c.h"


static void I2C_GenerateStartCondition(I2C_TypeDef *pI2C);
static void I2C_ExecuteAddrPhase(I2C_TypeDef *pI2C,uint8_t slaveAddr,uint8_t R_W);

static void I2C_GenerateStartCondition(I2C_TypeDef *pI2C){
	pI2C->CR1 |= (1 << I2C_CR1_START);
}
static void I2C_GenerateStopCondition(I2C_TypeDef *pI2C){
	pI2C->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ExecuteAddrPhase(I2C_TypeDef *pI2C,uint8_t slaveAddr,uint8_t R_W){
	uint8_t addrSent = slaveAddr << 1;
	slaveAddr &= ~(1 << 0);
	slaveAddr |= (R_W << 0);
	pI2C->DR = addrSent;
}

static void I2C_ClearAddrFlag(I2C_TypeDef *pI2C){
	uint32_t dummyRead = pI2C->SR1;
	dummyRead = pI2C->SR2;
	(void) dummyRead;
}

static void I2C_ManageACK(I2C_TypeDef *pI2C,uint8_t en_dis){
	if(en_dis == DISABLE){
		pI2C->CR1 &= ~(1 << I2C_CR1_ACK);
	}else if(en_dis == ENABLE){
		pI2C->CR1 |= (1 << I2C_CR1_ACK);
	}
}

void RCC_GetPLLClk(void){

}

uint32_t RCC_GetPLCK1Value(void){
	uint32_t pclk1;
	uint8_t pclk1_src;
	uint32_t systemCLK;

	uint16_t AHB_divider[8] = {2,4,8,16,64,128,256,512};
	uint8_t pre1;
	uint8_t AHB_Prescalar;

	uint8_t APB_divider[4] = {2,4,8,16};
	uint8_t pre2;
	uint8_t APB_Prescalar;

	pclk1_src = (RCC->CFGR >> 2) & 0x3;

	if(pclk1_src == 0){
		systemCLK = 16000000;
	}else if(pclk1_src == 1){
		pclk1 = 8000000;
	}else if(pclk1_src == 2){
		RCC_GetPLLClk();
	}


	pre1 = (RCC->CFGR >> 4) & 0xF;

	if(pre1 < 8){
		AHB_Prescalar = 1;
	}
	else{
		AHB_Prescalar = AHB_divider[pre1-8];
	}

	pre2 = (RCC->CFGR >> 10) & 0x7;

	if(pre2 < 4){
		APB_Prescalar = 1;
	}
	else{
		APB_Prescalar = APB_divider[pre2-4];
	}

	pclk1 = systemCLK/(AHB_Prescalar*APB_Prescalar);



	return pclk1;

}


void I2C_Init(I2C_Handle_Typedef *pI2C_Handle){
	uint32_t temp;

	I2C_PeriClockControl(pI2C_Handle->pI2Cx,ENABLE);
	//ACK configuration
	pI2C_Handle->pI2Cx->CR1 |= (pI2C_Handle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);

	//Speed Configuration
	pI2C_Handle->pI2Cx->CR2 &= ~(0x3f);
	pI2C_Handle->pI2Cx->CR2 |= (RCC_GetPLCK1Value()/1000000U) & 0x3f;

	//Device own address
	pI2C_Handle->pI2Cx->OAR1 |= pI2C_Handle->I2C_Config.I2C_DeviceAddress << 1;
	pI2C_Handle->pI2Cx->OAR1 |= (1 << 14);

	//Clock configuration :
	pI2C_Handle->pI2Cx->CCR &= ~(1 << I2C_CCR_FS);
	uint16_t ccr_value = 0;

	if(pI2C_Handle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM){

		//Standard Mode
		pI2C_Handle->pI2Cx->CCR &= ~(1 << I2C_CCR_FS);

		ccr_value = RCC_GetPLCK1Value()/(2*I2C_SCL_SPEED_SM);
		pI2C_Handle->pI2Cx->CCR &= ~(0xfff);
		pI2C_Handle->pI2Cx->CCR |= ccr_value & 0xfff;


	}else{
		//Fast Mode
		pI2C_Handle->pI2Cx->CCR |= (1 << I2C_CCR_FS);

		pI2C_Handle->pI2Cx->CCR &= ~(1 << I2C_CCR_DUTY);
		pI2C_Handle->pI2Cx->CCR |= (pI2C_Handle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);

		if(pI2C_Handle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			ccr_value = RCC_GetPLCK1Value()/(3*I2C_SCL_SPEED_FM);
		}else{
			ccr_value = RCC_GetPLCK1Value()/(25*I2C_SCL_SPEED_FM);
		}
		pI2C_Handle->pI2Cx->CCR |= ccr_value & 0xfff;
	}

	//TRISE Configuration
	if(pI2C_Handle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM){

		//Standard Mode

		temp = (RCC_GetPLCK1Value()/1000000U) + 1;

	}else{
		//Fast Mode
		temp = ((RCC_GetPLCK1Value()*300)/1000000000U) + 1;
	}
	pI2C_Handle->pI2Cx->TRISE = (temp & 0x3f);

}
void I2C_DeInit(I2C_TypeDef *pI2Cx);

void I2C_MasterSendData(I2C_Handle_Typedef *pI2C_Handle,uint8_t *pTxBuffer,uint8_t Len,uint8_t SlaveAddr){

	//1-Generate the START condition :
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);
	//2- SB=1, cleared by reading SR1 register followed by writing DR register with Address.
	while(!I2C_GetStatusRegister1(pI2C_Handle->pI2Cx, I2C_SB_FLAG));
	//3- send the slave addr with W/nR bit (8 bits)
	I2C_ExecuteAddrPhase(pI2C_Handle->pI2Cx,SlaveAddr,I2C_WRITE);
	//4- checking the addr flag in SR1
	while(!I2C_GetStatusRegister1(pI2C_Handle->pI2Cx, I2C_ADDR_FLAG));
	//5-clear the addr flag  according to its software sequence
	I2C_ClearAddrFlag(pI2C_Handle->pI2Cx);
	//6- send the data until the Len becomes 0
	while(Len > 0){
		while(!I2C_GetStatusRegister1(pI2C_Handle->pI2Cx, I2C_TxE_FLAG));
		pI2C_Handle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}
	//7-wait TxE=1, BTF = 1, Program Stop request. TxE and BTF are cleared by hardware by the Stop condition
	while(!I2C_GetStatusRegister1(pI2C_Handle->pI2Cx, I2C_TxE_FLAG));
	while(!I2C_GetStatusRegister1(pI2C_Handle->pI2Cx, I2C_BTF_FLAG));
	//8-generating STOP, automatically clears BTF
	I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);

}

void I2C_MasterRecieveData(I2C_Handle_Typedef *pI2C_Handle,uint8_t *pRxBuffer,uint8_t Len,uint8_t SlaveAddr){

	//1-Generate the START condition
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);
	//2-confirm that the start condition is completed by checking the SB flag in SR1
	//Until the SB is cleared the SCL will be stretched
	while(!I2C_GetStatusRegister1(pI2C_Handle->pI2Cx, I2C_SB_FLAG));
	//3- Send the address of the slave with R/nW with R(1)
	I2C_ExecuteAddrPhase(pI2C_Handle->pI2Cx,SlaveAddr,I2C_READ);
	//4-wait until the address phase is completed by checking the ADDR flag in SR1
	while(!I2C_GetStatusRegister1(pI2C_Handle->pI2Cx, I2C_ADDR_FLAG));
	//==>Procedure to read 1 byte :
	if(Len == 1){
		//1- Disable the ACK
		I2C_ManageACK(pI2C_Handle->pI2Cx, DISABLE);


		//3- Clear the ADDR flag
		I2C_ClearAddrFlag(pI2C_Handle->pI2Cx);

		//4-wait until RxNE become 1
		while(!I2C_GetStatusRegister1(pI2C_Handle->pI2Cx, I2C_RxNE_FLAG));

		//2-generate the stop condition
		I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
		//5-read 1 byte
		*pRxBuffer = pI2C_Handle->pI2Cx->DR;
		return;
	}

	//==>Procedure to read more than 1 byte :
	if (Len > 1){
		//clear the ADDR Flag
		I2C_ClearAddrFlag(pI2C_Handle->pI2Cx);
		//read the data until the Len become 0
		while(!I2C_GetStatusRegister1(pI2C_Handle->pI2Cx, I2C_RxNE_FLAG));
		for(uint32_t i = Len; i>0 ;i--){

			//wait until RxNE become 1
			while(!I2C_GetStatusRegister1(pI2C_Handle->pI2Cx, I2C_RxNE_FLAG));
			if(i == 2){
				//clear the ACK bit
				I2C_ManageACK(pI2C_Handle->pI2Cx, DISABLE);
				//generate STOP Condition
				I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
			}
			//read the data
			*pRxBuffer = pI2C_Handle->pI2Cx->DR;
			//increment the buffer address
			pRxBuffer++;

		}


	}

	//re-enable ACK
	I2C_ManageACK(pI2C_Handle->pI2Cx, ENABLE);

}


uint8_t I2C_GetStatusRegister1(I2C_TypeDef *pI2C,uint32_t FlagName){
	if(pI2C->SR1 & FlagName){
		return 1;
	}
	else{
		return 0;
	}
}

uint8_t I2C_GetStatusRegister2(I2C_TypeDef *pI2C,uint32_t FlagName){
	if(pI2C->SR1 & FlagName){
		return 1;
	}
	else{
		return 0;
	}
}

void I2C_PeriClockControl(I2C_TypeDef *pI2Cx,uint8_t EnDi){

		if (EnDi == ENABLE) {
			if (pI2Cx == I2C1) {
				I2C1_CLK_EN();
			} else if (pI2Cx == I2C2) {
				I2C2_CLK_EN();
			} else if (pI2Cx == I2C3) {
				I2C3_CLK_EN();
			}
		} else if (EnDi == DISABLE) {
			I2C1_CLK_DIS();
		} else if (pI2Cx == I2C2) {
			I2C2_CLK_DIS();
		} else if (pI2Cx == I2C3) {
			I2C3_CLK_DIS();
		}
}


void I2C_PeripheralControl(I2C_TypeDef *pI2Cx,uint8_t EnDi){
	if(EnDi == ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}
