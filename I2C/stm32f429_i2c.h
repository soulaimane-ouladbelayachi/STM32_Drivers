/*
 * stm32f429_i2c.h
 *
 *  Created on: Oct 9, 2022
 *      Author: soulaimane Oulad Belayachi
 */

#ifndef INC_STM32F429_I2C_H_
#define INC_STM32F429_I2C_H_

#include "stm32f4xx.h"


/*
 * configuration structure for I2CX Peripheral
 */

typedef struct {
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t	 I2C_ACKControl;
	uint8_t	 I2C_FMDutyCycle;
}I2C_Config_TypeDef;


/*
 * Handle structure for I2CX Peripheral
 */

typedef struct {
	I2C_TypeDef	*pI2Cx;
	I2C_Config_TypeDef I2C_Config;
}I2C_Handle_Typedef;


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM	400000

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE		ENABLE
#define I2C_ACK_DISABLE		DISABLE

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1



/*
 * Bit position I2C_CR1
 */

#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15

/*
 * Bit position I2C_CR2
 */
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERRE			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12

/*
 * Bit position I2C_SR1
 */
#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RxNE			6
#define I2C_SR1_TxE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15

/*
 * Bit position I2C_SR2
 */
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAUL		5
#define I2C_SR2_SMBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC				8

/*
 * Bit position I2C_CCR
 */
#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS				15


/*
 * I2C status Flag :
 */
#define I2C_SB_FLAG				(1 << I2C_SR1_SB)
#define I2C_ADDR_FLAG		    (1 << I2C_SR1_ADDR)
#define I2C_ADD10_FLAG			(1 << I2C_SR1_ADD10)
#define I2C_STOPF_FLAG			(1 << I2C_SR1_STOPF)
#define I2C_RxNE_FLAG			(1 << I2C_SR1_RxNE)
#define I2C_TxE_FLAG			(1 << I2C_SR1_TxE)
#define I2C_BTF_FLAG			(1 << I2C_SR1_BTF)
#define I2C_AF_FLAG				(1 << I2C_SR1_AF)
#define I2C_OVR_FLAG			(1 << I2C_SR1_OVR)

#define I2C_READ				1
#define I2C_WRITE				0

/*********************************************************/
/*				 I2C Driver Prototypes :				 */


/*Peripheral Clock setup*/
void I2C_PeriClockControl(I2C_TypeDef *pI2Cx,uint8_t EnDi);

/*init and deinit */
void I2C_Init(I2C_Handle_Typedef *pI2C_Handle);
void I2C_DeInit(I2C_TypeDef *pI2Cx);

/*
 * Data Send and Recieve
 */
void I2C_MasterSendData(I2C_Handle_Typedef *pI2C_Handle,uint8_t *pTxBuffer,uint8_t Len,uint8_t SlaveAddr);
void I2C_MasterRecieveData(I2C_Handle_Typedef *pI2C_Handle,uint8_t *pRxBuffer,uint8_t Len,uint8_t SlaveAddr);
/*
 * IRQ
 */

uint8_t I2C_SendDataIT(I2C_Handle_Typedef *pI2CHandle,uint8_t *TxBuffer,uint32_t Len);
uint8_t I2C_RecieveDataIT(I2C_Handle_Typedef *pI2CHandle,uint8_t *RxBuffer,uint32_t Len);

void I2C_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnDi);
void I2C_IRQPriorityConfig(uint8_t IRQPriority);


void I2C_PeripheralControl(I2C_TypeDef *pI2Cx,uint8_t EnDi);
uint8_t I2C_GetStatusRegister1(I2C_TypeDef *pI2C,uint32_t FlagName);
uint8_t I2C_GetStatusRegister2(I2C_TypeDef *pI2C,uint32_t FlagName);


#endif /* INC_STM32F429_I2C_H_ */
