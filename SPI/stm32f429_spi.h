/*
 * stm32f429_spi.h
 *
 *  Created on: Jul 20, 2022
 *      Author: soulaimane Oulad Belayachi
 */

#ifndef INC_STM32F429_SPI_H_
#define INC_STM32F429_SPI_H_

#include "stm32f4xx.h"


typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_BusCPOL;
	uint8_t SPI_BusCPHA;
	uint8_t SPI_SSM;
}SPI_Config_Typedef;


typedef struct {
	SPI_TypeDef *pSPIx;
	SPI_Config_Typedef SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;

}SPI_Handle_Typedef;

/*
 * SPI Application Event
 */

#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4




/*
 * SPI master Config :
 */
#define SPI_MODE_MASTER				1
#define SPI_MODE_SLAVE				0




/*
 * SPI Bus Config :
 */
#define SPI_BUS_FD					0
#define SPI_BUS_HD					1
#define SPI_BUS_SIMPLEX_RXONLY		2

/*
 * SPI Clock Speed
 */
#define SPI_SPEED_DIV_2				0
#define SPI_SPEED_DIV_4				1
#define SPI_SPEED_DIV_8				2
#define SPI_SPEED_DIV_16			3
#define SPI_SPEED_DIV_32			4
#define SPI_SPEED_DIV_64			5
#define SPI_SPEED_DIV_128			6
#define SPI_SPEED_DIV_256			7

/*
 * SPI DFF
 */
#define SPI_DFF_8BITS				0
#define SPI_DFF_16BITS				1

/*
 * SPI CPHA
 */
#define SPI_CPHA_LOW				0
#define SPI_CPHA_HIGH				1

/*
 * SPI CPOL
 */
#define SPI_CPOL_LOW				0
#define SPI_CPOL_HIGH				1

/*
 * SPI SSM
 */
#define SPI_SSM_DIS					0
#define SPI_SSM_EN					1





/*
 * Bit position CR1
 */

#define SPI_CR_CPHA					0
#define SPI_CR_CPOL					1
#define SPI_CR_MSTR					2
#define SPI_CR_BR					3
#define SPI_CR_SPE					6
#define SPI_CR_LSBFIRST				7
#define SPI_CR_SSI					8
#define SPI_CR_SSM					9
#define SPI_CR_RXONLY				10
#define SPI_CR_DFF					11
#define SPI_CR_CRCNEXT				12
#define SPI_CR_CRCEN				13
#define SPI_CR_BIDIOE				14
#define SPI_CR_BIDIMODE				15

/*
 * Bit position CR2
 */
#define SPI_CR_RXDMAEN				0
#define SPI_CR_TXDMAEN				1
#define SPI_CR_SSOE					2
#define SPI_CR_FRF					4
#define SPI_CR_ERRIE				5
#define SPI_CR_RXNEIE				6
#define SPI_CR_TXEIE				7

/*
 * Bit position SR
 */
#define SPI_SR_RXNE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF 				5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8

/*
 *	Status Registers flags
 */

#define SPI_RXNE_FLAG				(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG				(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG				(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG				(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG				(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG				(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG				(1 << SPI_SR_OVR)
#define SPI_BSY_FLAG				(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG				(1 << SPI_SR_FRE)

/*
 * SPI application states
 */

#define SPI_READY		1
#define SPI_BUSY_IN_RX	2
#define SPI_BUSY_IN_TX	3


/*********************************************************/
/*				 SPI Driver Prototypes :				  /



/*Peripheral Clock setup*/
void SPI_PeriClockControl(SPI_TypeDef *pSPIx,uint8_t EnDi);

/*init and deinit */
void SPI_Init(SPI_Handle_Typedef *pSPI_Handle);
void SPI_DeInit(SPI_TypeDef *pSPIx);

/*
 * Data Send and Recieve
 */

void SPI_SendData(SPI_TypeDef *pSPIx,uint8_t *TxBuffer,uint32_t Len);
void SPI_SendRecieve(SPI_TypeDef *pSPIx,uint8_t *RxBuffer,uint32_t Len);

void SPI_PeriphiralControl(SPI_TypeDef *pSPIx,uint8_t EnDi);

void SPI_SSIConfig(SPI_TypeDef *pSPIx,uint8_t EnDi);

/*
 * IRQ
 */

uint8_t SPI_SendDataIT(SPI_Handle_Typedef *pSPIHandle,uint8_t *TxBuffer,uint32_t Len);
uint8_t SPI_RecieveDataIT(SPI_Handle_Typedef *pSPIHandle,uint8_t *RxBuffer,uint32_t Len);

void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnDi);
void SPI_IRQPriorityConfig(uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_Typedef *pHandle);

uint8_t SPI_GetStatusRegister(SPI_TypeDef *pSPI,uint32_t FlagName);


#endif /* INC_STM32F429_SPI_H_ */
