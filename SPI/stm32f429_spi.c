/*
 * stm32f429_spi.c
 *
 *  Created on: Jul 20, 2022
 *      Author: soula
 */

#include "stm32f429_spi.h"

/*********************************************************/
/*				 SPI Driver source code :				 */

static void spi_TXE_handle(SPI_Handle_Typedef *pSPI_Handle);
static void spi_RXNE_handle(SPI_Handle_Typedef *pSPI_Handle);
static void spi_OVR_ERRIE_handle(SPI_Handle_Typedef *pSPI_Handle);

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_TypeDef *pSPIx, uint8_t EnDi) {
	if (EnDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_CLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_CLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_CLK_EN();
		} else if (pSPIx == SPI4) {
			SPI4_CLK_EN();
		} else if (pSPIx == SPI5) {
			SPI5_CLK_EN();
		} else if (pSPIx == SPI6) {
			SPI6_CLK_EN();
		}
	} else if (EnDi == DISABLE) {
		SPI1_CLK_DIS();
	} else if (pSPIx == SPI2) {
		SPI2_CLK_DIS();
	} else if (pSPIx == SPI3) {
		SPI3_CLK_DIS();
	} else if (pSPIx == SPI4) {
		SPI4_CLK_DIS();
	} else if (pSPIx == SPI5) {
		SPI5_CLK_DIS();
	} else if (pSPIx == SPI6) {
		SPI6_CLK_DIS();
	}
}

/*init and deinit
 *
 */
void SPI_Init(SPI_Handle_Typedef *pSPI_Handle) {
	//Enable clock
	SPI_PeriClockControl(pSPI_Handle->pSPIx, ENABLE);
	//Configure device mode
	pSPI_Handle->pSPIx->CR1 &= ~(1 << SPI_CR_MSTR);
	pSPI_Handle->pSPIx->CR1 |= (pSPI_Handle->SPIConfig.SPI_DeviceMode
			<< SPI_CR_MSTR);

	//Configure the bus
	if (pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_FD) {
		//clear bidimode
		pSPI_Handle->pSPIx->CR1 &= ~(1 << SPI_CR_BIDIMODE);

	} else if (pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_HD) {
		//set bidimode
		pSPI_Handle->pSPIx->CR1 |= (1 << SPI_CR_BIDIMODE);

	} else if (pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_SIMPLEX_RXONLY) {
		//clear bidimode
		pSPI_Handle->pSPIx->CR1 &= ~(1 << SPI_CR_BIDIMODE);
		//set RXONLY
		pSPI_Handle->pSPIx->CR1 &= ~(1 << SPI_CR_RXONLY);
	}

	//configure CPOL
	pSPI_Handle->pSPIx->CR1 &= ~(1 << SPI_CR_CPOL);
	pSPI_Handle->pSPIx->CR1 |= (pSPI_Handle->SPIConfig.SPI_BusCPOL
			<< SPI_CR_CPOL);

	//configure CPOL
	pSPI_Handle->pSPIx->CR1 &= ~(1 << SPI_CR_CPHA);
	pSPI_Handle->pSPIx->CR1 |= (pSPI_Handle->SPIConfig.SPI_BusCPHA
			<< SPI_CR_CPHA);

	//configure the speed
	pSPI_Handle->pSPIx->CR1 &= ~(7 << SPI_CR_BR);
	pSPI_Handle->pSPIx->CR1 |= (pSPI_Handle->SPIConfig.SPI_SclkSpeed
			<< SPI_CR_BR);

	//Configure SSM
	pSPI_Handle->pSPIx->CR1 &= ~(1 << SPI_CR_SSM);
	pSPI_Handle->pSPIx->CR1 |= (pSPI_Handle->SPIConfig.SPI_SSM << SPI_CR_SSM);

	//configure DFF
	pSPI_Handle->pSPIx->CR1 &= ~(1 << SPI_CR_DFF);
	pSPI_Handle->pSPIx->CR1 |= (pSPI_Handle->SPIConfig.SPI_DFF << SPI_CR_DFF);

}
void SPI_DeInit(SPI_TypeDef *pSPIx);

void SPI_PeriphiralControl(SPI_TypeDef *pSPIx, uint8_t EnDi) {
	if (EnDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR_SPE);
	}
}

void SPI_SSIConfig(SPI_TypeDef *pSPIx, uint8_t EnDi) {
	if (EnDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR_SSI);
	}
}

void SPI_SSOEConfig(SPI_TypeDef *pSPIx, uint8_t EnDi) {
	if (EnDi == ENABLE) {
		pSPIx->CR2 |= (1 << 2);
	} else {
		pSPIx->CR2 &= ~(1 << 2);
	}
}

/*
 *
 */

uint8_t SPI_GetStatusRegister(SPI_TypeDef *pSPI, uint32_t FlagName) {

	if (pSPI->SR & FlagName) {
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/*
 * Data Send and Recieve
 */

void SPI_SendData(SPI_TypeDef *pSPIx, uint8_t *TxBuffer, uint32_t Len) {
	uint32_t dataLength = Len;

	SPI_PeriphiralControl(SPI2, ENABLE);

	while (dataLength > 0) {

		//1-wait until TXE set (SPI_SR)
		while (SPI_GetStatusRegister(pSPIx, SPI_TXE_FLAG) == FLAG_RESET)
			;

		//2-Check the DFF bit
		if (pSPIx->CR1 & (1 << SPI_CR_DFF)) {
			pSPIx->DR = *((uint16_t*) TxBuffer);
			dataLength--;
			dataLength--;
			(uint16_t*) TxBuffer++;
		} else {
			pSPIx->DR = *TxBuffer;
			dataLength--;
			TxBuffer++;
		}

	}
}
void SPI_SendRecieve(SPI_TypeDef *pSPIx, uint8_t *RxBuffer, uint32_t Len) {
	uint32_t dataLength = Len;

	SPI_PeriphiralControl(SPI2, ENABLE);

	while (dataLength > 0) {
		while (SPI_GetStatusRegister(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET)
			;

		//2-Check the DFF bit
		if (pSPIx->CR1 & (1 << SPI_CR_DFF)) {
			*(RxBuffer) = (uint8_t) pSPIx->DR;
			*(RxBuffer + 1) = (uint8_t) (pSPIx->DR >> 8);
			(uint16_t*) RxBuffer++;
			dataLength--;
			dataLength--;
		} else {
			*(RxBuffer) = (uint8_t) pSPIx->DR;
			RxBuffer++;
			dataLength--;
		}

	}

}

uint8_t SPI_SendDataIT(SPI_Handle_Typedef *pSPIHandle, uint8_t *TxBuffer,
		uint32_t Len) {

	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX) {

		//1-Save the TxBuffer and Len information in some global variable
		pSPIHandle->pTxBuffer = TxBuffer;
		pSPIHandle->TxLen = Len;

		//2-Mark the SPI state busy in transmission so that no other code can take over the SPI peripheral until the transmission is ended
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3-Enable the TXEIE control bit to get interrupt whenever the TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR_TXEIE);

		//4-Data transmission will be handled by the ISR code
	}
	return state;

}

uint8_t SPI_RecieveDataIT(SPI_Handle_Typedef *pSPIHandle, uint8_t *RxBuffer,
		uint32_t Len) {

	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX) {

		//1-Save the TxBuffer and Len information in some global variable
		pSPIHandle->pRxBuffer = RxBuffer;
		pSPIHandle->RxLen = Len;

		//2-Mark the SPI state busy in transmission so that no other code can take over the SPI peripheral until the transmission is ended
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3-Enable the TXEIE control bit to get interrupt whenever the TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR_RXNEIE);

		//4-Data transmission will be handled by the ISR code
	}
	return state;

}

void SPI_IRQHandling(SPI_Handle_Typedef *pHandle) {

	uint8_t temp1, temp2;

	//1_check TXE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR_TXEIE);

	if (temp1 && temp2) {
		//handle the TXE interrupt
		spi_TXE_handle();
	}

	//2_check RXNE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR_RXNEIE);

	if (temp1 && temp2) {
		//handle the RXNE interrupt
		spi_RXNE_handle();
	}

	//1_check RXNE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR_ERRIE);

	if (temp1 && temp2) {
		//handle the RXNE interrupt
		spi_OVR_ERRIE_handle();
	}

}

static void spi_TXE_handle(SPI_Handle_Typedef *pSPI_Handle) {

	if (pSPI_Handle->pSPIx->CR1 & (1 << SPI_CR_DFF)) {
		pSPI_Handle->pSPIx->DR = *((uint16_t*) pSPI_Handle->pTxBuffer);
		(uint16_t*) pSPI_Handle->pTxBuffer--;
		pSPI_Handle->TxLen = pSPI_Handle->TxLen - 2;
	} else {
		pSPI_Handle->pSPIx->DR = *(pSPI_Handle->pTxBuffer);
		pSPI_Handle->pTxBuffer--;
		pSPI_Handle->TxLen = pSPI_Handle->TxLen - 1;
	}

	if (!pSPI_Handle->TxLen) {
		//TX is over
		pSPI_Handle->pSPIx->CR2 &= ~(1 << SPI_CR_TXEIE);
		pSPI_Handle->TxLen = 0;
		pSPI_Handle->pTxBuffer = NULL;
		pSPI_Handle->TxState = SPI_READY;

		SPI_ApplicationEventCallback(pSPI_Handle, SPI_EVENT_TX_CMPLT);
	}
}
static void spi_RXNE_handle(SPI_Handle_Typedef *pSPI_Handle) {

	if (pSPI_Handle->pSPIx->CR1 & (1 << SPI_CR_DFF)) {
		*((uint16_t*) pSPI_Handle->pTxBuffer) =
				(uint16_t) pSPI_Handle->pSPIx->DR;
		(uint16_t*) pSPI_Handle->pRxBuffer--;
		pSPI_Handle->TxLen = pSPI_Handle->TxLen - 2;
	} else {
		*(pSPI_Handle->pRxBuffer) = pSPI_Handle->pSPIx->DR;
		pSPI_Handle->pRxBuffer--;
		pSPI_Handle->RxLen--;
	}

	if (!pSPI_Handle->RxLen) {
		//TX is over
		pSPI_Handle->pSPIx->CR2 &= ~(1 << SPI_CR_RXNEIE);
		pSPI_Handle->RxLen = 0;
		pSPI_Handle->pRxBuffer = NULL;
		pSPI_Handle->RxState = SPI_READY;

		SPI_ApplicationEventCallback(pSPI_Handle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_OVR_ERRIE_handle(SPI_Handle_Typedef *pSPI_Handle) {

	uint8_t temp;
	//1-clear the ovr flag
	if(!(pSPI_Handle->TxState == SPI_BUSY_IN_TX)){
		temp = pSPI_Handle->pSPIx->DR;
		temp = pSPI_Handle->pSPIx->SR;
	}

	//2-inform the application
	SPI_ApplicationEventCallback(pSPI_Handle,SPI_EVENT_OVR_ERR);
}

