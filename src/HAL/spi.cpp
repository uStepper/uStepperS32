#include "spi.h"

Spi::Spi(bool csPolarity, GPIO mosi, GPIO miso, GPIO sck, GPIO cs, SPI_TypeDef *spiChannel)
{
	_mosi = mosi;
	_miso = miso;
	_sck = sck;
	_cs = cs;
	this->_spiChannel = spiChannel;
}

void Spi::init()
{
	LL_SPI_InitTypeDef SPI_InitStruct = {0};

	/* Peripheral clock enable */
	if (this->_spiChannel == SPI1){LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);}
	else if (this->_spiChannel == SPI2){LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);}
	else if (this->_spiChannel == SPI3){LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);}
	else if (this->_spiChannel == SPI4){LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI4);}
	/**SPI2 GPIO Configuration
	PB13   ------> SPI2_SCK
	PB14   ------> SPI2_MISO
	PB15   ------> SPI2_MOSI
	*/

	this->_mosi.configureSpi(this->_spiChannel);
	this->_miso.configureSpi(this->_spiChannel);
	this->_sck.configureSpi(this->_spiChannel);
	this->_cs.configureOutput();

	this->csSet();

	/* SPI2 parameter configuration*/
	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 10;
	LL_SPI_Init(this->_spiChannel, &SPI_InitStruct);
	LL_SPI_SetStandard(this->_spiChannel, LL_SPI_PROTOCOL_MOTOROLA);
	LL_SPI_Enable(this->_spiChannel);
}

uint8_t Spi::transmit8BitData(uint8_t data)
{
	LL_SPI_SetDataWidth(this->_spiChannel, LL_SPI_DATAWIDTH_8BIT);
	while (((this->_spiChannel->SR) & (1 << 7)))
		; // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
	while (!((this->_spiChannel->SR) & (1 << 1)))
		;						  // wait for TXE bit to set -> This will indicate that the buffer is empty
	this->_spiChannel->DR = data; // send data
	while (!((this->_spiChannel->SR) & (1 << 0)))
		; // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
	while (((this->_spiChannel->SR) & (1 << 7)))
		; // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
	while (!((this->_spiChannel->SR) & (1 << 1)))
		;						  // wait for TXE bit to set -> This will indicate that the buffer is empty
	return this->_spiChannel->DR; //Return data
}

uint16_t Spi::transmit16BitData(uint16_t data)
{
	LL_SPI_SetDataWidth(this->_spiChannel, LL_SPI_DATAWIDTH_16BIT);
	while (((this->_spiChannel->SR) & (1 << 7)))
		; // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
	while (!((this->_spiChannel->SR) & (1 << 1)))
		;						  // wait for TXE bit to set -> This will indicate that the buffer is empty
	this->_spiChannel->DR = data; // send data
	while (!((this->_spiChannel->SR) & (1 << 0)))
		;						  // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
	return this->_spiChannel->DR; //Return data
}

void Spi::transmitBytes(uint8_t *data, uint8_t *response, uint8_t bytes)
{
	while (bytes)
	{
		*response++ = this->transmit8BitData(*data++);
		bytes--;
	}
}

void Spi::csSet()
{
	this->_cs.set();		//Set cs pin high
}

void Spi::csReset()
{
	this->_cs.reset(); //Set cs pin low
}

void Spi::releaseMosi()
{
	this->_mosi.configureInput();
}

void Spi::engageMosi()
{
	this->_mosi.configureSpi(this->_spiChannel);
}