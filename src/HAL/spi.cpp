#include "spi.h"

Spi::Spi() 
{
	this->settings._pins.mosi = LL_GPIO_PIN_15;
	this->settings._pins.miso = LL_GPIO_PIN_14;
	this->settings._pins.sck = LL_GPIO_PIN_13;
	this->settings._pins.cs = LL_GPIO_PIN_2;
}

void Spi::init()
{
	LL_SPI_InitTypeDef SPI_InitStruct = {0};
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
	/**SPI2 GPIO Configuration
	PB13   ------> SPI2_SCK
	PB14   ------> SPI2_MISO
	PB15   ------> SPI2_MOSI
	*/

	GPIO_InitStruct.Pin = this->settings._pins.mosi | this->settings._pins.miso | this->settings._pins.sck;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = this->settings._pins.cs;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	this->csSet();

	/* SPI2 parameter configuration*/
	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 10;
	LL_SPI_Init(SPI2, &SPI_InitStruct);
	LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
	LL_SPI_Enable(SPI2);
}

uint8_t Spi::transmit8BitData(uint8_t data)
{
	LL_SPI_SetDataWidth(SPI2, LL_SPI_DATAWIDTH_8BIT);
	while (((SPI2->SR) & (1 << 7)));			  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
	while (!((SPI2->SR) & (1 << 1)));				 // wait for TXE bit to set -> This will indicate that the buffer is empty
	SPI2->DR = data; // send data
	while (!((SPI2->SR) & (1 << 0))); // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
	while (((SPI2->SR) & (1 << 7))); // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
	while (!((SPI2->SR) & (1 << 1)));					// wait for TXE bit to set -> This will indicate that the buffer is empty
	return SPI2->DR;		//Return data
}

uint16_t Spi::transmit16BitData(uint16_t data)
{
	LL_SPI_SetDataWidth(SPI2, LL_SPI_DATAWIDTH_16BIT);
	while (((SPI2->SR) & (1 << 7))); // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
	while (!((SPI2->SR) & (1 << 1)));			 // wait for TXE bit to set -> This will indicate that the buffer is empty
	SPI2->DR = data; // send data
	while (!((SPI2->SR) & (1 << 0)));			 // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
	return SPI2->DR; //Return data
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
	GPIOB->BSRR |= this->settings._pins.cs;		//Set cs pin high
}

void Spi::csReset()
{
	GPIOB->BSRR |= this->settings._pins.cs << 16; //Set cs pin low
}