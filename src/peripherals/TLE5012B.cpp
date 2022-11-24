#include "TLE5012B.h"

TLE5012B::TLE5012B() : spiHandle(
						   csActivePolarity_t(activeLow),
						   ENCODERMOSI,
						   ENCODERMISO,
						   ENCODERSCK,
						   ENCODERCS,
						   SPI3),
					   velocityEstimator(MAINTIMERINTERRUPTPERIOD)
{
}

void TLE5012B::init()
{
	this->spiHandle.init();
	this->sample();
	this->encoderOffset = this->angle;
	this->angle = 0;
	this->angleMoved = 0;
	this->sample();
}

void TLE5012B::sendCommand(uint16_t rw, uint16_t lock, uint16_t upd, uint16_t addr, uint16_t nd)
{
	uint16_t cmd = ((rw << 15) | (lock << 11) | (upd << 10) | (addr << 4) | nd);

	spiHandle.transmit16BitData(cmd);
}

void TLE5012B::sample()
{
	int16_t deltaAngle;
	uint16_t newAngle;

	spiHandle.csReset();
	sendCommand(0x1, 0x0, 0x0, 0x02, 0x0);
	this->spiHandle.releaseMosi();
	newAngle = spiHandle.transmit16BitData(0x0000) & 0x7FFF;
	spiHandle.csSet();
	this->spiHandle.engageMosi();
	if (this->spiHandle.getTransmissionStatus() != SPI_TRANSMISSION_SUCCESS)
	{
		return;
	}

	newAngle -= this->encoderOffset;

	if (newAngle > 0x7FFF)
	{
		newAngle -= 0x8000;
	}

	
	deltaAngle = (int16_t)this->angle - (int16_t)newAngle;

	if (deltaAngle < -0x4000)
	{
		deltaAngle += 0x8000;
	}
	else if (deltaAngle > 0x4000)
	{
		deltaAngle -= 0x8000;
	}

	this->angleMoved -= deltaAngle;
	this->angle = newAngle;
	this->velocityEstimator.runIteration(this->angleMoved);
}

float TLE5012B::getAngle()
{
	return CONVERTENCODERRAWTOANGLE(this->angle);
}

void TLE5012B::setHome(float initialAngle)
{
	mainTimerPause();
	this->encoderOffset = 0;
	this->sample();
	this->encoderOffset = this->angle;
	//#TODO: Remember to reset driver position 
	this->angle = 0;
	this->angleMoved = CONVERTENCODERANGLETORAW(initialAngle);
	this->velocityEstimator.reset();
	mainTimerStart();
}

uint16_t TLE5012B::getAngleRaw()
{
	return this->angle;
}

float TLE5012B::getAngleMoved(bool filtered)
{
	return CONVERTENCODERRAWTOANGLE(this->angleMoved);
}

int32_t TLE5012B::getAngleMovedRaw(bool filtered)
{
	return this->angleMoved;
}

float TLE5012B::getSpeed(uint32_t stepSize)
{
	return ENCODERRAWTOSTEP((float)stepSize) * this->velocityEstimator.getVelocityEstimation();
}

float TLE5012B::getRPM(){
	return this->velocityEstimator.getVelocityEstimation() * ENCODERRAWTOREVOLUTIONS;
}

uint8_t TLE5012B::getStatus(void)
{
	return 0;
}

bool TLE5012B::detectMagnet(void)
{
	return true;
}