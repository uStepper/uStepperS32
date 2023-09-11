#include "../UstepperS32.h"
extern UstepperS32 *ptr;

TLE5012B::TLE5012B() : spiHandle(
						   csActivePolarity_t(activeLow),
						   ENCODERMOSI,
						   ENCODERMISO,
						   ENCODERSCK,
						   ENCODERCS,
						   SPI3),
					   velocityEstimator(MAINTIMERINTERRUPTPERIOD),
					   semaphore()
{
}

void TLE5012B::init()
{
	this->spiHandle.init();
	volatile uint16_t angle;
	//Invert angle direction of encoder
	spiHandle.csReset();
	sendCommand(0x0, 0xA, 0x0, 0x08, 0x1);
	spiHandle.transmit16BitData(0x0800);
	this->spiHandle.releaseMosi();
	angle = spiHandle.transmit16BitData(0x0000) & 0x7FFF;
	spiHandle.csSet();
	this->spiHandle.engageMosi();
	
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

uint16_t TLE5012B::readAngle()
{
	uint16_t angle;
	spiHandle.csReset();
	sendCommand(0x1, 0x0, 0x0, 0x02, 0x0);
	this->spiHandle.releaseMosi();
	angle = spiHandle.transmit16BitData(0x0000) & 0x7FFF;
	spiHandle.csSet();
	this->spiHandle.engageMosi();
	return angle;
}

int16_t TLE5012B::readSpeed()
{
	int16_t speed;
	spiHandle.csReset();
	sendCommand(0x1, 0x0, 0x0, 0x03, 0x0);
	this->spiHandle.releaseMosi();
	speed = spiHandle.transmit16BitData(0x0000) & 0x7FFF;
	spiHandle.csSet();
	this->spiHandle.engageMosi();
	
	if(speed & 0x4000)		//Check bit 14 for sign
	{
		speed -= 0x8000;
	}
	
	return speed;
}

bool TLE5012B::sample()
{
	int16_t deltaAngle;
	uint16_t newAngle;

	if (!semaphore.getLock())
	{
		return false;
	}

	newAngle = readAngle();

	if (this->spiHandle.getTransmissionStatus() != SPI_TRANSMISSION_SUCCESS)
	{
		semaphore.releaseLock();
		return false;
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

	/*this->speed = readSpeed();
	
	if (this->spiHandle.getTransmissionStatus() != SPI_TRANSMISSION_SUCCESS)
	{
		semaphore.releaseLock();
		return false;
	}*/
	
	this->angleMoved += deltaAngle;
	this->angle = newAngle;
	this->velocityEstimator.runIteration(this->angleMoved);
	
	semaphore.releaseLock();
	return true;
}

float TLE5012B::getAngle()
{
	return CONVERTENCODERRAWTOANGLE(this->angle);
}

void TLE5012B::setHome(float initialAngle)
{
	mainTimerPause();
	ptr->driver.getVelocity();
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
	//float temp;
	return ENCODERRAWTOSTEP((float)stepSize) * this->velocityEstimator.getVelocityEstimation();
	/*temp = (float)this->speed;
	temp *= (0.5 * 23419.2037470726 * 0.010986328125 * 142.22222222222);

	return temp;*/
}

float TLE5012B::getRPM(){
	float rpm = this->velocityEstimator.getVelocityEstimation() * ENCODERRAWTOREVOLUTIONS;
	return rpm;
}

uint8_t TLE5012B::getStatus(void)
{
	return 0;
}

bool TLE5012B::detectMagnet(void)
{
	return true;
}