#ifndef __TLE5012B_H_
#define __TLE5012B_H_

#include "../HAL/gpio.h"
#include "../HAL/spi.h"
#include "../HAL/timer.h"
#include "utils/velocityEstimator.h"

#define ANGLETOENCODERRAW 32768.0 * (1.0 / 360.0) /**< Constant to convert angle to raw encoder data */
#define CONVERTENCODERRAWTOANGLE(x) ((1.0 / ANGLETOENCODERRAW) * (float)x)
#define CONVERTENCODERANGLETORAW(x) ((uint16_t)(x * ANGLETOENCODERRAW))
#define ENCODERRAWTOSTEP(x) (x*256.0*200.0) * (1.0/32768.0)		/**< Constant to convert raw encoder data to 1/256th steps*/
#define ENCODERRAWTOREVOLUTIONS 60.0 * (1.0 / 32768.0) /**< Constant to convert raw encoder data to revolutions */


class TLE5012B
{
  public:
	TLE5012B();
	void init();
	float getAngle();
	void setHome(float initialAngle = 0);
	uint16_t getAngleRaw();
	float getAngleMoved(bool filtered = true);
	int32_t getAngleMovedRaw(bool filtered = true);
	float getSpeed(uint32_t stepSize = 256);
	float getRPM();
	uint8_t getStatus(void);
	bool detectMagnet(void);

  private:
	Spi spiHandle;
	uint16_t angle;
	volatile int32_t angleMoved;
	int32_t userAngleOffset = 0;
	/** Angle of the shaft at the reference position. */
	volatile uint16_t encoderOffset;
	void sendCommand(uint16_t rw, uint16_t lock, uint16_t upd, uint16_t addr, uint16_t nd);
	void sample();
	VelocityEstimator velocityEstimator;
	friend void mainTimerCallback();
};

#endif