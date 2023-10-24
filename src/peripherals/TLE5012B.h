#ifndef __TLE5012B_H_
#define __TLE5012B_H_

#include "../HAL/gpio.h"
#include "../HAL/spi.h"
#include "../HAL/timer.h"
#include "utils/velocityEstimator.h"
#include "utils/semaphore.h"
#include "../callbacks.h"

#define ANGLETOENCODERRAW 32768.0 / 360.0 /**< Constant to convert angle to raw encoder data */
#define CONVERTENCODERRAWTOANGLE(x) ((360.0 / 32768.0) * (float)x)
#define CONVERTENCODERANGLETORAW(x) ((uint16_t)(x * (32768.0 / 360.0)))
#define ENCODERRAWTOSTEP(x) (x*200.0) * (1.0/32768.0)		/**< Constant to convert raw encoder data to 1/256th steps*/
#define ENCODERRAWTOREVOLUTIONS 60.0 * (1.0 / 32768.0) /**< Constant to convert raw encoder data to revolutions */
#define ANGULARUPDATESPEED 42.7		//microseconds


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
	/** Encoder stalldetect enable - enabled when set to 1 **/
	volatile  bool encoderStallDetectEnable = 0;
	/** Encoder stalldetect return value - stall = 1 **/
	volatile  bool encoderStallDetect;
	/** Encoder stalldetect sensitivity - From -10 to 1 where lower number is less sensitive and higher is more sensitive. **/
	volatile  float encoderStallDetectSensitivity = -0.5;	
	
  private:
	Spi spiHandle;
	uint16_t angle;
	int16_t speed;
	volatile int32_t angleMoved;
	int32_t userAngleOffset = 0;
	/** Angle of the shaft at the reference position. */
	volatile uint16_t encoderOffset;
	void sendCommand(uint16_t rw, uint16_t lock, uint16_t upd, uint16_t addr, uint16_t nd);
	bool sample();
	uint16_t readAngle();
	int16_t readSpeed();
	VelocityEstimator velocityEstimator;
	Semaphore semaphore;
	friend void mainTimerCallback();
	/** Encoder stall detect counter - Delay start of the stalldetect to let the encoder velocity filter initialize properly */
	volatile uint16_t startDelay = 0;
	/** Encoder stall detect counter - Counter for number of consecutive samples that must show a stall */
	volatile uint16_t errorCnt = 0;	
};

#endif