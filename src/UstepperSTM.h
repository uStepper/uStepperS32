
#ifndef _USTEPPER_STM_H_
#define _USTEPPER_STM_H_

#include <Arduino.h>
#include "HAL/spi.h"
#include "peripherals/TMC5130.h"
#include "peripherals/TLE5012B.h"
#include "HAL/timer.h"
#include "callbacks.h"


class UstepperSTM
{
	friend class TMC5130;
	friend class TLE5012B;
	
	public:
	TLE5012B encoder;
	TMC5130 driver;
	/**
	 * @brief	Constructor of uStepper class
	 */
	UstepperSTM();
	void init();
	
  private:
	friend void mainTimerCallback();
	friend void dropInStepInputEXTI();
	friend void dropInDirInputEXTI();
	friend void dropInEnableInputEXTI();

	float stepTime;
	float rpmToVel;
	float velToRpm;

	/** This variable contains the maximum velocity in steps/s, the motor is
	 * allowed to reach at any given point. The user of the library can
	 * set this by use of the setMaxVelocity()
	 */
	float maxVelocity;

	/** This variable contains the maximum acceleration in steps/s to be used. The
	 * can be set and read by the user of the library using the
	 * functions setMaxAcceleration()
	 */
	float maxAcceleration;
	float maxDeceleration;
	bool invertPidDropinDirection;
	float rpmToVelocity;
	float angleToStep;

	uint16_t microSteps;
	uint16_t fullSteps;

	uint16_t dropinStepSize;

	int32_t stepCnt;

	float stepsPerSecondToRPM;
	float RPMToStepsPerSecond;

	float currentPidSpeed;
	/** This variable is used to indicate which mode the uStepper is
	* running in (Normal, dropin or pid)*/
	volatile uint8_t mode;
	float pTerm;
	/** This variable contains the integral coefficient used by the PID */
	float iTerm;

	float dTerm;
	bool brake;
	volatile bool pidDisabled;
	/** This variable sets the threshold for activating/deactivating closed loop position control - i.e. it is the allowed error in steps for the control**/
	volatile float controlThreshold = 10;
	/** This variable holds information on wether the motor is stalled or not.
	0 = OK, 1 = stalled */
	volatile bool stall;
	// SPI functions

	volatile int32_t pidPositionStepsIssued = 0;
	volatile float currentPidError;

	/** This variable holds the default stall threshold, but can be updated by the user. */
	int8_t stallThreshold = 4;

	/** This variable hold the default state for Stallguard (stopOnStall) */
	bool stallStop = false;

	/** Flag to keep track of stallguard */
	bool stallEnabled = false;

	/** Flag to keep track of shaft direction setting */
	volatile bool shaftDir = 0;

	//float pid(float error);

	//dropinCliSettings_t dropinSettings;
	//bool loadDropinSettings(void);
	//void saveDropinSettings(void);
	//uint8_t dropinSettingsCalcChecksum(dropinCliSettings_t *settings);
	
};

extern UstepperSTM *ptr;

#endif