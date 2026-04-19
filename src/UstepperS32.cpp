/********************************************************************************************
* 	 	File: 		uStepperS.cpp															*
*		Version:    2.3.0                                          						    *
*      	Date: 		December 27th, 2021  	                                    			*
*      	Authors: 	Thomas Hørring Olsen                                   					*
*					Emil Jacobsen															*
*                                                   										*
*********************************************************************************************
*	(C) 2020																				*
*																							*
*	uStepper ApS																			*
*	www.ustepper.com 																		*
*	administration@ustepper.com 															*
*																							*
*	The code contained in this file is released under the following open source license:	*
*																							*
*			Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International			*
* 																							*
* 	The code in this file is provided without warranty of any kind - use at own risk!		*
* 	neither uStepper ApS nor the author, can be held responsible for any damage				*
* 	caused by the use of the code contained in this file ! 									*
*                                                                                           *
********************************************************************************************/
/**
* @file uStepperS.cpp
*
* @brief      Function and class implementation for the uStepper S library
*
*             This file contains class and function implementations for the library.
*
* @author     Thomas Hørring Olsen (thomas@ustepper.com)
*/
#include <UstepperS32.h>
Callbacks_t callbacks = {
							._closedLoopCallback = closedLoopCallback,
							._mainTimerCallback = mainTimerCallback,
							._dropInStepInputEXTI = dropInStepInputEXTI,
							._dropInDirInputEXTI = dropInDirInputEXTI,
							._dropInEnableInputEXTI = dropInEnableInputEXTI,
							._dropInHandler = dropInHandler
						};
UstepperS32 *ptr;
UstepperS32::UstepperS32() : driver(), encoder(), dropin()
{
	ptr = this;

	this->microSteps = 256;

	this->setMaxAcceleration(2000.0, false);
	this->setMaxDeceleration(2000.0, false);
	this->setMaxVelocity(100.0, false);
}

UstepperS32::UstepperS32(float acceleration, float velocity) : driver(), encoder()
{
	ptr = this;

	this->microSteps = 256;

	this->setMaxDeceleration(acceleration, false);
	this->setMaxAcceleration(acceleration, false);
	this->setMaxVelocity(velocity, false);
}

void UstepperS32::setup(uint8_t mode,
						uint16_t fullstepsPerRevolution,
						float pTerm,
						float iTerm,
						float dTerm,
						uint16_t dropinStepSize,
						bool setHome,
						uint8_t invert,
						uint8_t runCurrent,
						uint8_t holdCurrent)
{
	this->encoder.init();
	this->driver.init();

	dropinCliSettings_t tempSettings;
	this->pidDisabled = 1;

	this->fullSteps = fullstepsPerRevolution;
	this->angleToStep = (float)this->fullSteps * (float)this->microSteps / 360.0;
	this->rpmToVelocity = (float)(279620.267 * fullSteps * microSteps) / (DRIVERCLOCKFREQ);
	this->stepsPerSecondToRPM = 60.0 / (this->microSteps * this->fullSteps);
	this->RPMToStepsPerSecond = (this->microSteps * this->fullSteps) / 60.0;

	this->stepTime = 16777216.0 / DRIVERCLOCKFREQ; // 2^24/DRIVERCLOCKFREQ
	this->rpmToVel = (this->fullSteps * this->microSteps) / (60.0 / this->stepTime);
	this->velToRpm = 1.0 / this->rpmToVel;

	this->driver.setDeceleration((uint32_t)(this->maxDeceleration));
	this->driver.setAcceleration((uint32_t)(this->maxAcceleration));

	this->setCurrent(40.0);
	this->setHoldCurrent(40.0);


	if (setHome == true)
	{
		encoder.setHome();
	}

	// Load encoder calibration from flash if available
	encoderCalibration.loadFromFlash();
	// Wire calibration into encoder for runtime linearization
	this->encoder.setCalibration(&encoderCalibration);

	if (mode == DROPIN)
	{
		this->checkOrientation(10);
	}
	this->mode = mode;

	mainTimerInit();
	if (mode == DROPIN)
	{
		dropin.init(dropinStepSize);
		dropin.setProportional(pTerm);
		dropin.setIntegral(iTerm);
		dropin.setDifferential(dTerm);
	}
}

void UstepperS32::runContinous(bool direction)
{
	this->driver.setDeceleration((uint32_t)(this->maxDeceleration));
	this->driver.setAcceleration((uint32_t)(this->maxAcceleration));
	this->driver.setVelocity((uint32_t)(this->maxVelocity));

	// Make sure we use velocity mode
	this->driver.setRampMode(VELOCITY_MODE_POS);

	// Set the direction
	this->driver.setDirection(direction);
}

bool UstepperS32::getMotorState(uint8_t statusType)
{
	this->driver.readMotorStatus();
	if (this->driver.status & statusType)
	{
		return 0;
	}
	return 1;
}

void UstepperS32::checkOrientation(float distance)
{
	float startAngle;
	uint8_t inverted = 0;
	uint8_t noninverted = 0;
	bool pidEnabled = this->mode == NORMAL ? false : true;

	if (pidEnabled)
	{
		this->disablePid();
	}

	this->shaftDir = 0;
	this->driver.setShaftDirection(this->shaftDir);

	while(inverted < 2 && noninverted < 2)
	{
		startAngle = this->encoder.getAngleMoved();
		this->moveAngle(distance);
		while (this->getMotorState());
		if (this->encoder.getAngleMoved() < startAngle)
		{
			inverted++;
		}
		else
		{
			noninverted++;
		}
		this->moveAngle(-distance);
		while (this->getMotorState());
		if (noninverted > 0 && inverted > 0)
		{
			inverted = 0;
			noninverted = 0;
		}
	}
	if (inverted > 0)
	{
		this->shaftDir = 1;
		this->driver.setShaftDirection(this->shaftDir);
	}
	if (pidEnabled)
	{
		this->enablePid();
	}
}

void UstepperS32::setMaxVelocity(float velocity, bool applyToDriver)
{
	velocity *= (float)this->microSteps;
	velocity = abs(velocity) * VELOCITYCONVERSION;

	this->maxVelocity = velocity;

	if (applyToDriver == false)
	{
		return;
	}
	// Steps per second, has to be converted to microsteps
	this->driver.setVelocity((uint32_t)(this->maxVelocity));
}

void UstepperS32::setMaxAcceleration(float acceleration, bool applyToDriver)
{
	acceleration *= (float)this->microSteps;
	acceleration = abs(acceleration) * ACCELERATIONCONVERSION;

	this->maxAcceleration = acceleration;

	if (applyToDriver == false)
	{
		return;
	}

	// Steps per second, has to be converted to microsteps
	this->driver.setAcceleration((uint32_t)(this->maxAcceleration));
}

void UstepperS32::setMaxDeceleration(float deceleration, bool applyToDriver)
{
	deceleration *= (float)this->microSteps;
	deceleration = abs(deceleration) * ACCELERATIONCONVERSION;

	this->maxDeceleration = deceleration;

	if (applyToDriver == false)
	{
		return;
	}

	// Steps per second, has to be converted to microsteps
	this->driver.setDeceleration((uint32_t)(this->maxDeceleration));
}

void UstepperS32::moveSteps(int32_t steps)
{
	this->driver.setDeceleration((uint16_t)(this->maxDeceleration));
	this->driver.setAcceleration((uint16_t)(this->maxAcceleration));
	this->driver.setVelocity((uint32_t)(this->maxVelocity));

	// Get current position
	int32_t current = this->driver.getPosition();

	// Set new position
	this->driver.setPosition(current + steps);
}

void UstepperS32::moveAngle(float angle)
{
	int32_t steps;

	if (angle < 0.0)
	{
		steps = (int32_t)((angle * angleToStep) - 0.5);
		this->moveSteps(steps);
	}
	else
	{
		steps = (int32_t)((angle * angleToStep) + 0.5);
		this->moveSteps(steps);
	}
}

void UstepperS32::moveToAngle(float angle)
{
	float diff;
	int32_t steps;

	diff = angle - this->angleMoved();
	steps = (int32_t)((abs(diff) * angleToStep) + 0.5);

	if (diff < 0.0)
	{
		this->moveSteps(-steps);
	}
	else
	{
		this->moveSteps(steps);
	}
}

void UstepperS32::setCurrent(double current)
{
	if (current <= 100.0 && current >= 0.0)
	{
		// The current needs to be in the range of 0-31
		this->driver.current = ceil(0.31 * current);
	}
	else
	{
		// If value is out of range, set default
		this->driver.current = 16;
	}

	driver.updateCurrent();
}

void UstepperS32::enableStallguard(int8_t threshold, bool stopOnStall, float rpm)
{
	this->clearStall();
	this->stallThreshold = threshold;
	this->stallStop = stopOnStall;

	ptr->driver.enableStallguard(threshold, stopOnStall, rpm);

	this->stallEnabled = true;
}

void UstepperS32::disableStallguard(void)
{
	ptr->driver.disableStallguard();

	this->stallEnabled = false;
}

void UstepperS32::clearStall(void)
{
	ptr->driver.clearStall();
}

bool UstepperS32::isStalled(void)
{
	return this->isStalled(this->stallThreshold);
}

bool UstepperS32::isStalled(int8_t threshold)
{
	// If the threshold is different from what is configured..
	if (threshold != this->stallThreshold || this->stallEnabled == false)
	{
		// Reconfigure stallguard
		this->enableStallguard(threshold, this->stallStop, 10);
	}

	int32_t stats = ptr->driver.readRegister(RAMP_STAT);

	// Only interested in 'status_sg', with bit position 13 (last bit in RAMP_STAT).
	return (stats >> 13);
}

void UstepperS32::setBrakeMode(uint8_t mode, float brakeCurrent)
{
	int32_t registerContent = this->driver.readRegister(PWMCONF);
	registerContent &= ~(3UL << 20);
	if (mode == FREEWHEELBRAKE)
	{
		this->setHoldCurrent(0.0);
		this->driver.writeRegister(PWMCONF, PWM_AUTOSCALE(1) | PWM_GRAD(1) | PWM_AMPL(128) | PWM_FREQ(0) | FREEWHEEL(1));
	}
	else if (mode == COOLBRAKE)
	{
		this->setHoldCurrent(0.0);
		this->driver.writeRegister(PWMCONF, PWM_AUTOSCALE(1) | PWM_GRAD(1) | PWM_AMPL(128) | PWM_FREQ(0) | FREEWHEEL(2));
	}
	else
	{
		this->setHoldCurrent(brakeCurrent);
		this->driver.writeRegister(PWMCONF, PWM_AUTOSCALE(1) | PWM_GRAD(1) | PWM_AMPL(128) | PWM_FREQ(0) | FREEWHEEL(0));
	}
}

float UstepperS32::angleMoved(void)
{
	return this->encoder.getAngleMoved();
}

float UstepperS32::getDriverRPM(void)
{
	int32_t velocity = this->driver.getVelocity();

	return (float)velocity * this->velToRpm;
}

void UstepperS32::setControlThreshold(float threshold)
{
	this->controlThreshold = threshold;
}
void UstepperS32::enablePid(void)
{
	this->pidDisabled = 0;
	this->mode = CLOSEDLOOP;
}

void UstepperS32::disablePid(void)
{
	this->pidDisabled = 1;
	this->mode = NORMAL;
}

void UstepperS32::enableClosedLoop(void)
{
	this->enablePid();
}

void UstepperS32::disableClosedLoop(void)
{
	this->disablePid();
}

void UstepperS32::setRPM(float rpm)
{
	int32_t velocityDir = rpmToVelocity * rpm;

	if (velocityDir > 0)
	{
		driver.setDirection(1);
	}
	else
	{
		driver.setDirection(0);
	}

	// The velocity cannot be signed
	uint32_t velocity = abs(velocityDir);

	driver.setVelocity((uint32_t)velocity);
}

void UstepperS32::stop(bool mode)
{

	if (mode == HARD)
	{
		this->driver.setDeceleration(0xFFFE);
		this->driver.setAcceleration(0xFFFE);
		this->setRPM(0);
		while (this->driver.readRegister(VACTUAL) != 0)
			;
		this->driver.setDeceleration((uint32_t)(this->maxDeceleration));
		this->driver.setAcceleration((uint32_t)(this->maxAcceleration));
	}
	else
	{
		this->setRPM(0);
		while (this->driver.readRegister(VACTUAL) != 0)
			;
	}
	// Get current position
	int32_t current = this->driver.getPosition();
	// Set new position
	this->driver.setPosition(current);
}

float UstepperS32::moveToEnd(bool dir, float rpm, int8_t threshold, uint32_t timeOut)
{
	uint32_t timeOutStart = micros();
	// Lowest reliable speed for stallguard
	if (rpm < 10.0)
		rpm = 10.0;

	if (dir == CW)
		this->setRPM(abs(rpm));
	else
		this->setRPM(-abs(rpm));

	delay(100);

	this->isStalled();
	// Enable stallguard to detect hardware stop (use driver directly, as to not override user stall settings)
	ptr->driver.enableStallguard(threshold, true, rpm);

	float length = this->encoder.getAngleMoved();

	while (!this->isStalled())
	{
		delay(1);
		if ((micros() - timeOutStart) > (timeOut * 1000))
		{
			break; // TimeOut !! break out and exit
		}
	}
	this->stop();
	ptr->driver.clearStall();

	// Return to normal operation
	ptr->driver.disableStallguard();

	length -= this->encoder.getAngleMoved();
	delay(1000);
	return abs(length);
}

float UstepperS32::getPidError(void)
{
	return this->currentPidError;
}

void UstepperS32::setHoldCurrent(double current)
{
	// The current needs to be in the range of 0-31
	if (current <= 100.0 && current >= 0.0)
	{
		// The current needs to be in the range of 0-31
		this->driver.holdCurrent = ceil(0.31 * current);
	}
	else
	{
		// If value is out of range, set default
		this->driver.holdCurrent = 16;
	}

	driver.updateCurrent();
}
extern "C" uint8_t getUstepperMode()
{
	return ptr->mode;
}

// ========== Encoder Calibration Implementation ==========

bool UstepperS32::calibrateEncoder(uint8_t current)
{
	// Disable PID and any motion
	bool wasPidEnabled = !this->pidDisabled;
	uint8_t prevMode = this->mode;
	this->disablePid();
	this->stop(HARD);
	delay(500);

	// Pause main timer to prevent interference with calibration sampling
	mainTimerPause();

	// Set calibration current
	this->setCurrent((double)current);

	// Microsteps per full revolution
	int32_t stepsPerRev = (int32_t)this->fullSteps * (int32_t)this->microSteps;
	// Bin size in encoder counts
	const int32_t binSize = ENCODER_COUNTS_PER_REV / CALIBRATION_TABLE_SIZE; // 64

	// Accumulator arrays for error averaging (allocated on stack — 2KB each)
	int32_t errorSum[CALIBRATION_TABLE_SIZE];
	uint16_t errorCount[CALIBRATION_TABLE_SIZE];
	memset(errorSum, 0, sizeof(errorSum));
	memset(errorCount, 0, sizeof(errorCount));

	// Zero the driver position counter
	this->driver.writeRegister(XACTUAL, 0);
	this->driver.writeRegister(XTARGET, 0);
	delay(200);

	// Configure velocity mode: slow constant speed (~2 RPM)
	// TMC5130 velocity units: v [Hz] = VMAX * fCLK / 2^24
	// For 200 full steps * 256 microsteps = 51200 usteps/rev
	// At 2 RPM: 51200 * 2 / 60 = 1706.7 usteps/s
	// With fCLK ~12MHz: VMAX = 1707 * 2^24 / 12e6 ≈ 2389
	// Use a conservative low speed for accuracy
	uint32_t calVelocity = 3000;  // ~2-3 RPM depending on clock

	this->driver.setAcceleration(500);
	this->driver.setDeceleration(500);
	this->driver.setVelocity(calVelocity);
	this->driver.setRampMode(VELOCITY_MODE_POS);

	Serial.println(F("Calibration: running at constant speed..."));
	Serial.print(F("  Revolutions: "));
	Serial.println(CALIBRATION_NUM_REVOLUTIONS);

	// Wait for motor to reach steady speed
	delay(2000);

	Serial.println(F("  Sampling..."));

	// Sample continuously until we've completed the target number of revolutions
	int32_t lastXactual = 0;
	uint32_t totalSamples = 0;
	uint32_t startTime = millis();

	while (true)
	{
		// Read XACTUAL (cumulative microstep count) and encoder simultaneously
		int32_t xactual = this->driver.readRegister(XACTUAL);
		uint16_t encRaw = this->encoder.readAngleAbsolute();

		// Check if we've completed enough revolutions
		if (xactual >= stepsPerRev * CALIBRATION_NUM_REVOLUTIONS)
			break;

		// Timeout safety (120 seconds max)
		if (millis() - startTime > 120000UL)
		{
			Serial.println(F("  Timeout!"));
			this->driver.setVelocity(0);
			delay(500);
			mainTimerStart();
			return false;
		}

		// Compute expected encoder angle from XACTUAL
		// expected = (xactual % stepsPerRev) * 32768 / stepsPerRev
		int32_t posInRev = xactual % stepsPerRev;
		if (posInRev < 0) posInRev += stepsPerRev;
		int32_t expectedEnc = (int32_t)((int64_t)posInRev * ENCODER_COUNTS_PER_REV / stepsPerRev);

		// Error = actual - expected (wrap to [-16384, 16384))
		int32_t error = (int32_t)encRaw - expectedEnc;
		if (error > ENCODER_COUNTS_PER_REV / 2) error -= ENCODER_COUNTS_PER_REV;
		if (error < -ENCODER_COUNTS_PER_REV / 2) error += ENCODER_COUNTS_PER_REV;

		// Bin by expected encoder position (this is the "ideal" angle axis)
		uint16_t bin = (uint16_t)(expectedEnc / binSize);
		if (bin >= CALIBRATION_TABLE_SIZE) bin = CALIBRATION_TABLE_SIZE - 1;

		errorSum[bin] += error;
		errorCount[bin]++;
		totalSamples++;

		// ~1ms between samples → ~1kHz sampling rate
		delayMicroseconds(1000);

		// Progress every 2 seconds
		if ((totalSamples & 0x7FF) == 0)
		{
			int32_t rev100 = (int32_t)((int64_t)xactual * 100 / stepsPerRev);
			Serial.print(F("  Rev "));
			Serial.print(rev100 / 100);
			Serial.print('.');
			Serial.print(rev100 % 100);
			Serial.print('/');
			Serial.print(CALIBRATION_NUM_REVOLUTIONS);
			Serial.print(F("  samples="));
			Serial.println(totalSamples);
		}
	}

	// Stop motor
	this->driver.setVelocity(0);
	delay(1000);

	Serial.print(F("  Total samples: "));
	Serial.println(totalSamples);

	// Compute average error per bin → this becomes the correction table
	// The correction is NEGATIVE of the error: corrected = raw + correction
	// where correction = -error (we want to subtract the encoder's error)
	uint16_t emptyBins = 0;
	for (uint16_t b = 0; b < CALIBRATION_TABLE_SIZE; b++)
	{
		if (errorCount[b] > 0)
		{
			int16_t avgError = (int16_t)(errorSum[b] / (int32_t)errorCount[b]);
			encoderCalibration.setCorrectionEntry(b, -avgError);
		}
		else
		{
			encoderCalibration.setCorrectionEntry(b, 0);
			emptyBins++;
		}
	}

	if (emptyBins > 0)
	{
		Serial.print(F("  Warning: "));
		Serial.print(emptyBins);
		Serial.println(F(" bins had no samples (interpolated as 0)"));
	}

	// Apply smoothing: 5-tap moving average on the correction table
	// Read back, smooth, write back
	int16_t tempBuf[CALIBRATION_TABLE_SIZE];
	const int16_t *corr = encoderCalibration.getCorrectionTable();
	for (uint16_t b = 0; b < CALIBRATION_TABLE_SIZE; b++)
	{
		int32_t sum = 0;
		for (int8_t k = -2; k <= 2; k++)
		{
			uint16_t idx = (b + k + CALIBRATION_TABLE_SIZE) % CALIBRATION_TABLE_SIZE;
			sum += corr[idx];
		}
		tempBuf[b] = (int16_t)(sum / 5);
	}
	for (uint16_t b = 0; b < CALIBRATION_TABLE_SIZE; b++)
	{
		encoderCalibration.setCorrectionEntry(b, tempBuf[b]);
	}

	// Finalize and save
	encoderCalibration.finalizeCalibration();
	bool saved = encoderCalibration.saveToFlash();

	// Wire calibration into encoder for runtime use
	this->encoder.setCalibration(&encoderCalibration);

	// Restart main timer
	mainTimerStart();

	// Re-home encoder now that linearization is active
	this->encoder.setHome();

	// Restore previous state
	if (wasPidEnabled)
	{
		this->enablePid();
	}
	this->mode = prevMode;

	Serial.println(saved ? F("Calibration saved to flash!") : F("Flash write FAILED!"));
	return saved;
}

bool UstepperS32::isEncoderCalibrated(void)
{
	return encoderCalibration.isCalibrated();
}

bool UstepperS32::eraseEncoderCalibration(void)
{
	return encoderCalibration.eraseCalibration();
}
