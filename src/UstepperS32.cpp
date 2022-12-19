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
							._dropInEnableInputEXTI = dropInEnableInputEXTI
						};
UstepperS32 *ptr;
UstepperS32::UstepperS32() : driver(), encoder()
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
						uint16_t stepsPerRevolution,
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
	// Should setup mode etc. later
	this->mode = mode;
	this->fullSteps = stepsPerRevolution;
	this->dropinStepSize = 256 / dropinStepSize;
	this->angleToStep = (float)this->fullSteps * (float)this->microSteps / 360.0;
	this->rpmToVelocity = (float)(279620.267 * fullSteps * microSteps) / (DRIVERCLOCKFREQ);
	this->stepsPerSecondToRPM = 60.0 / (this->microSteps * this->fullSteps);
	this->RPMToStepsPerSecond = (this->microSteps * this->fullSteps) / 60.0;

	this->stepTime = 16777216.0 / DRIVERCLOCKFREQ; // 2^24/DRIVERCLOCKFREQ
	this->rpmToVel = (this->fullSteps * this->microSteps) / (60.0 / this->stepTime);
	this->velToRpm = 1.0 / this->rpmToVel;
	
	this->driver.setDeceleration((uint32_t)(this->maxDeceleration));
	this->driver.setAcceleration((uint32_t)(this->maxAcceleration));
	
	this->setCurrent(0.0);
	this->setHoldCurrent(0.0);

	this->setCurrent(40.0);
	this->setHoldCurrent(0.0);

	if (setHome == true)
	{
		encoder.setHome();
	}

	this->pidDisabled = 0;
	
	mainTimerInit();
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
	// TODO: FIX THIS FUNCTION!
	return;
	float startAngle;
	uint8_t inverted = 0;
	uint8_t noninverted = 0;
	this->disablePid();
	this->shaftDir = 0;
	this->driver.setShaftDirection(this->shaftDir);
	return;

	while(inverted < 3 && noninverted < 3)
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
	this->enablePid();
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
}

void UstepperS32::disablePid(void)
{
	this->pidDisabled = 1;
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

float UstepperS32::pid(float error)
{
	float u;
	float limit = abs(this->currentPidSpeed) + 10000.0;
	static float integral;
	static bool integralReset = 0;
	static float errorOld, differential = 0.0;

	this->currentPidError = error;

	u = error * this->pTerm;

	if (u > 0.0)
	{
		if (u > limit)
		{
			u = limit;
		}
	}
	else if (u < 0.0)
	{
		if (u < -limit)
		{
			u = -limit;
		}
	}

	integral += error * this->iTerm;

	if (integral > 200000.0)
	{
		integral = 200000.0;
	}
	else if (integral < -200000.0)
	{
		integral = -200000.0;
	}

	if (error > -10 && error < 10)
	{
		if (!integralReset)
		{
			integralReset = 1;
			integral = 0;
		}
	}
	else
	{
		integralReset = 0;
	}

	u += integral;

	differential *= 0.9;
	differential += 0.1 * ((error - errorOld) * this->dTerm);

	errorOld = error;

	u += differential;

	u *= this->stepsPerSecondToRPM * 16.0;
	this->setRPM(u);
	this->driver.setDeceleration(0xFFFE);
	this->driver.setAcceleration(0xFFFE);
}

void UstepperS32::setProportional(float P)
{
	this->pTerm = P;
}

void UstepperS32::setIntegral(float I)
{
	this->iTerm = I * MAINTIMERINTERRUPTPERIOD;
}

void UstepperS32::setDifferential(float D)
{
	this->dTerm = D * MAINTIMERINTERRUPTFREQUENCY;
}

void UstepperS32::invertDropinDir(bool invert)
{
	this->invertPidDropinDirection = invert;
}

void UstepperS32::parseCommand(String *cmd)
{
	uint8_t i = 0;
	String value;

	if (cmd->charAt(2) == ';')
	{
		Serial.println("COMMAND NOT ACCEPTED");
		return;
	}

	value.remove(0);
	/****************** SET P Parameter ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
	if (cmd->substring(0, 2) == String("P="))
	{
		for (i = 2;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == '.')
			{
				value.concat(cmd->charAt(i));
				i++;
				break;
			}
			else if (cmd->charAt(i) == ';')
			{
				break;
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}

		for (;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == ';')
			{
				Serial.print("COMMAND ACCEPTED. P = ");
				Serial.println(value.toFloat(), 4);
				this->dropinSettings.P = value.toFloat();
				this->saveDropinSettings();
				this->setProportional(value.toFloat());
				return;
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}
	}

	/****************** SET I Parameter ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 2) == String("I="))
	{
		for (i = 2;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == '.')
			{
				value.concat(cmd->charAt(i));
				i++;
				break;
			}
			else if (cmd->charAt(i) == ';')
			{
				break;
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}

		for (;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == ';')
			{
				Serial.print("COMMAND ACCEPTED. I = ");
				Serial.println(value.toFloat(), 4);
				this->dropinSettings.I = value.toFloat();
				this->saveDropinSettings();
				this->setIntegral(value.toFloat());
				return;
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}
	}

	/****************** SET D Parameter ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 2) == String("D="))
	{
		for (i = 2;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == '.')
			{
				value.concat(cmd->charAt(i));
				i++;
				break;
			}
			else if (cmd->charAt(i) == ';')
			{
				break;
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}

		for (;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == ';')
			{
				Serial.print("COMMAND ACCEPTED. D = ");
				Serial.println(value.toFloat(), 4);
				this->dropinSettings.D = value.toFloat();
				this->saveDropinSettings();
				this->setDifferential(value.toFloat());
				return;
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}
	}

	/****************** invert Direction ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 6) == String("invert"))
	{
		if (cmd->charAt(6) != ';')
		{
			Serial.println("COMMAND NOT ACCEPTED");
			return;
		}
		if (this->invertPidDropinDirection)
		{
			Serial.println(F("Direction normal!"));
			this->dropinSettings.invert = 0;
			this->saveDropinSettings();
			this->invertDropinDir(0);
			return;
		}
		else
		{
			Serial.println(F("Direction inverted!"));
			this->dropinSettings.invert = 1;
			this->saveDropinSettings();
			this->invertDropinDir(1);
			return;
		}
	}

	/****************** get Current Pid Error ********************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 5) == String("error"))
	{
		if (cmd->charAt(5) != ';')
		{
			Serial.println("COMMAND NOT ACCEPTED");
			return;
		}
		Serial.print(F("Current Error: "));
		Serial.print(this->getPidError());
		Serial.println(F(" Steps"));
	}

	/****************** Get run/hold current settings ************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 7) == String("current"))
	{
		if (cmd->charAt(7) != ';')
		{
			Serial.println("COMMAND NOT ACCEPTED");
			return;
		}
		Serial.print(F("Run Current: "));
		Serial.print(ceil(((float)this->driver.current) / 0.31));
		Serial.println(F(" %"));
		Serial.print(F("Hold Current: "));
		Serial.print(ceil(((float)this->driver.holdCurrent) / 0.31));
		Serial.println(F(" %"));
	}

	/****************** Get PID Parameters ***********************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 10) == String("parameters"))
	{
		if (cmd->charAt(10) != ';')
		{
			Serial.println("COMMAND NOT ACCEPTED");
			return;
		}
		Serial.print(F("P: "));
		Serial.print(this->dropinSettings.P, 4);
		Serial.print(F(", "));
		Serial.print(F("I: "));
		Serial.print(this->dropinSettings.I, 4);
		Serial.print(F(", "));
		Serial.print(F("D: "));
		Serial.println(this->dropinSettings.D, 4);
	}

	/****************** Help menu ********************************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 4) == String("help"))
	{
		if (cmd->charAt(4) != ';')
		{
			Serial.println("COMMAND NOT ACCEPTED");
			return;
		}
		this->dropinPrintHelp();
	}

	/****************** SET run current ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 11) == String("runCurrent="))
	{
		for (i = 11;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == '.')
			{
				value.concat(cmd->charAt(i));
				i++;
				break;
			}
			else if (cmd->charAt(i) == ';')
			{
				break;
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}

		for (;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == ';')
			{
				if (value.toFloat() >= 0.0 && value.toFloat() <= 100.0)
				{
					i = (uint8_t)value.toFloat();
					break;
				}
				else
				{
					Serial.println("COMMAND NOT ACCEPTED");
					return;
				}
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}

		Serial.print("COMMAND ACCEPTED. runCurrent = ");
		Serial.print(i);
		Serial.println(F(" %"));
		this->dropinSettings.runCurrent = i;
		this->saveDropinSettings();
		this->setCurrent(i);
	}

	/****************** SET run current ***************************
  *                                                            *
  *                                                            *
  **************************************************************/
	else if (cmd->substring(0, 12) == String("holdCurrent="))
	{
		for (i = 12;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == '.')
			{
				value.concat(cmd->charAt(i));
				i++;
				break;
			}
			else if (cmd->charAt(i) == ';')
			{
				break;
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}

		for (;; i++)
		{
			if (cmd->charAt(i) >= '0' && cmd->charAt(i) <= '9')
			{
				value.concat(cmd->charAt(i));
			}
			else if (cmd->charAt(i) == ';')
			{
				if (value.toFloat() >= 0.0 && value.toFloat() <= 100.0)
				{
					i = (uint8_t)value.toFloat();
					break;
				}
				else
				{
					Serial.println("COMMAND NOT ACCEPTED");
					return;
				}
			}
			else
			{
				Serial.println("COMMAND NOT ACCEPTED");
				return;
			}
		}

		Serial.print("COMMAND ACCEPTED. holdCurrent = ");
		Serial.print(i);
		Serial.println(F(" %"));
		this->dropinSettings.holdCurrent = i;
		this->saveDropinSettings();
		this->setHoldCurrent(i);
	}

	/****************** DEFAULT (Reject!) ************************
  *                                                            *
  *                                                            *
  **************************************************************/
	else
	{
		Serial.println("COMMAND NOT ACCEPTED");
		return;
	}
}

void UstepperS32::dropinCli()
{
	static String stringInput;
	static uint32_t t = millis();

	while (1)
	{
		while (!Serial.available())
		{
			delay(1);
			if ((millis() - t) >= 500)
			{
				stringInput.remove(0);
				t = millis();
			}
		}
		t = millis();
		stringInput += (char)Serial.read();
		if (stringInput.lastIndexOf(';') > -1)
		{
			this->parseCommand(&stringInput);
			stringInput.remove(0);
		}
	}
}

void UstepperS32::dropinPrintHelp()
{
	Serial.println(F("uStepper S Dropin !"));
	Serial.println(F(""));
	Serial.println(F("Usage:"));
	Serial.println(F("Show this command list: 'help;'"));
	Serial.println(F("Get PID Parameters: 'parameters;'"));
	Serial.println(F("Set Proportional constant: 'P=10.002;'"));
	Serial.println(F("Set Integral constant: 'I=10.002;'"));
	Serial.println(F("Set Differential constant: 'D=10.002;'"));
	Serial.println(F("Invert Direction: 'invert;'"));
	Serial.println(F("Get Current PID Error: 'error;'"));
	Serial.println(F("Get Run/Hold Current Settings: 'current;'"));
	Serial.println(F("Set Run Current (percent): 'runCurrent=50.0;'"));
	Serial.println(F("Set Hold Current (percent): 'holdCurrent=50.0;'"));
	Serial.println(F(""));
	Serial.println(F(""));
}

bool UstepperS32::loadDropinSettings(void)
{
	dropinCliSettings_t tempSettings;
	//TODO: FIX THIS !
	//EEPROM.get(0, tempSettings);

	if (this->dropinSettingsCalcChecksum(&tempSettings) != tempSettings.checksum)
	{
		return 0;
	}

	this->dropinSettings = tempSettings;
	this->setProportional(this->dropinSettings.P);
	this->setIntegral(this->dropinSettings.I);
	this->setDifferential(this->dropinSettings.D);
	this->invertDropinDir((bool)this->dropinSettings.invert);
	this->setCurrent(this->dropinSettings.runCurrent);
	this->setHoldCurrent(this->dropinSettings.holdCurrent);
	return 1;
}

void UstepperS32::saveDropinSettings(void)
{
	this->dropinSettings.checksum = this->dropinSettingsCalcChecksum(&this->dropinSettings);
	//TODO: FIX THIS !
	//EEPROM.put(0, this->dropinSettings);
}

uint8_t UstepperS32::dropinSettingsCalcChecksum(dropinCliSettings_t *settings)
{
	uint8_t i;
	uint8_t checksum = 0xAA;
	uint8_t *p = (uint8_t *)settings;

	for (i = 0; i < 15; i++)
	{
		checksum ^= *p++;
	}

	return checksum;
}
