#ifndef __TMC5130_H_
#define __TMC5130_H_

#include <Arduino.h>
#include "../HAL/spi.h"
#include "../HAL/gpio.h"
#include "TMC5130RegDef.h"
#include "utils/semaphore.h"
#include "../callbacks.h"
#include "../UstepperS32.h"

#define DRIVERCLOCKFREQ 10000000.0 /**< MCU Clock frequency */

enum TMC5130OperationModes
{
	stepDir = 0,
	spi
};

class TMC5130
{
	friend class UstepperS32;
  public:
	TMC5130();
	/**
		 * @brief		Initiation of the motor driver
		 *
		 *				This function initiates all the registers of the motor driver.
		 *
		 */
	void init();
	/**
		 * @brief		Set the motor position
		 *
		 *				This function tells the motor to go to an absolute position.
		 *
		 * @param[in]	position - position the motor should move to, in micro steps (1/256th default)
		 */
	void setPosition(int32_t position);
	/**
		 * @brief		Set motor velocity
		 *
		 *				This function tells the motor driver to make the motor spin at a specific velocity.
		 *				input unit is in ((microsteps/s)/8MHz)/2^23. See page 74 of datasheet for more information 
		 *
		 * @param[in]	velocity - see description for unit
		 */
	void setVelocity(uint32_t velocity);
	/**
		 * @brief		Set motor acceleration
		 *
		 *				This function tells the motor driver to use a specific acceleration while ramping up the velocity.
		 *				input unit is in (microsteps/s^2)/116.42. See page 74 of datasheet for more information 
		 *
		 * @param[in]	acceleration - see description for unit
		 */
	void setAcceleration(uint32_t acceleration);
	/**
		 * @brief		Set motor deceleration
		 *
		 *				This function tells the motor driver to use a specific deceleration while ramping down the velocity.
		 *				input unit is in (microsteps/s^2)/116.42. See page 74 of datasheet for more information 
		 *
		 * @param[in]	deceleration - see description for unit
		 */
	void setDeceleration(uint32_t deceleration);
	/**
		 * @brief		Set motor driver current
		 *
		 * @param[in]	current - sets the current to use during movements in percent (0-100)
		 */
	void setCurrent(uint8_t current);

	/**
		 * @brief		Set motor driver hold current
		 *
		 * @param[in]	current - sets the current to use during standstill in percent (0-100)
		 */
	void setHoldCurrent(uint8_t current);

	/**
		 * @brief		Set motor driver direction
		 *
		 *				This function is used to set the direction of the motor driver
		 *				to either normal or inverted. In normal mode, a positiv target
		 *				position corresponds to a CW movement, while a negative target
		 *				position corresponds to a CCW movement.
		 *
		 * @param[in]	direction - 0 = normal direction, 1 = inverted direction
		 */
	void setShaftDirection(bool direction);

	/**
		 * @brief		Stops any ongoing movement with deceleration
		 */
	void stop(void);

	/**
		 * @brief		Returns the current speed of the motor driver
		 *
		 *				This function returns the current speed of the 
		 *				internal ramp generator of the motor driver.
		 *				unit is in ((microsteps/s)/8MHz)/2^23. 
		 *				See page 74 of datasheet for more information 
		 *
		 * @return		see description for unit
		 */
	int32_t getVelocity(void);

	/**
		 * @brief		Returns the current position of the motor driver
		 *
		 *				This function returns the position of the motor
		 *				drivers internal position counter.
		 *				unit is in microsteps (default 1/256th). 
		 *
		 * @return		microsteps (default 1/256th).
		 */
	int32_t getPosition(void);

	/**
		 * @brief		Resets the internal position counter of the motor driver
		 * 
		 * @param[in]  initialSteps - Home step offset from zero
		 */
	void setHome(int32_t initialSteps = 0);

	/**
		 * @brief		Write a register of the motor driver
		 *
		 *				This function is used to write a value into
		 *				a register in the TMC5130 motor driver. Please
		 *				refer to datasheet for details.
		 *				
		 *				When using this function you are on your own and expect you know what you are doing !
		 *
		 * @return 		Return data associated with last SPI command
		 *
		 * @param[in]	address - Register to write
		 *
		 * @param[in]	datagram - data to write into the register
		 */
	int32_t writeRegister(uint8_t address, uint32_t datagram);

	/**
		 * @brief		Reads a register from the motor driver
		 *
		 *				This function is used to read the content of
		 *				a register in the TMC5130 motor driver. Please
		 *				refer to datasheet for details.
		 *
		 * @return 		Return data of the read register
		 *
		 * @param[in]	address - Register to read
		 */
	int32_t readRegister(uint8_t address);

	/**
		 * @brief		Returns the load measurement used for Stall detection
		 */
	uint16_t getStallValue(void);

	/** target position in microsteps*/
	volatile int32_t xTarget = 0;

	/** current position in microsteps*/
	volatile int32_t xActual = 0;

	void setRPM(float rpm);
	GPIO stepPin;
  private:
	/**
		 * @brief		Writes the current setting registers of the motor driver  
		 */
	void updateCurrent(void);

	void setOperationMode(uint8_t mode);

	/**
		 * @brief		Set motor driver to position mode or velocity mode
		 *
		 * @param[in]	mode - can be either POSITIONING_MODE or VELOCITY_MODE_POS
		 */
	void setRampMode(uint8_t mode);

	void setDirection(bool direction);

	void reset(void);

	void enableStealth(void);

	void enableStallguard(int8_t threshold, bool stopOnStall, float rpm);

	void disableStallguard(void);

	void clearStall(void);

	uint8_t readMotorStatus(void);
	
	Spi spiHandle;
	GPIO enablePin;
	GPIO sdPin;
	GPIO spiPin;
	
	GPIO dirPin;
	/** Default acceleration profile for positioning mode */
	uint32_t VSTART = 0;
	uint32_t V1 = 0;
	uint32_t VMAX = 200000;
	uint32_t VSTOP = 10;
	uint16_t A1 = 600;
	uint16_t AMAX = 100;
	uint16_t DMAX = 600;
	uint16_t D1 = 600;
	
	uint8_t status;
	uint32_t lastReadValue;
	uint8_t current = 16;
	uint8_t holdCurrent = 0;
	uint8_t holdDelay = 0;
	/** STOP, VELOCITY, POSITION*/
	uint8_t mode = DRIVER_STOP;
	Semaphore semaphore;
	float rpmToVelocity = (float)(279620.267 * 200 * 256) / (DRIVERCLOCKFREQ);
	friend void closedLoopCallback();
	friend void dropInStepInputEXTI();
	friend void dropInDirInputEXTI();
	friend void dropInEnableInputEXTI();
	friend class Dropin;
};

#endif