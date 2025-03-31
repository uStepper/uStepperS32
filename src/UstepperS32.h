
#ifndef _USTEPPER_STM_H_
#define _USTEPPER_STM_H_

#include <Arduino.h>
#include "HAL/spi.h"
#include "HAL/gpio.h"
#include "peripherals/TMC5130.h"
#include "peripherals/TLE5012B.h"
#include "HAL/timer.h"
#include "callbacks.h"
#include "utils/dropin.h"

class UstepperS32;

#define FREEWHEELBRAKE 0 /**< Define label users can use as argument for setBrakeMode() function to specify freewheeling as brake mode. This will result in no holding torque at standstill */
#define COOLBRAKE 1		 /**< Define label users can use as argument for setBrakeMode() function to make the motor brake by shorting the two bottom FET's of the H-Bridge. This will provide less holding torque, but will significantly reduce driver heat */
#define HARDBRAKE 2		 /**< Define label users can use as argument for setBrakeMode() function to use full specified current for braking. This will provide high holding torque, but will make the driver (and motor) dissipate power*/

#define CW 1  /**< Define label users can use as argument for runContinous() function to specify clockwise direction */
#define CCW 0 /**< Define label users can use as argument for runContinous() function to specify counterclockwise direction */

#define POSITION_REACHED 0x20 /**< Define label users can use as argument for getMotorState() function to check if target position has been reached*/
#define VELOCITY_REACHED 0x10 /**< Define label users can use as argument for getMotorState() function to check if target velocity has been reached */
#define STANDSTILL 0x08		  /**< Define label users can use as argument for getMotorState() function to check if motor is not currently running */
#define STALLGUARD2 0x04	  /**< Define label users can use as argument for getMotorState() function to check stallguard status */

#define HARD 0 /**< Define label users can use as argument for stop() function to specify that the motor should stop immediately (without decelerating) */
#define SOFT 1 /**< Define label users can use as argument for stop() function to specify that the motor should decelerate before stopping */

#define NORMAL 0	   /**< Value defining normal mode*/
#define DROPIN 1	   /**< Value defining dropin mode for 3d printer/CNC controller boards*/
#define CLOSEDLOOP 2   /**< Value defining closed loop mode for normal library functions*/
#define PID CLOSEDLOOP /**< Value defining PID mode for normal library functions. only here for backwards compatibility*/

#define ACCELERATIONCONVERSION 0.0220 //1.0 / (DRIVERCLOCKFREQ*DRIVERCLOCKFREQ / (512*256) / 16777216) /**< page 74 datasheet*/
#define VELOCITYCONVERSION 1.677 //1.0 / (DRIVERCLOCKFREQ/2/8388608)   /**< page 74 datasheet*/

#ifdef __cplusplus
extern "C" {
#endif
	uint8_t getUstepperMode();
#ifdef __cplusplus
}
#endif

class UstepperS32
{
	friend class TMC5130;
	friend class TLE5012B;
	friend class Dropin;

  public:
	TLE5012B encoder;
	TMC5130 driver;
	Dropin dropin;
	/**
	 * @brief	Constructor of uStepper class
	 */
	UstepperS32();
	/**
	 * @brief	Overloaded Constructor of uStepper class
	 */
	UstepperS32(float acceleration, float velocity);
	/**
	 * @brief      Initializes the different parts of the uStepper S object
	 *
	 *             This function initializes the different parts of the uStepper S
	 *             object, and should be called in the setup() function of the
	 *             arduino sketch. This function is needed as some things, like
	 *             the timer can not be setup in the constructor, since arduino
	 *             for some strange reason, resets a lot of the AVR registers
	 *             just before entering the setup() function.
	 *
	 * @param[in]  mode             	Default is normal mode. Pass the constant
	 *                              	"DROPIN" to configure the uStepper to act as
	 *                              	dropin compatible to the stepstick. Pass the
	 *                              	constant "PID", to enable closed loop feature for
	 *                              	regular movement functions, such as
	 *                              	moveSteps()
	 * @param[in]  stepsPerRevolution   Number of fullsteps per revolution
	 *
	 * @param[in]  pTerm            	The proportional coefficent of the DROPIN PID
	 *                              	controller
	 * @param[in]  iTerm            	The integral coefficent of the DROPIN PID
	 *                              	controller
	 * @param[in]  dTerm            	The differential coefficent of the DROPIN PID
	 *                              	controller
	 * @param[in]  dropinStepSize		number of steps per fullstep, send from
	 *									external dropin controller
	 * @param[in]  setHome          	When set to true, the encoder position is
	 *									Reset. When set to false, the encoder
	 *									position is not reset.
	 * @param[in]  invert           	Inverts the motor direction for dropin
	 *									feature. 0 = NOT invert, 1 = invert.
	 *									this has no effect for other modes than dropin
	 * @param[in]  runCurrent       	Sets the current (in percent) to use while motor is running.
	 * @param[in]  holdCurrent      	Sets the current (in percent) to use while motor is NOT running
	 */
	void setup(uint8_t mode = NORMAL,
			   uint16_t fullstepsPerRevolution = 200,
			   float pTerm = 10,
			   float iTerm = 0.2,
			   float dTerm = 0.0,
			   uint16_t dropinStepSize = 16,
			   bool setHome = true,
			   uint8_t invert = 0,
			   uint8_t runCurrent = 50,
			   uint8_t holdCurrent = 30);

	/**
	 * @brief      Set the velocity in rpm
	 *
	 *             This function lets the user set the velocity of the motor in rpm.
	 *             A negative value switches direction of the motor.
	 *
	 * @param[in]  rpm  - The velocity in rotations per minute
	 */
	void setRPM(float rpm);

	/**
	 * @brief      Get the RPM from driver
	 *
	 *             This function returns the driver velocity of the motor
	 *
	 * @return     The velocity in rpm
	 */
	float getDriverRPM(void);

	/**
	 * @brief      Make the motor perform a predefined number of microsteps
	 *
	 *             This function makes the motor perform a predefined number of
	 *             steps, using the acceleration profile. The motor will accelerate
	 *             at the rate set by setMaxAcceleration(), decelerate at the rate set by setMaxDeceleration()
     *             and eventually reach the speed set by setMaxVelocity() function.
     *             The direction of rotation is set by the sign of the commanded microsteps to perform.
	 *
	 * @param[in]      microsteps -	Number of microsteps to be performed. an input value of
	 *								300 makes the motor go 300 microsteps in CW direction, and
	 *								an input value of -300 makes the motor move 300 microsteps
	 *								in CCW direction.
	 */
	void moveSteps(int32_t microsteps);

	/**
	 * @brief      	Makes the motor rotate a specific angle relative to the current position
	 *
	 *              This function makes the motor a rotate by a specific angle relative to
	 *			    the current position, using the acceleration profile. The motor will accelerate
	 *              at the rate set by setMaxAcceleration(), decelerate at the rate set by
	 *				setMaxDeceleration() and eventually reach the speed set
	 *              by setMaxVelocity() function. The direction of rotation
	 *              is set by the sign of the commanded angle to move
	 *
	 * @param[in]  	    angle     -	Angle to move. An input value of
	 *								300 makes the motor go 300 degrees in CW direction, and
	 *								an input value of -300 makes the motor move 300 degrees
	 *								in CCW direction.
	 */
	void moveAngle(float angle);

	/**
	 * @brief      	Makes the motor rotate to a specific absolute angle
	 *
	 *              This function makes the motor a rotate to a specific angle,
	 *			    using the acceleration profile. The motor will accelerate
	 *              at the rate set by setMaxAcceleration(), decelerate at the rate set by
	 *				setMaxDeceleration() and eventually reach the speed set
	 *              by setMaxVelocity() function. The direction of rotation
	 *              is set by the sign of the commanded angle to move
	 *
	 * @param[in]  	    angle     -	Angle to move to. An input value of
	 *								300 makes the motor go to absolute 300 degrees,
	 *								and an input value of -300 makes the motor move
	 *								to absolute -300 degrees.
	 */
	void moveToAngle(float angle);

	/**
	 * @brief      Get the current motor driver state
	 *
	 *				This function is used to check some internal status flags of the driver.
	 *				The argument is used to specify the flag to check
	 *
	 *	param[in]	statusType - status flag to check. Possible values:
	 *					POSITION_REACHED - has last commanded position been reached?
	 *					VELOCITY_REACHED - has last commanded velocity been reached?
	 *					STANDSTILL - Are the motor currently stopped?
	 *					STALLGUARD2 - Has the stallguard been trickered?
	 *
	 *
	 * @return     The angle moved.
	 */
	bool getMotorState(uint8_t statusType = POSITION_REACHED);

	/**
	 * @brief      Make the motor rotate continuously
	 *
	 *             This function makes the motor rotate continuously, using the
	 *             acceleration profile.
	 *
	 * @param[in]      dir   - Can be set to "CCW" or "CW" (without the quotes)
	 */
	void runContinous(bool dir);

	/**
	 * @brief      Get the angle moved from reference position in degrees
	 *
	 * @return     The angle moved in degrees.
	 */
	float angleMoved(void);

	/**
	 * @brief      Set motor output current.
	 *
	 *             This function allows the user to change the current setting of the motor driver.
	 *
	 * @param[in]      current  - Desired current in percent (0% - 100%)
	 */
	void setCurrent(double current);

	/**
	 * @brief      Set motor hold current.
	 *
	 *             This function allows the user to change the current setting of the motor driver.
	 *
	 * @param[in]      current  - Desired hold current in percent (0% - 100%)
	 */
	void setHoldCurrent(double current);

	/**
	 * @brief      Set the maximum acceleration of the stepper motor.
	 *
	 *             This function lets the user set the max acceleration used
	 *             by the stepper driver.
	 *
	 * @param[in]      acceleration  - Maximum acceleration in fullsteps/s^2
	 */
	void setMaxAcceleration(float acceleration, bool applyToDriver = true);

	/**
	 * @brief      Set the maximum deceleration of the stepper motor.
	 *
	 *             This function lets the user set the max deceleration used
	 *             by the stepper driver.
	 *
	 * @param[in]      deceleration  - Maximum deceleration in fullsteps/s^2
	 */
	void setMaxDeceleration(float deceleration, bool applyToDriver = true);

	/**
	 * @brief      Set the maximum velocity of the stepper motor.
	 *
	 *             This function lets the user set the max velocity used
	 *             by the stepper driver.
	 *
	 * @param[in]      velocity  - Maximum velocity in fullsteps/s
	 */
	void setMaxVelocity(float velocity, bool applyToDriver = true);

	/**
	 * @brief      Enable TMC5130 StallGuard
	 *
	 *             	This function enables the builtin stallguard offered from TMC5130 stepper driver.
	 * 				The threshold should be tuned as to trigger stallguard before a step is lost.
	 *
	 * @param	   threshold 	- stall sensitivity. A value between -64 and +63
	 * @param      stopOnStall  - should the driver automatic stop the motor on a stall
	 */
	void enableStallguard(int8_t threshold = 4, bool stopOnStall = false, float rpm = 10.0);

	/**
	 * @brief      	Disables the builtin stallguard offered from TMC5130, and reenables StealthChop.
	 */
	void disableStallguard(void);

	/**
	 * @brief      	Clear the stallguard, reenabling the motor to return to its previous operation.
	 */
	void clearStall(void);

	/**
	 * @brief      	This method returns a bool variable indicating wether the motor is stalled or not.
	 * 				Uses the default stallguard threshold, unless this has been changed by .enableStallguard()
	 *
	 * @return     	0 = not stalled, 1 = stalled
	 */
	bool isStalled(void);

	/**
	 * @brief      	This method returns a bool variable indicating wether the motor is stalled or not.
	 * 				The stallguard is sensitive to the speed of the motor, as the torque available is a
	 * 				function of the speed. Therefore, it is necessary to change the treshold according
	 * 				to the application. A higher treshold makes the stallguard less sensitive to external
	 * 				loads, meaning that, the higher the application speed, the higher the treshold has to
	 * 				be for the stall guard to perform well
	 *
	 * @param[in]   threshold  -  Threshold for stallguard. A value between -64 and +63
	 *
	 * @return     	0 = not stalled, 1 = stalled
	*/
	bool isStalled(int8_t threshold);

	/**
	 * @brief
	 *
	 * @param[in]   mode  -  this parameter specifies how the motor should brake during standstill.
	 * 				available modes:
	 * 				FREEWHEELBRAKE - This will result in no holding torque at standstill
	 *				COOLBRAKE 1 - This will make the motor brake by shorting the two bottom FET's of the H-Bridge. This will provide less holding torque, but will significantly reduce driver heat
	 *				HARDBRAKE 2 - This will make the motor brake by sending the full specified current through the coils. This will provide high holding torque, but will make the driver (and motor) dissipate power
	 *
	 * @param[in]   brakeCurrent (optional) -  if HARDBRAKE is use as mode, this argument can set the current to use for braking (0-100% of 2A).
	 * 				If argument is not specified, the motor will brake with 25% of max current
	*/
	void setBrakeMode(uint8_t mode, float brakeCurrent = 25.0);

	/**
	 * @brief      	This method reenables the PID after being disabled.
	 *
	 */
	void enablePid(void);

	/**
	 * @brief      	This method disables the PID until calling enablePid.
	 *
	 */
	void disablePid(void);

	/**
	 * @brief      	This method reenables the closed loop mode after being disabled.
	 *
	 */
	void enableClosedLoop(void);

	/**
	 * @brief      	This method disables the closed loop mode until calling enableClosedLoop.
	 *
	 */
	void disableClosedLoop(void);

	/**
	 * @brief      	This method sets the control threshold for the closed loop position control in microsteps - i.e. it is the allowed control error. 10 microsteps is suitable in most applications.
	 *
	 */
	void setControlThreshold(float threshold);

	/**
	 * @brief      	Moves the motor to its physical limit, without limit switch
	 *
	 *              This function, makes the motor run continously, untill the
	 *				encoder detects a stall, at which point the motor is assumed
	 *				to be at it's limit.
	 *
	 * @param[in]  	dir  Direction to search for limit
	 *
	 * @param[in]   rpm   RPM of the motor while searching for limit
	 *
	 * @param[in]  	threshold  Sensitivity of stall detection (-64 to +63), low is more sensitive
	 *
	 * @param[in]  	Timeout in milliseconds to exit function if homing doesnt behave as expected
	 *
	 * @return 		Degrees turned from calling the function, till end was reached
	 */
	float moveToEnd(bool dir, float rpm = 40.0, int8_t threshold = 4, uint32_t timeOut = 100000);

	/**
	 * @brief      Stop the motor
	 *
	 *             	This function stops any ongoing motor movement. The "mode" argument
	 *				determines whether the motor should stop with or without
	 *				a deceleration phase
	 *
	 * @param      mode  -	can be set to "HARD" for no deceleration phase
	 *						or "SOFT" for deceleration phase.
	 */
	void stop(bool mode = HARD);

	/**
	 * @brief      This method returns the current PID error
	 * @return     PID error (float)
	 */
	float getPidError(void); //*delete*/

	/**
	 * @brief      	This method is used to check the orientation of the motor connector.
	 *
	 * @param[in]  	distance - the amount of degrees the motor shaft should rotate during orientation determination.
	 *
	 */

	void checkOrientation(float distance = 10);

  private:
	friend void mainTimerCallback();
	friend void dropInStepInputEXTI();
	friend void dropInDirInputEXTI();
	friend void dropInEnableInputEXTI();
	friend void closedLoopCallback();

	float stepTime;
	float rpmToVel;
	float velToRpm;

	/** This variable contains the maximum velocity, the motor is allowed to reach at any given point.
     *  The variable is in units of ( 1.0 / ( 2^24 * |f_CLK| ) ) * ( microsteps / second ).
     *  The user of the library can set this by use of the setMaxVelocity() function,
     *  which takes in values with units of ( fullsteps / s ). */
	float maxVelocity;

	/** These variables contain the maximum acceleration/deceleration in steps/s to be used. The
     *  The variables are in units of ( |f_CLK|^2 / 2^41 ) * ( microsteps / s^2 ).
	 *  can be set and read by the user of the library using the setMaxAcceleration() and setMaxDeceleration() functions,
     *  which take in values with units of ( fullsteps / s^2 ). */
	float maxAcceleration;
	float maxDeceleration;

	float rpmToVelocity;
	float angleToStep;

	uint16_t microSteps;
	uint16_t fullSteps;

	float stepsPerSecondToRPM;
	float RPMToStepsPerSecond;
	/** This variable is used to indicate which mode the uStepper is
	* running in (Normal, dropin or pid)*/
	volatile uint8_t mode;
	bool brake;
	volatile bool pidDisabled;
	/** This variable sets the threshold for activating/deactivating closed loop position control - i.e. it is the allowed error in steps for the control**/
	volatile float controlThreshold = 10;
	/** This variable holds information on wether the motor is stalled or not.
	0 = OK, 1 = stalled */
	volatile bool stall;
	// SPI functions
	volatile float currentPidError;

	/** This variable holds the default stall threshold, but can be updated by the user. */
	int8_t stallThreshold = 4;

	/** This variable hold the default state for Stallguard (stopOnStall) */
	bool stallStop = false;

	/** Flag to keep track of stallguard */
	bool stallEnabled = false;

	/** Flag to keep track of shaft direction setting */
	volatile bool shaftDir = 0;
	friend uint8_t getUstepperMode();
	friend void dropInHandler();
};

extern UstepperS32 *ptr;


#endif
