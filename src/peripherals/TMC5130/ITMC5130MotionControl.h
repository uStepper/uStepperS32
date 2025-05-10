#ifndef __TMC5130MOTIONCONTROL_H
#define __TMC5130MOTIONCONTROL_H

#include "../../UstepperS32.h"

typedef struct TMC5130MotionControlSettings_t
{
	uint32_t VSTART = 0;
	uint32_t V1 = 0;
	uint32_t VMAX = 200000;
	uint32_t VSTOP = 10;
	uint16_t A1 = 600;
	uint16_t AMAX = 100;
	uint16_t DMAX = 600;
	uint16_t D1 = 600;
	/** target position in microsteps*/
	volatile int32_t xTarget = 0;

	/** current position in microsteps*/
	volatile int32_t xActual = 0;
	bool invertShaftDirection = false;
	TMC5130MotionControlMode_e mode = DRIVER_STOP;
};

typedef enum TMC5130MotionControlMode_e
{
	DRIVER_POSITION = 0,
	DRIVER_VELOCITY,
	DRIVER_STEPPER
};

class ITMC5130MotionControl
{
public:
	virtual void setRPM(float RPM) = 0;
	virtual void init(TMC5130 *driver) = 0;
	virtual void deInit() = 0;
	virtual void setPosition(int32_t position) = 0;
	virtual void setVelocity(uint32_t velocity) = 0;
	virtual void setAcceleration(uint32_t acceleration) = 0;
	virtual void setDeceleration(uint32_t deceleration) = 0;
	virtual void setShaftDirection(bool direction) = 0;
	virtual void stop(void) = 0;
	virtual int32_t getVelocity(void) = 0;
	virtual int32_t getPosition(void) = 0;
	virtual void setHome(int32_t initialSteps = 0) = 0;

private:
	TMC5130MotionControlSettings_t settings;
	TMC5130 *driver = nullptr;
	TMC5130MotionControlSettings_t getSettings(void);
	void setSettings(TMC5130MotionControlSettings_t settings);
};

#endif