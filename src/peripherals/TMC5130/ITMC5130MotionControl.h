#ifndef __TMC5130MOTIONCONTROL_H
#define __TMC5130MOTIONCONTROL_H

// Intentionally avoid including full driver header here to prevent circular include.
#include <stdint.h>
// Only a forward declaration is needed for pointer members and parameters.
class TMC5130;

typedef enum TMC5130MotionControllers_e
{
	stepDir = 0,
	internalRamp
};

typedef enum TMC5130MotionControlMode_e
{
	DRIVER_STOP = 0,
	DRIVER_POSITION,
	DRIVER_VELOCITY,
	DRIVER_STEPPER
};

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

class ITMC5130MotionControl
{
public:
	virtual ~ITMC5130MotionControl() {}
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
	virtual void setDirection(bool direction) = 0; // Select motion (velocity/ramp) direction
	virtual void setRampMode(uint8_t mode) = 0;    // Configure ramp generator mode

protected:
	TMC5130MotionControlSettings_t settings;
	TMC5130 *driver = nullptr;
	// Provide derived classes access to previous settings when switching controllers
	inline TMC5130MotionControlSettings_t getSettings() { return settings; }
	inline void setSettings(const TMC5130MotionControlSettings_t &s) { settings = s; }
	// Helper wrappers for driver register access (only valid after init)
	int32_t writeRegister(uint8_t address, uint32_t datagram);
	int32_t readRegister(uint8_t address);
	friend class TMC5130;
};

#endif