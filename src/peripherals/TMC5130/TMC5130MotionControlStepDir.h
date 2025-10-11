#ifndef __TMC5130MOTIONCONTROLSTEPDIR_H
#define __TMC5130MOTIONCONTROLSTEPDIR_H

#include "../../UstepperS32.h"
#include "ITMC5130MotionControl.h"

class TMC5130MotionControlStepDir : public ITMC5130MotionControl
{
public:
	TMC5130MotionControlStepDir();
	void setRPM(float RPM) override;
	void init(TMC5130 *driver) override;
	void deInit() override;
	void setPosition(int32_t position) override;
	void setVelocity(uint32_t velocity) override;
	void setAcceleration(uint32_t acceleration) override;
	void setDeceleration(uint32_t deceleration) override;
	void setShaftDirection(bool direction) override;
	void stop(void) override;
	int32_t getVelocity(void) override;
	int32_t getPosition(void) override;
	void setHome(int32_t initialSteps = 0) override;
	void setDirection(bool direction) override;
	void setRampMode(uint8_t mode) override;
	
private:
	GPIO stepPin;
	GPIO dirPin;
	friend class TMC5130;
};

#endif