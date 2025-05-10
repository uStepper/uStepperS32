#ifndef __TMC5130MOTIONCONTROL_H
#define __TMC5130MOTIONCONTROL_H

#include "../../UstepperS32.h"

class ITMC5130MotionControl
{
public:
	virtual void setRPM(float RPM) = 0;
private:

};

#endif