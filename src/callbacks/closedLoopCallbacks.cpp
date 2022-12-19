#include "../UstepperS32.h"
void closedLoopCallback()
{
	int32_t stepsMoved = ptr->driver.getPosition();
	int32_t stepsMeasured = ENCODERRAWTOSTEP(ptr->encoder.getAngleMovedRaw());
	
	if (!ptr->pidDisabled)
	{
		ptr->currentPidError = stepsMoved - stepsMeasured;
		if (abs(ptr->currentPidError) >= ptr->controlThreshold)
		{
			ptr->driver.writeRegister(XACTUAL, stepsMeasured);
			ptr->driver.writeRegister(XTARGET, ptr->driver.xTarget);
		}

		//ptr->currentPidSpeed = ptr->encoder.encoderFilter.velIntegrator * ENCODERDATATOSTEP;
	}
}