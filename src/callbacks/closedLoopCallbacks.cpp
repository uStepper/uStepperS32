#include "../UstepperS32.h"
void closedLoopCallback()
{
	if (!ptr->driver.semaphore.isLocked()){
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
		}
	}
}