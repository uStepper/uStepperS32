#include "../UstepperS32.h"
void mainTimerCallback()
{
	if (!ptr->encoder.semaphore.isLocked())
	{
		ptr->encoder.sample();
		if (ptr->mode == CLOSEDLOOP)
		{
			callbacks._closedLoopCallback();
		}
	}
}