#include "../UstepperS32.h"
void mainTimerCallback()
{
	digitalToggle(D0);
	if (!ptr->encoder.semaphore.isLocked())
	{
		ptr->encoder.sample();
		if (ptr->mode == CLOSEDLOOP)
		{
			callbacks._closedLoopCallback();
		}
	}
	
	
	digitalToggle(D0);
}