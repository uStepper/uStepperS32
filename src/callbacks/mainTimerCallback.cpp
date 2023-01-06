#include "../UstepperS32.h"
void mainTimerCallback()
{
	digitalToggle(D0);
	ptr->encoder.sample();
	if (ptr->mode == CLOSEDLOOP)
	{
		callbacks._closedLoopCallback();
	}
	digitalToggle(D0);
}