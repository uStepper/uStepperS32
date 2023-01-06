#include "../UstepperS32.h"
void mainTimerCallback()
{
	digitalToggle(D0);
	ptr->encoder.sample();
	//ptr->encoder.angleMoved = 2000;
	if (ptr->mode == CLOSEDLOOP)
	{
		callbacks._closedLoopCallback();
	}
	digitalToggle(D0);
}