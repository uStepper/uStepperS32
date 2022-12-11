#include "../UstepperS32.h"
void mainTimerCallback()
{
	digitalToggle(D0);
	ptr->encoder.sample();
	digitalToggle(D0);
}