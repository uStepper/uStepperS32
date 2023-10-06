#include "../UstepperS32.h"
void dropInStepInputEXTI()
{
	if (ptr->dropin.stepPin.read())
	{
		ptr->driver.stepPin.set();
	}
	else
	{
		ptr->driver.stepPin.reset();
	}
	
}
void dropInDirInputEXTI()
{
}
void dropInEnableInputEXTI()
{
	if (ptr->dropin.enaPin.read())
	{
		ptr->driver.enablePin.set();
	}
	else
	{
		ptr->driver.enablePin.reset();
	}
}