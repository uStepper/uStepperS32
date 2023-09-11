#include "../UstepperS32.h"
void dropInStepInputEXTI()
{
	ptr->driver.stepPin.toggle();
}
void dropInDirInputEXTI()
{
	if (ptr->dropin.dirPin.read())
	{
		ptr->driver.dirPin.set();
	}
	else
	{
		ptr->driver.dirPin.reset();
	}
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