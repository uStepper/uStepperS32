#include <UstepperS32.h>

UstepperS32 stepper;

void setup()
{
	// put your setup code here, to run once:
	stepper.setup(DROPIN);				  //Initialisation of the uStepper S32
	stepper.dropin.setProportional(10.0);
	stepper.dropin.setIntegral(0.0);
	stepper.dropin.setDifferential(0.0);
}

void loop()
{
	stepper.dropin.cli();
}

