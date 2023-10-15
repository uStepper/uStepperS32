#include <UstepperS32.h>

UstepperS32 stepper;

void setup()
{
	// put your setup code here, to run once:
	stepper.setup(DROPIN);				  //Initialisation of the uStepper S
}

void loop()
{
	stepper.dropin.cli();
}

