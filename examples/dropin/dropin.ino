#include <UstepperS32.h>

UstepperS32 stepper;
float angle = 360.0; //amount of degrees to move

void setup()
{
	// put your setup code here, to run once:
	stepper.setup(DROPIN);				  //Initialisation of the uStepper S
}

void loop()
{/*
	stepper.driver.stepPin.set();
	delayMicroseconds(10000);
	stepper.driver.stepPin.reset();
	delayMicroseconds(10000);
*/
}

