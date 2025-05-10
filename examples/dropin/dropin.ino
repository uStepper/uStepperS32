#include <UstepperS32.h>

UstepperS32 stepper;

void setup()
{
	// put your setup code here, to run once:
	stepper.setup(DROPIN);				  //Initialisation of the uStepper S32
	stepper.dropin.setProportional(10.0);
	stepper.dropin.setIntegral(0.0);
	stepper.dropin.setDifferential(0.0);
	/*pinMode(24, OUTPUT);
	pinMode(25, OUTPUT);
	pinMode(26, OUTPUT);
	pinMode(27, OUTPUT);
	pinMode(28, OUTPUT);
	digitalWrite(26, HIGH);
	digitalWrite(27, LOW);
	digitalWrite(28, HIGH);
	digitalWrite(25, HIGH);
	digitalWrite(24, LOW);
	digitalWrite(26, LOW);*/
}

void loop()
{
	stepper.dropin.cli();
	/*digitalWrite(24, HIGH);
	delay(1);
	digitalWrite(24, LOW);
	delay(1);*/
	
}

