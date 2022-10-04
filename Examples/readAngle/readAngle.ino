#include <UstepperSTM.h>

UstepperSTM stepper;

void setup()
{
	stepper.init();
	//stepper.driver.setRPM(50);
}

void loop()
{
	delay(1000);
	stepper.encoder.getAngle();
	//stepper.driver.writeRegister(XACTUAL, 23);
	//delay(100);
	//stepper.driver.readRegister(CHOPCONF);
}
