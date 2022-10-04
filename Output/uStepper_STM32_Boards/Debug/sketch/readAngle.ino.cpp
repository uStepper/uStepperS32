#include <Arduino.h>
#line 1 "C:\\Users\\Thomas\\Documents\\Arduino\\libraries\\uStepperSTM\\Examples\\readAngle\\readAngle.ino"
#include <UstepperSTM.h>

UstepperSTM stepper;

#line 5 "C:\\Users\\Thomas\\Documents\\Arduino\\libraries\\uStepperSTM\\Examples\\readAngle\\readAngle.ino"
void setup();
#line 11 "C:\\Users\\Thomas\\Documents\\Arduino\\libraries\\uStepperSTM\\Examples\\readAngle\\readAngle.ino"
void loop();
#line 5 "C:\\Users\\Thomas\\Documents\\Arduino\\libraries\\uStepperSTM\\Examples\\readAngle\\readAngle.ino"
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

