#include <UstepperSTM.h>

UstepperSTM stepper;
float angle = 360.0; //amount of degrees to move
void setup()
{
	// put your setup code here, to run once:
	stepper.setup();				  //Initialisation of the uStepper S
	stepper.setMaxAcceleration(2000); //use an acceleration of 2000 fullsteps/s^2
	stepper.setMaxVelocity(500);	  //Max velocity of 500 fullsteps/s
	//stepper.checkOrientation(30.0);   //Check orientation of motor connector with +/- 30 microsteps movement
	Serial.begin(9600);
	
}

void loop()
{
	// put your main code here, to run repeatedly:
	if (!stepper.getMotorState()) //If motor is at standstill
	{
		delay(1000);
		stepper.moveAngle(angle); //start new movement
		angle = -angle;			  //invert angle variable, so the next move is in opposite direction
	}
	Serial.print("Angle: ");
	Serial.print(stepper.encoder.getAngleMoved()); //print out angle moved since last reset
	Serial.println(" Degrees");
}