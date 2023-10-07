/********************************************************************************************
* 	    File:  bounce.ino                                                    	        	*
*		Version: 2.3.0                                          						    *
*      	Date: 	 October 7th, 2023  	                                    				*
*       Author:  Mogens Groth Nicolaisen                                                       	*
*  Description:  Example sketch for exercising the stepper motor while printing out encoder *
*				 data on the serial monitor											        *
*                                                                                           *
*                                                                                           *
* For more information, check out the documentation:                                        *
*    http://ustepper.com/docs/usteppers/html/index.html                                     *
*                                                                                           *
*                                                                                           *
*********************************************************************************************
*	(C) 2023                                                                                *
*                                                                                           *
*	uStepper ApS                                                                            *
*	www.ustepper.com                                                                        *
*	administration@ustepper.com                                                             *
*                                                                                           *
*	The code contained in this file is released under the following open source license:    *
*                                                                                           *
*			Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International         *
*                                                                                           *
* 	The code in this file is provided without warranty of any kind - use at own risk!       *
* 	neither uStepper ApS nor the author, can be held responsible for any damage             *
* 	caused by the use of the code contained in this file !                                  *
*                                                                                           *
*                                                                                           *
********************************************************************************************/
#include <UstepperS32.h>

UstepperS32 stepper;
float angle = 360.0; //amount of degrees to move

void setup(){
	stepper.setup();				  //Initialize uStepper S32
	stepper.checkOrientation(30.0);   //Check orientation of motor connector with +/- 30 microsteps movement
	stepper.setMaxAcceleration(2000); //use an acceleration of 2000 fullsteps/s^2
	stepper.setMaxVelocity(500);	  //Max velocity of 500 fullsteps/s
	Serial.begin(9600);
}

void loop()
{
	if (!stepper.getMotorState()) //If motor is at standstill
	{
		delay(1000);
		stepper.moveAngle(angle); //start new movement
		angle = -angle;			  //invert angle variable, so the next move is in opposite direction
	}
	Serial.print("Angle: ");
	float angle = stepper.encoder.getAngleMoved();
	Serial.print(angle); //print out angle moved since last reset
	Serial.println(" Degrees");
}