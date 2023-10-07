/********************************************************************************************
* 	    	File:  velocityTest.ino                                                           *
*		Version:    2.3.0                                          						    *
*      	Date: 		October 7th, 2023  	                                    			*
*       Author:  Hans Henrik Skovgaard                                                      *
*  Description:  By entering the commands f, m and s the RPM of the stepper can be changed  *
*                 on the fly.                                                               *
*  Commands...:  f: fast, set rpm = 1500,                                                   *
*                m: medium, set rpm = 1000,                                                 *
*                s: slow, set rpm = 500,                                                    *
*                h: halt                                                                    *
*                                                                                           *
*  Note.......:  Place the stepper motor on a solid surface where it cannot harm anyone     *
*                during the test.                                                           *
*                                                                                           *
*********************************************************************************************
*	(C) 2023                                                                                  *
*                                                                                           *
*	uStepper ApS                                                                              *
*	www.ustepper.com                                                                          *
*	administration@ustepper.com                                                               *
*                                                                                           *
*	The code contained in this file is released under the following open source license:      *
*                                                                                           *
*			Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International               *
*                                                                                           *
* 	The code in this file is provided without warranty of any kind - use at own risk!       *
* 	neither uStepper ApS nor the author, can be held responsible for any damage             *
* 	caused by the use of the code contained in this file !                                  *
*                                                                                           *
*                                                                                           *
********************************************************************************************/
#include <UstepperS32.h>

#define MAXACCELERATION 1500  //Max acceleration = 1500 Steps/s^2
#define MAXVELOCITY 1500      //Max velocity = 1500 steps/s

UstepperS32 stepper(MAXACCELERATION, MAXVELOCITY);

void setup(){
  stepper.setup();					//Initialize uStepper S32
  stepper.checkOrientation(15.0);	//Check orientation of motor connector with +/- 15 microsteps movement
  Serial.begin(9600);
  delay(1000);						//Give it a second for the UART to be ready
  Serial.println("Speed test.\n Commands: f:fast, m:medium, s:slow, h:halt");  
  stepper.runContinous(CCW);		//Start running Counter Clockwise
}

void loop() 
{
  char cmd;
  while(!Serial.available());
  Serial.println("ACK!");
  cmd = Serial.read();
  if( (cmd == 'f') || (cmd == 'F') ) //Run fast
  {
    stepper.setMaxVelocity(1500);
    stepper.runContinous(CCW);
  }
  else if( (cmd == 'm') || (cmd == 'M') ) //Run medium
  {
    stepper.setMaxVelocity(1000);
    stepper.runContinous(CCW);
  }
  else if( (cmd == 's') || (cmd == 'S') ) // Run slow
  {
    stepper.setMaxVelocity(500);
    stepper.runContinous(CCW);
  }
  else if( (cmd == 'h') || (cmd == 'H') ) // Halt
  {
    stepper.stop( );
  }
}
