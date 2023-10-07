/********************************************************************************************
* 	    	File:  limitDetection.ino                                                         *
*		Version:    2.3.0                                          						    *
*      	Date: 		October 7th, 2023  	                                    			*
*       Author:  Thomas Hørring Olsen                                                       *
*  Description:  Limit Detection Example Sketch!                                            *
*                This example demonstrates how the library can be used to detect hard       *
*                limits in an application, without the use of mechanical endstop switches.  *
*                Stallguard is very sensitive and provides seamless stall detection when 	  *
*				         tuned for the application. It is dependent on speed, current setting		    *
*				         and load conditions amongst others. The encoder stall detection is 		    *
*				         unaffected by most of these but can be a bit less sensitive.				        *
*                                                                                           *
* For more information, check out the documentation:                                        *
*                       http://ustepper.com/docs/usteppers/html/index.html                  *
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
UstepperS32 stepper;

#define STEPSPERREV 200 //Number of steps pr revolution. 200 for a 1.8deg motor, 400 for a 0.9deg motor
#define RES (STEPSPERREV *256)/360.0//calculate microstep pr. degree
#define STEPPRMM 53.55//full step pr. mm for the rail used in the demo
#define MMPRSTEP 1/(STEPPRMM*256)//mm pr. microstep
#define MMPRDEG MMPRSTEP*RES//mm pr. degree
#define STALLSENSITIVITY 2//sensitivity of the stall detection, between -64 and 63 - higher number is less sensitive

// Desired rpm for homing
int16_t rpm = 50;

void setup(){
  stepper.setup(NORMAL, STEPSPERREV); 	//Initialize uStepper S32
  stepper.checkOrientation(15.0);       //Check orientation of motor connector with +/- 15 microsteps movement
  stepper.setMaxAcceleration(2000);		//Use an acceleration of 2000 fullsteps/s^2
  stepper.setMaxVelocity(500);			//Max velocity of 500 fullsteps/s
  Serial.begin(9600);
}

void loop() {
  float railLength;

  stepper.moveToEnd(CW, rpm, STALLSENSITIVITY);      //Reset to CW endpoint
  Serial.println(railLength*MMPRDEG);//find end positions and read out the recorded end position
  railLength = stepper.moveToEnd(CCW, rpm, STALLSENSITIVITY);    //Go to CCW end
  Serial.println(railLength*MMPRDEG);//find end positions and read out the recorded end position
  
  while(1);
}
