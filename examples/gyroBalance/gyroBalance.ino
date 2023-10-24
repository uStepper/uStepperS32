/********************************************************************************************
* 	    	File:  gyroBalance.ino                                                            *
*		      Version:    2.3.0                                          						            *
*      	  Date: 	 October 7th, 2023  	                                    			          *
*         Author:  Mogens Groth Nicolaisen                                                        *
*         Description:  Gyro Example Sketch!                                                *
*                Requires uStepper Gyro Shield !                                            *
*                This example demonstrates the use of a gyro/accelerometer for feedback to  *
*                the uStepper.                                                              * 
*                                                                                           *
*                The MPU6050 src files are derivatives from the TinyMPU6050 library         *
*                 - licensing info is found in "LICENSE_TinyMPU6050"                        *
*                                                                                           *
* For more information, check out the documentation:                                        *
*    http://ustepper.com/docs/usteppers/html/index.html                                     *
*                                                                                           *
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
#include "TinyMPU6050.h"

MPU6050 mpu;

void setup(){
  stepper.setup(CLOSEDLOOP,200);   //Initialize uStepper S32 to use closed loop control with 200 steps per revolution motor - i.e. 1.8 deg stepper
  stepper.checkOrientation(30.0);  //Check orientation of motor connector with +/- 30 microsteps movement
  stepper.setMaxAcceleration(500); //Use an acceleration of 500 fullsteps/s^2
  stepper.setMaxVelocity(500);     //Max velocity of 500 fullsteps/s
  mpu.Initialize();				   // Initialization of MPU
  mpu.Calibrate();				   // Calibration - LED will blink until calibration is done !
  stepper.encoder.setHome(-mpu.GetAngZ());
}

void loop(){
  mpu.Execute();
  stepper.moveToAngle(-mpu.GetAngZ());
}