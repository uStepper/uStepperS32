/********************************************************************************************
* 	    	File:  rcStepperServoSpeed.ino                                                    *
*		   Version:  1.0.0                                                                      *
*         Date:  October 7th, 2023                                                          *
*       Author:  Mogens Groth Nicolaisen                                                    *
*  Description:  Example sketch for control of stepper using an RC radio!                   *         
*                This example demonstrates the use of uStepper S32 an RC servo             *
*                taking input from an RC receiver and running continuous at a given speed.  *
*                Set the desired max and min speed and acceleration.                        *
*                The example requires the user to give a low signal (hold the transmitter   * 
*                stick low for 1 second followed by moving it to the highest positon        *
*                for a second to calibrate the speed range.                                 *
*                                                                                           *
*  Pin connections:                                                                         *
* -----------------------------                                                             *
* | Controller | uStepper S32    |                                                          *
* |----------------------------|                                                            *
* | Signal   |    D3     |                                                                  *
* | GND      |    GND    |                                                                  *
* | 5V       |    5V     | <- This is only neded if you do not have a supply for your       *
*                             receiver - ONLY add one 5V connection to your receiver !!     *
* ------------------------------                                                            *
*                                                                                           *
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

#define MINSPD -200
#define MAXSPD 200

UstepperS32 stepper;
uint16_t deltaTime = 0;
uint16_t time1 = 0;
uint16_t lowVal = 2000;// Ususally 1000us - Is set to 2000 for RC range calibarion procedure
uint16_t highVal = 0;// Usually 2000us - Is set to 0 for RC range calibration procedure
bool RCinit = 0;// If you do not wish to do RC calibration procedure, set the above two values to 1000 and 2000 respectively (or your own min/max servo pulse widths) and set RCinit = 1;
uint8_t k = 1;
int16_t velFilt;

void setup(void){
  stepper.setup();    						//Initialize uStepper S32
  stepper.checkOrientation(30.0);       	//Check orientation of motor connector with +/- 30 microsteps movement
  pinMode(3,INPUT_PULLUP);    				//Hardware interrupt input on D3 with pull-up
  attachInterrupt(3, interruptRC, CHANGE);  //Attach interrupt to the pin and react on both rising and falling edges
  pinMode(PC4,OUTPUT);						//Setting the LED up
  digitalWrite(PC4,LOW);  					//Setting the LED low
  stepper.setMaxAcceleration(50000);     	//Use an acceleration of 2000 fullsteps/s^2
}

void loop(void)
{
  while(!RCinit)// Calibrate max/min of RC input before doing anything else
  {
    initRC();
  }
  int16_t servoVel = map(deltaTime, lowVal, highVal, MINSPD, MAXSPD);
  velFilt = (velFilt<< 2)-velFilt;//  Filtering a bit on the signal to make it nice
  velFilt += servoVel;
  velFilt >>= 2;
  stepper.setRPM(velFilt);// Now move as requested
}

void initRC(void)// Find max and min limits of control input. Requires that the user moves the stick to the low point and keeps it there for a second and then move in the opposite direction and keep it there for a second
{
  if(deltaTime < 1200 && k<50)
  {
    if(deltaTime<lowVal && deltaTime>800)
    {
      lowVal = deltaTime;
      k=0;
    }
    k++;
    digitalWrite(PC4,LOW);
    delay(20);
  }
  else if(deltaTime > 1700 && k>49 && k<100)
  {
    if(deltaTime>highVal)
    {
      highVal = deltaTime;
      k=50;
    }
    k++;
    digitalWrite(PC4,LOW);
    delay(20);
  }
  else
  {
    digitalWrite(PC4,HIGH);
  }
  if(k==100 && deltaTime > ((highVal+lowVal)/2)-10 && deltaTime < ((highVal+lowVal)/2)+10)
  {
    RCinit = 1;
  }
}

void interruptRC(void)
{
  if(GPIOA->IDR & LL_GPIO_PIN_6)// Check if input is high
  {
    time1 = micros();
  }
  else//Now it is low - calc time
  {
    deltaTime = micros()-time1;
  }
}
