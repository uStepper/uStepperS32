#include <UstepperSTM.h>

UstepperSTM stepper;
uint8_t pin = D21;
volatile uint16_t analog0 = 0;
volatile uint16_t analog1 = 0;
volatile uint16_t analog2 = 0;
volatile uint16_t analog3 = 0;
volatile uint16_t analog4 = 0;
volatile uint16_t analog5 = 0;
volatile uint16_t analog6 = 0;
volatile uint16_t analog7 = 0;
volatile uint16_t analog8 = 0;
volatile uint16_t analog9 = 0;
volatile uint16_t analog10 = 0;
volatile uint16_t analog11 = 0;
volatile uint16_t analog12 = 0;
volatile uint16_t analog13 = 0;
void setup()
{
	stepper.init();
	Serial.begin(115200);
	//pinMode(pin, OUTPUT);
	//stepper.driver.setRPM(0);
}

void loop()
{
	//analog0 = analogRead(A0);
	//analog1 = analogRead(A1);
	analog2 = analogRead(A2);
	analog3 = analogRead(A3);
	analog4 = analogRead(A4);
	analog5 = analogRead(A5);
	analog6 = analogRead(A6);
	analog7 = analogRead(A7);
	analog8 = analogRead(A8);
	analog9 = analogRead(A9);
	analog10 = analogRead(A10);
	analog11 = analogRead(A11);
	analog12 = analogRead(A12);
	analog13 = analogRead(A13);
	
	delay(1000);
	//stepper.encoder.getAngle();
	//stepper.driver.writeRegister(XACTUAL, 23);
	//delay(100);
	//stepper.driver.readRegister(XACTUAL);
	//stepper.driver.setRPM(50);
}
