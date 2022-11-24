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
	//stepper.driver.setRPM(0);
}

void loop()
{
	Serial.println("Hejsa");
	
	delay(1000);
	//stepper.encoder.getAngle();
	//stepper.driver.writeRegister(XACTUAL, 23);
	//delay(100);
	//stepper.driver.readRegister(XACTUAL);
	//stepper.driver.setRPM(50);
}
