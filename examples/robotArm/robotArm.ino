#include "robotArmControl.h"

robotArmControl arm;

void setup()
{
	delay(5000);
	arm.begin(SHOULDER);
}

void loop()
{
	arm.run();
}
