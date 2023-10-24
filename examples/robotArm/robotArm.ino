#include <robotArmControl.h>

robotArmControl arm;

void setup()
{
	arm.begin(BASE);
}

void loop()
{
	arm.run();
}