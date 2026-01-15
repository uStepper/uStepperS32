#include <UstepperS32.h>

UstepperS32 stepper;

void setup() {
    stepper.setup();
    stepper.checkOrientation(30.0);

    // Enable Modbus with slave ID 1 and baud rate 500000
    stepper.modbusEnable(1, 500000);
}

void loop() {
    // Handle all Modbus-related tasks
    stepper.handleModbus();
}