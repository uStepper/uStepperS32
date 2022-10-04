# 1 "C:\\Users\\Thomas\\Documents\\Arduino\\libraries\\uStepperSTM\\Examples\\readAngle\\readAngle.ino"
# 2 "C:\\Users\\Thomas\\Documents\\Arduino\\libraries\\uStepperSTM\\Examples\\readAngle\\readAngle.ino" 2

UstepperSTM stepper;

void setup()
{
 stepper.init();
 //stepper.driver.setRPM(50);
}

void loop()
{
 delay(1000);
 stepper.encoder.getAngle();
 //stepper.driver.writeRegister(XACTUAL, 23);
 //delay(100);
 //stepper.driver.readRegister(CHOPCONF);
}
