#include "TMC5130MotionControlStepDir.h"

TMC5130MotionControlStepDir::TMC5130MotionControlStepDir() : 
                                                            stepPin(LL_GPIO_PIN_9, 9, GPIOA),
                                                            dirPin(LL_GPIO_PIN_10, 10, GPIOA)
{
	
}

void TMC5130MotionControlStepDir::setRPM(float RPM)
{
	
}

void TMC5130MotionControlStepDir::init(TMC5130 *driver){
    this->driver = driver;
    this->stepPin.configureOutput();
    this->dirPin.configureOutput();
    this->stepPin.reset();
    this->dirPin.reset();
}

void TMC5130MotionControlStepDir::deInit(){
    this->stepPin.configureInput();
    this->dirPin.configureInput();
}

void TMC5130MotionControlStepDir::setPosition(int32_t position){

}
void TMC5130MotionControlStepDir::setVelocity(uint32_t velocity){

}
void TMC5130MotionControlStepDir::setAcceleration(uint32_t acceleration){

}
void TMC5130MotionControlStepDir::setDeceleration(uint32_t deceleration){

}
void TMC5130MotionControlStepDir::setShaftDirection(bool direction){

}
void TMC5130MotionControlStepDir::stop(void){

}
int32_t TMC5130MotionControlStepDir::getVelocity(void){
    return 0; // Not yet implemented
}
int32_t TMC5130MotionControlStepDir::getPosition(void){
    return 0; // Not yet implemented
}
void TMC5130MotionControlStepDir::setHome(int32_t initialSteps){

}

void TMC5130MotionControlStepDir::setDirection(bool direction)
{
}

void TMC5130MotionControlStepDir::setRampMode(uint8_t mode)
{
}