#include "TMC5130MotionControlInternalRamp.h"
#include "../../UstepperS32.h"

TMC5130MotionControlInternalRamp::TMC5130MotionControlInternalRamp()
{
	
}

void TMC5130MotionControlInternalRamp::setRPM(float RPM)
{
	if(!driver) return;
	// convert RPM using accessor to rpm->velocity factor
	int32_t velocityDir = 0;
	if(ptr){
		velocityDir = (int32_t)(ptr->getRpmToVelocityFactor() * RPM);
	}

	if (velocityDir > 0)
	{
		this->setDirection(1);
	}
	else
	{
		this->setDirection(0);
	}

	// The velocity cannot be signed
	uint32_t velocity = abs(velocityDir);

	this->setVelocity((uint32_t)velocity);
}

void TMC5130MotionControlInternalRamp::init(TMC5130 *driver){
	this->driver = driver;
}

void TMC5130MotionControlInternalRamp::deInit(){

}

void TMC5130MotionControlInternalRamp::setPosition(int32_t position){
    this->settings.mode = DRIVER_POSITION;
	this->setRampMode(POSITIONING_MODE);
	this->writeRegister(XTARGET, position);
	this->settings.xTarget = position;
}
void TMC5130MotionControlInternalRamp::setVelocity(uint32_t velocity){
    this->settings.VMAX = velocity;

	if (this->settings.VMAX > 0x7FFE00)
	{
		this->settings.VMAX = 0x7FFE00;
	}

	this->writeRegister(VMAX_REG, this->settings.VMAX);
}
void TMC5130MotionControlInternalRamp::setAcceleration(uint32_t acceleration){
    this->settings.AMAX = acceleration;

	if (this->settings.AMAX > 0xFFFE)
	{
		this->settings.AMAX = 0xFFFE;
	}

	this->writeRegister(AMAX_REG, this->settings.AMAX);
}
void TMC5130MotionControlInternalRamp::setDeceleration(uint32_t deceleration){
    this->settings.DMAX = deceleration;

	if (this->settings.DMAX > 0xFFFE)
	{
		this->settings.DMAX = 0xFFFE;
	}

	this->writeRegister(DMAX_REG, this->settings.DMAX);
}

void TMC5130MotionControlInternalRamp::stop(void){
    this->settings.mode = DRIVER_STOP;
	this->setVelocity(0);
}
int32_t TMC5130MotionControlInternalRamp::getVelocity(void){
    int32_t value = this->readRegister(VACTUAL);

	// VACTUAL is 24bit two's compliment
	if (value & 0x00800000)
		value |= 0xFF000000;

	return (value);
}
int32_t TMC5130MotionControlInternalRamp::getPosition(void){
    return this->readRegister(XACTUAL);
}
void TMC5130MotionControlInternalRamp::setHome(int32_t initialSteps){
    int32_t xActual, xTarget;

	if (this->settings.mode == DRIVER_POSITION)
	{
		xActual = this->getPosition();
		xTarget = this->readRegister(XTARGET);

		xTarget -= xActual;
		this->settings.xTarget = xTarget + initialSteps;
		this->settings.xActual = initialSteps;
		this->writeRegister(XACTUAL, initialSteps);
		this->writeRegister(XTARGET, this->settings.xTarget);
	}
	else
	{
		this->settings.xTarget = initialSteps;
		this->settings.xActual = initialSteps;
		this->writeRegister(XACTUAL, initialSteps);
		this->writeRegister(XTARGET, initialSteps);
	}
}

void TMC5130MotionControlInternalRamp::setDirection(bool direction)
{
    this->settings.mode = DRIVER_VELOCITY;
    if (direction == 1)
    {
        this->writeRegister(RAMPMODE, VELOCITY_MODE_POS);
    }
    else
    {
        this->writeRegister(RAMPMODE, VELOCITY_MODE_NEG);
    }
}

void TMC5130MotionControlInternalRamp::setRampMode(uint8_t mode)
{
    switch (mode)
	{
	case POSITIONING_MODE:
		// Positioning mode
		this->writeRegister(VSTART_REG, this->settings.VSTART);
		this->writeRegister(A1_REG, this->settings.A1);
		this->writeRegister(V1_REG, this->settings.V1);
		this->writeRegister(AMAX_REG, this->settings.AMAX);
		this->writeRegister(VMAX_REG, this->settings.VMAX);
		this->writeRegister(DMAX_REG, this->settings.DMAX);
		this->writeRegister(D1_REG, this->settings.D1);
		this->writeRegister(VSTOP_REG, this->settings.VSTOP < 10 ? 10 : this->settings.VSTOP);	 /* Minimum 10 in POSITIONING_MODE */
		this->writeRegister(RAMPMODE, POSITIONING_MODE); /* RAMPMODE = POSITIONING_MODE */
		break;

	case VELOCITY_MODE_POS:
		// Velocity mode (only AMAX and VMAX is used)
		this->writeRegister(VSTART_REG, this->settings.VSTART);
		this->writeRegister(A1_REG, 0);
		this->writeRegister(V1_REG, 0);
		this->writeRegister(AMAX_REG, this->settings.AMAX);
		this->writeRegister(VMAX_REG, this->settings.VMAX);
		this->writeRegister(DMAX_REG, 0);
		this->writeRegister(D1_REG, 0);
		this->writeRegister(VSTOP_REG, 0);
		this->writeRegister(RAMPMODE, VELOCITY_MODE_POS); /* RAMPMODE = VELOCITY_MODE_POS */
		break;
	}
}

void TMC5130MotionControlInternalRamp::setShaftDirection(bool direction)
{
	this->settings.invertShaftDirection = direction;
	if(!driver) return;
	// Read the register to save the settings via driver
	int32_t value = driver->readRegister(GCONF);
	if (direction == 1)
	{
		value |= (0x01 << 4);
	}
	else
	{
		value &= ~(0x01 << 4);
	}
	driver->writeRegister(GCONF, value);
}