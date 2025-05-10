#include "../../UstepperS32.h"

TMC5130MotionControlSettings_t ITMC5130MotionControl::getSettings(void)
{
    return this->settings;
}

void ITMC5130MotionControl::setSettings(TMC5130MotionControlSettings_t settings)
{
    this->settings = settings;
}