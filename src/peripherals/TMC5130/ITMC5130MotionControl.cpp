#include "ITMC5130MotionControl.h"
#include "TMC5130.h"
#include <stdint.h>

int32_t ITMC5130MotionControl::writeRegister(uint8_t address, uint32_t datagram){
    if(!driver) return 0;
    return driver->writeRegister(address, datagram);
}

int32_t ITMC5130MotionControl::readRegister(uint8_t address){
    if(!driver) return 0;
    return driver->readRegister(address);
}