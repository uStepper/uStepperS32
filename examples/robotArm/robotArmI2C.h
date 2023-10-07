#ifndef _ROBOTARMI2C_H_
#define _ROBOTARMI2C_H_

#include "robotArmConfig.h"

class robotArmI2C {

public:
  uint8_t addressNum = 0;

  robotArmI2C();

  void begin(uint8_t device);

  void setAngle(uint8_t slave, float angle);
  void writeCommand(uint8_t slave, char command, float value);

  float requestAngle(uint8_t slave);
  state_t requestState(uint8_t slave);
  uint8_t requestMotorState(uint8_t slave);
  void setSpeed(uint8_t slave, float speed);
  void setAcceleration(uint8_t slave, float acceleration);
  void setBrakeMode(uint8_t slave, uint8_t brakeMode);
  void runContinously(uint8_t slave, float speed);
  void stopSlave(uint8_t slave);
  void setHomingSpeed(uint8_t slave, float speed);
  void setStallSense(uint8_t slave, float sense);

private:
  // I2C addresses used
  uint8_t address[3] = {8, 9, 10};

  void writeFloat(float value);
};

#endif
