#include "robotArmI2C.h"

robotArmI2C::robotArmI2C(void) {}

void robotArmI2C::begin(uint8_t device) {

  // Master-slave determination
  addressNum = device;
  I2CPORT.setSDA(PB9);
  I2CPORT.setSCL(PB8);
  /*
  uint16_t addressVal = analogRead(ADDRESS_PIN);
  
  if (addressVal < 250) // Less than 400 = Slave 1
    addressNum = SHOULDER;
  else if (addressVal > 450) // More than 600 = Slave 2
    addressNum = ELBOW;
  else // Else master
    addressNum = BASE;
    */
  // Initiation of I2C object at address
  I2CPORT.begin(address[addressNum]);

  if (addressNum == BASE)
    DEBUG_PRINTLN("-- I2C Master --");
  else
    DEBUG_PRINTLN("-- I2C Slave --");

  DEBUG_PRINT("Address: ");
  DEBUG_PRINTLN(address[addressNum]);
}

void robotArmI2C::writeCommand(uint8_t slave, char command, float value) {

  binaryFloat temp;
  temp.f = value;

  I2CPORT.beginTransmission(address[slave]);

  I2CPORT.write(command);
  I2CPORT.write(temp.b, 4);

  I2CPORT.endTransmission();
}

// Master function to set slave angles
void robotArmI2C::setAngle(uint8_t slave, float angle) {
  writeCommand(slave, 'a', angle);
}

// Master function to set slave speed
void robotArmI2C::setSpeed(uint8_t slave, float speed) {
  writeCommand(slave, 's', speed);
}

// Master function to set slave acceleration
void robotArmI2C::setAcceleration(uint8_t slave, float acceleration) {
  writeCommand(slave, 'i', acceleration);
}

// Master function to set slave brake mode
void robotArmI2C::setBrakeMode(uint8_t slave, uint8_t brakeMode) {
  writeCommand(slave, 'b', (float)brakeMode);
}

// Master function to make slave run Continously
void robotArmI2C::runContinously(uint8_t slave, float speed) {
  writeCommand(slave, 'S', speed);
}

// Master function to change slave homing speed
void robotArmI2C::setHomingSpeed(uint8_t slave, float speed) {
  writeCommand(slave, 'H', speed);
}

// Master function to change slave stall sense
void robotArmI2C::setStallSense(uint8_t slave, float sense) {
  writeCommand(slave, 'f', sense);
}

// Master function to stop slave
void robotArmI2C::stopSlave(uint8_t slave) { writeCommand(slave, 'B', 0.0); }

// Master function to request slave angle
float robotArmI2C::requestAngle(uint8_t slave) {
  binaryFloat value;
  int i = 0;
  char c;

  I2CPORT.requestFrom(address[slave], 4);

  while (I2CPORT.available()) {
    if (i < 4) {
      c = I2CPORT.read();
      value.b[i++] = c;
    }
  }

  return value.f;
}

// Master function to request slave state
state_t robotArmI2C::requestState(uint8_t slave) {
  state_t temp;
  int i = 0;

  writeCommand(slave, 'q', 1.0);

  I2CPORT.requestFrom(address[slave], 1);

  temp = (state_t)I2CPORT.read();

  return temp;
}

// Master function to request slave motor state
uint8_t robotArmI2C::requestMotorState(uint8_t slave) {
  uint8_t temp;
  int i = 0;

  writeCommand(slave, 'q', 2.0);

  I2CPORT.requestFrom(address[slave], 1);

  temp = (uint8_t)I2CPORT.read();

  return temp;
}
