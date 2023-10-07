#ifndef _ROBOTARMCONTROL_H_
#define _ROBOTARMCONTROL_H_

#include "robotArmComm.h"
#include "robotArmConfig.h"
#include "robotArmI2C.h"
#include <UstepperS32.h>

#define FEEDRATETOANGULARFEEDRATE(x) x * 10.0
#define HOMEFEEDRATEFAST 20.0f
#define HOMEFEEDRATESLOW 5.0f

class robotArmControl {

public:
  robotArmControl();

  void begin(uint8_t device);
  void masterLoop();
  void slaveLoop();
  /* Main control function, listening for gcode and reading commands when robot
   * is ready */
  void run(void);

  void execute(char *command);

  void setMotorAngle(uint8_t num, float angle);

  void setMotorSpeed(uint8_t num, float speed);

  void setMotorAcceleration(uint8_t num, float acceleration);

  void setMotorHomingSpeed(uint8_t num, float speed);

  void setMotorStallSense(uint8_t num, float sense);

  void runContinously(uint8_t num, float speed);

  void setXYZ();

  void xyzToAngles(float &rot, float &left, float &right, float x_target,
                   float y_target, float z_target);

  void angleToxyz(float rot, float left, float right, float &x_actual,
                  float &y_actual, float &z_actual);

  void homeArm(void);

  uint8_t checkLimits(bool correction = false);

  void setPump(bool state);

  void sendXYZ(void);
  void setbrakeMode(uint8_t num, float brakeMode);
  float direction = 1.0;
  void checkConnOrientation(void);

private:
  // Object for manipulating i2c slaves
  robotArmI2C bus;

  // Object for receiving and sending gcode
  robotArmComm comm;

  // I2C slave function when receiving a command
  static void busReceiveEvent(int numBytes);

  // I2C slave function for sending current angle position */
  static void busRequestEvent(void);

  uint8_t calcVelocityProfile(float baseTarget, float elbowTarget, float shoulderTarget, bool correction = false);

  void calcVelocityProfileMovement(void);

  bool inRange( float value, float target, float limit );

  float setServo(float servoVal);

  void setServo();

  bool isRecording = false;
  bool targetReached = true;

  int8_t baseTargetReached = 1;
  int8_t elbowTargetReached = 1;
  int8_t shoulderTargetReached = 1;

  bool movementInProgress = 0;

  // Current angles
  float angleBase = 0.0;
  float angleShoulder = 0.0;
  float angleElbow = 0.0;

  // Target angles
  float angleTargetBase = 0.0;
  float angleTargetShoulder = 0.0;
  float angleTargetElbow = 0.0;

  // Current position
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;

  // Target position
  float tx = 145.0;
  float ty = 0.0;
  float tz = 145.0;

  // Target speed
  float sx = 0.0;
  float sy = 0.0;
  float sz = 0.0;

    // Target angular speed
  float targetBaseSpeed = 0.0;
  float targetElbowSpeed = 0.0;
  float targetShoulderSpeed = 0.0;

  bool targetPumpState = 0;
  float targetServo = 0.0;
  float currentServo = 0.0;
  float filteredServo = 0.0;
  bool currentPumpState = 0;
  // Feedrate in mm/s
  float feedrate = 10.0;

  // Used for timeout purposes
  uint32_t lastCommand = 0;

  bool valveOn = 0;
  int32_t valveOnTime;

  int8_t stallSense;
  float homeSpeed;
};

#endif
