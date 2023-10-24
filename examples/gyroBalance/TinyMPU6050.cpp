/*
 *	Register map for the MPU6050 available at: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 */
#include "TinyMPU6050.h"

/*
 *	Constructor
 */
MPU6050::MPU6050(int i2cAddress) {

	// Setting I²C stuff
	if(i2cAddress == MPU6050_ADDRESS_LOW || i2cAddress == MPU6050_ADDRESS_HIGH){
		address = i2cAddress;
	} else{
		address = MPU6050_ADDRESS_LOW;
	}
	
}

/*
 *	Wrap an angle to the interval [-180, +180]
 *  --> Suggestion by edgar-bonet at https://github.com/gabriel-milan/TinyMPU6050/issues/6
 */
static float wrap(float angle)
{
	while (angle > +180) angle -= 360;
	while (angle < -180) angle += 360;
	return angle;
}

/*
 *	Compute the weighted average of a (with weight wa)
 *	and b (weight wb), treating both a and b as angles.
 *	It is assumed the sum of the weights is 1.
 */
static float angle_average(float wa, float a, float wb, float b)
{
	return wrap(wa * a + wb * (a + wrap(b-a)));
}

void MPU6050::Initialize () {

	// Beginning Wire
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
	Wire.begin();
	this->BaseInititalize();
}

void MPU6050::BaseInititalize () {
	// Setting attributes with default values
	filterAccelCoeff = DEFAULT_ACCEL_COEFF;
	filterGyroCoeff = DEFAULT_GYRO_COEFF;

	// Setting sample rate divider
	this->RegisterWrite(MPU6050_SMPLRT_DIV, 0x00);

	// Setting frame synchronization and the digital low-pass filter
	this->RegisterWrite(MPU6050_CONFIG, 0x00);

	// Setting gyro self-test and full scale range
	this->RegisterWrite(MPU6050_GYRO_CONFIG, 0x08);

	// Setting accelerometer self-test and full scale range
	this->RegisterWrite(MPU6050_ACCEL_CONFIG, 0x00);

	// Waking up MPU6050
	this->RegisterWrite(MPU6050_PWR_MGMT_1, 0x01);

	// Setting angles to zero
	angX = 0;
	angY = 0;
	angZ = 0;

	// Beginning integration timer
	intervalStart = millis();
}

/*
 *	Method that executes all readings and updates all attributes
 */
void MPU6050::Execute () {

	// Updating raw data before processing it
	this->UpdateRawAccel();
	this->UpdateRawGyro();

	// Computing readable accel/gyro data
	accX = (float)(rawAccX) * ACCEL_TRANSFORMATION_NUMBER;
	accY = (float)(rawAccY) * ACCEL_TRANSFORMATION_NUMBER;
	accZ = (float)(rawAccZ) * ACCEL_TRANSFORMATION_NUMBER;
	gyroX = (float)(rawGyroX - gyroXOffset) * GYRO_TRANSFORMATION_NUMBER;
	gyroY = (float)(rawGyroY - gyroYOffset) * GYRO_TRANSFORMATION_NUMBER;
	gyroZ = (float)(rawGyroZ - gyroZOffset) * GYRO_TRANSFORMATION_NUMBER;

	// Computing accel angles
	angAccX = wrap((atan2(accY, sqrt(accZ * accZ + accX * accX))) * RAD_TO_DEG);
	angAccY = wrap((-atan2(accX, sqrt(accZ * accZ + accY * accY))) * RAD_TO_DEG);

	// Computing gyro angles
	dt = (millis() - intervalStart) * 0.001;
	angGyroX = wrap(angGyroX + gyroX * dt);
	angGyroY = wrap(angGyroY + gyroY * dt);
	angGyroZ = wrap(angGyroZ + gyroZ * dt);

	// Computing complementary filter angles
	angX = angle_average(filterAccelCoeff, angAccX, filterGyroCoeff, angX + gyroX * dt);
	angY = angle_average(filterAccelCoeff, angAccY, filterGyroCoeff, angY + gyroY * dt);
	angZ = angGyroZ;

	// Reseting the integration timer
	intervalStart = millis();
}

/*
 *	Raw accel data update method
 */
void MPU6050::UpdateRawAccel () {

	// Beginning transmission for MPU6050
	Wire.beginTransmission(address);

	// Accessing accel data registers
	Wire.write(MPU6050_ACCEL_XOUT_H);
	Wire.endTransmission(false);

	// Requesting accel data
	Wire.requestFrom(address, 6, (int) true);

	// Storing raw accel data
	rawAccX = Wire.read() << 8;
	rawAccX |= Wire.read();

	rawAccY = Wire.read() << 8;
	rawAccY |= Wire.read();

	rawAccZ = Wire.read() << 8;
	rawAccZ |= Wire.read();
}

/*
 *	Raw gyro data update method
 */
void MPU6050::UpdateRawGyro () {

	// Beginning transmission for MPU6050
	Wire.beginTransmission(address);

	// Accessing gyro data registers
	Wire.write(MPU6050_GYRO_XOUT_H);
	Wire.endTransmission(false);

	// Requesting gyro data
	Wire.requestFrom(address, 6, (int) true);

	// Storing raw gyro data
	rawGyroX = Wire.read() << 8;
	rawGyroX |= Wire.read();

	rawGyroY = Wire.read() << 8;
	rawGyroY |= Wire.read();

	rawGyroZ = Wire.read() << 8;
	rawGyroZ |= Wire.read();
}

/*
 *	Register write method
 */
void MPU6050::RegisterWrite (byte registerAddress, byte data) {

	// Starting transmission for MPU6050
	Wire.beginTransmission(address);

	// Accessing register
	Wire.write(registerAddress);

	// Writing data
	Wire.write(data);

	// Closing transmission
	Wire.endTransmission();
}

/*
 *	MPU-6050 calibration method inspired by https://42bots.com/tutorials/arduino-script-for-mpu-6050-auto-calibration/
 */
void MPU6050::Calibrate () {
  pinMode(PC4, OUTPUT);
	for (int i = 0; i < DISCARDED_MEASURES; i++) {
		this->UpdateRawAccel();
		this->UpdateRawGyro();
		delay(2);
	}

	float sumGyroX = 0;
	float sumGyroY = 0;
	float sumGyroZ = 0;
	int cnt = 0;
	for (int i = 0; i < CALIBRATION_MEASURES; i++) {
		cnt++;
		this->UpdateRawAccel();
		this->UpdateRawGyro();
		sumGyroX += this->GetRawGyroX();
		sumGyroY += this->GetRawGyroY();
		sumGyroZ += this->GetRawGyroZ();
		delay(2);
		if(cnt % 30 == 0)
		digitalWrite(PC4, !digitalRead(PC4));
	}
  pinMode(PC4, INPUT);
	sumGyroX  /= CALIBRATION_MEASURES;
	sumGyroY  /= CALIBRATION_MEASURES;
	sumGyroZ  /= CALIBRATION_MEASURES;

	this->SetGyroOffsets(sumGyroX, sumGyroY, sumGyroZ);
}

/*
 *	Set function for gyro offsets
 */
void MPU6050::SetGyroOffsets (float x, float y, float z) {

	// Setting offsets
	gyroXOffset = x;
	gyroYOffset = y;
	gyroZOffset = z;
}

/*
 *	Set function for the acc coefficient on complementary filter
 */
void MPU6050::SetFilterAccCoeff (float coeff) {

	// Setting coefficient
	filterAccelCoeff = coeff;
}

/*
 *	Set function for the gyro coefficient on complementary filter
 */
void MPU6050::SetFilterGyroCoeff (float coeff) {

	// Setting coefficient
	filterGyroCoeff = coeff;
}
