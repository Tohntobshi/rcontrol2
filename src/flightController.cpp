#include <pigpio.h>
#include "flightController.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <math.h>
#include <algorithm>
#include "utils.h"
#include "glm/gtx/rotate_vector.hpp"
#include <fstream>

using namespace glm;

#define MPU9250_ADDRESS 0x68
#define AK8963_ADDRESS 0x0C

#define ACCEL_XOUT_H 0x3B // Accel data first register
#define GYRO_XOUT_H 0x43  // Gyro data first register
#define USER_CTRL 0x6A	  // MPU9250 config
#define INT_PIN_CFG 0x37  // MPU9250 config
#define CONFIG 0x1A	// MPU9250 config
#define ACCEL_CONFIG_2 0x1D // MPU9250 config

#define AK8963_CNTL 0x0A   // Mag config
#define AK8963_XOUT_L 0x03 // Mag data first register
#define AK8963_ASAX 0x10   // Mag adj vals first register

static unsigned int MIN_VAL = 1000;
static unsigned int MAX_VAL = 2000;

static unsigned int AUX_READY_PIN = 25;

FlightController * FlightController::instance = nullptr;

FlightController * FlightController::Init()
{
	if (instance != nullptr)
	{
		return instance;
	}
	instance = new FlightController();
	return instance;
}

void FlightController::Destroy()
{
	delete instance;
	instance = nullptr;
}

FlightController::FlightController()
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	if (gpioInitialise() < 0)
	{
		std::cout << "cant init gpio\n";
	}

	accGyroFD = i2cOpen(1, MPU9250_ADDRESS, 0);
	if (accGyroFD < 0)
	{
		std::cout << "cant open accel/gyro sensor\n";
	}
	uint8_t intPinCfg = readByte(accGyroFD, INT_PIN_CFG);

	writeByte(accGyroFD, INT_PIN_CFG, intPinCfg | 0b00000010); // set pass though mode for mpu9250 in order to have direct access to magnetometer
	writeByte(accGyroFD, CONFIG, imuLPFMode); // enable low pass filter for gyro
	writeByte(accGyroFD, ACCEL_CONFIG_2, imuLPFMode); // enable low pass filter for accel
	
	restorePrevGyroCalibration();
	
	magFD = i2cOpen(1, AK8963_ADDRESS, 0);
	if (magFD < 0)
	{
		std::cout << "cant open mag sensor\n";
	}
	writeByte(magFD, AK8963_CNTL, 0b00000000); // set power down mode for magnetometer
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	writeByte(magFD, AK8963_CNTL, 0b00001111); // set rom mode for magnetometer
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	uint8_t rawMagAdjVals[3];
	readBytes(magFD, AK8963_ASAX, rawMagAdjVals, 3);
	magAdjustment.x = ((float)(rawMagAdjVals[0] - 128) * 0.5f) / 128.f + 1.f;
	magAdjustment.y = ((float)(rawMagAdjVals[1] - 128) * 0.5f) / 128.f + 1.f;
	magAdjustment.z = ((float)(rawMagAdjVals[2] - 128) * 0.5f) / 128.f + 1.f;
	writeByte(magFD, AK8963_CNTL, 0b00000000); // set power down mode for magnetometer
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	writeByte(magFD, AK8963_CNTL, 0b00010110); // set 16bit continious measurement mode for magnetometer

	restorePrevMagCalibration();

	barFD = i2cOpen(1, BMP280_I2C_ADDR_PRIM, 0);
	if (barFD < 0)
	{
		std::cout << "cant open bar sensor\n";
	}
	bmp.delay_ms = [](uint32_t period_ms) { std::this_thread::sleep_for(std::chrono::milliseconds(period_ms)); };
	bmp.dev_id = barFD;
	bmp.read = readBytes;
	bmp.write = writeBytes;
	bmp.intf = BMP280_I2C_INTF;
	bmp280_init(&bmp);
	bmp280_config bmpConf;
	bmp280_get_config(&bmpConf, &bmp);
	bmpConf.filter = BMP280_FILTER_COEFF_2;
	bmpConf.os_pres = BMP280_OS_4X;
	bmpConf.odr = BMP280_ODR_62_5_MS;
	bmp280_set_config(&bmpConf, &bmp);
	bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);

	auxControllerFD = spiOpen(0, 1000000, 0);
	if (auxControllerFD < 0)
	{
		std::cout << "cant open auxillary controller\n";
	}
	gpioSetMode(AUX_READY_PIN, PI_INPUT);
	gpioSetPullUpDown(AUX_READY_PIN, PI_PUD_DOWN);
	controlAllMotors(0);
}

FlightController::~FlightController()
{
	gpioTerminate();
}

std::array<float, 2> FlightController::calibrateAccel()
{
	std::array<float, 2> result = { 0.f, 0.f };
	for (int i = 0; i < 10000; i++) {
		vec3 acc = normalize(getAccData());
		float accPitch = degrees(atan2(acc.x, acc.z));
		result[0] += accPitch;
		float accRoll = -degrees(atan2(acc.y, acc.z));
		result[1] += accRoll;
	}
	result[0] /= 10000;
	result[1] /= 10000;
	return result;
	
}
void FlightController::calibrateGyro()
{
	vec3 values;
	for (int i = 0; i < 10000; i++) {
		vec3 gyro = getGyroData();
		values += gyro;
	}
	values /= 10000;
	gyroCalibration = values;
	std::ofstream file;
	file.open("/home/pi/gyrocalibration", std::ofstream::out | std::ofstream::trunc);
	file << values.x << ","<< values.y << ","<< values.z << "\n";
	file.close();
}

void FlightController::restorePrevGyroCalibration()
{
	std::ifstream file;
	file.open("/home/pi/gyrocalibration");
	if (!file.is_open()) return;
	std::string line;
	std::getline(file, line);
	file.close();
	float x;
	float y;
	float z;
	sscanf(line.c_str(), "%f,%f,%f", &x, &y, &z);
	gyroCalibration.x = x;
	gyroCalibration.y = y;
	gyroCalibration.z = z;
}

void FlightController::controlAllMotors(uint16_t val)
{
	uint32_t value = val == 0 ? 0 : std::min((int)MAX_VAL, std::max((int)MIN_VAL, (int)val));
	controlAllMotorsAux(value, value, value, value);
}

void FlightController::controlAllMotors(uint16_t fl, uint16_t fr, uint16_t bl, uint16_t br)
{
	fl = std::min((int)MAX_VAL, std::max((int)MIN_VAL, (int)fl));
	fr = std::min((int)MAX_VAL, std::max((int)MIN_VAL, (int)fr));
	bl = std::min((int)MAX_VAL, std::max((int)MIN_VAL, (int)bl));
	br = std::min((int)MAX_VAL, std::max((int)MIN_VAL, (int)br));
	controlAllMotorsAux(fl, fr, bl, br);
}

void FlightController::controlAllMotorsAux(uint16_t fl, uint16_t fr, uint16_t bl, uint16_t br)
{
	uint8_t dummy[9];
	uint8_t command[1] = { 1 }; // command to update motor vals
	uint8_t values[9] = {
		uint8_t(fl >> 8),
		uint8_t(fl),
		uint8_t(fr >> 8),
		uint8_t(fr),
		uint8_t(bl >> 8),
		uint8_t(bl),
		uint8_t(br >> 8),
		uint8_t(br),
		uint8_t((fl >> 8) + fl + (fr >> 8) + fr + (bl >> 8) + bl + (br >> 8) + br) // checksum
	};
	while(!gpioRead(AUX_READY_PIN)) {}
	spiXfer(auxControllerFD, (char *)command, (char *)dummy, 1);
	std::this_thread::sleep_for(std::chrono::microseconds(10));
	while(!gpioRead(AUX_READY_PIN)) {}
	spiXfer(auxControllerFD, (char *)values, (char *)dummy, 9);
}


uint8_t FlightController::readByte(uint8_t device, uint8_t reg)
{
	return i2cReadByteData(device, reg);
}

int8_t FlightController::readBytes(uint8_t device, uint8_t reg, uint8_t *bytes, uint16_t count)
{
	int res = i2cReadI2CBlockData(device, reg, (char *)bytes, count);
	return res > 0 ? 0 : -1;
}

void FlightController::writeByte(uint8_t device, uint8_t reg, uint8_t byte)
{
	i2cWriteByteData(device, reg, byte);
}

int8_t FlightController::writeBytes(uint8_t device, uint8_t reg, uint8_t *bytes, uint16_t count)
{
	int res = i2cWriteI2CBlockData(device, reg, (char *)bytes, count);
	return res == 0 ? 0 : -1;
}

vec3 FlightController::getAccData()
{
	uint8_t rawData[6];
	readBytes(accGyroFD, ACCEL_XOUT_H, rawData, 6);
	int16_t rawX = ((int16_t)rawData[0] << 8) | rawData[1];
	int16_t rawY = ((int16_t)rawData[2] << 8) | rawData[3];
	int16_t rawZ = ((int16_t)rawData[4] << 8) | rawData[5];
	vec3 result;

	result.x = (float)rawX * aRes;
	result.y = (float)rawY * aRes;
	result.z = (float)rawZ * aRes;
	return result;
}

float FlightController::getUltrasonicHeightFromSensor()
{
	while (true)
	{
		uint8_t dummy[4];
		uint8_t command[1] = { 2 }; // command to send duration
		uint8_t values[4];
		while(!gpioRead(AUX_READY_PIN)) {}
		spiXfer(auxControllerFD, (char *)command, (char *)dummy, 1);
		std::this_thread::sleep_for(std::chrono::microseconds(10));
		while(!gpioRead(AUX_READY_PIN)) {}
		spiXfer(auxControllerFD, (char *)dummy, (char *)values, 4);

		uint8_t msb = values[0];
		uint8_t lsb = values[1];
		uint8_t chksum = values[2];
		uint8_t chksum2 = values[3];
		uint8_t currentChkSum = (msb >> 4) + (msb & 0b00001111) + (lsb >> 4) + (lsb & 0b00001111);
		uint8_t currentChkSum2 = (~msb) + (~lsb);
		if (chksum != currentChkSum || chksum2 != currentChkSum2 || (msb == 0 && lsb == 0)) {
			std::cout << "corrupted us data\n";
			// corrupted data
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}
		uint16_t duration = ((uint16_t)msb << 8) | lsb;
		return (duration * 343.f) / 2000000.f;
	}
}

vec3 FlightController::getGyroData()
{
	uint8_t rawData[6];
	readBytes(accGyroFD, GYRO_XOUT_H, rawData, 6);
	int16_t rawX = ((int16_t)rawData[0] << 8) | rawData[1];
	int16_t rawY = ((int16_t)rawData[2] << 8) | rawData[3];
	int16_t rawZ = ((int16_t)rawData[4] << 8) | rawData[5];
	vec3 result;

	result.x = (float)rawX * gRes;
	result.y = (float)rawY * gRes;
	result.z = (float)rawZ * gRes;
	return result;
}

vec3 FlightController::getGyroCalibratedData()
{
	return getGyroData() - gyroCalibration;
}


vec3 FlightController::getMagData()
{
	while (true)
	{
		uint8_t rawData[7];
		readBytes(magFD, AK8963_XOUT_L, rawData, 7);
		if ((rawData[6] & 0x08))
		{
			// data corrupted, try again
			continue;
		}
		int16_t rawX = ((int16_t)rawData[1] << 8) | rawData[0];
		int16_t rawY = ((int16_t)rawData[3] << 8) | rawData[2];
		int16_t rawZ = ((int16_t)rawData[5] << 8) | rawData[4];
		vec3 result;
		result.x = (float)rawX * magAdjustment.x;
		result.y = (float)rawY * magAdjustment.y;
		result.z = (float)rawZ * magAdjustment.z;
		return result;
	}
}

void FlightController::calibrateMag()
{
	vec3 result = getMagData();
	vec3 magMinVals = result;
	vec3 magMaxVals = result;
	for (int i = 0; i < 20000; i++) {
		vec3 result = getMagData();
		if (result.x < magMinVals.x)
			magMinVals.x = result.x;
		if (result.x > magMaxVals.x)
			magMaxVals.x = result.x;
		if (result.y < magMinVals.y)
			magMinVals.y = result.y;
		if (result.y > magMaxVals.y)
			magMaxVals.y = result.y;
		if (result.z < magMinVals.z)
			magMinVals.z = result.z;
		if (result.z > magMaxVals.z)
			magMaxVals.z = result.z;
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	magRange = (magMaxVals - magMinVals) / 2.f;
	magMidVals = magMinVals + magRange;
	std::ofstream file;
	file.open("/home/pi/magcalibration", std::ofstream::out | std::ofstream::trunc);
	file << magRange.x << ","<< magRange.y << ","<< magRange.z << ","<< magMidVals.x << ","<< magMidVals.y << ","<< magMidVals.z << "\n";
	file.close();
}

void FlightController::restorePrevMagCalibration()
{
	std::ifstream file;
	file.open("/home/pi/magcalibration");
	if (!file.is_open()) return;
	std::string line;
	std::getline(file, line);
	file.close();
	float mRangeX;
	float mRangeY;
	float mRangeZ;
	float midX;
	float midY;
	float midZ;
	sscanf(line.c_str(), "%f,%f,%f,%f,%f,%f", &mRangeX, &mRangeY, &mRangeZ, &midX, &midY, &midZ);
	magRange.x = mRangeX;
	magMidVals.x = midX;
	magRange.y = mRangeY;
	magMidVals.y = midY;
	magRange.z = mRangeZ;
	magMidVals.z = midZ;
}

vec3 FlightController::getMagNormalizedData()
{
	vec3 result = getMagData();
	vec3 noralizedSeparately({(result.x - magMidVals.x) / magRange.x, (result.y - magMidVals.y) / magRange.y, (result.z - magMidVals.z) / magRange.z});
	return normalize(noralizedSeparately);
}

float FlightController::getBarData()
{
	bmp280_uncomp_data ucomp_data;
	double pres;
	bmp280_get_uncomp_data(&ucomp_data, &bmp);
	bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
	return pres;
}

void FlightController::start(InfoAdapter * infoAdapter)
{
	setShouldStop(false);
	int64_t prevTimeStamp = 0;
	int64_t infoSentTimeStamp = 0;
	int64_t heightTakenTimeStamp = 0;
	float prevPitch = 0.f;
	float prevRoll = 0.f;
	float prevYaw = 0.f;
	float prevHeight = 0.f;
	float prevHeightDer = 0.f;
	float pitchErrInt = 0.f;
	float rollErrInt = 0.f;
	float yawErrInt = 0.f;
	float heightErrInt = 0.f;
	bool usePressureForHeightEstimation = false;
	float groundPressure = 0.f;
	float passivePressureChangeRate = 0.f;

	std::unique_lock<std::mutex> lck(commonCommandMtx);
	int currentLPFMode = imuLPFMode;
	float pitchAccelCalibration = 0.f;
	float rollAccelCalibration = 0.f;
	lck.unlock();
	while(!getShouldStop()) {
		
		if (getNeedCalibrateGyro()) {
			calibrateGyro();
			setNeedCalibrateGyro(false);
			prevYaw = 0.f; // temp solution
			continue;
		}
		if (getNeedCalibrateAccel()) {
			auto result = calibrateAccel();
			pitchAccelCalibration = result[0];
			rollAccelCalibration = result[1];
			setNeedCalibrateAccel(false);
			continue;
		}
		if (getNeedCalibrateMag()) {
			calibrateMag();
			prevYaw = 0.f;
			setNeedCalibrateMag(false);
			continue;
		}
		if (getNeedCalibrate()) {
			// todo: signal aux controller to calibrate esc's
			setNeedCalibrate(false);
			continue;
		}
		// copy adjustable from outside values
		lck.lock();
		float desiredPitch = this->desiredPitch;
		float desiredRoll = this->desiredRoll;
		float pitchAdjust = this->pitchAdjust;
		float rollAdjust = this->rollAdjust;
		float desiredDirection = this->desiredDirection;
		float desiredHeight = this->acceleration * 4.f;
		float accTrust = this->accTrust;
		float magTrust = this->magTrust;
		float turnOffInclineAngle = this->turnOffInclineAngle;
		int desiredLPFMode = this->imuLPFMode;

		float pitchPropCoef = this->pitchPropCoef;
		float pitchDerCoef = this->pitchDerCoef;
		float pitchIntCoef = this->pitchIntCoef;
		float rollPropCoef = this->rollPropCoef;
		float rollDerCoef = this->rollDerCoef;
		float rollIntCoef = this->rollIntCoef;
		float yawPropCoef = this->yawPropCoef;
		float yawDerCoef = this->yawDerCoef;
		float yawIntCoef = this->yawIntCoef;
		float heightPropCoef = this->heightPropCoef;
		float heightDerCoef = this->heightDerCoef;
		float heightIntCoef = this->heightIntCoef;
		float acceleration = this->acceleration;
		float baseAcceleration = this->baseAcceleration;
		bool turnOffTrigger = this->turnOffTrigger;
		bool sendingInfo = this->sendingInfo;
		lck.unlock();

		if (desiredLPFMode != currentLPFMode)
		{
			writeByte(accGyroFD, CONFIG, desiredLPFMode);
			writeByte(accGyroFD, ACCEL_CONFIG_2, desiredLPFMode);
			currentLPFMode = desiredLPFMode;
		}

		vec3 acc = getAccData();
		vec3 gyro = getGyroCalibratedData();
		vec3 mag = getMagNormalizedData();
		float sensorHeight = getUltrasonicHeightFromSensor();
		float pressure = getBarData();

		int64_t currentTimestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
		float secondsElapsed = (double)(currentTimestamp - prevTimeStamp) / 1000.0;

		float accPitch = degrees(atan2(acc.x, acc.z)) - pitchAccelCalibration;
		float accRoll = -degrees(atan2(acc.y, glm::length(vec2(acc.z, acc.x)) )) - rollAccelCalibration;

		float gyroPitch = prevPitch - gyro.y * secondsElapsed;
		float gyroRoll = prevRoll - gyro.x * secondsElapsed;
		float gyroYaw = prevYaw + gyro.z * secondsElapsed;
		float gyroPitchTmp = gyroPitch;
		float gyroRollTmp = gyroRoll;
		gyroPitch -= gyroRollTmp * sin(radians(gyro.z * secondsElapsed));
		gyroRoll += gyroPitchTmp * sin(radians(gyro.z * secondsElapsed));
		gyroYaw += gyro.x * secondsElapsed * sin(radians(gyroPitchTmp));
		gyroYaw -= gyro.y * secondsElapsed * sin(radians(gyroRollTmp));

		float currentPitch = accPitch * accTrust + (1.f - accTrust) * gyroPitch;
		float currentRoll = accRoll * accTrust + (1.f - accTrust) * gyroRoll;

		vec3 staticMag = glm::rotateY(glm::rotateX(mag, radians(-currentPitch) ), radians(-currentRoll));
		float magYaw = degrees(atan2(staticMag.y, staticMag.x)) - 90.f;
		auto yawAngles = Utils::pepareAnglesForCombination(magYaw, gyroYaw);
		
		
		float currentYaw = Utils::trimAngleTo360(yawAngles[0] * magTrust + (1.f - magTrust) * yawAngles[1]);
		
		float currentHeight = 0.f;
		float currentHeightError = 0.f;
		if (abs(currentPitch) > 30.f || abs(currentRoll) > 30.f)
		{
			currentHeight = prevHeight;
			currentHeightError = desiredHeight - currentHeight;
		}
		else
		{
			currentHeight = sensorHeight * sqrt(1.f / (pow(tan(radians(currentRoll)), 2) + pow(tan(radians(currentPitch)), 2) + 1)) * 0.2f + prevHeight * 0.8f;
			currentHeightError = desiredHeight - currentHeight;
			heightErrInt += currentHeightError * secondsElapsed;
		}
		// float currentHeight = sensorHeight;
		float heightDer = ((currentHeight - prevHeight) / secondsElapsed) * 0.2f + prevHeightDer * 0.8f;
		prevHeight = currentHeight;
		prevHeightDer = heightDer;
		

		float currentPitchError = desiredPitch - currentPitch + pitchAdjust;
		float currentRollError = desiredRoll - currentRoll + rollAdjust;
		auto yawAngles2 = Utils::pepareAnglesForCombination(-desiredDirection, currentYaw);
		float currentYawError = yawAngles2[0] - yawAngles2[1];

		float pitchErrorChangeRate = gyro.y;
		float rollErrorChangeRate = gyro.x;
		float yawErrorChangeRate = -gyro.z;
		float heightErrorChangeRate = -heightDer;

		pitchErrInt += currentPitchError * secondsElapsed;
		rollErrInt += currentRollError * secondsElapsed;
		yawErrInt += currentYawError * secondsElapsed;

		prevPitch = currentPitch;
		prevRoll = currentRoll;
		prevYaw = currentYaw;
		prevHeight = currentHeight;
		prevTimeStamp = currentTimestamp;
		if (currentHeight < 0.1f) {
			currentYawError = 0.f;
			yawErrInt = 0.f;
		}
		if (abs(currentPitch) > turnOffInclineAngle || abs(currentRoll) > turnOffInclineAngle) {
			setTurnOffTrigger(true);
		}
		
		int pitchMotorAdjust = (currentPitchError * pitchPropCoef + pitchErrorChangeRate * pitchDerCoef + pitchErrInt * pitchIntCoef);
		int rollMotorAdjust = (currentRollError * rollPropCoef + rollErrorChangeRate * rollDerCoef + rollErrInt * rollIntCoef);
		int yawMotorAdjust = currentYawError * yawPropCoef + yawErrorChangeRate * yawDerCoef + yawErrInt * yawIntCoef;
		int heightMotorAdjust = (currentHeightError * heightPropCoef + heightErrorChangeRate * heightDerCoef + heightErrInt * heightIntCoef) * 1000;

		int baseMotorVal = MIN_VAL + (MAX_VAL - MIN_VAL) * baseAcceleration;

		int frontLeft = baseMotorVal + heightMotorAdjust + pitchMotorAdjust - rollMotorAdjust + yawMotorAdjust;
		int frontRight = baseMotorVal + heightMotorAdjust + pitchMotorAdjust + rollMotorAdjust - yawMotorAdjust;
		int backLeft = baseMotorVal + heightMotorAdjust - pitchMotorAdjust - rollMotorAdjust - yawMotorAdjust;
		int backRight = baseMotorVal + heightMotorAdjust - pitchMotorAdjust + rollMotorAdjust + yawMotorAdjust;

		if (acceleration < 0.01f || turnOffTrigger)
		{
			frontLeft = MIN_VAL;
			frontRight = MIN_VAL;
			backLeft = MIN_VAL;
			backRight = MIN_VAL;
			pitchErrInt = 0.f;
			rollErrInt = 0.f;
			heightErrInt = 0.f;
			yawErrInt = 0.f;
			usePressureForHeightEstimation = false;
			groundPressure = pressure;
			passivePressureChangeRate = 0.f;
			// pressure values look like this
			// pressure 96828.2
			// pressure 96827
			// pressure 96825.7
			// pressure 96825.7
			// pressure 96826.3
			// pressure 96825.7

		}
		controlAllMotors(frontLeft, frontRight, backLeft, backRight);
		if (sendingInfo) {
			if (currentTimestamp - infoSentTimeStamp >= 50)
			{
				infoSentTimeStamp = currentTimestamp;
				// std::cout << "pressure " << pressure << "\n";
				infoAdapter->sendInfo(
					currentPitchError,
					currentRollError,

					pitchErrorChangeRate,
					rollErrorChangeRate,

					currentHeightError,
					heightErrorChangeRate,

					currentYawError,
					yawErrorChangeRate,

					frontLeft,
					frontRight,
					backLeft,
					backRight,
					
					1.f / secondsElapsed,

					pitchErrInt,
					rollErrInt,
					yawErrInt,
					heightErrInt
				);
			}
		}
	}
}

void FlightController::stop()
{
	setShouldStop(true);
}

void FlightController::scheduleArm()
{
	setNeedArm(true);
}

void FlightController::scheduleCalibrate()
{
	setNeedCalibrate(true);
}

void FlightController::scheduleCalibrateGyro()
{
	setNeedCalibrateGyro(true);
}

void FlightController::scheduleCalibrateAccel()
{
	setNeedCalibrateAccel(true);
}

void FlightController::scheduleCalibrateMag()
{
	setNeedCalibrateMag(true);
}

void FlightController::setShouldStop(bool val)
{
	std::unique_lock<std::mutex> lck(shouldStopMtx);
	shouldStop = val;
}

bool FlightController::getShouldStop()
{
	std::unique_lock<std::mutex> lck(shouldStopMtx);
	return shouldStop;
}

void FlightController::setNeedArm(bool val)
{
	std::unique_lock<std::mutex> lck(needArmMtx);
	needArm = val;
}

bool FlightController::getNeedArm()
{
	std::unique_lock<std::mutex> lck(needArmMtx);
	return needArm;
}

void FlightController::setNeedCalibrate(bool val)
{
	std::unique_lock<std::mutex> lck(needCalibrateMtx);
	needCalibrate = val;
}

bool FlightController::getNeedCalibrate()
{
	std::unique_lock<std::mutex> lck(needCalibrateMtx);
	return needCalibrate;
}

void FlightController::setNeedCalibrateGyro(bool val)
{
	std::unique_lock<std::mutex> lck(needCalibrateGyroMtx);
	needCalibrateGyro = val;
}

bool FlightController::getNeedCalibrateGyro()
{
	std::unique_lock<std::mutex> lck(needCalibrateGyroMtx);
	return needCalibrateGyro;
}

void FlightController::setNeedCalibrateAccel(bool val)
{
	std::unique_lock<std::mutex> lck(needCalibrateAccelMtx);
	needCalibrateAccel = val;
}

bool FlightController::getNeedCalibrateAccel()
{
	std::unique_lock<std::mutex> lck(needCalibrateAccelMtx);
	return needCalibrateAccel;
}

void FlightController::setNeedCalibrateMag(bool val)
{
	std::unique_lock<std::mutex> lck(needCalibrateMagMtx);
	needCalibrateMag = val;
}

bool FlightController::getNeedCalibrateMag()
{
	std::unique_lock<std::mutex> lck(needCalibrateMagMtx);
	return needCalibrateMag;
}

void FlightController::setDesiredPitchAndRoll(float pitch, float roll)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	desiredPitch = pitch;
	desiredRoll = roll;
}

void FlightController::setPitchPropCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	pitchPropCoef = value;
}

void FlightController::setPitchDerCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	pitchDerCoef = value;
}

void FlightController::setPitchIntCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	pitchIntCoef = value;
}

void FlightController::setRollPropCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	rollPropCoef = value;
}

void FlightController::setRollDerCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	rollDerCoef = value;
}

void FlightController::setRollIntCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	rollIntCoef = value;
}

void FlightController::setYawPropCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	yawPropCoef = value;
}

void FlightController::setYawDerCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	yawDerCoef = value;
}

void FlightController::setYawIntCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	yawIntCoef = value;
}

void FlightController::setHeightPropCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	heightPropCoef = value;
}

void FlightController::setHeightDerCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	heightDerCoef = value;
}

void FlightController::setHeightIntCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	heightIntCoef = value;
}


void FlightController::setAcceleration(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	acceleration = value;
}

void FlightController::setBaseAcceleration(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	baseAcceleration = value;
}

void FlightController::setAccTrust(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	accTrust = value;
}

void FlightController::setMagTrust(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	magTrust = value;
}

void FlightController::setTurnOffInclineAngle(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	turnOffInclineAngle = value;
}

void FlightController::setTurnOffTrigger(bool value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	turnOffTrigger = value;
}

void FlightController::resetTurnOffTrigger()
{
	setTurnOffTrigger(false);
}

void FlightController::setImuLPFMode(int value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	int mode = min(6, max(1, value));
	imuLPFMode = value;
}

void FlightController::setSendingInfo(bool value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	sendingInfo = value;
}

void FlightController::setPitchAdjust(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	pitchAdjust = value;
}

void FlightController::setRollAdjust(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	rollAdjust = value;
}

void FlightController::setDirection(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	desiredDirection = value;
}