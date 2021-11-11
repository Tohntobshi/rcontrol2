#include <pigpio.h>
#include "flightController.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <math.h>
#include <algorithm>

using namespace glm;

#define MPU9250_ADDRESS 0x68
#define AK8963_ADDRESS 0x0C
#define ULTRASONIC_ADDRESS 0x13

#define ACCEL_XOUT_H 0x3B // Accel data first register
#define GYRO_XOUT_H 0x43  // Gyro data first register
#define USER_CTRL 0x6A	  // MPU9250 config
#define INT_PIN_CFG 0x37  // MPU9250 config

#define AK8963_CNTL 0x0A   // Mag config
#define AK8963_XOUT_L 0x03 // Mag data first register
#define AK8963_ASAX 0x10   // Mag adj vals first register

static unsigned int MIN_VAL = 1000;
static unsigned int MAX_VAL = 2000;

static unsigned int MOTOR_FL_PIN = 21;
static unsigned int MOTOR_FR_PIN = 20;
static unsigned int MOTOR_BL_PIN = 19;
static unsigned int MOTOR_BR_PIN = 26;


static vec3 flVec = normalize(vec3(-1.0, 1.0, 0.0));
static vec3 frVec = normalize(vec3(1.0, 1.0, 0.0));
static vec3 blVec = normalize(vec3(-1.0, -1.0, 0.0));
static vec3 brVec = normalize(vec3(1.0, -1.0, 0.0));

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
	magFD = i2cOpen(1, AK8963_ADDRESS, 0);
	if (magFD < 0)
	{
		std::cout << "cant open mag sensor\n";
	}
	writeByte(magFD, AK8963_CNTL, 0b00001111); // set rom mode for magnetometer
	uint8_t rawMagAdjVals[3];
	readBytes(magFD, AK8963_ASAX, rawMagAdjVals, 3);
	magAdjustment.x = (float)(rawMagAdjVals[0] - 128) / 128.f + 1.f;
	magAdjustment.y = (float)(rawMagAdjVals[1] - 128) / 128.f + 1.f;
	magAdjustment.z = (float)(rawMagAdjVals[2] - 128) / 128.f + 1.f;
	writeByte(magFD, AK8963_CNTL, 0b00010110); // set 16bit continious measurement mode for magnetometer
	vec3 magVals = getMagData();
	magMinVals = magVals;
	magMaxVals = magVals;

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

	ultrasonicFD = i2cOpen(1, ULTRASONIC_ADDRESS, 0);
	if (ultrasonicFD < 0)
	{
		std::cout << "cant open ultrasonic sensor\n";
	}

	gpioSetMode(MOTOR_FL_PIN, PI_OUTPUT);
	gpioSetMode(MOTOR_FR_PIN, PI_OUTPUT);
	gpioSetMode(MOTOR_BL_PIN, PI_OUTPUT);
	gpioSetMode(MOTOR_BR_PIN, PI_OUTPUT);
	controlAll(0);
}

FlightController::~FlightController()
{
	gpioTerminate();
}

void FlightController::arm()
{
	std::cout << "arm\n";
	controlAll(0);
	std::this_thread::sleep_for(std::chrono::seconds(1));
	controlAll(MAX_VAL);
	std::this_thread::sleep_for(std::chrono::seconds(1));
	controlAll(MIN_VAL);
	std::this_thread::sleep_for(std::chrono::seconds(1));
	std::cout << "arm finished\n";
}

void FlightController::calibrate()
{
	std::cout << "calibrate\n";
	controlAll(0);
	std::this_thread::sleep_for(std::chrono::seconds(3));
	std::cout << "calibrate max\n";
	controlAll(MAX_VAL);
	std::this_thread::sleep_for(std::chrono::seconds(6));
	std::cout << "calibrate min\n";
	controlAll(MIN_VAL);
	std::this_thread::sleep_for(std::chrono::seconds(6));
	std::cout << "calibrate finished\n";
	controlAll(0);
}

void FlightController::controlAll(uint32_t val)
{
	uint32_t value = val == 0 ? 0 : std::min(MAX_VAL, std::max(MIN_VAL, val));
	gpioServo(MOTOR_FL_PIN, value);
	gpioServo(MOTOR_FR_PIN, value);
	gpioServo(MOTOR_BL_PIN, value);
	gpioServo(MOTOR_BR_PIN, value);
}

void FlightController::controlAll(uint32_t fl, uint32_t fr, uint32_t bl, uint32_t br)
{
	gpioServo(MOTOR_FL_PIN, std::min(MAX_VAL, std::max(MIN_VAL, fl)));
	gpioServo(MOTOR_FR_PIN, std::min(MAX_VAL, std::max(MIN_VAL, fr)));
	gpioServo(MOTOR_BL_PIN, std::min(MAX_VAL, std::max(MIN_VAL, bl)));
	gpioServo(MOTOR_BR_PIN, std::min(MAX_VAL, std::max(MIN_VAL, br)));
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

float FlightController::getUltrasonicHeight()
{
	uint8_t rawData[2];
	readBytes(ultrasonicFD, 0x0, rawData, 2);
	uint16_t duration = ((uint16_t)rawData[1] << 8) | rawData[0];
	return (duration * 343.f) / 2000000.f;
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

vec3 FlightController::getMagData()
{
	uint8_t rawData[7];
	readBytes(magFD, AK8963_XOUT_L, rawData, 7);
	if ((rawData[6] & 0x08))
	{
		// data corrupted, do something
	}
	int16_t rawX = ((int16_t)rawData[1] << 8) | rawData[0];
	int16_t rawY = ((int16_t)rawData[3] << 8) | rawData[2];
	int16_t rawZ = ((int16_t)rawData[5] << 8) | rawData[4];
	vec3 result;
	result.x = (float)rawX * mRes * magAdjustment.x;
	result.y = (float)rawY * mRes * magAdjustment.y;
	result.z = (float)rawZ * mRes * magAdjustment.z;
	return result;
}

vec3 FlightController::getMagNormalizedData()
{
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
	magMidVals.x = (magMaxVals.x + magMinVals.x) / 2.f;
	magMidVals.y = (magMaxVals.y + magMinVals.y) / 2.f;
	magMidVals.z = (magMaxVals.z + magMinVals.z) / 2.f;
	float xRange = abs(magMaxVals.x - magMinVals.x) / 2.f;
	float yRange = abs(magMaxVals.y - magMinVals.y) / 2.f;
	float zRange = abs(magMaxVals.z - magMinVals.z) / 2.f;
	vec3 noralizedSeparately({(result.x - magMidVals.x) / xRange, (result.y - magMidVals.y) / yRange, (result.z - magMidVals.z) / zRange});
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
	std::thread i2cSensorsThread([&]() -> void {
		std::unique_lock<std::mutex> lck(commonCommandMtx);
		lck.unlock();
		// previous iteration values
		int64_t prevTimeStamp = 0;
		float prevPitch = 0.f;
		float prevRoll = 0.f;
		float prevYawSpeed = 0.f;
		float prevHeight = 0.f;
		float prevPitchErrChangeRate = 0.f;
		float prevRollErrChangeRate = 0.f;
		float prevYawSpeedErrChangeRate = 0.f;
		while (!getShouldStop()) {
			lck.lock();
			float desiredPitch = this->desiredPitch;
			float desiredRoll = this->desiredRoll;
			float desiredYawSpeed = this->desiredYawSpeed;
			float desiredHeight = this->acceleration * 4.f;
			float accTrust = this->accTrust;
			float inclineFilteringCoef = this->inclineFilteringCoef;
			float inclineChangeRateFilteringCoef = this->inclineChangeRateFilteringCoef;
			float yawSpeedFilteringCoef = this->yawSpeedFilteringCoef;
			float yawSpeedChangeRateFilteringCoef = this->yawSpeedChangeRateFilteringCoef;
			lck.unlock();

			vec3 acc = normalize(getAccData());
			vec3 gyro = getGyroData();
			vec3 mag = getMagNormalizedData();
			float pressure = getBarData();
			float ultrasonicHeight = getUltrasonicHeight();

			int64_t currentTimestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
			float secondsElapsed = (double)(currentTimestamp - prevTimeStamp) / 1000.0;

			float accPitch = degrees(atan2(acc.x, acc.z));
			float accRoll = -degrees(atan2(acc.y, acc.z));
			float magYaw = degrees(atan2(mag.y, mag.x));

			float pitchChangeRate = -gyro.y;
			float rollChangeRate = -gyro.x;

			float currentYawSpeed = gyro.z * (1.f - yawSpeedFilteringCoef) + prevYawSpeed * yawSpeedFilteringCoef;

			float currentPitch = (accPitch * accTrust + (1.f - accTrust) * (prevPitch + pitchChangeRate * secondsElapsed)) * (1.f - inclineFilteringCoef) + prevPitch * inclineFilteringCoef;
			float currentRoll = (accRoll * accTrust + (1.f - accTrust) * (prevRoll + rollChangeRate * secondsElapsed)) * (1.f - inclineFilteringCoef) + prevRoll * inclineFilteringCoef;
			float currentHeight = ultrasonicHeight * 0.2 + prevHeight * 0.8; // adjustable filter maybe ?

			float currentPitchError = desiredPitch - currentPitch;
			float currentRollError = desiredRoll - currentRoll;
			float currentYawSpeedError = desiredYawSpeed - currentYawSpeed;
			float currentHeightError = desiredHeight - currentHeight;
			setPitchErr(currentPitchError);
			setRollErr(currentRollError);
			setHeightErr(currentHeightError);
			float prevPitchError = desiredPitch - prevPitch;
			float prevRollError = desiredRoll - prevRoll;
			float prevYawSpeedError = desiredYawSpeed - prevYawSpeed;

			float pitchErrorChangeRate = -pitchChangeRate * (1.f - inclineChangeRateFilteringCoef) + prevPitchErrChangeRate * inclineChangeRateFilteringCoef;
			float rollErrorChangeRate = -rollChangeRate * (1.f - inclineChangeRateFilteringCoef) + prevRollErrChangeRate * inclineChangeRateFilteringCoef;
			float yawSpeedErrorChangeRate = ((currentYawSpeedError - prevYawSpeedError) / secondsElapsed) * (1.f - yawSpeedChangeRateFilteringCoef) + prevYawSpeedErrChangeRate * yawSpeedChangeRateFilteringCoef;
			float heightErrorChangeRate = (currentHeightError - (desiredHeight - prevHeight)) / secondsElapsed;
			setPitchErrDer(pitchErrorChangeRate);
			setRollErrDer(rollErrorChangeRate);
			setHeightErrDer(heightErrorChangeRate);

			setPitchErrInt(getPitchErrInt() + currentPitchError * secondsElapsed);
			setRollErrInt(getRollErrInt() + currentRollError * secondsElapsed);
			setYawSpeedErrInt(getYawSpeedErrInt() + currentYawSpeedError * secondsElapsed);
			setHeightErrInt(getHeightErrInt() + currentHeightError * secondsElapsed);

			prevPitch = currentPitch;
			prevRoll = currentRoll;
			prevYawSpeed = currentYawSpeed;
			prevHeight = currentHeight;
			prevTimeStamp = currentTimestamp;
			prevPitchErrChangeRate = pitchErrorChangeRate;
			prevRollErrChangeRate = rollErrorChangeRate;
			prevYawSpeedErrChangeRate = yawSpeedErrorChangeRate;
		}
		
	});

	std::unique_lock<std::mutex> lck(commonCommandMtx);
	lck.unlock();
	while(!getShouldStop()) {
		if (getNeedArm()) {
			arm();
			setNeedArm(false);
			continue;
		}
		if (getNeedCalibrate()) {
			calibrate();
			setNeedCalibrate(false);
			continue;
		}
		// copy adjustable from outside values
		lck.lock();
		float pitchPropCoef = this->pitchPropCoef;
		float pitchDerCoef = this->pitchDerCoef;
		float pitchIntCoef = this->pitchIntCoef;
		float rollPropCoef = this->rollPropCoef;
		float rollDerCoef = this->rollDerCoef;
		float rollIntCoef = this->rollIntCoef;
		float yawSpPropCoef = this->yawSpPropCoef;
		float yawSpDerCoef = this->yawSpDerCoef;
		float yawSpIntCoef = this->yawSpIntCoef;
		float heightPropCoef = this->heightPropCoef;
		float heightDerCoef = this->heightDerCoef;
		float heightIntCoef = this->heightIntCoef;
		bool onlyPositiveAdjustMode = this->onlyPositiveAdjustMode;
		float acceleration = this->acceleration;
		float baseAcceleration = this->baseAcceleration;
		lck.unlock();
		

		// get this values
		float currentYawSpeedError = 0.f;
		float yawSpeedErrorChangeRate = 0.f;

		

		int pitchAdjust = (getPitchErr() * pitchPropCoef + getPitchErrDer() * pitchDerCoef + getPitchErrInt() * pitchIntCoef) * 10;
		int rollAdjust = (getRollErr() * rollPropCoef + getRollErrDer() * rollDerCoef + getRollErrInt() * rollIntCoef) * 10;
		int yawAdjust = currentYawSpeedError * yawSpPropCoef + yawSpeedErrorChangeRate * yawSpDerCoef + getYawSpeedErrInt() * yawSpIntCoef;
		int heightAdjust = (getHeightErr() * heightPropCoef + getHeightErrDer() * heightDerCoef + getHeightErrInt() * heightIntCoef) * 1000;

		int baseVal = MIN_VAL + (MAX_VAL - MIN_VAL) * baseAcceleration;


		int frontLeft = baseVal + heightAdjust + pitchAdjust - rollAdjust + yawAdjust;
		int frontRight = baseVal + heightAdjust + pitchAdjust + rollAdjust - yawAdjust;
		int backLeft = baseVal + heightAdjust - pitchAdjust - rollAdjust - yawAdjust;
		int backRight = baseVal + heightAdjust - pitchAdjust + rollAdjust + yawAdjust;
		if (acceleration < 0.01f)
		{
			frontLeft = MIN_VAL;
			frontRight = MIN_VAL;
			backLeft = MIN_VAL;
			backRight = MIN_VAL;
			controlAll(MIN_VAL);
			setPitchErrInt(0.f);
			setRollErrInt(0.f);
			setHeightErrInt(0.f);
			setYawSpeedErrInt(0.f);
		}
		else
		{
			controlAll(frontLeft, frontRight, backLeft, backRight);
		}
		// std::cout << "height error" << getHeightErr() << "\n";
		infoAdapter->sendInfo(
			getPitchErr(),
			getRollErr(),
			getPitchErrDer(),
			getRollErrDer(),
			getHeightErr(),
			getHeightErrDer(),
			frontLeft,
			frontRight,
			backLeft,
			backRight
		);

		std::this_thread::sleep_for(std::chrono::milliseconds(20)); // TODO make adjustable
	}
	i2cSensorsThread.join();
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

void FlightController::setPitchErr(float val)
{
	std::unique_lock<std::mutex> lck(pitchErrMtx);
	pitchErr = val;
}

float FlightController::getPitchErr()
{
	std::unique_lock<std::mutex> lck(pitchErrMtx);
	return pitchErr;
}

void FlightController::setPitchErrDer(float val)
{
	std::unique_lock<std::mutex> lck(pitchErrDerMtx);
	pitchErrDer = val;
}

float FlightController::getPitchErrDer()
{
	std::unique_lock<std::mutex> lck(pitchErrDerMtx);
	return pitchErrDer;
}

void FlightController::setPitchErrInt(float val)
{
	std::unique_lock<std::mutex> lck(pitchErrIntMtx);
	pitchErrInt = val;
}

float FlightController::getPitchErrInt()
{
	std::unique_lock<std::mutex> lck(pitchErrIntMtx);
	return pitchErrInt;
}

void FlightController::setRollErr(float val)
{
	std::unique_lock<std::mutex> lck(rollErrMtx);
	rollErr = val;
}

float FlightController::getRollErr()
{
	std::unique_lock<std::mutex> lck(rollErrMtx);
	return rollErr;
}

void FlightController::setRollErrDer(float val)
{
	std::unique_lock<std::mutex> lck(rollErrDerMtx);
	rollErrDer = val;
}

float FlightController::getRollErrDer()
{
	std::unique_lock<std::mutex> lck(rollErrDerMtx);
	return rollErrDer;
}

void FlightController::setRollErrInt(float val)
{
	std::unique_lock<std::mutex> lck(rollErrIntMtx);
	rollErrInt = val;
}

float FlightController::getRollErrInt()
{
	std::unique_lock<std::mutex> lck(rollErrIntMtx);
	return rollErrInt;
}

void FlightController::setYawSpeedErrInt(float val)
{
	std::unique_lock<std::mutex> lck(yawSpeedErrIntMtx);
	yawSpeedErrInt = val;
}

float FlightController::getYawSpeedErrInt()
{
	std::unique_lock<std::mutex> lck(yawSpeedErrIntMtx);
	return yawSpeedErrInt;
}

void FlightController::setHeightErr(float val)
{
	std::unique_lock<std::mutex> lck(heightErrMtx);
	heightErr = val;
}

float FlightController::getHeightErr()
{
	std::unique_lock<std::mutex> lck(heightErrMtx);
	return heightErr;
}

void FlightController::setHeightErrDer(float val)
{
	std::unique_lock<std::mutex> lck(heightErrDerMtx);
	heightErrDer = val;
}

float FlightController::getHeightErrDer()
{
	std::unique_lock<std::mutex> lck(heightErrDerMtx);
	return heightErrDer;
}

void FlightController::setHeightErrInt(float val)
{
	std::unique_lock<std::mutex> lck(heightErrIntMtx);
	heightErrInt = val;
}

float FlightController::getHeightErrInt()
{
	std::unique_lock<std::mutex> lck(heightErrIntMtx);
	return heightErrInt;
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
	lck.unlock();
	setPitchErrInt(0.f);
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
	setRollErrInt(0.f);
}

void FlightController::setYawSpPropCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	yawSpPropCoef = value;
}

void FlightController::setYawSpDerCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	yawSpDerCoef = value;
}

void FlightController::setYawSpIntCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	yawSpIntCoef = value;
	setYawSpeedErrInt(0.f);
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
	lck.unlock();
	setHeightErrInt(0.f);
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

void FlightController::setIncFilteringCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	inclineFilteringCoef = value;
}

void FlightController::setIncChangeRateFilteringCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	inclineChangeRateFilteringCoef = value;
}

void FlightController::setYawSpFilteringCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	yawSpeedFilteringCoef = value;
}

void FlightController::setYawSpChangeRateFilteringCoef(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	yawSpeedChangeRateFilteringCoef = value;
}

void FlightController::setOnlyPositiveAdjustMode(bool value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	onlyPositiveAdjustMode = value;
}