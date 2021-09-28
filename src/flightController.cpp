#include <pigpio.h>
#include "flightController.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <math.h>

using namespace glm;

#define MPU9250_ADDRESS 0x68
#define AK8963_ADDRESS 0x0C

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

static unsigned int ULTRASONIC_TRIG_PIN = 24;
static unsigned int ULTRASONIC_ECHO_PIN = 23;

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
	gpioSetMode(ULTRASONIC_ECHO_PIN, PI_INPUT);
	gpioSetMode(ULTRASONIC_TRIG_PIN, PI_OUTPUT);
	gpioWrite(ULTRASONIC_TRIG_PIN, 0);

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
	gpioServo(MOTOR_FL_PIN, val);
	gpioServo(MOTOR_FR_PIN, val);
	gpioServo(MOTOR_BL_PIN, val);
	gpioServo(MOTOR_BR_PIN, val);
}

void FlightController::controlAll(uint32_t fl, uint32_t fr, uint32_t bl, uint32_t br)
{
	gpioServo(MOTOR_FL_PIN, fl);
	gpioServo(MOTOR_FR_PIN, fr);
	gpioServo(MOTOR_BL_PIN, bl);
	gpioServo(MOTOR_BR_PIN, br);
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

float FlightController::getUltrasonicHeight()
{
	gpioTrigger(ULTRASONIC_TRIG_PIN, 10, 1);
	int64_t start;
	int64_t stop;
	while (gpioRead(ULTRASONIC_ECHO_PIN) == 0)
	{
		start = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
	}
	while (gpioRead(ULTRASONIC_ECHO_PIN) == 1)
	{
		stop = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
	}
	return ((stop - start) * 343.f) / 2000000.f;
}

void FlightController::start(InfoAdapter * infoAdapter)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	lck.unlock();
	setShouldStop(false);
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
		float acceleration = this->acceleration;
		float desiredPitch = this->desiredPitch;
		float desiredRoll = this->desiredRoll;
		float desiredYawSpeed = this->desiredYawSpeed;
		float pitchPropCoef = this->pitchPropCoef;
		float pitchDerCoef = this->pitchDerCoef;
		float pitchIntCoef = this->pitchIntCoef;
		float rollPropCoef = this->rollPropCoef;
		float rollDerCoef = this->rollDerCoef;
		float rollIntCoef = this->rollIntCoef;
		float yawSpPropCoef = this->yawSpPropCoef;
		float yawSpDerCoef = this->yawSpDerCoef;
		float yawSpIntCoef = this->yawSpIntCoef;
		float pitchBias = this->pitchBias;
		float rollBias = this->rollBias;
		float yawSpeedBias = this->yawSpeedBias;
		float accTrust = this->accTrust;
		float inclineChangeRateFilteringCoef = this->inclineChangeRateFilteringCoef;
		float yawSpeedFilteringCoef = this->yawSpeedFilteringCoef;
		float yawSpeedChangeRateFilteringCoef = this->yawSpeedChangeRateFilteringCoef;
		bool onlyPositiveAdjustMode = this->onlyPositiveAdjustMode;
		lck.unlock();
		
		vec3 acc = normalize(getAccData());
		vec3 gyro = getGyroData();
		vec3 mag = getMagNormalizedData();
		float pressure = getBarData();

		float accPitch = degrees(atan2(acc.x, acc.z)) + pitchBias;
		float accRoll = -degrees(atan2(acc.y, acc.z)) + rollBias;
		float magYaw = degrees(atan2(mag.y, mag.x));

		// std::cout << "yaw " << magYaw << " mag x " << mag.x << " mag y " << mag.y << " mag z " << mag.z << "\n";
		// std::cout << "desired p and r " << desiredPitch << "  " << desiredRoll << "\n";

		int64_t currentTimestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
		float secondsElapsed = (double)(currentTimestamp - prevTimeStamp) / 1000.0;

		float pitchChangeRate = -gyro.y;
		float rollChangeRate = -gyro.x;

		float currentYawSpeed = (gyro.z + yawSpeedBias) * (1.f - yawSpeedFilteringCoef) + prevYawSpeed * yawSpeedFilteringCoef;

		float currentPitch = (accPitch * accTrust + (1.f - accTrust) * (prevPitch + pitchChangeRate * secondsElapsed)) * (1.f - inclineFilteringCoef) + prevPitch * inclineFilteringCoef;
		float currentRoll = (accRoll * accTrust + (1.f - accTrust) * (prevRoll + rollChangeRate * secondsElapsed)) * (1.f - inclineFilteringCoef) + prevRoll * inclineFilteringCoef;

		float currentPitchError = desiredPitch - currentPitch;
		float currentRollError = desiredRoll - currentRoll;
		float currentYawSpeedError = desiredYawSpeed - currentYawSpeed;
		float prevPitchError = desiredPitch - prevPitch;
		float prevRollError = desiredRoll - prevRoll;
		float prevYawSpeedError = desiredYawSpeed - prevYawSpeed;

		float pitchErrorChangeRate = -pitchChangeRate * (1.f - inclineChangeRateFilteringCoef) + prevPitchErrChangeRate * inclineChangeRateFilteringCoef;
		float rollErrorChangeRate = -rollChangeRate * (1.f - inclineChangeRateFilteringCoef) + prevRollErrChangeRate * inclineChangeRateFilteringCoef;
		float yawSpeedErrorChangeRate = ((currentYawSpeedError - prevYawSpeedError) / secondsElapsed) * (1.f - yawSpeedChangeRateFilteringCoef) + prevYawSpeedErrChangeRate * yawSpeedChangeRateFilteringCoef;

		setPitchErrInt(getPitchErrInt() + currentPitchError * secondsElapsed);
		setRollErrInt(getRollErrInt() + currentRollError * secondsElapsed);
		setYawSpeedErrInt(getYawSpeedErrInt() + currentYawSpeedError * secondsElapsed);

		int pitchAdjust = (currentPitchError * pitchPropCoef + pitchErrorChangeRate * pitchDerCoef + getPitchErrInt() * pitchIntCoef) * 10;
		int rollAdjust = (currentRollError * rollPropCoef + rollErrorChangeRate * rollDerCoef + getRollErrInt() * rollIntCoef) * 10;
		int yawAdjust = currentYawSpeedError * yawSpPropCoef + yawSpeedErrorChangeRate * yawSpDerCoef + getYawSpeedErrInt() * yawSpIntCoef;

		float height = getUltrasonicHeight();
		std::cout << "height " << height << "\n";

		int baseVal = (int)(acceleration * 1000.f + 1000.f);
		if (baseVal < MIN_VAL + 300)
		{
			controlAll(baseVal);
			setPitchErrInt(0.f);
			setRollErrInt(0.f);
			setYawSpeedErrInt(0.f);
		}
		else
		{
			if (onlyPositiveAdjustMode)
			{
				int fronLeft = baseVal + (pitchAdjust > 0 ? pitchAdjust : 0) - (rollAdjust < 0 ? rollAdjust : 0) + (yawAdjust > 0 ? yawAdjust : 0);
				int frontRight = baseVal + (pitchAdjust > 0 ? pitchAdjust : 0) + (rollAdjust > 0 ? rollAdjust : 0) - (yawAdjust < 0 ? yawAdjust : 0);
				int backLeft = baseVal - (pitchAdjust < 0 ? pitchAdjust : 0) - (rollAdjust < 0 ? rollAdjust : 0) - (yawAdjust < 0 ? yawAdjust : 0);
				int backRight = baseVal - (pitchAdjust < 0 ? pitchAdjust : 0) + (rollAdjust > 0 ? rollAdjust : 0) + (yawAdjust > 0 ? yawAdjust : 0);
				controlAll(fronLeft, frontRight, backLeft, backRight);
			}
			else
			{
				int fronLeft = baseVal + pitchAdjust - rollAdjust + yawAdjust;
				int frontRight = baseVal + pitchAdjust + rollAdjust - yawAdjust;
				int backLeft = baseVal - pitchAdjust - rollAdjust - yawAdjust;
				int backRight = baseVal - pitchAdjust + rollAdjust + yawAdjust;
				controlAll(fronLeft, frontRight, backLeft, backRight);
			}
		}
		prevPitch = currentPitch;
		prevRoll = currentRoll;
		prevYawSpeed = currentYawSpeed;
		prevTimeStamp = currentTimestamp;
		prevPitchErrChangeRate = pitchErrorChangeRate;
		prevRollErrChangeRate = rollErrorChangeRate;
		prevYawSpeedErrChangeRate = yawSpeedErrorChangeRate;

		infoAdapter->sendInfo(currentPitchError, currentRollError, pitchErrorChangeRate, rollErrorChangeRate, currentYawSpeedError, yawSpeedErrorChangeRate);

		std::this_thread::sleep_for(std::chrono::milliseconds(20)); // TODO make adjustable
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

void FlightController::setPitchBias(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	pitchBias = value;
}

void FlightController::setRollBias(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	rollBias = value;
}

void FlightController::setYawSpeedBias(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	rollBias = value;
}


void FlightController::setAcceleration(float value)
{
	std::unique_lock<std::mutex> lck(commonCommandMtx);
	acceleration = value;
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