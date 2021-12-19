#include <pigpio.h>
#include "flightController.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <math.h>
#include <algorithm>
#include "utils.h"
#include <fstream>
#include "messageTypes.h"

#define AUX_READY_PIN 25

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
	if (gpioInitialise() < 0)
	{
		std::cout << "cant init gpio\n";
	}

	flightControllerFD = spiOpen(0, 1000000, 0);
	if (flightControllerFD < 0)
	{
		std::cout << "cant open flight controller\n";
	}
	gpioSetMode(AUX_READY_PIN, PI_INPUT);
	gpioSetPullUpDown(AUX_READY_PIN, PI_PUD_DOWN);
	restorePrevGyroCalibration();
	restorePrevMagCalibration();
}

FlightController::~FlightController()
{
	gpioTerminate();
}

bool FlightController::readBytes(uint8_t reg, uint8_t * dest, uint8_t count)
{
	uint8_t data[count + 1];
	std::unique_lock<std::mutex> lck(spiInterfaceMutex);
	uint8_t registerToRead[1] = { reg | 0b10000000 }; // register + read flag
	while(!gpioRead(AUX_READY_PIN)) {}
	spiXfer(flightControllerFD, (char *)registerToRead, nullptr, 1);
	std::this_thread::sleep_for(std::chrono::microseconds(10));
	while(!gpioRead(AUX_READY_PIN)) {}
	spiXfer(flightControllerFD, nullptr, (char *)data, count + 1);
	uint8_t crc = reg;
	for (int i = 0; i < count; i++)
	{
		crc = crc ^ data[i];
		dest[i] = data[i];
	}
	std::this_thread::sleep_for(std::chrono::microseconds(10));
	return data[count] == crc;
}

void FlightController::writeBytes(uint8_t reg, uint8_t * bytes, uint8_t count)
{
	uint8_t crc = reg;
	uint8_t data[count + 1];
	for (int i = 0; i < count; i++)
	{
		crc = crc ^ bytes[i];
		data[i] = bytes[i];
	}
	data[count] = crc;
	
	std::unique_lock<std::mutex> lck(spiInterfaceMutex);
	uint8_t registerToWrite[1] = { reg };
	while(!gpioRead(AUX_READY_PIN)) {}
	spiXfer(flightControllerFD, (char *)registerToWrite, nullptr, 1);
	std::this_thread::sleep_for(std::chrono::microseconds(10));
	while(!gpioRead(AUX_READY_PIN)) {}
	spiXfer(flightControllerFD, (char *)data, nullptr, count + 1);
	std::this_thread::sleep_for(std::chrono::microseconds(10));
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
	// std::cout << "gonna send gyro calib " << x << " " << y << " " << z << "\n";
	uint8_t data[12];
	Utils::setFloatToNet(x, data);
	Utils::setFloatToNet(y, data + 4);
	Utils::setFloatToNet(z, data + 8);
	writeBytes((uint8_t)FlightControllerRegisters::SET_GYRO_CALIBRATION, data, 12);
}

void FlightController::restorePrevMagCalibration()
{
	std::ifstream file;
	file.open("/home/pi/magcalibration");
	if (!file.is_open()) return;
	std::string line;
	std::getline(file, line);
	file.close();
	float midX;
	float midY;
	float midZ;
	float mRangeX;
	float mRangeY;
	float mRangeZ;
	sscanf(line.c_str(), "%f,%f,%f,%f,%f,%f", &midX, &midY, &midZ, &mRangeX, &mRangeY, &mRangeZ);
	// std::cout << "gonna send mag calib mid " << midX << " " << midY << " " << midZ << " range " << mRangeX << " " << mRangeY << " " << mRangeZ << "\n";
	uint8_t data[24];
	Utils::setFloatToNet(midX, data);
	Utils::setFloatToNet(midY, data + 4);
	Utils::setFloatToNet(midZ, data + 8);
	Utils::setFloatToNet(mRangeX, data + 12);
	Utils::setFloatToNet(mRangeY, data + 16);
	Utils::setFloatToNet(mRangeZ, data + 20);
	writeBytes((uint8_t)FlightControllerRegisters::SET_MAG_CALIBRATION, data, 24);
}

void FlightController::setInfoAdapter(InfoAdapter * adapter)
{
	infoAdapter = adapter;
}

void FlightController::startSendingInfo()
{
	if (sendInfo) return;
	sendInfo = true;
	shouldStopSendInfo = false;
	std::thread thread([&]() -> void {
		while (!shouldStopSendInfo && infoAdapter)
		{
			uint8_t pitchAndRollInfo[24];
			if (!readBytes((uint8_t)FlightControllerRegisters::GET_PITCH_AND_ROLL_INFO, pitchAndRollInfo, 24)) continue;
			uint8_t yawAndHeightInfo[24];
			if (!readBytes((uint8_t)FlightControllerRegisters::GET_YAW_AND_HEIGHT_INFO, yawAndHeightInfo, 24)) continue;
			uint8_t motorValsAndFreq[12];
			if (!readBytes((uint8_t)FlightControllerRegisters::GET_MOTOR_VALS_AND_FREQ, motorValsAndFreq, 12)) continue;

			float currentPitchError = Utils::getFloatFromNet(pitchAndRollInfo);
			float pitchErrorChangeRate = Utils::getFloatFromNet(pitchAndRollInfo + 4);
			float pitchErrInt = Utils::getFloatFromNet(pitchAndRollInfo + 8);
			float currentRollError = Utils::getFloatFromNet(pitchAndRollInfo + 12);
			float rollErrorChangeRate = Utils::getFloatFromNet(pitchAndRollInfo + 16);
			float rollErrInt = Utils::getFloatFromNet(pitchAndRollInfo + 20);

			float currentYawError = Utils::getFloatFromNet(yawAndHeightInfo);
			float yawErrorChangeRate = Utils::getFloatFromNet(yawAndHeightInfo + 4);
			float yawErrInt = Utils::getFloatFromNet(yawAndHeightInfo + 8);
			float currentHeightError = Utils::getFloatFromNet(yawAndHeightInfo + 12);
			float heightErrorChangeRate = Utils::getFloatFromNet(yawAndHeightInfo + 16);
			float heightErrInt = Utils::getFloatFromNet(yawAndHeightInfo + 20);

			uint16_t frontLeft = Utils::getShortFromNet(motorValsAndFreq);
			uint16_t frontRight = Utils::getShortFromNet(motorValsAndFreq + 2);
			uint16_t backLeft = Utils::getShortFromNet(motorValsAndFreq + 4);
			uint16_t backRight = Utils::getShortFromNet(motorValsAndFreq + 6);
			float freq = Utils::getFloatFromNet(motorValsAndFreq + 8);

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
				freq,
				pitchErrInt,
				rollErrInt,
				yawErrInt,
				heightErrInt
			);
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
		sendInfo = false;
	});
	thread.detach();
}

void FlightController::stopSendingInfo()
{
	shouldStopSendInfo = true;
}

void FlightController::scheduleCalibrateEsc()
{
	uint8_t data = 0;
	writeBytes((uint8_t)FlightControllerRegisters::CALIBRATE_ESC, &data, 1);
}

void FlightController::scheduleCalibrateGyro()
{
	if (calibratingGyro) return;
	calibratingGyro = true;
	std::thread thread([&]() -> void {
		uint8_t data = 0;
		writeBytes((uint8_t)FlightControllerRegisters::CALIBRATE_GYRO, &data, 1);
		std::this_thread::sleep_for(std::chrono::seconds(10));
		uint8_t calibData[12];

		while (!readBytes((uint8_t)FlightControllerRegisters::GET_GYRO_CALIBRATION, calibData, 12)) {}
		
		float x = Utils::getFloatFromNet(calibData);
		float y = Utils::getFloatFromNet(calibData + 4);
		float z = Utils::getFloatFromNet(calibData + 8);
		// std::cout << "got gyro calib " << x << " " << y << " " << z << "\n";
		std::ofstream file;
		file.open("/home/pi/gyrocalibration", std::ofstream::out | std::ofstream::trunc);
		file << x << "," << y << "," << z << "\n";
		file.close();
		calibratingGyro = false;
	});
	thread.detach();
}


void FlightController::scheduleCalibrateMag()
{
	if (calibratingMag) return;
	calibratingMag = true;
	std::thread thread([&]() -> void {
		uint8_t data = 0;
		writeBytes((uint8_t)FlightControllerRegisters::CALIBRATE_MAG, &data, 1);
		std::this_thread::sleep_for(std::chrono::seconds(35));
		uint8_t calibData[24];
		while (!readBytes((uint8_t)FlightControllerRegisters::GET_MAG_CALIBRATION, calibData, 24)) {};
		float midx = Utils::getFloatFromNet(calibData);
		float midy = Utils::getFloatFromNet(calibData + 4);
		float midz = Utils::getFloatFromNet(calibData + 8);
		float rangex = Utils::getFloatFromNet(calibData + 12);
		float rangey = Utils::getFloatFromNet(calibData + 16);
		float rangez = Utils::getFloatFromNet(calibData + 20);
		// std::cout << "got mag calib mid " << midx << " " << midy << " " << midz << " range " << rangex << " " << rangey << " " << rangez << "\n";
		std::ofstream file;
		file.open("/home/pi/magcalibration", std::ofstream::out | std::ofstream::trunc);
		file << midx << "," << midy << "," << midz << "," << rangex << "," << rangey << "," << rangez << "\n";
		file.close();
		calibratingMag = false;
	});
	thread.detach();
}


void FlightController::setDesiredPitchAndRoll(float pitch, float roll)
{
	uint8_t data[8];
	Utils::setFloatToNet(pitch, data);
	Utils::setFloatToNet(roll, data + 4);
	writeBytes((uint8_t)FlightControllerRegisters::SET_PITCH_AND_ROLL, data, 8);
}

void FlightController::setPitchPropCoef(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_PITCH_PROP_COEF, data, 4);
}

void FlightController::setPitchDerCoef(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_PITCH_DER_COEF, data, 4);
}

void FlightController::setPitchIntCoef(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_PITCH_INT_COEF, data, 4);
}

void FlightController::setRollPropCoef(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_ROLL_PROP_COEF, data, 4);
}

void FlightController::setRollDerCoef(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_ROLL_DER_COEF, data, 4);
}

void FlightController::setRollIntCoef(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_ROLL_INT_COEF, data, 4);
}

void FlightController::setYawPropCoef(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_YAW_PROP_COEF, data, 4);
}

void FlightController::setYawDerCoef(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_YAW_DER_COEF, data, 4);
}

void FlightController::setYawIntCoef(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_YAW_INT_COEF, data, 4);
}

void FlightController::setHeightPropCoef(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_HEIGHT_PROP_COEF, data, 4);
}

void FlightController::setHeightDerCoef(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_HEIGHT_DER_COEF, data, 4);
}

void FlightController::setHeightIntCoef(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_HEIGHT_INT_COEF, data, 4);
}


void FlightController::setHeight(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_HEIGHT, data, 4);
}

void FlightController::setBaseAcceleration(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_BASE_ACCELERATION, data, 4);
}

void FlightController::setAccTrust(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_ACC_TRUST, data, 4);
}

void FlightController::setMagTrust(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_MAG_TRUST, data, 4);
}

void FlightController::setTurnOffInclineAngle(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_TURN_OFF_INCLINE_ANGLE, data, 4);
}


void FlightController::resetTurnOffTrigger()
{
	uint8_t data = 0;
	writeBytes((uint8_t)FlightControllerRegisters::RESET_TURN_OFF_TRIGGER, &data, 1);
}

void FlightController::setImuLPFMode(int value)
{
	uint8_t mode = std::min(6, std::max(1, value));
	writeBytes((uint8_t)FlightControllerRegisters::SET_IMU_LPF_MODE, &mode, 1);
}

void FlightController::setPitchAdjust(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_PITCH_ADJUST, data, 4);
}

void FlightController::setRollAdjust(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_ROLL_ADJUST, data, 4);
}

void FlightController::setDirection(float value)
{
	uint8_t data[4];
	Utils::setFloatToNet(value, data);
	writeBytes((uint8_t)FlightControllerRegisters::SET_DIRECTION, data, 4);
}