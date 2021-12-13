#pragma once
#include <stdint.h>
#include "glm/glm.hpp"
#include "bmp280.h"
#include <mutex>
#include "infoAdapter.h"
#include <array>

class FlightController
{
private:
	static FlightController * instance;
	FlightController();
	~FlightController();
	
	void controlAllMotors(uint16_t val);
	void controlAllMotors(uint16_t fl, uint16_t fr, uint16_t bl, uint16_t br);
	void controlAllMotorsAux(uint16_t fl, uint16_t fr, uint16_t bl, uint16_t br);

	std::array<float, 2> calibrateAccel();
	void calibrateGyro();
	void restorePrevGyroCalibration();
	void calibrateMag();
	void restorePrevMagCalibration();

	int accGyroFD = -1;
	int magFD = -1;
	int barFD = -1;
	int ultrasonicFD = -1;
	int auxControllerFD = -1;
	struct bmp280_dev bmp;

	float aRes = 2.0 / 32768.0;
	float gRes = 250.0 / 32768.0;
	float mRes = 4912.0 / 32768.0;
	glm::vec3 magAdjustment;
	glm::vec3 magRange;
	glm::vec3 magMidVals;
	glm::vec3 gyroCalibration;


	static uint8_t readByte(uint8_t device, uint8_t reg);
	static int8_t readBytes(uint8_t device, uint8_t reg, uint8_t * bytes, uint16_t count);
	static void writeByte(uint8_t device, uint8_t reg, uint8_t byte);
	static int8_t writeBytes(uint8_t device, uint8_t reg, uint8_t * bytes, uint16_t count);
	glm::vec3 getAccData();
	glm::vec3 getGyroData();
	glm::vec3 getGyroCalibratedData();
	glm::vec3 getMagData();
	glm::vec3 getMagNormalizedData();
	float getUltrasonicHeightFromSensor();
	float getBarData();

	std::mutex shouldStopMtx;
	bool shouldStop = false;
	void setShouldStop(bool);
	bool getShouldStop();
	
	std::mutex needArmMtx;
	bool needArm = false;
	void setNeedArm(bool);
	bool getNeedArm();
	
	std::mutex needCalibrateMtx;
	bool needCalibrate = false;
	void setNeedCalibrate(bool);
	bool getNeedCalibrate();

	std::mutex needCalibrateGyroMtx;
	bool needCalibrateGyro = false;
	void setNeedCalibrateGyro(bool);
	bool getNeedCalibrateGyro();

	std::mutex needCalibrateAccelMtx;
	bool needCalibrateAccel = false;
	void setNeedCalibrateAccel(bool);
	bool getNeedCalibrateAccel();

	std::mutex needCalibrateMagMtx;
	bool needCalibrateMag = false;
	void setNeedCalibrateMag(bool);
	bool getNeedCalibrateMag();

	std::mutex commonCommandMtx;

	// desired values
	float desiredPitch = 0.f;
	float desiredRoll = 0.f;
	float desiredDirection = 0.f;
	float acceleration = 0.f; // desired height

	float baseAcceleration = 0.f;

	// PID coefficients for pitch
	float pitchPropCoef = 0.f;
	float pitchDerCoef = 0.f;
	float pitchIntCoef = 0.f;

	// PID coefficients for roll
	float rollPropCoef = 0.f;
	float rollDerCoef = 0.f;
	float rollIntCoef = 0.f;

	// PID coefficients for yaw
	float yawPropCoef = 0.f;
	float yawDerCoef = 0.f;
	float yawIntCoef = 0.f;

	// PID coefficients for height
	float heightPropCoef = 0.f;
	float heightDerCoef = 0.f;
	float heightIntCoef = 0.f;

	// filtering
	float accTrust = 0.1f;
	float magTrust = 0.1f;
	int imuLPFMode = 3;

	bool turnOffTrigger = false;
	float turnOffInclineAngle = 30.f;
	bool sendingInfo = false;

	float pitchAdjust = 0.f;
	float rollAdjust = 0.f;

	void setTurnOffTrigger(bool val);
public:
	static FlightController * Init();
	static void Destroy();

	void start(InfoAdapter * infoAdapter);
	void stop();

	void scheduleArm();
	void scheduleCalibrate();
	void scheduleCalibrateGyro();
	void scheduleCalibrateAccel();
	void scheduleCalibrateMag();

	void setDesiredPitchAndRoll(float, float);
	void setDirection(float);
	void setAcceleration(float val);
	void setBaseAcceleration(float val);

	void setPitchPropCoef(float val);
	void setPitchDerCoef(float val);
	void setPitchIntCoef(float val);

	void setRollPropCoef(float val);
	void setRollDerCoef(float val);
	void setRollIntCoef(float val);

	void setYawPropCoef(float val);
	void setYawDerCoef(float val);
	void setYawIntCoef(float val);

	void setHeightPropCoef(float val);
	void setHeightDerCoef(float val);
	void setHeightIntCoef(float val);

	void setAccTrust(float val);
	void setMagTrust(float val);
	void setImuLPFMode(int val); // from 1 to 6

	void setTurnOffInclineAngle(float val);
	
	void resetTurnOffTrigger();
	
	void setSendingInfo(bool val);

	void setPitchAdjust(float);
	void setRollAdjust(float);
};
