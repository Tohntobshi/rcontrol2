#pragma once
#include <stdint.h>
#include <mutex>
#include "infoAdapter.h"

class FlightController
{
private:
	static FlightController * instance;
	FlightController();
	~FlightController();
	void restorePrevMagCalibration();
	void restorePrevGyroCalibration();
	int flightControllerFD = -1;

	bool sendInfo = false;
	bool shouldStopSendInfo = false;
	bool calibratingGyro = false;
	bool calibratingMag = false;

	std::mutex spiInterfaceMutex;
	bool readBytes(uint8_t reg, uint8_t *bytes, uint8_t count);
	bool writeBytes(uint8_t reg, uint8_t *bytes, uint8_t count);

	InfoAdapter * infoAdapter = nullptr;
public:
	// not supposed to be called from different threads simultaneously
	static FlightController * Init();
	static void Destroy();

	void setInfoAdapter(InfoAdapter * infoAdapter);

	void scheduleCalibrateEsc();
	void scheduleCalibrateGyro();
	void scheduleCalibrateMag();

	void move(float, float);
	void setDirection(float);
	void setHeight(float val);
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
	
	void startSendingInfo();
	void stopSendingInfo();

	void setPitchAdjust(float);
	void setRollAdjust(float);
};
