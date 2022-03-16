#pragma once
#include <stdint.h>
#include <mutex>
#include "infoAdapter.h"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <tuple>

class FlightController
{
private:
	static FlightController * instance;
	FlightController();
	~FlightController();
	void restorePrevAccCalibration();
	void restorePrevMagCalibration();
	void restorePrevGyroCalibration();
	int flightControllerFD = -1;

	bool sendSecondaryInfo = false;
	bool shouldStopSendSecondaryInfo = false;

	bool sendPrimaryInfo = false;
	bool shouldStopSendPrimaryInfo = false;

	bool writeSecondaryInfo = false;
	bool shouldStopWritingSecondaryInfo = false;

	bool calibratingAcc = false;
	bool calibratingGyro = false;
	bool calibratingMag = false;
	bool positionControlRunning = false;

	std::mutex spiInterfaceMutex;
	bool readBytes(uint8_t reg, uint8_t *bytes, uint8_t count);
	bool writeBytes(uint8_t reg, uint8_t *bytes, uint8_t count);

	InfoAdapter * infoAdapter = nullptr;

	float currentMoveCommandX = 0.f;
	float currentMoveCommandY = 0.f;
	float positionPropCoef = 0.f;
	float positionDerCoef = 0.f;
	float positionIntCoef = 0.f;
	float positionIntLimit = 0.f;
	float positionFiltering = 0.f;
	float positionDerFiltering = 0.f;

	float positionXErrorOut = 0.f;
	float positionXErrorDerOut = 0.f;
	float positionXErrorIntOut = 0.f;
	float positionYErrorOut = 0.f;
	float positionYErrorDerOut = 0.f;
	float positionYErrorIntOut = 0.f;

	cv::VideoCapture videoCap;
	cv::Mat image;
	cv::Mat oldImageGray;
	cv::Mat imageGray;
	std::vector<cv::Point2f> startCoordinatesFromCenterInMeters;
	std::vector<cv::Point2f> currentCoordinates;
	void initCV();
	void readFrame();
	void writePositionCameraSampleIfNeeded(std::chrono::system_clock::time_point time);
	void findPositionHoldTrackingPoints();
	void updatePositionHoldTrackingPoints();
	std::tuple<float, float> calculateShiftFromInitialPosition();
	std::vector<cv::Point2f> convertCoordinatesToMetersFromCenter(const std::vector<cv::Point2f>& pixelCoordinates);

	std::tuple<float, float> getHeightAndDirection();
	bool needToResetTrackingPoints = true;


	// info from flight controller
	std::mutex fetchInfoMutex;
	std::chrono::time_point<std::chrono::system_clock> fetchTimestamp;
	void fetchControllerInfoIfOld();
	float currentPitchErrorOut = 0.f;
	float pitchErrorChangeRateOut = 0.f;
	float pitchErrIntOut = 0.f;
	float currentRollErrorOut = 0.f;
	float rollErrorChangeRateOut = 0.f;
	float rollErrIntOut = 0.f;
	float currentYawErrorOut = 0.f;
	float yawErrorChangeRateOut = 0.f;
	float yawErrIntOut = 0.f;
	float currentHeightErrorOut = 0.f;
	float heightErrorChangeRateOut = 0.f;
	float heightErrIntOut = 0.f;
	uint16_t frontLeftOut = 0;
	uint16_t frontRightOut = 0;
	uint16_t backLeftOut = 0;
	uint16_t backRightOut = 0;
	float freqOut = 0.f; 
	float voltageOut = 0.f;

	float currentHeight = 0.f;
	float currentDirection = 0.f;

	int positionHoldMode = 1;
	bool needSamplePositionCamera = false;
public:
	// not supposed to be called from different threads simultaneously
	static FlightController * Init();
	static void Destroy();

	void setInfoAdapter(InfoAdapter * infoAdapter);

	void scheduleCalibrateEsc();
	void scheduleCalibrateGyro();
	void scheduleCalibrateMag();
	void scheduleCalibrateAcc();

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
	void setAccFiltering(float val);
	void setAccLPFMode(int val); // from 1 to 6
	void setGyroLPFMode(int val); // from 1 to 6

	void setTurnOffInclineAngle(float val);
	
	void resetTurnOffTrigger();

	void startPositionControl();
	
	void startSendingSecondaryInfo();
	void stopSendingSecondaryInfo();

	void startSendingPrimaryInfo();
	void stopSendingPrimaryInfo();

	void setPitchAdjust(float);
	void setRollAdjust(float);

	void resetLandingFlag();
	void switchToRelativeAcceleration();
	void setRelativeAcceleration(float);

	void setUsHeightFiltering(float);
	void setUsHeightDerFiltering(float);

	void setPitchIntLimit(float val);
	void setRollIntLimit(float val);
	void setYawIntLimit(float val);
	void setHeightIntLimit(float val);

	void setMotorCurveA(float val);
	void setMotorCurveB(float val);
	void setVoltageDropCurveA(float val);
	void setVoltageDropCurveB(float val);
	void setPowerLossCurveA(float val);
	void setPowerLossCurveB(float val);

	void setHeightNegativeIntCoef(float val);

	void setPositionPropCoef(float val);
	void setPositionDerCoef(float val);
	void setPositionIntCoef(float val);
	void setPositionIntLimit(float val);
	void setBarHeightPropCoef(float val);
	void setBarHeightDerCoef(float val);
	void setBarHeightIntCoef(float val);
	void setBarHeightFiltering(float val);
	void setBarHeightDerFiltering(float val);
	void setPositionFiltering(float val);
	void setPositionDerFiltering(float val);
	void setHoldMode(int val);
	void schedulePositionCameraShot();
	void startDataRecording();
	void stopDataRecording();
};
