
#include "controlsAdapter.h"
#include "messageTypes.h"
#include "utils.h"
#include "iostream"

ControlsAdapter::ControlsAdapter(Connection * conn, FlightController * flightContr)
: connection(conn), flightController(flightContr) {}

void ControlsAdapter::start() {
    setShouldStop(false);
    while (!getShouldStop()) {
		Connection::Message msg = connection->getMessage((uint8_t)MessageTypes::CONTROLS, true);
		if (msg.size == 0) {
			printf("received empty message\n");
			flightController->move(0.f, 0.f);
			continue;
		}
		if (msg.size < 2) {
			printf("received invalid message\n");
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::MOVE && msg.size == 10) {
			float x = Utils::getFloatFromNet(msg.data + 2);
			float y = Utils::getFloatFromNet(msg.data + 6);
			flightController->move(x, y);
			// printf("set pitch and roll %f %f\n", x, y);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_PITCH_PROP_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPitchPropCoef(value);
			// printf("set pitch prop coef %f\n", value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_PITCH_INT_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPitchIntCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_PITCH_DER_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPitchDerCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_ROLL_PROP_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setRollPropCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_ROLL_INT_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setRollIntCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_ROLL_DER_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setRollDerCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_ACC_TRUST && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setAccTrust(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_DIRECTION && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setDirection(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_HEIGHT && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setHeight(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_BASE_ACCELERATION && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setBaseAcceleration(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_HEIGHT_PROP_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setHeightPropCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_HEIGHT_DER_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setHeightDerCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_HEIGHT_INT_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setHeightIntCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_TURN_OFF_INCLINE_ANGLE && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setTurnOffInclineAngle(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::RESET_TURN_OFF_TRIGGER && msg.size == 2) {
			flightController->resetTurnOffTrigger();
			delete[] msg.data;
			continue;
		}

		if (msg.data[1] == (uint8_t)Controls::SET_YAW_PROP_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setYawPropCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_YAW_DER_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setYawDerCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_YAW_INT_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setYawIntCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_IMU_LPF_MODE && msg.size == 6) {
			int value = Utils::getIntFromNet(msg.data + 2);
			flightController->setImuLPFMode(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::START_SENDING_INFO && msg.size == 2) {
			flightController->startSendingInfo();
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::STOP_SENDING_INFO && msg.size == 2) {
			flightController->stopSendingInfo();
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::CALIBRATE_ESC && msg.size == 2) {
			flightController->scheduleCalibrateEsc();
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::CALIBRATE_GYRO && msg.size == 2) {
			flightController->scheduleCalibrateGyro();
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::CALIBRATE_MAG && msg.size == 2) {
			flightController->scheduleCalibrateMag();
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_PITCH_ADJUST && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPitchAdjust(value);
			delete[] msg.data;
			continue;
		}

		if (msg.data[1] == (uint8_t)Controls::SET_ROLL_ADJUST && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setRollAdjust(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_MAG_TRUST && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setMagTrust(value);
			delete[] msg.data;
			continue;
		}
		printf("got message\n");
		delete[] msg.data;
	}
}

void ControlsAdapter::stop() {
    setShouldStop(true);
    connection->stop();
}

void ControlsAdapter::setShouldStop(bool val) {
    std::unique_lock<std::mutex> lck(shouldStopMtx);
    shouldStop = val;
}

bool ControlsAdapter::getShouldStop() {
    std::unique_lock<std::mutex> lck(shouldStopMtx);
    return shouldStop;
}
