
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
			#ifdef PRINT_COMMANDS
			printf("received empty message\n");
			#endif
			flightController->move(0.f, 0.f);
			continue;
		}
		if (msg.size < 2) {
			#ifdef PRINT_COMMANDS
			printf("received invalid message\n");
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::MOVE && msg.size == 10) {
			float x = Utils::getFloatFromNet(msg.data + 2);
			float y = Utils::getFloatFromNet(msg.data + 6);
			flightController->move(x, y);
			#ifdef PRINT_COMMANDS
			printf("move %f %f\n", x, y);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_PITCH_PROP_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPitchPropCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set pitch prop coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_PITCH_INT_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPitchIntCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set pitch int coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_PITCH_DER_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPitchDerCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set pitch der coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_ROLL_PROP_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setRollPropCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set roll prop coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_ROLL_INT_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setRollIntCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set roll int coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_ROLL_DER_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setRollDerCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set roll der coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_ACC_TRUST && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setAccTrust(value);
			#ifdef PRINT_COMMANDS
			printf("set acc trust %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_DIRECTION && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setDirection(value);
			#ifdef PRINT_COMMANDS
			printf("set direction %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_HEIGHT && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setHeight(value);
			#ifdef PRINT_COMMANDS
			printf("set height %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_BASE_ACCELERATION && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setBaseAcceleration(value);
			#ifdef PRINT_COMMANDS
			printf("set base accel %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_HEIGHT_PROP_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setHeightPropCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set height prop coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_HEIGHT_DER_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setHeightDerCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set height der coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_HEIGHT_INT_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setHeightIntCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set height int coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_TURN_OFF_INCLINE_ANGLE && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setTurnOffInclineAngle(value);
			#ifdef PRINT_COMMANDS
			printf("set turn off angle %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::RESET_TURN_OFF_TRIGGER && msg.size == 2) {
			flightController->resetTurnOffTrigger();
			#ifdef PRINT_COMMANDS
			printf("reset turn off\n");
			#endif
			delete[] msg.data;
			continue;
		}

		if (msg.data[1] == (uint8_t)Controls::SET_YAW_PROP_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setYawPropCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set yaw prop coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_YAW_DER_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setYawDerCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set yaw der coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_YAW_INT_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setYawIntCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set yaw int coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_IMU_LPF_MODE && msg.size == 6) {
			int value = Utils::getIntFromNet(msg.data + 2);
			flightController->setImuLPFMode(value);
			#ifdef PRINT_COMMANDS
			printf("set imu lpf mode %d\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::START_SENDING_INFO && msg.size == 2) {
			flightController->startSendingInfo();
			#ifdef PRINT_COMMANDS
			printf("start send info\n");
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::STOP_SENDING_INFO && msg.size == 2) {
			flightController->stopSendingInfo();
			#ifdef PRINT_COMMANDS
			printf("stop send info\n");
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::CALIBRATE_ESC && msg.size == 2) {
			flightController->scheduleCalibrateEsc();
			#ifdef PRINT_COMMANDS
			printf("calibrate esc\n");
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::CALIBRATE_GYRO && msg.size == 2) {
			flightController->scheduleCalibrateGyro();
			#ifdef PRINT_COMMANDS
			printf("calibrate gyro\n");
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::CALIBRATE_MAG && msg.size == 2) {
			flightController->scheduleCalibrateMag();
			#ifdef PRINT_COMMANDS
			printf("calibrate mag\n");
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_PITCH_ADJUST && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPitchAdjust(value);
			#ifdef PRINT_COMMANDS
			printf("set pitch adjust %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}

		if (msg.data[1] == (uint8_t)Controls::SET_ROLL_ADJUST && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setRollAdjust(value);
			#ifdef PRINT_COMMANDS
			printf("set roll adjust %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_MAG_TRUST && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setMagTrust(value);
			#ifdef PRINT_COMMANDS
			printf("set mag trust %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		#ifdef PRINT_COMMANDS
		printf("got message\n");
		#endif
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
