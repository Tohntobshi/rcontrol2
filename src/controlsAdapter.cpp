
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
		if (msg.data[1] == (uint8_t)Controls::SET_ACC_LPF_MODE && msg.size == 6) {
			int value = Utils::getIntFromNet(msg.data + 2);
			flightController->setAccLPFMode(value);
			#ifdef PRINT_COMMANDS
			printf("set acc lpf mode %d\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_GYRO_LPF_MODE && msg.size == 6) {
			int value = Utils::getIntFromNet(msg.data + 2);
			flightController->setGyroLPFMode(value);
			#ifdef PRINT_COMMANDS
			printf("set gyro lpf mode %d\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::START_SENDING_PRIMARY_INFO && msg.size == 2) {
			flightController->startSendingPrimaryInfo();
			#ifdef PRINT_COMMANDS
			printf("start send primary info\n");
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::STOP_SENDING_PRIMARY_INFO && msg.size == 2) {
			flightController->stopSendingPrimaryInfo();
			#ifdef PRINT_COMMANDS
			printf("stop send primary info\n");
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::START_SENDING_SECONDARY_INFO && msg.size == 2) {
			flightController->startSendingSecondaryInfo();
			#ifdef PRINT_COMMANDS
			printf("start send secondary info\n");
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::STOP_SENDING_SECONDARY_INFO && msg.size == 2) {
			flightController->stopSendingSecondaryInfo();
			#ifdef PRINT_COMMANDS
			printf("stop send secondary info\n");
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
		if (msg.data[1] == (uint8_t)Controls::SET_ACC_FILTERING && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setAccFiltering(value);
			#ifdef PRINT_COMMANDS
			printf("set acc filtering %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::RESET_LANDING_FLAG && msg.size == 2) {
			flightController->resetLandingFlag();
			#ifdef PRINT_COMMANDS
			printf("reset landing flag\n");
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SWITCH_TO_RELATIVE_ACCELERATION && msg.size == 2) {
			flightController->switchToRelativeAcceleration();
			#ifdef PRINT_COMMANDS
			printf("switch to relative acceleration\n");
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_RELATIVE_ACCELERATION && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setRelativeAcceleration(value);
			#ifdef PRINT_COMMANDS
			printf("set relative acceleration %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_US_HEIGHT_FILTERING && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setUsHeightFiltering(value);
			#ifdef PRINT_COMMANDS
			printf("set us height filtering %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_US_HEIGHT_DER_FILTERING && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setUsHeightDerFiltering(value);
			#ifdef PRINT_COMMANDS
			printf("set us height der filtering %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_PITCH_I_LIMIT && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPitchIntLimit(value);
			#ifdef PRINT_COMMANDS
			printf("set pitch i limit %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_ROLL_I_LIMIT && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setRollIntLimit(value);
			#ifdef PRINT_COMMANDS
			printf("set roll i limit %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_YAW_I_LIMIT && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setYawIntLimit(value);
			#ifdef PRINT_COMMANDS
			printf("set yaw i limit %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_HEIGHT_I_LIMIT && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setHeightIntLimit(value);
			#ifdef PRINT_COMMANDS
			printf("set height i limit %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::CALIBRATE_ACC && msg.size == 2) {
			flightController->scheduleCalibrateAcc();
			#ifdef PRINT_COMMANDS
			printf("calibrate acc\n");
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_MOTOR_CURVE_A && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setMotorCurveA(value);
			#ifdef PRINT_COMMANDS
			printf("set motor curve a %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_MOTOR_CURVE_B && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setMotorCurveB(value);
			#ifdef PRINT_COMMANDS
			printf("set motor curve b %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_VOLTAGE_DROP_CURVE_A && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setVoltageDropCurveA(value);
			#ifdef PRINT_COMMANDS
			printf("set voltage drop curve a %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_VOLTAGE_DROP_CURVE_B && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setVoltageDropCurveB(value);
			#ifdef PRINT_COMMANDS
			printf("set voltage drop curve b %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_POWER_LOSS_CURVE_A && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPowerLossCurveA(value);
			#ifdef PRINT_COMMANDS
			printf("set power loss curve a %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_POWER_LOSS_CURVE_B && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPowerLossCurveB(value);
			#ifdef PRINT_COMMANDS
			printf("set power loss curve a %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_HEIGHT_NEGATIVE_INT_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setHeightNegativeIntCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set height negative int coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_POSITION_PROP_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPositionPropCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set position prop coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_POSITION_DER_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPositionDerCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set position der coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_POSITION_INT_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPositionIntCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set position int coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_POSITION_I_LIMIT && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPositionIntLimit(value);
			#ifdef PRINT_COMMANDS
			printf("set position int limit %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_BAR_HEIGHT_PROP_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setBarHeightPropCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set bar height prop coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_BAR_HEIGHT_DER_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setBarHeightDerCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set bar height der coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_BAR_HEIGHT_INT_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setBarHeightIntCoef(value);
			#ifdef PRINT_COMMANDS
			printf("set bar height int coef %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_BAR_HEIGHT_FILTERING && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setBarHeightFiltering(value);
			#ifdef PRINT_COMMANDS
			printf("set bar height filtering %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_BAR_HEIGHT_DER_FILTERING && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setBarHeightDerFiltering(value);
			#ifdef PRINT_COMMANDS
			printf("set bar height der filtering %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_POSITION_FILTERING && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPositionFiltering(value);
			#ifdef PRINT_COMMANDS
			printf("set position filtering %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_POSITION_DER_FILTERING && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPositionDerFiltering(value);
			#ifdef PRINT_COMMANDS
			printf("set position der filtering %f\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::SET_HOLD_MODE && msg.size == 6) {
			int value = Utils::getIntFromNet(msg.data + 2);
			flightController->setHoldMode(value);
			#ifdef PRINT_COMMANDS
			printf("set hold mode %d\n", value);
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::TAKE_POSITION_CAMERA_SHOT && msg.size == 2) {
			flightController->schedulePositionCameraShot();
			#ifdef PRINT_COMMANDS
			printf("schedule position cam shot\n");
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::START_DATA_RECORDING && msg.size == 2) {
			flightController->startDataRecording();
			#ifdef PRINT_COMMANDS
			printf("start data recording\n");
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::STOP_DATA_RECORDING && msg.size == 2) {
			flightController->stopDataRecording();
			#ifdef PRINT_COMMANDS
			printf("stop data recording\n");
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::START_VIDEO_TRANSMISSION && msg.size == 2) {
			flightController->startVideoTransmission();
			#ifdef PRINT_COMMANDS
			printf("start video transmission\n");
			#endif
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == (uint8_t)Controls::STOP_VIDEO_TRANSMISSION && msg.size == 2) {
			flightController->stopVideoTransmission();
			#ifdef PRINT_COMMANDS
			printf("stop video transmission\n");
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
