
#include "controlsAdapter.h"
#include "messageTypes.h"
#include "utils.h"

ControlsAdapter::ControlsAdapter(Connection * conn, FlightController * flightContr)
: connection(conn), flightController(flightContr) {}

void ControlsAdapter::start() {
    setShouldStop(false);
    while (!getShouldStop()) {
		Connection::Message msg = connection->getMessage(MessageTypes::CONTROLS, true);
		if (msg.size == 0) {
			printf("received empty message\n");
			flightController->setDesiredPitchAndRoll(0.f, 0.f);
			continue;
		}
		if (msg.size < 2) {
			printf("received invalid message\n");
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == Controls::SET_PITCH_AND_ROLL && msg.size == 10) {
			float x = Utils::getFloatFromNet(msg.data + 2);
			float y = Utils::getFloatFromNet(msg.data + 6);
			float pitch = std::min(std::max(-1.f, y), 1.f) * -20.f;
			float roll = std::min(std::max(-1.f, x), 1.f) * -20.f;
			flightController->setDesiredPitchAndRoll(pitch, roll);
			// printf("set pitch and roll %f %f\n", x, y);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == Controls::SET_PITCH_PROP_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPitchPropCoef(value);
			// printf("set pitch prop coef %f\n", value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == Controls::SET_PITCH_INT_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPitchIntCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == Controls::SET_PITCH_DER_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setPitchDerCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == Controls::SET_ROLL_PROP_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setRollPropCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == Controls::SET_ROLL_INT_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setRollIntCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == Controls::SET_ROLL_DER_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setRollDerCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == Controls::SET_ACC_TRUST && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setAccTrust(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == Controls::SET_INCL_CH_RATE_FILTERING_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setIncChangeRateFilteringCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == Controls::SET_INCL_FILTERING_COEF && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setIncFilteringCoef(value);
			delete[] msg.data;
			continue;
		}
		if (msg.data[1] == Controls::SET_ACCELERATION && msg.size == 6) {
			float value = Utils::getFloatFromNet(msg.data + 2);
			flightController->setAcceleration(value);
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
