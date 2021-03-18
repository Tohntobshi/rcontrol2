
#include "controlsAdapter.h"
#include "messageTypes.h"

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
			uint32_t tmp1 = ntohl(*(uint32_t *)(msg.data + 2));
			uint32_t tmp2 = ntohl(*(uint32_t *)(msg.data + 6));
			float x = *(float *)(&tmp1);
			float y = *(float *)(&tmp2);
			float pitch = std::min(std::max(-1.f, y), 1.f) * -20.f;
			float roll = std::min(std::max(-1.f, x), 1.f) * -20.f;
			flightController->setDesiredPitchAndRoll(pitch, roll);
			// printf("set pitch and yaw %f %f\n", x, y);
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
