#include <thread>
#include "flightController.h"
#include "controlsAdapter.h"
#include "connection.h"
#include "bluetoothHelper.h"

int main()
{
	Connection *connection = Connection::Init();
	BluetoothHelper *btHelper = BluetoothHelper::Init();
	FlightController *flightController = FlightController::Init();
	std::thread connThread([&]() -> void {
		connection->start(btHelper);
	});
	InfoAdapter infoAdapter(connection);
	flightController->setInfoAdapter(&infoAdapter);
	flightController->startPositionControl();
	ControlsAdapter controlsAdapter(connection, flightController);
	controlsAdapter.start();
	connThread.join();
	return 0;
}