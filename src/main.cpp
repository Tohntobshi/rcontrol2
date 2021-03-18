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
	std::thread flightThread([&]() -> void {
		flightController->start(&infoAdapter);
	});
	ControlsAdapter controlsAdapter(connection, flightController);
	controlsAdapter.start();

	flightThread.join();
	connThread.join();
	return 0;
}