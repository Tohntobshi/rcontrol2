#pragma once
#include "connection.h"
#include "flightController.h"
#include <mutex>

class ControlsAdapter {
private:
    Connection * connection;
    FlightController * flightController;
    std::mutex shouldStopMtx;
    bool shouldStop = false;
    void setShouldStop(bool);
    bool getShouldStop();
public:
    ControlsAdapter(Connection * conn, FlightController * flightContr);
    void start();
    void stop();
};