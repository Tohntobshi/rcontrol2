#pragma once
#include <dbus-1.0/dbus/dbus.h>
#include <string>
#include <list>
#include <mutex>
#include <bluetooth/sdp.h>
#include <bluetooth/sdp_lib.h>

class BluetoothHelper
{
private:
	static BluetoothHelper *instance;
	BluetoothHelper();
	~BluetoothHelper();

	static void printMessage(DBusMessage *msg);
	static void printRecursively(DBusMessageIter &iter, int depth);
	class RemoteObject
	{
	public:
		std::string serviceName;
		std::string path;
		std::string iface;
	};
	std::list<RemoteObject> adapters;
	std::list<RemoteObject> agentManagers;
	DBusConnection *dbusConn = nullptr;
	void populateRemoteObjects();
	void acquireDBusServiceName();
	void registerAgent();
	void unregisterAgent();
	void beDefaultAgent();
	void handleDBusMessage(DBusMessage *msg);
	void establishDBusConnection();
	void closeDBusConnection();
	void setDiscoverable(bool val);
	bool isRunning = false;
	std::mutex mtx;
	int stopPipe[2];
	sdp_session_t *sdpSession = nullptr;

public:
	static BluetoothHelper *Init();
	static void Destroy();
	/// Name says by itself
	void beDiscoverableAndAcceptAnyPairing();
	/// Stop being discoverable and waiting for pairing. May be called from another thread
	void abort();
	/// Register service in Service Discovery Proptocol
	void registerService(uint8_t rfcommChannel);
	/// Unregister service from Service Discovery Proptocol
	void unregisterService();
};
