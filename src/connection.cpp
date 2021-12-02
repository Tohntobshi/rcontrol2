#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/rfcomm.h>
#include <sys/socket.h>
#include <unistd.h>
#include "connection.h"
#include <thread>
#include "btConstants.h"
#include "poll.h"
#include <cstring>

void Connection::start(BluetoothHelper *btHelper)
{
	if (getIsRunning())
		return;
	setIsRunning(true);
	setShouldStop(false);
	createAbortReadPipe();

	sockaddr_rc loc_addr = {0};
	loc_addr.rc_channel = RFCOMM_CHAN;
	loc_addr.rc_family = AF_BLUETOOTH;
	loc_addr.rc_bdaddr = BDADDR_ANY_VAL;

	int listeningSocket = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
	if (listeningSocket == -1)
	{
		printf("no listening socket\n");
		return;
	}

	int bound = bind(listeningSocket, (sockaddr *)&loc_addr, sizeof(loc_addr));
	if (bound == -1)
	{
		printf("not bound\n");
		return;
	}

	int listening = listen(listeningSocket, 1);
	if (listening == -1)
	{
		printf("not listening\n");
		return;
	}

	while (true)
	{
		std::thread btHelperThread([&]() -> void {
			btHelper->beDiscoverableAndAcceptAnyPairing();
		});
		btHelper->registerService(RFCOMM_CHAN);
		sockaddr_rc rem_addr = {0};
		socklen_t rem_addr_size = sizeof(rem_addr);
		printf("waiting for connection...\n");
		if (isAbortRead(listeningSocket))
		{
			btHelper->abort();
			btHelper->unregisterService();
			btHelperThread.join();
			close(listeningSocket);
			return;
		}
		clientConnectionSocket = accept(listeningSocket, (sockaddr *)&rem_addr, &rem_addr_size);
		setIsConnected(true);
		char string_rem_addr[1024] = {0};
		ba2str(&rem_addr.rc_bdaddr, string_rem_addr);
		printf("accepted connection from %s\n", string_rem_addr);
		btHelper->abort();
		std::thread senderThread([&]() -> void {
			sendMessages();
		});
		receiveMessages();
		printf("connection failed\n");

		std::unique_lock<std::mutex> lck(outgoingQueueMtx);
		outgoingQueue.remove_if([](Message &val) -> bool {
			return val.ignoreWithoutConnection;
		});
		lck.unlock();

		setIsConnected(false);
		close(clientConnectionSocket);
		emptyOutgoingQueueCV.notify_all();
		btHelperThread.join();
		senderThread.join();
		
		btHelper->unregisterService();
		if (getShouldStop())
			break;
	}
	closeAbortReadPipe();
	setIsRunning(false);
}

void Connection::stop()
{
	emitAbortRead();
	setShouldStop(true);
	emptyOutgoingQueueCV.notify_all();
	std::unique_lock<std::mutex> lck(incomingQueueMapsMtx);
	for (auto &pair : emptyIncomingQueueCVs)
	{
		pair.second.notify_all();
	}
}

void Connection::receiveMessages()
{
	while (true)
	{
		if (getShouldStop())
			break;
		uint32_t messageSize;
		if (!receiveP((uint8_t *)&messageSize, 4))
			break;
		messageSize = ntohl(messageSize);
		if (messageSize <= 0 || messageSize > MAX_MESSAGE_SIZE)
		{
			break;
		}
		uint8_t *data2 = new uint8_t[messageSize];
		if (!receiveP(data2, messageSize))
		{
			delete[] data2;
			break;
		}
		uint8_t messageType = data2[0];
		std::unique_lock<std::mutex> lck(incomingQueueMapsMtx);
		auto &thisMessageTypeMtx = incomingQueueMtxs[messageType];
		auto &thisMessageTypeQueue = incomingQueues[messageType];
		auto &thisMessageTypeCV = emptyIncomingQueueCVs[messageType];
		lck.unlock();
		std::unique_lock<std::mutex> lck2(thisMessageTypeMtx);
		thisMessageTypeQueue.push_back({data2, messageSize, false});
		thisMessageTypeCV.notify_all();
	}
}

void Connection::sendMessages()
{
	bool fail = false;
	while (true)
	{
		if (getShouldStop() || !getIsConnected())
			break;
		std::unique_lock<std::mutex> lck(outgoingQueueMtx);
		if (outgoingQueue.empty())
		{
			std::unique_lock<std::mutex> cvlck(emptyOutgoingQueueCVMtx);
			lck.unlock();
			emptyOutgoingQueueCV.wait(cvlck);
			continue;
		}
		Message msg = outgoingQueue.front();
		outgoingQueue.pop_front();
		lck.unlock();
		uint32_t size = htonl(msg.size);
		if (!sendP((uint8_t *)&size, 4))
			fail = true;
		if (fail || !sendP(msg.data, msg.size))
			fail = true;
		delete[] msg.data;
		if (fail)
			break;
	}
}

bool Connection::sendP(uint8_t *data, uint32_t size)
{
	int bytesSent = 0;
	while (bytesSent < size)
	{
		int currentlySent = send(clientConnectionSocket, &(data[bytesSent]), size - bytesSent, 0);
		if (currentlySent < 1)
		{
			return false;
		}
		bytesSent += currentlySent;
	}
	return true;
}

bool Connection::receiveP(uint8_t *data, uint32_t size)
{
	int bytesReceived = 0;
	while (bytesReceived < size)
	{
		if (isAbortRead(clientConnectionSocket))
		{
			return false;
		}
		int currentlyReceived = recv(clientConnectionSocket, &(data[bytesReceived]), size - bytesReceived, 0);
		if (currentlyReceived < 1)
		{
			return false;
		}
		bytesReceived += currentlyReceived;
	}
	return true;
}

void Connection::enqueueToSend(Message msg)
{
	if (msg.ignoreWithoutConnection && !getIsConnected())
		return;
	std::unique_lock<std::mutex> lck(outgoingQueueMtx);
	outgoingQueue.push_back(msg);
	emptyOutgoingQueueCV.notify_all();
}

Connection::Message Connection::getMessage(uint8_t messageType, bool blockIfNoDataYet)
{
	std::unique_lock<std::mutex> lck(incomingQueueMapsMtx);
	auto &thisMessageTypeMtx = incomingQueueMtxs[messageType];
	auto &thisMessageTypeQueue = incomingQueues[messageType];
	auto &thisMessageTypeCVMtx = emptyIncomingQueueCVMtxs[messageType];
	auto &thisMessageTypeCV = emptyIncomingQueueCVs[messageType];
	lck.unlock();
	while (true)
	{
		std::unique_lock<std::mutex> lck2(thisMessageTypeMtx);
		if (thisMessageTypeQueue.empty())
		{
			std::unique_lock<std::mutex> cvlck(thisMessageTypeCVMtx);
			lck2.unlock();
			if (blockIfNoDataYet && !getShouldStop())
			{
				thisMessageTypeCV.wait(cvlck);
				continue;
			}
			return {nullptr, 0};
		}
		Message msg = thisMessageTypeQueue.front();
		thisMessageTypeQueue.pop_front();
		return msg;
	}
}

void Connection::setIsRunning(bool val)
{
	std::unique_lock<std::mutex> lck(isRunningMtx);
	isRunning = val;
}

bool Connection::getIsRunning()
{
	std::unique_lock<std::mutex> lck(isRunningMtx);
	return isRunning;
}

void Connection::setIsConnected(bool val)
{
	std::unique_lock<std::mutex> lck(isConnectedMtx);
	isConnected = val;
}

bool Connection::getIsConnected()
{
	std::unique_lock<std::mutex> lck(isConnectedMtx);
	return isConnected;
}

void Connection::setShouldStop(bool val)
{
	std::unique_lock<std::mutex> lck(shouldStopMtx);
	shouldStop = val;
}

bool Connection::getShouldStop()
{
	std::unique_lock<std::mutex> lck(shouldStopMtx);
	return shouldStop;
}

void Connection::createAbortReadPipe()
{
	std::unique_lock<std::mutex> lck(abortReadPipeMtx);
	pipe(abortReadPipe);
}

void Connection::emitAbortRead()
{
	std::unique_lock<std::mutex> lck(abortReadPipeMtx);
	char a = 0;
	write(abortReadPipe[1], &a, 1);
}

bool Connection::isAbortRead(int fd)
{
	pollfd descriptors[2];
	descriptors[0].fd = fd;
	descriptors[0].events = POLLIN;
	std::unique_lock<std::mutex> lck(abortReadPipeMtx);
	descriptors[1].fd = abortReadPipe[1];
	lck.unlock();
	descriptors[1].events = POLLIN;
	poll(descriptors, 2, -1);
	if (descriptors[1].revents & POLLIN)
	{
		return true;
	}
	return false;
}

void Connection::closeAbortReadPipe()
{
	std::unique_lock<std::mutex> lck(abortReadPipeMtx);
	close(abortReadPipe[0]);
	close(abortReadPipe[1]);
}

Connection::Connection() {}
Connection::~Connection() {}

Connection *Connection::instance = nullptr;

Connection *Connection::Init()
{
	if (instance != nullptr)
	{
		return instance;
	}
	instance = new Connection();
	return instance;
}

void Connection::Destroy()
{
	delete instance;
	instance = nullptr;
}