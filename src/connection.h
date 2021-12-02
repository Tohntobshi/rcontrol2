#pragma once
#include <functional>
#include <list>
#include <mutex>
#include <map>
#include <condition_variable>
#include "bluetoothHelper.h"

#define RFCOMM_CHAN 11
#define MAX_MESSAGE_SIZE 5000

class Connection
{
public:
	static Connection *Init();
	static void Destroy();
	class Message
	{
	public:
		uint8_t *data;
		unsigned int size;
		bool ignoreWithoutConnection;
	};
	/// will block untill stopped from another thread
	void start(BluetoothHelper *btHelper);
	/// stops connection, may be called from another thread
	void stop();
	/**
	 *  get message by type, thread safe
	 *  @param messageType first byte of message data which is used to sort messages
	 *  @param blockIfNoDataYet if true call will block until there's data to return,
	 *  may return 0 sized Message even if parameter is true
	 *  @returns Message, you should delete[] data yourself after use
	 */
	Message getMessage(uint8_t messageType, bool blockIfNoDataYet);
	/**
	 *  will place messsage to the send queue, thread safe
	 *  first byte of message data is used to sort messages
	 *  @param msg message to send, don't do any operations with its data after this call
	 *  @param ignoreWithoutConnection if true message will be igrored and deleted if no connection
	 */
	void enqueueToSend(Message msg);

private:
	int clientConnectionSocket = 0;

	void receiveMessages();
	void sendMessages();
	bool sendP(uint8_t *data, uint32_t size);
	bool receiveP(uint8_t *data, uint32_t size);

	std::mutex isRunningMtx;
	bool isRunning = false;
	void setIsRunning(bool val);
	bool getIsRunning();

	std::mutex isConnectedMtx;
	bool isConnected = false;
	void setIsConnected(bool val);
	bool getIsConnected();

	std::mutex shouldStopMtx;
	bool shouldStop = false;
	void setShouldStop(bool val);
	bool getShouldStop();

	std::mutex abortReadPipeMtx;
	int abortReadPipe[2];
	void createAbortReadPipe();
	void emitAbortRead();
	bool isAbortRead(int fd);
	void closeAbortReadPipe();

	std::mutex emptyOutgoingQueueCVMtx;
	std::condition_variable emptyOutgoingQueueCV;
	std::mutex outgoingQueueMtx;
	std::list<Message> outgoingQueue;

	std::mutex incomingQueueMapsMtx;
	std::map<uint8_t, std::mutex> emptyIncomingQueueCVMtxs;
	std::map<uint8_t, std::condition_variable> emptyIncomingQueueCVs;
	std::map<uint8_t, std::mutex> incomingQueueMtxs;
	std::map<uint8_t, std::list<Message>> incomingQueues;

	static Connection *instance;
	Connection();
	~Connection();
};
