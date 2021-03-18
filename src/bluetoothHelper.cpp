#include "bluetoothHelper.h"
#include "btConstants.h"
#include <cstring>
#include <unistd.h>
#include <poll.h>

void BluetoothHelper::printRecursively(DBusMessageIter &iter, int depth)
{
	while (true)
	{
		int argType = dbus_message_iter_get_arg_type(&iter);
		if (argType == DBUS_TYPE_INVALID)
			break;
		int i = 0;
		while (i < depth)
		{
			printf("|  ");
			i++;
		}
		if (argType == DBUS_TYPE_STRING || argType == DBUS_TYPE_OBJECT_PATH)
		{
			char *value;
			dbus_message_iter_get_basic(&iter, &value);
			printf("val: %s\n", value);
		}
		else if (argType == DBUS_TYPE_ARRAY)
		{
			printf("arr:\n");
			DBusMessageIter iter2;
			dbus_message_iter_recurse(&iter, &iter2);
			printRecursively(iter2, depth + 1);
		}
		else if (argType == DBUS_TYPE_DICT_ENTRY)
		{
			printf("pair:\n");
			DBusMessageIter iter2;
			dbus_message_iter_recurse(&iter, &iter2);
			printRecursively(iter2, depth + 1);
		}
		else if (argType == DBUS_TYPE_VARIANT)
		{
			printf("variant:\n");
			DBusMessageIter iter2;
			dbus_message_iter_recurse(&iter, &iter2);
			printRecursively(iter2, depth + 1);
		}
		else
		{
			switch (argType)
			{
			case DBUS_TYPE_DOUBLE:
				printf("val type double\n");
				break;
			case DBUS_TYPE_BOOLEAN:
				printf("val type boolean\n");
				break;
			case DBUS_TYPE_BYTE:
				printf("val type byte\n");
				break;
			case DBUS_TYPE_STRUCT:
				printf("val type struct\n");
				break;
			case DBUS_TYPE_INT16:
			case DBUS_TYPE_UINT16:
			case DBUS_TYPE_INT32:
			case DBUS_TYPE_UINT32:
			case DBUS_TYPE_INT64:
			case DBUS_TYPE_UINT64:
				printf("val type int\n");
				break;
			case DBUS_TYPE_SIGNATURE:
				printf("val type sugnature\n");
				break;
			case DBUS_TYPE_UNIX_FD:
				printf("val type unix fd\n");
				break;
			default:
				printf("val type other\n");
				break;
			}
		}
		dbus_message_iter_next(&iter);
	}
}

void BluetoothHelper::printMessage(DBusMessage *message)
{
	int type = dbus_message_get_type(message);
	const char *memberName = dbus_message_get_member(message);
	const char *ifaceName = dbus_message_get_interface(message);
	switch (type)
	{
	case DBUS_MESSAGE_TYPE_METHOD_CALL:
		printf("received DBUS_MESSAGE_TYPE_METHOD_CALL %s %s\n", ifaceName ? ifaceName : "no iface", memberName ? memberName : "no name");
		break;
	case DBUS_MESSAGE_TYPE_METHOD_RETURN:
		printf("received DBUS_MESSAGE_TYPE_METHOD_RETURN\n");
		break;
	case DBUS_MESSAGE_TYPE_ERROR:
		printf("received DBUS_MESSAGE_TYPE_ERROR\n");
		break;
	case DBUS_MESSAGE_TYPE_SIGNAL:
		printf("received DBUS_MESSAGE_TYPE_SIGNAL %s %s\n", ifaceName ? ifaceName : "no iface", memberName ? memberName : "no name");
		break;
	default:
		printf("received another message\n");
		break;
	}
	DBusMessageIter iter;
	if (!dbus_message_iter_init(message, &iter))
	{
		printf("no args to print\n");
	}
	printRecursively(iter, 0);
}

void BluetoothHelper::populateRemoteObjects()
{
	DBusMessage *msg = dbus_message_new_method_call(
		"org.bluez", "/", "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
	// dbus_message_append_args(msg, DBUS_TYPE_INVALID);  // ????
	DBusMessage *reply = dbus_connection_send_with_reply_and_block(
		dbusConn, msg, -1, nullptr);
	dbus_message_unref(msg);
	if (!reply)
	{
		printf("populateRemoteObjects failed\n");
		return;
	}
	DBusMessageIter iter;
	if (!dbus_message_iter_init(reply, &iter))
	{
		printf("populateRemoteObjects failed\n");
		dbus_message_unref(reply);
		return;
	}
	if (dbus_message_iter_get_arg_type(&iter) != DBUS_TYPE_ARRAY)
	{
		printf("populateRemoteObjects failed\n");
		dbus_message_unref(reply);
		return;
	}
	DBusMessageIter iter2;
	dbus_message_iter_recurse(&iter, &iter2);
	while (dbus_message_iter_get_arg_type(&iter2) == DBUS_TYPE_DICT_ENTRY)
	{
		DBusMessageIter iter3;
		dbus_message_iter_recurse(&iter2, &iter3);
		if (dbus_message_iter_get_arg_type(&iter3) != DBUS_TYPE_OBJECT_PATH)
		{
			dbus_message_iter_next(&iter2);
			continue;
		}
		char *currentObjPath = nullptr;
		dbus_message_iter_get_basic(&iter3, &currentObjPath);
		dbus_message_iter_next(&iter3);
		if (dbus_message_iter_get_arg_type(&iter3) != DBUS_TYPE_ARRAY)
		{
			dbus_message_iter_next(&iter2);
			continue;
		}
		DBusMessageIter iter4;
		dbus_message_iter_recurse(&iter3, &iter4);
		while (dbus_message_iter_get_arg_type(&iter4) == DBUS_TYPE_DICT_ENTRY)
		{
			DBusMessageIter iter5;
			dbus_message_iter_recurse(&iter4, &iter5);
			if (dbus_message_iter_get_arg_type(&iter5) != DBUS_TYPE_STRING)
			{
				dbus_message_iter_next(&iter4);
				continue;
			}
			char *currentIface = nullptr;
			dbus_message_iter_get_basic(&iter5, &currentIface);
			if (strcmp(currentIface, "org.bluez.AgentManager1") == 0)
			{
				agentManagers.push_back({"org.bluez", currentObjPath, currentIface});
				// printf("found agent manager %s %s\n", currentObjPath, currentIface);
			}
			if (strcmp(currentIface, "org.bluez.Adapter1") == 0)
			{
				adapters.push_back({"org.bluez", currentObjPath, currentIface});
				// printf("found adapter %s %s\n", currentObjPath, currentIface);
			}
			dbus_message_iter_next(&iter4);
		}
		dbus_message_iter_next(&iter2);
	}
	dbus_message_unref(reply);
}

void BluetoothHelper::acquireDBusServiceName()
{
	int res = dbus_bus_request_name(
		dbusConn, "org.example.MyExample",
		DBUS_NAME_FLAG_REPLACE_EXISTING | DBUS_NAME_FLAG_ALLOW_REPLACEMENT, nullptr);
	if (res == DBUS_REQUEST_NAME_REPLY_PRIMARY_OWNER)
	{
		printf("got name\n");
	}
	else
	{
		printf("cant get name\n");
	}
}

void BluetoothHelper::registerAgent()
{
	if (agentManagers.empty())
	{
		printf("cant register agent, no agent managers\n");
		return;
	}
	auto agentManager = agentManagers.front();
	DBusMessage *msg = dbus_message_new_method_call(
		agentManager.serviceName.c_str(), agentManager.path.c_str(),
		agentManager.iface.c_str(), "RegisterAgent");
	DBusMessageIter iter;
	dbus_message_iter_init_append(msg, &iter);
	const char *path = "/";
	const char *capabilities = "DisplayYesNo";
	dbus_message_iter_append_basic(&iter, DBUS_TYPE_OBJECT_PATH, &path);
	dbus_message_iter_append_basic(&iter, DBUS_TYPE_STRING, &capabilities);
	DBusMessage *reply = dbus_connection_send_with_reply_and_block(
		dbusConn, msg, -1, nullptr);
	if (dbus_message_get_error_name(reply) != nullptr)
	{
		printf("can't register agent, got error\n");
	}
	else
	{
		printf("registered agent\n");
	}
	dbus_message_unref(msg);
	dbus_message_unref(reply);
}

void BluetoothHelper::beDefaultAgent()
{
	if (agentManagers.empty())
	{
		printf("cant be default agent, no agent managers\n");
		return;
	}
	auto agentManager = agentManagers.front();
	DBusMessage *msg = dbus_message_new_method_call(
		agentManager.serviceName.c_str(), agentManager.path.c_str(),
		agentManager.iface.c_str(), "RequestDefaultAgent");
	DBusMessageIter iter;
	dbus_message_iter_init_append(msg, &iter);
	const char *path = "/";
	dbus_message_iter_append_basic(&iter, DBUS_TYPE_OBJECT_PATH, &path);
	DBusMessage *reply = dbus_connection_send_with_reply_and_block(
		dbusConn, msg, -1, nullptr);
	if (dbus_message_get_error_name(reply) != nullptr)
	{
		printf("can't be default agent, got error\n");
	}
	else
	{
		printf("became default agent\n");
	}
	dbus_message_unref(msg);
	dbus_message_unref(reply);
}

void BluetoothHelper::unregisterAgent()
{
	if (agentManagers.empty())
	{
		printf("cant unregister agent, no agent managers\n");
		return;
	}
	auto agentManager = agentManagers.front();
	DBusMessage *msg = dbus_message_new_method_call(
		agentManager.serviceName.c_str(), agentManager.path.c_str(),
		agentManager.iface.c_str(), "UnregisterAgent");
	DBusMessageIter iter;
	dbus_message_iter_init_append(msg, &iter);
	const char *path = "/";
	dbus_message_iter_append_basic(&iter, DBUS_TYPE_OBJECT_PATH, &path);
	DBusMessage *reply = dbus_connection_send_with_reply_and_block(
		dbusConn, msg, -1, nullptr);
	if (dbus_message_get_error_name(reply) != nullptr)
	{
		printf("can't unregister agent, got error\n");
	}
	else
	{
		printf("unregistered agent\n");
	}
	dbus_message_unref(msg);
	dbus_message_unref(reply);
}

void BluetoothHelper::handleDBusMessage(DBusMessage *msg)
{
	printMessage(msg);
	if (
		dbus_message_is_method_call(msg, "org.bluez.Agent1", "RequestConfirmation") ||
		dbus_message_is_method_call(msg, "org.bluez.Agent1", "AuthorizeService"))
	{
		DBusMessage *reply = dbus_message_new_method_return(msg);
		dbus_connection_send(dbusConn, reply, nullptr);
		dbus_connection_flush(dbusConn);
		dbus_message_unref(reply);
	}
}

void BluetoothHelper::establishDBusConnection()
{
	dbusConn = dbus_bus_get(DBUS_BUS_SYSTEM, nullptr);
}

void BluetoothHelper::closeDBusConnection()
{
	dbus_connection_unref(dbusConn);
}

void BluetoothHelper::setDiscoverable(bool val)
{
	if (adapters.empty())
	{
		printf("can't make discoverable, no adapters\n");
		return;
	}
	auto adapter = adapters.front();
	DBusMessage *msg = dbus_message_new_method_call(
		adapter.serviceName.c_str(), adapter.path.c_str(), DBUS_INTERFACE_PROPERTIES, "Set");
	DBusMessageIter iter1;
	dbus_message_iter_init_append(msg, &iter1);
	const char *iface = adapter.iface.c_str();
	const char *property = "Discoverable";
	dbus_message_iter_append_basic(&iter1, DBUS_TYPE_STRING, &iface);
	dbus_message_iter_append_basic(&iter1, DBUS_TYPE_STRING, &property);

	DBusMessageIter iter2;
	dbus_message_iter_open_container(&iter1, DBUS_TYPE_VARIANT, DBUS_TYPE_BOOLEAN_AS_STRING, &iter2);
	int value = (int)val;
	dbus_message_iter_append_basic(&iter2, DBUS_TYPE_BOOLEAN, &value);
	dbus_message_iter_close_container(&iter1, &iter2);
	DBusMessage *reply = dbus_connection_send_with_reply_and_block(dbusConn, msg, -1, nullptr);
	if (dbus_message_get_error_name(reply) != nullptr)
	{
		printf("can't set discoverable, returned failed result\n");
	}
	else
	{
		printf("set discoverable %d\n", val);
	}
	dbus_message_unref(msg);
	dbus_message_unref(reply);
}

void BluetoothHelper::beDiscoverableAndAcceptAnyPairing()
{
	std::unique_lock<std::mutex> lck(mtx);
	if (isRunning)
		return;
	isRunning = true;
	establishDBusConnection();
	// acquireDBusServiceName();
	populateRemoteObjects();
	setDiscoverable(true);
	registerAgent();
	beDefaultAgent();
	pipe(stopPipe);
	lck.unlock();
	while (true)
	{
		DBusMessage *msg = dbus_connection_pop_message(dbusConn);
		if (msg == nullptr)
		{
			printf("message is null, calling readwrite\n");
			int fd = 0;
			dbus_connection_get_unix_fd(dbusConn, &fd);
			pollfd pollfds[2] = {{stopPipe[0], POLLIN, 0}, {fd, POLLIN, 0}};
			poll(pollfds, 2, -1);
			if (pollfds[0].revents & POLLIN)
			{
				printf("abort signal\n");
				break;
			}
			dbus_connection_read_write(dbusConn, -1);
		}
		else
		{
			handleDBusMessage(msg);
			dbus_message_unref(msg);
		}
	}
	unregisterAgent();
	setDiscoverable(false);
	closeDBusConnection();
	lck.lock();
	isRunning = false;
}

void BluetoothHelper::abort()
{
	std::unique_lock<std::mutex> lck(mtx);
	uint8_t a = 0;
	write(stopPipe[1], &a, 1);
}

void BluetoothHelper::registerService(uint8_t rfcommChannel)
{
	const char *service_name = "tohntobshi's device";
	const char *service_dsc = "device remote control";
	const char *service_prov = "tohntobshi";
	uint8_t service_uuid_int[] = {0x84, 0x8d, 0x82, 0x8b, 0xc4, 0x86, 0x44, 0xdf, 0x83, 0xfa, 0x54, 0x13, 0xb0, 0x61, 0x46, 0xe0};

	sdp_record_t record = {0};

	uuid_t svc_uuid;
	sdp_uuid128_create(&svc_uuid, &service_uuid_int);
	sdp_set_service_id(&record, svc_uuid);

	sdp_list_t *svc_class_list = sdp_list_append(0, &svc_uuid);
	sdp_set_service_classes(&record, svc_class_list);

	uuid_t root_uuid;
	sdp_uuid16_create(&root_uuid, PUBLIC_BROWSE_GROUP);
	sdp_list_t *root_list = sdp_list_append(0, &root_uuid);
	sdp_set_browse_groups(&record, root_list);

	uuid_t rfcomm_uuid;
	sdp_uuid16_create(&rfcomm_uuid, RFCOMM_UUID);
	sdp_data_t *channel = sdp_data_alloc(SDP_UINT8, &rfcommChannel);
	sdp_list_t *rfcomm_list = sdp_list_append(0, &rfcomm_uuid);
	sdp_list_append(rfcomm_list, channel);
	sdp_list_t *proto_list = sdp_list_append(0, rfcomm_list);
	sdp_list_t *access_proto_list = sdp_list_append(0, proto_list);
	sdp_set_access_protos(&record, access_proto_list);

	sdp_profile_desc_t profile;
	sdp_uuid16_create(&profile.uuid, SERIAL_PORT_PROFILE_ID);
	profile.version = 0x0100;
	sdp_list_t *profile_list = sdp_list_append(0, &profile);
	sdp_set_profile_descs(&record, profile_list);

	sdp_set_info_attr(&record, service_name, service_prov, service_dsc);

	sdpSession = sdp_connect(&BDADDR_ANY_VAL, &BDADDR_LOCAL_VAL, SDP_RETRY_IF_BUSY);
	if (sdpSession)
	{
		int err = sdp_record_register(sdpSession, &record, 0);
		if (err)
		{
			printf("error registering sdp record\n");
		}
		else
		{
			printf("sdp record seems to be registered\n");
		}
	}
	else
	{
		printf("sdp connect failed\n");
	}

	sdp_data_free(channel);
	sdp_list_free(svc_class_list, 0);
	sdp_list_free(root_list, 0);
	sdp_list_free(rfcomm_list, 0);
	sdp_list_free(proto_list, 0);
	sdp_list_free(access_proto_list, 0);
	sdp_list_free(profile_list, 0);
}

void BluetoothHelper::unregisterService()
{
	sdp_close(sdpSession);
}

BluetoothHelper *BluetoothHelper::instance = nullptr;

BluetoothHelper *BluetoothHelper::Init()
{
	if (instance != nullptr)
	{
		return instance;
	}
	instance = new BluetoothHelper();
	return instance;
}

void BluetoothHelper::Destroy()
{
	delete instance;
	instance = nullptr;
}

BluetoothHelper::BluetoothHelper() {}
BluetoothHelper::~BluetoothHelper() {}