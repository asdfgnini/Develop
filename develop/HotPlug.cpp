#include "HotPlug.h"
#include <map>
#include <mutex>
#include <thread>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/netlink.h>
#include <sys/mount.h>
#include <sys/stat.h>
 
#define UEVENT_MSG_LEN 4096
 
#define DEV_BASE_PATH   "/dev/"
#define MOUNT_BASE_PATH "/mnt/"
#define DEV_PATH_LENGTH 256
 
#define SUCCESS 0
#define FAIL -1
 
struct HotPlugEvent {
	const char* action;
	const char* subsystem;
	const char* devname;
	const char* devtype;
};
 
static std::mutex g_observeTypeToCallbackMapMutex;
static std::multimap<ObserveDeviceType, ObserveCallback> g_observeTypeToCallbackMap;
 
static std::mutex g_HotPlugInitMutex;
static bool g_isHotPlugObserved = false;
 
static void observeHotPlug();
 
void initHotPlugObserver()
{
	// printf(" initHotPlugObserver \n");
 
	g_HotPlugInitMutex.lock();
 
	if (!g_isHotPlugObserved)
	{
		g_isHotPlugObserved = true;
		std::thread t(observeHotPlug);
                t.detach();
	}
 
	g_HotPlugInitMutex.unlock();
}
 
void unInitHotPlugObserver()
{
	printf(" unInitHotPlugObserver \n");
 
	g_HotPlugInitMutex.lock();
 
	if (g_isHotPlugObserved)
	{
		g_isHotPlugObserved = false;
	}
 
	g_HotPlugInitMutex.unlock();
 
	g_observeTypeToCallbackMapMutex.lock();
	g_observeTypeToCallbackMap.clear();
	g_observeTypeToCallbackMapMutex.unlock();
}
 
void registerObserveCallback(const ObserveDeviceType observeDeviceType, const ObserveCallback observeCallback)
{
	if (nullptr == observeCallback)
	{
		return;
	}
 
	g_observeTypeToCallbackMapMutex.lock();
 
	bool observeCallbackExist = false;
 
	auto observeCallbackPairs = g_observeTypeToCallbackMap.equal_range(observeDeviceType);
 
	for (auto p = observeCallbackPairs.first; p != observeCallbackPairs.second; ++p)
	{
		if (p->second == observeCallback)
		{
			observeCallbackExist = true;
			break;
		}
	}
 
	// printf("registerObserveCallback observeCallback=%p exist=%d \n", observeCallback, observeCallbackExist);
 
	if (!observeCallbackExist)
	{
		g_observeTypeToCallbackMap.insert(std::pair<ObserveDeviceType, ObserveCallback>(observeDeviceType, observeCallback));
	}
 
	g_observeTypeToCallbackMapMutex.unlock();
}
 
void unregisterObserveCallback(const ObserveDeviceType observeDeviceType, const ObserveCallback observeCallback)
{
	if (nullptr == observeCallback)
	{
		return;
	}
 
	g_observeTypeToCallbackMapMutex.lock();
 
	auto observeCallbackPairs = g_observeTypeToCallbackMap.equal_range(observeDeviceType);
 
	for (auto p = observeCallbackPairs.first; p != observeCallbackPairs.second; ++p)
	{
		if (p->second == observeCallback)
		{
			printf("unregisterObserveCallback find observeCallback=%p then erase \n", observeCallback);
			g_observeTypeToCallbackMap.erase(p);
			break;
		}
	}
 
	g_observeTypeToCallbackMapMutex.unlock();
}
 
static int openHotPlugEventSocket();
 
static void parseHotPlugEvent(const char* msg, struct HotPlugEvent* hotPlugEvent);
 
static int parseDevAction(const char* eventAction, DevAction* devAction);
 
static int parseDevType(const char* devSubsystem, DevType* devType);
 
static int parseDevNode(const char* devname, char* devNode);
 
static int mountDevNodeToDevPath(const char* devNode, const char* devPath);
 
static int umountDevPath(const char* devPath);
 
static void callbackDevEvent(DevType devType, DevAction devAction, const char* devPath);
 
static void observeHotPlug()
{
	// printf(" observeHotPlug \n");
 
	///< 1, open HotPlugEvent Socket
	int hotPlugEventSocketFd = openHotPlugEventSocket();
 
	if (FAIL == hotPlugEventSocketFd)
	{
		g_HotPlugInitMutex.lock();
 
		if (g_isHotPlugObserved)
		{
			g_isHotPlugObserved = false;
		}
		g_HotPlugInitMutex.unlock();
 
		return;
	}
 
	///< 2 , receive hot plug socket data and parse it to be HotPlugEvent
	char msg[UEVENT_MSG_LEN + 2];
	int n = 0;
	struct HotPlugEvent hotPlugEvent = {0};
 
	DevAction devAction = DevAction_Add;
	DevType devType = DevType_Block;
	char devNode[DEV_PATH_LENGTH] = { 0 };
	char devPath[DEV_PATH_LENGTH+1] = { 0 };
 
	fd_set rfds;
	struct timeval tv = { 0 };
 
	int ret = SUCCESS;
 
	while(g_isHotPlugObserved)
	{
 
		FD_ZERO(&rfds);
		FD_SET(hotPlugEventSocketFd, &rfds);
 
		tv.tv_sec = 3;   ///< timeout for 3 seconds
		tv.tv_usec = 0;
 
		int selectRet = select(hotPlugEventSocketFd + 1, &rfds, NULL, NULL, &tv);
 
		if (selectRet == -1)
		{
			printf(" observeHotPlug select error \n");
			break;
		}
		
		if (selectRet == 0)
		{
			// printf(" observeHotPlug select timeout then try again \n");
			continue;
		}
 
		///< there are some hot plug events to receive
		if (FD_ISSET(hotPlugEventSocketFd, &rfds))
		{
			n = recv(hotPlugEventSocketFd, msg, UEVENT_MSG_LEN, 0);
 
			// printf(" observeHotPlug recv n=%d bytes data \n", n);
 
			if (n <= 0 || n >= UEVENT_MSG_LEN)
			{
				// printf(" observeHotPlug recv n=%d bytes data , we cannot parse it \n", n);
				continue;
			}
 
			memset(&hotPlugEvent, 0, sizeof(hotPlugEvent));
 
			msg[n] = '\0';
			msg[n + 1] = '\0';
 
			parseHotPlugEvent(msg, &hotPlugEvent);
 
			// printf("observeHotPlug parsed an hot plug event action=%s subsystem=%s devname=%s devtype=%s \n", hotPlugEvent.action, hotPlugEvent.subsystem, hotPlugEvent.devname, hotPlugEvent.devtype);
 
			///< 1, parse device action
			ret = parseDevAction(hotPlugEvent.action, &devAction);
 
			if (FAIL == ret)
			{
				continue;
			}
 
			///< 2, parse device type
			ret = parseDevType(hotPlugEvent.subsystem,&devType);
 
			if (FAIL == ret)
			{
				continue;
			}
 
			///< 3,parse device node
			memset(devNode, 0, sizeof(devNode));
			ret = parseDevNode(hotPlugEvent.devname, devNode);
 
			if (FAIL == ret)
			{
				continue;
			}
 
			///< 4, mount device node to an accessed directory or umount the directory mounted before for block device of which devType is partition
			if (devType == DevType_Block)
			{
				if (NULL == hotPlugEvent.devtype || strncmp(hotPlugEvent.devtype, "partition", 9) != 0)
				{
					printf("observeHotPlug unhandled block device with devtype=%s \n", hotPlugEvent.devtype);
					continue;
				}
 
				///< get device path to be mounted or umounted , format /mnt/devname , such as /mnt/sdb1
				memset(devPath, 0, sizeof(devPath));
				sprintf(devPath, "%s%s/", MOUNT_BASE_PATH, hotPlugEvent.devname);
 
				if (devAction == DevAction_Add)
				{
					///< mount devNode to devPath
					ret = mountDevNodeToDevPath(devNode,devPath);
				}
				else
				{
					///< umount devPath
					ret = umountDevPath(devPath);
				}
 
				if (FAIL == ret)
				{
					continue;
				}
			}
			else
			{
				///< devPath is also devNode for non_block device
				memcpy(devPath,devNode,sizeof(devNode));
			}
 
			///< 5, callback device type,action and path
			callbackDevEvent(devType,devAction,devPath);
		}
	}
 
    //    printf(" observe hot plug thread exit \n");
 
	close(hotPlugEventSocketFd);
 
	g_HotPlugInitMutex.lock();
 
	if (g_isHotPlugObserved)
	{
		g_isHotPlugObserved = false;
	}
 
	g_HotPlugInitMutex.unlock();
}
 
static int openHotPlugEventSocket()
{
	struct sockaddr_nl addr;
 
	memset(&addr, 0, sizeof(addr));
 
	addr.nl_family = AF_NETLINK;
	addr.nl_pid = getpid();
	addr.nl_groups = 0xffffffff;
 
	int ret = socket(PF_NETLINK, SOCK_DGRAM, NETLINK_KOBJECT_UEVENT);
 
	if (ret < 0)
	{
		return FAIL;
	}
 
	int sz = 64 * 1024;
	setsockopt(ret, SOL_SOCKET, SO_RCVBUFFORCE, &sz, sizeof(sz));
 
	if (bind(ret, (struct sockaddr*) & addr, sizeof(addr)) < 0) 
	{
		close(ret);
		return FAIL;
	}
 
	return ret;
}
 
static void parseHotPlugEvent(const char* msg, struct HotPlugEvent* hotPlugEvent)
{
	while (*msg) {
 
		if (!strncmp(msg, "ACTION=", 7)) 
                {
			msg += 7;
			hotPlugEvent->action = msg;
		}
		else if (!strncmp(msg, "SUBSYSTEM=", 10)) 
                {
			msg += 10;
			hotPlugEvent->subsystem = msg;
		}
		else if (!strncmp(msg, "DEVNAME=", 8)) 
                {
			msg += 8;
			hotPlugEvent->devname = msg;
		}
		else if (!strncmp(msg, "DEVTYPE=", 8)) 
                {
			msg += 8;
			hotPlugEvent->devtype = msg;
		}
 
		while (*msg++);
	}
 
	// printf("event { \n\t action=%s,\n\t subsystem=%s,\n\t devname=%s,\n\t devtype=%s\n}\n", hotPlugEvent->action, hotPlugEvent->subsystem, hotPlugEvent->devname, hotPlugEvent->devtype);
}
 
 
 
static int parseDevAction(const char * eventAction,DevAction * devAction)
{
	if (eventAction == NULL)
	{
		return FAIL;
	}
 
	if (strncmp(eventAction, "add", 3) == 0)         ///< found a device plug in
	{
		*devAction = DevAction_Add;
	}
	else if (strncmp(eventAction, "remove", 6) == 0)  ///< found a device remove
	{
		*devAction = DevAction_Remove;
	}
	else
	{
		// printf(" parseDevAction unhandled devAction=%s \n", eventAction);
		return FAIL;
	}
 
	// printf(" parseDevAction devAction=%s success \n", eventAction);
 
	return SUCCESS;
}
 
static int parseDevType(const char * devSubsystem, DevType * devType)
{
	if (devSubsystem == NULL)
	{
		return FAIL;
	}
	
	if (strncmp(devSubsystem, "block", 5) == 0)             ///< found a block device 
	{
		*devType = DevType_Block;
        }
	else if (strncmp(devSubsystem, "tty", 3) == 0)  	///< found a tty device
	{
		*devType = DevType_Tty;
	}
	else
	{
		// printf(" observeHotPlug unhandled devType=%s \n", devSubsystem);
		return FAIL;
	}
 
	// printf(" parseDevType devType=%s success \n", devSubsystem);
 
	return SUCCESS;
}
 
static int parseDevNode(const char * devname, char *devNode)
{
	if (devname == NULL)
	{
		return FAIL;
	}
 
	if (strlen(DEV_BASE_PATH) + strlen(devname) + 1 > DEV_PATH_LENGTH)
	{
		// printf(" parseDevNode too long devNode=%s%s \n", DEV_BASE_PATH,devname);
		return FAIL;
	}
 
	///< get device node , format /dev/devname , such as /dev/sdb1
	sprintf(devNode, "%s%s", DEV_BASE_PATH, devname);
 
	// printf(" parseDevNode devNode=%s success \n", devNode);
 
	return SUCCESS;
}
 
static int createDir(const char* sPathName);
 
static int mountDevNodeToDevPath(const char * devNode, const char * devPath)
{
	///< check devPath to be mounted exists or not , if not exist then we create it
 
	if (access(devPath, F_OK) != 0)
	{
		int createDevMountPathRet = createDir(devPath);
 
		if (createDevMountPathRet == FAIL)
		{
			printf(" mountDevNodeToDevPath create devPath %s fail , please check whether current process's owner's power to create dir under %s \n", devPath, MOUNT_BASE_PATH);
			return FAIL;
		}
 
		printf(" mountDevNodeToDevPath devPath %s create success \n", devPath);
	}
	else
	{
		printf(" mountDevNodeToDevPath devPath %s has exist \n", devPath);
	}
 
	///< mount devNode to devPath
	int mountDevPathRet = mount(devNode, devPath, "vfat", MS_SYNCHRONOUS, "iocharset=utf8");
 
	if (mountDevPathRet != 0)
	{
		printf("mountDevNodeToDevPath mount %s to %s fail mountDevPathRet=%d !\n", devNode, devPath, mountDevPathRet);
		return FAIL;
	}
 
	printf("mountDevNodeToDevPath mount %s to %s success !\n", devNode, devPath);
 
	return SUCCESS;
}
 
static int createDir(const char* sPathName)
{
	char   DirName[256] = {0};
	strcpy(DirName, sPathName);
 
	int i, len = strlen(DirName);
 
	if (DirName[len - 1] != '/')
	{
		strcat(DirName, "/");
	}
 
	len = strlen(DirName);
 
	for (i = 1; i < len; i++)
	{
		if (DirName[i] == '/')
		{
			DirName[i] = 0;
 
			if (access(DirName, F_OK) != 0)
			{
				if (mkdir(DirName, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
				{
					return   FAIL;
				}
			}
 
			DirName[i] = '/';
		}
	}
 
	return SUCCESS;
}
 
static int umountDevPath(const char * devPath)
{
	///< umount devMountPath
	int mountDevPathRet = umount(devPath);
 
	if (mountDevPathRet != 0)
	{
		printf("umountDevPath umount %s failed mountDevPathRet=%d !\n", devPath, mountDevPathRet);
		return FAIL;
	}
 
	printf("umountDevPath umount %s success !\n", devPath);
 
	return SUCCESS;
}
 
static void callbackDevEvent(DevType devType,DevAction devAction,const char * devPath)
{
	ObserveDeviceType observeDeviceType = ObserveDeviceType_Block;
 
	switch (devType)
	{
	case DevType_Block:
		observeDeviceType = ObserveDeviceType_Block;
		break;
	case DevType_Tty:
		observeDeviceType = ObserveDeviceType_Tty;
		break;
	default:
		printf("callbackDevEvent unhandled devType=%d \n", devType);
		return;
	}
 
	g_observeTypeToCallbackMapMutex.lock();
 
        ///< callback device event to assigned type which is observed
	auto observeCallbackPairs = g_observeTypeToCallbackMap.equal_range(observeDeviceType);
 
	for (auto p = observeCallbackPairs.first; p != observeCallbackPairs.second; ++p)
	{
		p->second(devType, devAction, devPath);
	}
 
	///< callback device event to all type which is observed
	observeCallbackPairs = g_observeTypeToCallbackMap.equal_range(ObserveDeviceType_All);
 
	for (auto p = observeCallbackPairs.first; p != observeCallbackPairs.second; ++p)
	{
		p->second(devType, devAction, devPath);
	}
 
	g_observeTypeToCallbackMapMutex.unlock();
}
 
 
 