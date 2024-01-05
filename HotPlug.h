
 
#ifndef HOTPLUG_H
#define HOTPLUG_H

enum ObserveDeviceType
{
	ObserveDeviceType_Block,    ///< observe block devices , such as U disk , mobile disk etc 
	ObserveDeviceType_Tty,      ///< observe tty devices , such as serial or fake terminal device etc
  ObserveDevType_V4l2,
	ObserveDeviceType_All       ///< observe all devices above

};
 
enum DevType
{
    DevType_Block,    ///< block devices , such as U disk , mobile disk etc
    DevType_Tty,       ///< tty devices , such as serial or fake terminal device etc
    DevType_V4l2      
};
 
enum DevAction
{
	DevAction_Add,             ///< add device action , such as plugin a U disk
	DevAction_Remove           ///< remove device action, such as remove a U disk
};
 
/*****************************************************************************
  function name    : ObserveCallback
  used for         ：callback the observed device information
  input param      ：devType see enum DevType, devAction see enum DevAction ,devPath is device filse system access path , if device is block device , then devPath is the mounted path
  ouput param      ：no
  return value     ：no
******************************************************************************/
typedef void (*ObserveCallback)(const DevType devType,const DevAction devAction,const char * devPath);
 
#ifdef __cplusplus
extern "C" {
#endif
 
/*****************************************************************************
  function name    : initHotPlugObserver
  used for         ：initialize hot plug observer to lisen devices' change
  input param      ：no
  ouput param      ：no
  return value     ：no
******************************************************************************/
void initHotPlugObserver();
 
/*****************************************************************************
  function name    : unInitHotPlugObserver
  used for         ：uninitialize hot plug observer to release some resources
  input param      ：no
  ouput param      ：no
  return value     ：no
******************************************************************************/
void unInitHotPlugObserver();
 
/*****************************************************************************
  function name    : registerObserveCallback
  used for         ：resister a callback function to observe device's change
  input param      ：observeDeviceType see enum ObserveDeviceType, observeCallback is the callback function
  ouput param      ：no
  return value     ：no
******************************************************************************/
void registerObserveCallback(const ObserveDeviceType observeDeviceType,const ObserveCallback observeCallback);
 
/*****************************************************************************
  function name    : unregisterObserveCallback
  used for         ：unresister a callback function ,then you will not receive device's change
  input param      ：observeDeviceType see enum ObserveDeviceType, observeCallback is the callback function
  ouput param      ：no
  return value     ：no
******************************************************************************/
void unregisterObserveCallback(const ObserveDeviceType observeDeviceType, const ObserveCallback observeCallback);
 
#ifdef __cplusplus
}
#endif
 
#endif //HOT_PLUG_OBSERVER_H