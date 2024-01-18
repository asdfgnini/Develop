#include "HotPlug.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <stdbool.h>
#include <map>
#include <iostream>
#include <thread>
#include <string>
#include <mutex>
using namespace std;

// #define __Debug_info

//同时支持的设备数目
#define dev_num 10

const char* dev_name[dev_num] = {
      "/dev/ttyUSB0",
      "/dev/ttyUSB1",
      "/dev/ttyUSB2",
      "/dev/ttyUSB3",  
      "/dev/ttyUSB4",  
      "/dev/ttyUSB5"  
};

const char* order_info[dev_num] = {
    "sudo chmod 777 /dev/ttyUSB0",
    "sudo chmod 777 /dev/ttyUSB1",
    "sudo chmod 777 /dev/ttyUSB2",
    "sudo chmod 777 /dev/ttyUSB3",
    "sudo chmod 777 /dev/ttyUSB4",
    "sudo chmod 777 /dev/ttyUSB5"
};

std::map<const char*,int> dev_order;


                           
string port[dev_num] = {""};
int fd[dev_num] = {-1};
bool isstop[dev_num] = {false};
mutex mtx_stop;
bool isonline[dev_num] = {false};

typedef void (*function_type)();
std::map<const char*,int> pos_map;
std::map<const char*,function_type> dev_map;


typedef struct uart_hardware_cfg {
    unsigned int baudrate;      /* 波特率 */
    unsigned char dbit;         /* 数据位 */
    char parity;                /* 奇偶校验 */
    unsigned char sbit;         /* 停止位 */
} uart_cfg_t;

/****************************速度*********************************************/
/****************************brt*********************************************/
#define brt_Buffer_Length 60
typedef struct BRT_SaveData
{
    char SUDU_Buffer[brt_Buffer_Length];
    char order[8]; 
    char buf[5] = { 0 };
    int Data;
    int  pos = 0;
    float n = 0;  //转速
} BRT_SaveData;
BRT_SaveData Brt_Save_Data;
int getIndexOfSigns(char ch);
long hexToDec(char* source);
void cb_brt_test();
void cb_brt_test_singal();

/**************************OID*****************************************/
#define OID_Buffer_Length 60
typedef struct OID_SaveData
{
    char SUDU_Buffer[OID_Buffer_Length];
    char  order[8]; 
    char buf[5] = { 0 };
    int Data;
    int  pos = 0;
    float n = 0;  //转速
} OID_SaveData;
OID_SaveData OID_Save_Data;
int oid_getIndexOfSigns(char ch);
long oid_hexToDec(char* source);
void cb_oid_test();
void cb_oid_test_singal();

/****************************位置*********************************************/
/****************************ATGM336H****************************************/
#define GPS_Buffer_Length 1024
#define GPS_Buffer_Length_4 (GPS_Buffer_Length-524)
#define GPS_Buffer_Length_2 120
#define GPS_Buffer_Length_3 100
#define UTCTime_Length 11
#define latitude_Length 11
#define N_S_Length 2
#define longitude_Length 12
#define E_W_Length 2

typedef struct GPS_SaveData
{
    char GPS_Buffer[GPS_Buffer_Length];
    char Data[GPS_Buffer_Length_2];
    char GNRMC[GPS_Buffer_Length_3];
    char GNGGA[GPS_Buffer_Length_3];
    char GNVTG[GPS_Buffer_Length_3];

    char isGetData;		//是否获取到GPS数据
    char isParseData;	//是否解析完成
    char UTCTime[UTCTime_Length];		//UTC时间
    char latitude[latitude_Length];		//纬度
    char N_S[N_S_Length];		//N/S
    char longitude[longitude_Length];		//经度
    char E_W[E_W_Length];		//E/W
    char isUsefull;		//定位信息是否有效
    double raw_altitude; //海拔
    double raw_satellites;//卫星数量
    double raw_hdop;//精度因子
   double raw_speed;      //水平速率
   double raw_course;     //地面航向

    int m_read_idx;
} GPS_SaveData;
GPS_SaveData GPS_Save_Data;

typedef struct gps_zheng_data
{
    int pos;//获取目标帧的起始位置
    char* start;
    char* end;
    bool Give_up ;//判断是否放弃这帧数据
    bool find ;//判断是否找到这帧数据
}Gps_Zheng_data;
Gps_Zheng_data GNRMC;
Gps_Zheng_data GNGGA;
Gps_Zheng_data GNVTG;
static struct termios gps_old_cfg;  //用于保存终端的配置参数

double Convert_to_degrees(char* data);// GPS数据转化单位为度。
void printGpsBuffer();//打印GPS解码后相关数据
void printf_GNVTG();
void printf_GNGGA();
void GNRMC_calc_shuju();
void GNGGA_calc_shuju();
void GNVTG_calc_shuju();
static int gps_uart_init(const char *device);
static int gps_uart_cfg(const uart_cfg_t *cfg);

void cb_gps_test_singal();

/****************************WTGPS****************************************/
#define WTGPS_GPS_Buffer_Length 1024
#define WTGPS_GPS_Buffer_Length_4 (GPS_Buffer_Length-524)
#define WTGPS_GPS_Buffer_Length_2 120
#define WTGPS_GPS_Buffer_Length_3 100
#define WTGPS_UTCTime_Length 11
#define WTGPS_latitude_Length 11
#define WTGPS_N_S_Length 2
#define WTGPS_longitude_Length 12
#define WTGPS_E_W_Length 2

typedef struct WTGPS_GPS_SaveData
{
    char GPS_Buffer[GPS_Buffer_Length];
    char Data[GPS_Buffer_Length_2];
    char GNRMC[GPS_Buffer_Length_3];
    char GNGGA[GPS_Buffer_Length_3];
    char GNVTG[GPS_Buffer_Length_3];

    char isGetData;		//是否获取到GPS数据
    char isParseData;	//是否解析完成
    char UTCTime[UTCTime_Length];		//UTC时间
    char latitude[latitude_Length];		//纬度
    char N_S[N_S_Length];		//N/S
    char longitude[longitude_Length];		//经度
    char E_W[E_W_Length];		//E/W
    char isUsefull;		//定位信息是否有效
    double raw_altitude; //海拔
    double raw_satellites;//卫星数量
    double raw_hdop;//精度因子
   double raw_speed;      //水平速率
   double raw_course;     //地面航向

    int m_read_idx;
} WTGPS_GPS_SaveData;
WTGPS_GPS_SaveData WTGPS_GPS_Save_Data;

typedef struct WTGPS_gps_zheng_data
{
    int pos;//获取目标帧的起始位置
    char* start;
    char* end;
    bool Give_up ;//判断是否放弃这帧数据
    bool find ;//判断是否找到这帧数据
}WTGPS_Gps_Zheng_data;
WTGPS_Gps_Zheng_data WTGPS_GNRMC;
WTGPS_Gps_Zheng_data WTGPS_GNGGA;
WTGPS_Gps_Zheng_data WTGPS_GNVTG;
static struct termios WTGPS_gps_old_cfg;  //用于保存终端的配置参数

double WTGPS_Convert_to_degrees(char* data);// GPS数据转化单位为度。
void WTGPS_printGpsBuffer();//打印GPS解码后相关数据
void WTGPS_printf_GNVTG();
void WTGPS_printf_GNGGA();
void WTGPS_GNRMC_calc_shuju();
void WTGPS_GNGGA_calc_shuju();
void WTGPS_GNVTG_calc_shuju();
static int WTGPS_gps_uart_init(const char *device);
static int WTGPS_gps_uart_cfg(const uart_cfg_t *cfg);

void cb_wtgps_test_singal();

/****************************姿态*********************************************/
/**************************WIT*****************************************/

static struct termios wit_old_cfg;  //用于保存终端的配置参数
#define WIT_Buffer_Length 250
typedef struct WIT_SaveData
{
    char WIT_Buffer[WIT_Buffer_Length];
    struct
    {
        uint32_t x;
        uint32_t y;
       uint32_t z;
    }raw;
    float x;//x 角速度
    float y;//y 角速度
    float z;//z 角速度
    float a_x;//x 角加速度
    float a_y;//y 角加速度
    float a_z;//z 角加速度
    float roll;                     
    float pitch;
    float yaw;
    int isGet;
    int pos = 0;
    int a_isGet;
    int a_pos = 0;
    int j_isGet;
    int j_pos = 0;
}  WIT_SaveData;
 WIT_SaveData  WIT_Save_Data;
static int wit_uart_init(const char *device);
static int wit_uart_cfg(const uart_cfg_t *cfg);
void cb_wit_test_singal();
/**************************JY9*****************************************/
static struct termios JY9_old_cfg;  //用于保存终端的配置参数
#define JY9_Buffer_Length 250
typedef struct JY9_SaveData
{
    char JY9_Buffer[JY9_Buffer_Length];
    struct
    {
        uint32_t x;
        uint32_t y;
       uint32_t z;
    }raw;
    float x;//x 角速度
    float y;//y 角速度
    float z;//z 角速度
    float a_x;//x 角加速度
    float a_y;//y 角加速度
    float a_z;//z 角加速度
    float roll;                     
    float pitch;
    float yaw;
    int isGet;
    int pos = 0;
    int a_isGet;
    int a_pos = 0;
    int j_isGet;
    int j_pos = 0;
}  JY9_SaveData;
 JY9_SaveData  JY9_Save_Data;
static int jy9_uart_init(const char *device);
static int jy9_uart_cfg(const uart_cfg_t *cfg);
void cb_jy9_test_singal();

/****************************全局*********************************************/
void observeALLDeviceHotPlugEventCallback(const DevType devType,const DevAction devAction,const char * devPath);
/****************************brt*********************************************/
void observe_BRT_HotPlugEventCallback(const DevType devType,const DevAction devAction,const char * devPath);
/****************************OID*********************************************/
void observe_OID_HotPlugEventCallback(const DevType devType,const DevAction devAction,const char * devPath);
/****************************WIT*********************************************/
void observe_WIT_HotPlugEventCallback(const DevType devType,const DevAction devAction,const char * devPath);
/****************************JY9*********************************************/
void observe_JY9_HotPlugEventCallback(const DevType devType,const DevAction devAction,const char * devPath);
/****************************ATGM336H*********************************************/
void observe_ATGM336H_HotPlugEventCallback(const DevType devType,const DevAction devAction,const char * devPath);
/****************************WTGPS*********************************************/
void observe_WTGPS_HotPlugEventCallback(const DevType devType,const DevAction devAction,const char * devPath);
/****************************video*********************************************/
void observe_video_HotPlugEventCallback(const DevType devType,const DevAction devAction,const char * devPath);

//尾插
int arry_insert_back(const char* devpath);
//删除
void array_delete_pre(const char* devpath);
//获取
bool Get_isstop(int pos);
//停止
void stop_Thread(int pos , bool states);


int main(int argc , char * argv[])
{
    system("clear");
    // system("echo 3 > /proc/sys/kernel/printk");

    if(argc < 1 || argc >= 2)
    {
        if(strcmp(argv[1],"-all") == 0)
        {

            dev_order.insert(std::pair<const char*,int>("brt",0));
            dev_order.insert(std::pair<const char*,int>("oid",1));
            dev_order.insert(std::pair<const char*,int>("wit",2));
            dev_order.insert(std::pair<const char*,int>("jy9",3));
            dev_order.insert(std::pair<const char*,int>("atgm",4));
            dev_order.insert(std::pair<const char*,int>("wtgps",5));
        }
        else
        {
            dev_order.insert(std::pair<const char*,int>("brt",0));
            dev_order.insert(std::pair<const char*,int>("oid",0));
            dev_order.insert(std::pair<const char*,int>("wit",0));
            dev_order.insert(std::pair<const char*,int>("jy9",0));
            dev_order.insert(std::pair<const char*,int>("atgm",0));
            dev_order.insert(std::pair<const char*,int>("wtgps",0));
        }

        if(strcmp(argv[1],"-h") == 0)
        {
            printf("*****************速度******************\r\n");
            printf("-brt        支持_速度_传感器brt热插拔\r\n");
            printf("-oid        支持_速度_传感器oid热插拔\r\n\r\n");
            printf("*****************姿态******************\r\n");
            printf("-wit        支持_姿态_传感器wit热插拔\r\n");
            printf("-jy9        支持_姿态_传感器JY901B热插拔\r\n\r\n");
            printf("*****************位置******************\r\n");
            printf("-atgm       支持_位置_传感器atgm336h热插拔\r\n");
            printf("-wtgps      支持_位置_传感器WTGPS+BD热插拔\r\n\r\n");
            printf("***************************************\r\n");
            printf("-all        支持插入多个传感器(按一定顺序)\r\n");

            exit(-1);
        }
        else if(strcmp(argv[1],"-all") == 0)
        {
            printf("\r\n暂未开发, 敬请期待!!!\r\n");
            exit(-1);
            int loaction = -1;
            auto iter_order = dev_order.find("atgm");
            if(iter_order != pos_map.end())
            {
                loaction = iter_order->second;
            }
            //注册设备回调表
            dev_map.insert(std::pair<const char*,function_type>("/dev/ttyUSB0",cb_brt_test));
            dev_map.insert(std::pair<const char*,function_type>("/dev/ttyUSB1",cb_oid_test));
           
#ifdef __Debug_info 
            //注册设备号相对位置表
            pos_map.insert(std::pair<const char*,int>("/dev/ttyUSB0",0));
            //初始化热插拔服务器
            initHotPlugObserver();
            //注册热插拔事件回调
            registerObserveCallback(ObserveDeviceType_All,observe_BRT_HotPlugEventCallback);
            printf("系统初始化完毕\r\n");


            for(map<const char*,function_type>::iterator it=dev_map.begin();it!=dev_map.end();it++)
            {
                cout <<"[dev_map]: " << "key:" <<it->first<<" value:"<<it->second<<endl;
            }
#endif 
            //注册设备号相对位置表
            pos_map.insert(std::pair<const char*,int>("/dev/ttyUSB0",0));
            pos_map.insert(std::pair<const char*,int>("/dev/ttyUSB1",1));


#ifdef __Debug_info
            for(map<const char*,int>::iterator it=pos_map.begin();it!=pos_map.end();it++)
            {
                cout << "[pos_map]: "<< "key:" <<it->first<<" pos:"<<it->second<<endl;
            } 
#endif             
            //初始化热插拔服务器
            initHotPlugObserver();
        
            //注册热插拔事件回调
            registerObserveCallback(ObserveDeviceType_All,observeALLDeviceHotPlugEventCallback);
        
#if 0
            //注销热插拔服务器
            unInitHotPlugObserver();
        
            //注销热插拔事件回调
            unregisterObserveCallback(ObserveDeviceType_Block,observeBlockDeviceHotPlugEventCallback);    

#endif   
            printf("系统初始化完毕\r\n");

            //阻塞主线程
            pause();                
        } 
        else if(strcmp(argv[1],"-brt") == 0)
        {
            int loaction = -1;
            auto iter_order = dev_order.find("brt");
            if(iter_order != pos_map.end())
            {
                loaction = iter_order->second;
            }
            // printf("loaction=%d\r\n",loaction);
            //注册设备回调表
            dev_map.insert(std::pair<const char*,function_type>(dev_name[loaction],cb_brt_test_singal));
            //注册设备号相对位置表
            pos_map.insert(std::pair<const char*,int>(dev_name[loaction],loaction));

            // for(map<const char*,function_type>::iterator it=dev_map.begin();it!=dev_map.end();it++)
            // {
            //     cout <<"[dev_map]: " << "key:" <<it->first<<" value:"<<it->second<<endl;
            // }
            //初始化热插拔服务器
            initHotPlugObserver();
            //注册热插拔事件回调
            registerObserveCallback(ObserveDeviceType_All,observe_BRT_HotPlugEventCallback);
            printf("系统初始化完毕\r\n");

            //阻塞主线程
            pause();
            exit(-1);
        }
        else if(strcmp(argv[1],"-oid") == 0)
        {
            int loaction = -1;
            auto iter_order = dev_order.find("oid");
            if(iter_order != pos_map.end())
            {
                loaction = iter_order->second;
            }
            // printf("loaction=%d\r\n",loaction);
            //注册设备回调表
            dev_map.insert(std::pair<const char*,function_type>(dev_name[loaction],cb_oid_test_singal));
            //注册设备号相对位置表
            pos_map.insert(std::pair<const char*,int>(dev_name[loaction],loaction));
            //初始化热插拔服务器
            initHotPlugObserver();
            //注册热插拔事件回调
            registerObserveCallback(ObserveDeviceType_All,observe_OID_HotPlugEventCallback);
            printf("系统初始化完毕\r\n");

            //阻塞主线程
            pause();
            exit(-1);
        }
        else if(strcmp(argv[1],"-wit") == 0)
        {
            int loaction = -1;
            auto iter_order = dev_order.find("wit");
            if(iter_order != pos_map.end())
            {
                loaction = iter_order->second;
            }
            // printf("loaction=%d\r\n",loaction);
            //注册设备回调表
            dev_map.insert(std::pair<const char*,function_type>(dev_name[loaction],cb_wit_test_singal));
            //注册设备号相对位置表
            pos_map.insert(std::pair<const char*,int>(dev_name[loaction],loaction));
            //初始化热插拔服务器
            initHotPlugObserver();
            //注册热插拔事件回调
            registerObserveCallback(ObserveDeviceType_All,observe_WIT_HotPlugEventCallback);
            printf("系统初始化完毕\r\n");

            //阻塞主线程
            pause();
            exit(-1);
        }
        else if(strcmp(argv[1],"-jy9") == 0)
        {
            int loaction = -1;
            auto iter_order = dev_order.find("jy9");
            if(iter_order != pos_map.end())
            {
                loaction = iter_order->second;
            }
            // printf("loaction=%d\r\n",loaction);
            //注册设备回调表
            dev_map.insert(std::pair<const char*,function_type>(dev_name[loaction],cb_jy9_test_singal));
            //注册设备号相对位置表
            pos_map.insert(std::pair<const char*,int>(dev_name[loaction],loaction));
            //初始化热插拔服务器
            initHotPlugObserver();
            //注册热插拔事件回调
            registerObserveCallback(ObserveDeviceType_All,observe_JY9_HotPlugEventCallback);
            printf("系统初始化完毕\r\n");

            //阻塞主线程
            pause();
            exit(-1);
        }
        else if(strcmp(argv[1],"-atgm") == 0)
        {
            int loaction = -1;
            auto iter_order = dev_order.find("atgm");
            if(iter_order != pos_map.end())
            {
                loaction = iter_order->second;
            }
            // printf("loaction=%d\r\n",loaction);
            //注册设备回调表
            dev_map.insert(std::pair<const char*,function_type>(dev_name[loaction],cb_gps_test_singal));
            //注册设备号相对位置表
            pos_map.insert(std::pair<const char*,int>(dev_name[loaction],loaction));
            //初始化热插拔服务器
            initHotPlugObserver();
            //注册热插拔事件回调
            registerObserveCallback(ObserveDeviceType_All,observe_ATGM336H_HotPlugEventCallback);
            printf("系统初始化完毕\r\n");

            //阻塞主线程
            pause();
            exit(-1);
        }
        else if(strcmp(argv[1],"-wtgps") == 0)
        {
            int loaction = -1;
            auto iter_order = dev_order.find("wtgps");
            if(iter_order != pos_map.end())
            {
                loaction = iter_order->second;
            }
            // printf("loaction=%d\r\n",loaction);
            //注册设备回调表
            dev_map.insert(std::pair<const char*,function_type>(dev_name[loaction],cb_wtgps_test_singal));
            //注册设备号相对位置表
            pos_map.insert(std::pair<const char*,int>(dev_name[loaction],loaction));
            //初始化热插拔服务器
            initHotPlugObserver();
            //注册热插拔事件回调
            registerObserveCallback(ObserveDeviceType_All,observe_WTGPS_HotPlugEventCallback);
            printf("系统初始化完毕\r\n");

            //阻塞主线程
            pause();
            exit(-1);
        }
        else if(strcmp(argv[1],"-0") == 0)
        {

            //初始化热插拔服务器
            initHotPlugObserver();
            //注册热插拔事件回调
            registerObserveCallback(ObserveDeviceType_All,observe_video_HotPlugEventCallback);
            printf("系统初始化完毕\r\n");

            //阻塞主线程
            pause();
            exit(-1);
        }
        else
        {
            printf("\r\n请输入正确的参数!!! -h获取可用参数\r\n\r\n");    

            exit(-1);
        }       
    }
    else
    {
        printf("\r\n请输入正确的参数!!! -h获取可用参数\r\n\r\n");
        exit(-1);
    }
    return 0;
}
/*********************************全局************************************************/
void observeALLDeviceHotPlugEventCallback(const DevType devType,const DevAction devAction,const char * devPath)
{
    // printf(" observeBlockDeviceHotPlugEventCallback devType=%d devAction=%d devPath=%s ",devType,devAction,devPath);
    
    int pos = arry_insert_back(devPath);
    printf("\r\npos = %d\r\n",pos);

    if(pos != -1)
    {
        if(devType == DevType_Tty)
        {
            if(devAction == DevAction_Add)
            {
                if(strcmp(devPath,"/dev/ttyUSB0") == 0)
                {

                    auto iter = pos_map.find("/dev/ttyUSB0");
                    if(iter != pos_map.end())
                    {
                        isstop[iter->second] = false; 
                        auto iter2 = dev_map.find("/dev/ttyUSB0");
                        if(iter2 != dev_map.end())
                        {
                            std::thread hyxw(iter2->second);
                            hyxw.detach();
                            printf("力矩传感器hyxw-x6插入\r\n");
                        }
                        else
                        {
                            printf("[ADD]: hyxw dev_map没找到 %d\r\n",__LINE__);
                        }
                    }
                    else
                    {
                        printf("[ADD]:hyxw pos_map没找到 %d\r\n",__LINE__);
                    }
                }  
                else if(strcmp(devPath,"/dev/ttyUSB1") == 0)
                {
                    auto iter = pos_map.find("/dev/ttyUSB1");
                    if(iter != pos_map.end())
                    {
                        isstop[iter->second] = false; 
                        auto iter2 = dev_map.find("/dev/ttyUSB1");
                        if(iter2 != dev_map.end())
                        {
                            std::thread brt(iter2->second);
                            brt.detach();
                            printf("速度传感器brt插入\r\n");
                        }
                        else
                        {
                            printf("[ADD]: brt dev_map没找到 %d\r\n",__LINE__);
                        }
                    }
                    else
                    {
                        printf("[ADD]:brt pos_map没找到 %d\r\n",__LINE__);
                    }
                }  
            }
            else if(devAction == DevAction_Remove)
            {
                if(strcmp(devPath,"/dev/ttyUSB0") == 0)
                {
                    auto iter = pos_map.find("/dev/ttyUSB0");
                    if(iter != pos_map.end())
                    {
                        isstop[iter->second] = true;   
                        
                        array_delete_pre(devPath);
                        printf("力矩传感器 hyxw-x6拔出\r\n");
                    }
                    else
                    {
                        printf("[REMOVE]:pos_map没找到 %d\r\n",__LINE__);   
                    }
                }
                else if(strcmp(devPath,"/dev/ttyUSB1") == 0)
                {
                    if(strcmp(devPath,"/dev/ttyUSB1") == 0)
                    {
                        auto iter = pos_map.find("/dev/ttyUSB1");
                        if(iter != pos_map.end())
                        {
                            isstop[iter->second] = true;   
                           
                            array_delete_pre(devPath);
                            printf("速度传感器 brt拔出\r\n");
                        }
                        else
                        {
                            printf("[REMOVE]:pos_map没找到 %d\r\n",__LINE__);   
                        }
                    }
                } 
            }
        }
        // //获取设备数
        // int num1 = 0;
        // for (int i = 0; i < dev_num; i++)
        // {       
        //     if(port[i] == "")
        //     {
        //         continue;
        //     }
        //     else
        //     {
        //         num1++;
        //         cout << "dev: " << port[i] << endl;
        //     }
        // }
        // printf("\r\n设备数:%d\r\n",num1);
    }
}
/*********************************brt************************************************/
void observe_BRT_HotPlugEventCallback(const DevType devType,const DevAction devAction,const char * devPath)
{
    int loaction = -1;
    auto iter_order = dev_order.find("brt");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }
#ifdef __Debug_info
    printf("loaction=%d\r\n",loaction);
    printf("%s\r\n",devPath);
#endif
    if(strcmp(devPath,dev_name[loaction]) == 0)
    {
        int pos = arry_insert_back(devPath);
        if(pos != -1)
        {
            if(devType == DevType_Tty)
            {
                if(devAction == DevAction_Add)
                {   

                    system(order_info[loaction]);
                    auto iter = pos_map.find(dev_name[loaction]);
                    if(iter != pos_map.end())
                    {
                        isstop[iter->second] = false; 
                        auto iter2 = dev_map.find(dev_name[loaction]);
                        if(iter2 != dev_map.end())
                        {
                            std::thread brt(iter2->second);
                            brt.detach();
                            printf("速度传感器brt插入\r\n");
                        }
                        else
                        {
                            printf("[ADD]: brt dev_map没找到 %d\r\n",__LINE__);
                        }
                    }
                    else
                    {
                        printf("[ADD]:brt pos_map没找到 %d\r\n",__LINE__);
                    }
                
                }
                else if(devAction == DevAction_Remove)
                {
                    if(strcmp(devPath,dev_name[loaction]) == 0)
                    {
                        auto iter = pos_map.find(dev_name[loaction]);
                        if(iter != pos_map.end())
                        {
                            stop_Thread(iter->second,true);  
                            
                            array_delete_pre(devPath);
                            printf("速度传感器 brt拔出\r\n");
                        }
                        else
                        {
                            printf("[REMOVE]:pos_map没找到 %d\r\n",__LINE__);   
                        }
                    }
                }
            }
        }
    }
}
/*********************************OID************************************************/
void observe_OID_HotPlugEventCallback(const DevType devType,const DevAction devAction,const char * devPath)
{

    int loaction = -1;
    auto iter_order = dev_order.find("oid");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }
#ifdef __Debug_info
    printf("loaction=%d\r\n",loaction);
    printf("%s\r\n",devPath);
#endif
    if(strcmp(devPath,dev_name[loaction]) == 0)
    {
        int pos = arry_insert_back(devPath);
        if(pos != -1)
        {
            if(devType == DevType_Tty)
            {
                if(devAction == DevAction_Add)
                {
                    if(strcmp(devPath,dev_name[loaction]) == 0)
                    {
                        system(order_info[loaction]);
                        auto iter = pos_map.find(dev_name[loaction]);
                        if(iter != pos_map.end())
                        {
                            isstop[iter->second] = false; 
                            auto iter2 = dev_map.find(dev_name[loaction]);
                            if(iter2 != dev_map.end())
                            {
                                std::thread brt(iter2->second);
                                brt.detach();
                                printf("速度传感器OID插入\r\n");
                            }
                            else
                            {
                                printf("[ADD]: OID dev_map没找到 %d\r\n",__LINE__);
                            }
                        }
                        else
                        {
                            printf("[ADD]:OID pos_map没找到 %d\r\n",__LINE__);
                        }
                    }  
                }
                else if(devAction == DevAction_Remove)
                {
                    if(strcmp(devPath,dev_name[loaction]) == 0)
                    {
                        auto iter = pos_map.find(dev_name[loaction]);
                        if(iter != pos_map.end())
                        {
                            stop_Thread(iter->second,true);  
                            
                            array_delete_pre(devPath);
                            printf("速度传感器 OID拔出\r\n");
                        }
                        else
                        {
                            printf("[REMOVE]:pos_map没找到 %d\r\n",__LINE__);   
                        }
                    }
                }
            }
        }
    }
}
/*********************************WIT************************************************/
void observe_WIT_HotPlugEventCallback(const DevType devType,const DevAction devAction,const char * devPath)
{
    int loaction = -1;
    auto iter_order = dev_order.find("wit");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }
#ifdef __Debug_info
    printf("loaction=%d\r\n",loaction);
    printf("%s\r\n",devPath);
#endif
    if(strcmp(devPath,dev_name[loaction]) == 0)
    {
        int pos = arry_insert_back(devPath);
        if(pos != -1)
        {
            if(devType == DevType_Tty)
            {
                if(devAction == DevAction_Add)
                {

                    system(order_info[loaction]);

                    auto iter = pos_map.find(dev_name[loaction]);
                    if(iter != pos_map.end())
                    {
                        isstop[iter->second] = false; 
                        auto iter2 = dev_map.find(dev_name[loaction]);
                        if(iter2 != dev_map.end())
                        {
                            std::thread brt(iter2->second);
                            brt.detach();
                            printf("姿态传感器WIT插入\r\n");
                        }
                        else
                        {
                            printf("[ADD]: WIT dev_map没找到 %d\r\n",__LINE__);
                        }
                    }
                    else
                    {
                        printf("[ADD]:WIT pos_map没找到 %d\r\n",__LINE__);
                    }
                    
                }
                else if(devAction == DevAction_Remove)
                {
                    if(strcmp(devPath,dev_name[loaction]) == 0)
                    {
                        auto iter = pos_map.find(dev_name[loaction]);
                        if(iter != pos_map.end())
                        {
                            stop_Thread(iter->second,true);  
                            array_delete_pre(devPath);
                            printf("姿态传感器 WIT拔出\r\n");
                        }
                        else
                        {
                            printf("[REMOVE]:pos_map没找到 %d\r\n",__LINE__);   
                        }
                    }
                }
            }
        }
    }
}
/*********************************JY9************************************************/
void observe_JY9_HotPlugEventCallback(const DevType devType,const DevAction devAction,const char * devPath)
{
    int loaction = -1;
    auto iter_order = dev_order.find("jy9");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }
#ifdef __Debug_info
    printf("loaction=%d\r\n",loaction);
    printf("%s\r\n",devPath);
#endif
    if(strcmp(devPath,dev_name[loaction]) == 0)
    {
        int pos = arry_insert_back(devPath);
        if(pos != -1)
        {
            if(devType == DevType_Tty)
            {
                if(devAction == DevAction_Add)
                {
                    system(order_info[loaction]);
                    auto iter = pos_map.find(dev_name[loaction]);
                    if(iter != pos_map.end())
                    {
                        isstop[iter->second] = false; 
                        auto iter2 = dev_map.find(dev_name[loaction]);
                        if(iter2 != dev_map.end())
                        {
                            std::thread brt(iter2->second);
                            brt.detach();
                            printf("姿态传感器JY901B插入\r\n");
                        }
                        else
                        {
                            printf("[ADD]: JY9 dev_map没找到 %d\r\n",__LINE__);
                        }
                    }
                    else
                    {
                        printf("[ADD]:JY9 pos_map没找到 %d\r\n",__LINE__);
                    }
                     
                }
                else if(devAction == DevAction_Remove)
                {
                    if(strcmp(devPath,dev_name[loaction]) == 0)
                    {
                        auto iter = pos_map.find(dev_name[loaction]);
                        if(iter != pos_map.end())
                        {
                            stop_Thread(iter->second,true);  
                            array_delete_pre(devPath);
                            printf("姿态传感器 JY901B拔出\r\n");
                        }
                        else
                        {
                            printf("[REMOVE]:pos_map没找到 %d\r\n",__LINE__);   
                        }
                    }
                }
            }
        }
    }
}
/*********************************ATGM336H************************************************/
void observe_ATGM336H_HotPlugEventCallback(const DevType devType,const DevAction devAction,const char * devPath)
{
    int loaction = -1;
    auto iter_order = dev_order.find("atgm");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }


    if(strcmp(devPath,dev_name[loaction]) == 0)
    {
        int pos = arry_insert_back(devPath);
        if(pos != -1)
        {
            if(devType == DevType_Tty)
            {
                if(devAction == DevAction_Add)
                {

                    system(order_info[loaction]);
                    auto iter = pos_map.find(dev_name[loaction]);
                    if(iter != pos_map.end())
                    {
                        isstop[iter->second] = false; 
                        auto iter2 = dev_map.find(dev_name[loaction]);
                        if(iter2 != dev_map.end())
                        {
                            std::thread brt(iter2->second);
                            brt.detach();
                            printf("位置传感器ATGM336h插入\r\n");
                        }
                        else
                        {
                            printf("[ADD]: ATGM336h dev_map没找到 %d\r\n",__LINE__);
                        }
                    }
                    else
                    {
                        printf("[ADD]:ATGM336h pos_map没找到 %d\r\n",__LINE__);
                    }
                     
                }
                else if(devAction == DevAction_Remove)
                {
                    if(strcmp(devPath,dev_name[loaction]) == 0)
                    {
                        auto iter = pos_map.find(dev_name[loaction]);
                        if(iter != pos_map.end())
                        {
                            stop_Thread(iter->second,true);  
                            array_delete_pre(devPath);
                            printf("位置传感器ATGM336h拔出\r\n");
                        }
                        else
                        {
                            printf("[REMOVE]:pos_map没找到 %d\r\n",__LINE__);   
                        }
                    }
                }
            }
        }
    }
}
/*********************************WTGPS+BD************************************************/
void observe_WTGPS_HotPlugEventCallback(const DevType devType,const DevAction devAction,const char * devPath)
{
    int loaction = -1;
    auto iter_order = dev_order.find("wtgps");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }

    if(strcmp(devPath,dev_name[loaction]) == 0)
    {
        int pos = arry_insert_back(devPath);
        if(pos != -1)
        {
            if(devType == DevType_Tty)
            {
                if(devAction == DevAction_Add)
                {

                    system(order_info[loaction]);
                    auto iter = pos_map.find(dev_name[loaction]);
                    if(iter != pos_map.end())
                    {
                        isstop[iter->second] = false; 
                        auto iter2 = dev_map.find(dev_name[loaction]);
                        if(iter2 != dev_map.end())
                        {
                            std::thread brt(iter2->second);
                            brt.detach();
                            printf("位置传感器WTGPS插入\r\n");
                        }
                        else
                        {
                            printf("[ADD]: WTGPS dev_map没找到 %d\r\n",__LINE__);
                        }
                    }
                    else
                    {
                        printf("[ADD]:WTGPS pos_map没找到 %d\r\n",__LINE__);
                    }
                      
                }
                else if(devAction == DevAction_Remove)
                {
                    if(strcmp(devPath,dev_name[loaction]) == 0)
                    {
                        auto iter = pos_map.find(dev_name[loaction]);
                        if(iter != pos_map.end())
                        {
                            stop_Thread(iter->second,true);  
                            array_delete_pre(devPath);
                            printf("位置传感器WTGPS拔出\r\n");
                        }
                        else
                        {
                            printf("[REMOVE]:pos_map没找到 %d\r\n",__LINE__);   
                        }
                    }
                }
            }
        }
    }
}
/*********************************video************************************************/
void observe_video_HotPlugEventCallback(const DevType devType,const DevAction devAction,const char * devPath)
{
    if(strcmp(devPath,"/dev/video0") == 0)
    {
        if(devType == DevType_V4l2)
        {
            if(devAction == DevAction_Add)
            {
                printf("\r\n v4l2 插入\r\n");
            }
            else if(devAction == DevAction_Remove)
            {
                 printf("\r\nv4l2 拔出\r\n");
            }
        }
    }
}

//插
int arry_insert_back(const char* devpath)
{
    int num = 0;
    for (int i = 0; i < dev_num; i++)
    {       
        if(port[i] == "")
        {
            continue;
        }
        else
        {
            num++;
        }
    }
    if(num >= dev_num)
    {
        printf("设备太多\r\n");
        exit(-1);
    }

    if(strcmp(devpath,"/dev/ttyUSB0") == 0)
    {
        port[0] = devpath;

        return 0;
    }
    else if(strcmp(devpath,"/dev/ttyUSB1") == 0)
    {
        port[1] = devpath;
        return 1;
    }
    else if(strcmp(devpath,"/dev/ttyUSB2") == 0)
    {
        port[2] = devpath;
        return 2;
    }
    else if(strcmp(devpath,"/dev/ttyUSB3") == 0)
    {
        port[3] = devpath;
        return 3;
    }
    else if(strcmp(devpath,"/dev/ttyUSB4") == 0)
    {
        port[4] = devpath;
        return 4;
    }   
    else if(strcmp(devpath,"/dev/ttyUSB5") == 0)
    {
        port[5] = devpath;
        return 5;
    }

    return -1;
}
//删
void array_delete_pre(const char* devpath)
{
    if(strcmp(devpath,"/dev/ttyUSB0") == 0)
    {
        port[0] = "";
       
    }
    else if(strcmp(devpath,"/dev/ttyUSB1") == 0)
    {
        port[1] = "";
        
    }
    else if(strcmp(devpath,"/dev/ttyUSB2") == 0)
    {
        port[2] = ""; 
    }
    else if(strcmp(devpath,"/dev/ttyUSB3") == 0)
    {
        port[3] = ""; 
    }
    else if(strcmp(devpath,"/dev/ttyUSB4") == 0)
    {
        port[4] = ""; 
    }
    else if(strcmp(devpath,"/dev/ttyUSB5") == 0)
    {
        port[5] = ""; 
    }

}
//停止
void stop_Thread(int pos , bool states)
{
    mtx_stop.lock();
    isstop[pos] = states;
    mtx_stop.unlock();
}
//获取
bool Get_isstop(int pos)
{
    bool stat;
    mtx_stop.lock();
    stat = isstop[pos];
    mtx_stop.unlock();

    return stat;
}

/***********************************速度**************************************************/
/***********************************BRT**************************************************/
void cb_brt_test()
{
    printf("\r\ncb_brt_test thread :%ld\r\n",pthread_self());

    int ret,pos = -1;
	struct termios old_cfg; //用于保存终端配置之前的参数
	struct termios new_cfg; //用于保存终端新配置的参数
	speed_t speed = B9600;		//定义波特率为9600
    /*第一步，串口初始化*/

    auto it = pos_map.find("/dev/ttyUSB1");
    if (it != pos_map.end())
    {
        pos = it->second;
    }
    else
    {
        printf("cb_brt_test pos_map没找到 %d\r\n",__LINE__);
    }
    
	fd[pos] = open(port[pos].c_str(),O_RDWR | O_NOCTTY | O_NDELAY);//O_NOCTTY 标志，告知系统该节点不会成为进程的控制终端
    if(fd[pos] < 0)
    {
            printf("uart device open error\n");
            return;
    }
	ret = tcgetattr(fd[pos], &old_cfg);
	if(ret < 0)
        {
		printf("tcgetattr error\n");
		close(fd[pos]);
		return;
	}
	/*第二步，配置串口参数*/
	cfmakeraw(&new_cfg);//设置为原始模式 
	new_cfg.c_cflag |= CREAD;// 使能接收 
	cfsetspeed(&new_cfg, speed);//将波特率设置为9600
	new_cfg.c_cflag &= ~CSIZE; //将数据位相关的比特位清零
	new_cfg.c_cflag |= CS8;    //将数据位数设置为8位
	new_cfg.c_cflag &= ~PARENB;
	new_cfg.c_iflag &= ~INPCK;//设置为无校验模式
	new_cfg.c_cflag &= ~CSTOPB;//将停止位设置为1位
	new_cfg.c_cc[VTIME] = 0;// 将 MIN 和 TIME 设置为 0
	new_cfg.c_cc[VMIN] = 0;
	ret = tcflush(fd[pos], TCIOFLUSH);//清空缓冲区域

        if(ret < 0)
        {
            printf("tcflush error\n");
            return;
        }
        ret = tcsetattr(fd[pos], TCSANOW, &new_cfg);//写入配置、使配置生效
        if(ret < 0)
        {
            printf("tcsetattr error\n");
            return;
        }	
        Brt_Save_Data.order[0] = 0x01;
        Brt_Save_Data.order[1] = 0x03;
        Brt_Save_Data.order[2] = 0x00;
        Brt_Save_Data.order[3] = 0x20;
        Brt_Save_Data.order[4] = 0x00;
        Brt_Save_Data.order[5] = 0x02;
        Brt_Save_Data.order[6] = 0xC5;
        Brt_Save_Data.order[7] = 0xC1;
	
        while(Get_isstop(pos) != true)
        { 
            int ret = write(fd[pos],Brt_Save_Data.order,8);
            if(ret < 0)
            {
                printf("write fail %d\r\n",__LINE__);
                sleep(1);
            }
            else
            {
                printf("write data: %d pos :%d\r\n",ret,pos);
                bzero(Brt_Save_Data.SUDU_Buffer,sizeof(Brt_Save_Data.SUDU_Buffer));
                ret = read(fd[pos],Brt_Save_Data.SUDU_Buffer,10);
                if(ret < 0)
                {
                        printf("read fail %d\r\n",__LINE__);
                }
                else
                {
                    printf("read data: %d %d\r\n",ret,__LINE__);
#ifdef  __Debug_info
                    for(int i = 0;i < ret;i++)
                    {
                            printf("0x%x ",Brt_Save_Data.SUDU_Buffer[i]);
                    }
                    printf("\r\n");
#endif

                    for(int i = 0;i < ret;i++)
                    {
                        if(Brt_Save_Data.SUDU_Buffer[i] == 0x03 && Brt_Save_Data.SUDU_Buffer[i + 1] == 0x04)
                        {
                                Brt_Save_Data.pos = i + 1;
                                break;
                        }
                    }
                    sprintf(Brt_Save_Data.buf, "%x%x%x%x",Brt_Save_Data.SUDU_Buffer[ Brt_Save_Data.pos + 1],Brt_Save_Data.SUDU_Buffer[ Brt_Save_Data.pos + 2],Brt_Save_Data.SUDU_Buffer[ Brt_Save_Data.pos +3],Brt_Save_Data.SUDU_Buffer[ Brt_Save_Data.pos + 4]);

                    Brt_Save_Data.Data =  hexToDec(Brt_Save_Data.buf);
                    printf("SUDU_Data: %d\r\n", Brt_Save_Data.Data);
                    Brt_Save_Data.n =   (float)Brt_Save_Data.Data*0.00915;
                    printf("n: %f\r\n",Brt_Save_Data.n);
                }

                sleep(1);
                printf("\r\n");                    
            }
        }
        printf("brt线程退出\r\n");
        tcsetattr(fd[pos], TCSANOW, &old_cfg);//恢复到之前的配置
        close(fd[pos]);

}
void cb_brt_test_singal()
{
    int ret,pos = -1;
	struct termios old_cfg={0}; //用于保存终端配置之前的参数
	struct termios new_cfg={0}; //用于保存终端新配置的参数
	speed_t speed = B9600;		//定义波特率为9600
    /*第一步，串口初始化*/

    int loaction = -1;
    auto iter_order = dev_order.find("brt");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }

    for(map<const char*,int>::iterator it=pos_map.begin();it!=pos_map.end();it++)
    {
        if(strcmp(it->first,dev_name[loaction]) == 0)
        {
            pos = it->second;
            break;
        }
        else
        {
            printf("cb_brt_test pos_map没找到 %d\r\n",__LINE__);
        }
    }
    // printf("pos=%d\r\n ",pos);
    // cout << "port[pos]=" << port[pos] << endl;
	fd[pos] = open(port[pos].c_str(),O_RDWR | O_NOCTTY | O_NDELAY);//O_NOCTTY 标志，告知系统该节点不会成为进程的控制终端
    if(fd[pos] < 0)
    {
            printf("uart device open error\n");
            return;
    }
	ret = tcgetattr(fd[pos], &old_cfg);
	if(ret < 0)
    {
		printf("tcgetattr error\n");
		close(fd[pos]);
		return;
	}
	/*第二步，配置串口参数*/
	cfmakeraw(&new_cfg);//设置为原始模式 
	new_cfg.c_cflag |= CREAD;// 使能接收 
	cfsetspeed(&new_cfg, speed);//将波特率设置为9600
	new_cfg.c_cflag &= ~CSIZE; //将数据位相关的比特位清零
	new_cfg.c_cflag |= CS8;    //将数据位数设置为8位
	new_cfg.c_cflag &= ~PARENB;
	new_cfg.c_iflag &= ~INPCK;//设置为无校验模式
	new_cfg.c_cflag &= ~CSTOPB;//将停止位设置为1位
	new_cfg.c_cc[VTIME] = 0;// 将 MIN 和 TIME 设置为 0
	new_cfg.c_cc[VMIN] = 0;
	ret = tcflush(fd[pos], TCIOFLUSH);//清空缓冲区域

        if(ret < 0)
        {
            printf("tcflush error\n");
            return;
        }
        ret = tcsetattr(fd[pos], TCSANOW, &new_cfg);//写入配置、使配置生效
        if(ret < 0)
        {
            printf("tcsetattr error\n");
            return;
        }	
        Brt_Save_Data.order[0] = 0x01;
        Brt_Save_Data.order[1] = 0x03;
        Brt_Save_Data.order[2] = 0x00;
        Brt_Save_Data.order[3] = 0x20;
        Brt_Save_Data.order[4] = 0x00;
        Brt_Save_Data.order[5] = 0x02;
        Brt_Save_Data.order[6] = 0xC5;
        Brt_Save_Data.order[7] = 0xC1;
	
        while(Get_isstop(pos) != true)
        { 
            int ret = write(fd[pos],Brt_Save_Data.order,8);
            if(ret < 0)
            {
                printf("write fail %d\r\n",__LINE__);
                sleep(1);
            }
            else
            {
                // printf("write data: %d\r\n",ret);
                bzero(Brt_Save_Data.SUDU_Buffer,sizeof(Brt_Save_Data.SUDU_Buffer));
                ret = read(fd[pos],Brt_Save_Data.SUDU_Buffer,10);
                if(ret < 0)
                {
                    printf("read fail %d\r\n",__LINE__);
                }
                else
                {
                    // printf("read data: %d\r\n",ret);
#ifdef  __Debug_info
                    for(int i = 0;i < ret;i++)
                    {
                        printf("0x%x ",Brt_Save_Data.SUDU_Buffer[i]);
                    }
                    printf("\r\n");
#endif

                    for(int i = 0;i < ret;i++)
                    {
                        if(Brt_Save_Data.SUDU_Buffer[i] == 0x03 && Brt_Save_Data.SUDU_Buffer[i + 1] == 0x04)
                        {
                            Brt_Save_Data.pos = i + 1;
                            break;
                        }
                    }
                    sprintf(Brt_Save_Data.buf, "%x%x%x%x",Brt_Save_Data.SUDU_Buffer[ Brt_Save_Data.pos + 1],Brt_Save_Data.SUDU_Buffer[ Brt_Save_Data.pos + 2],Brt_Save_Data.SUDU_Buffer[ Brt_Save_Data.pos +3],Brt_Save_Data.SUDU_Buffer[ Brt_Save_Data.pos + 4]);
                    system("clear");
                    Brt_Save_Data.Data =  hexToDec(Brt_Save_Data.buf);
                    // printf("SUDU_Data: %d\t", Brt_Save_Data.Data);
                    Brt_Save_Data.n =   (float)Brt_Save_Data.Data*0.00915;
                    printf("n: %f\r\n",Brt_Save_Data.n);
                }
                usleep(100000);                  
            }
        }
        tcsetattr(fd[pos], TCSANOW, &old_cfg);//恢复到之前的配置
        close(fd[pos]);
}
/* 返回ch字符在sign数组中的序号 */
int getIndexOfSigns(char ch)
{
    if (ch >= '0' && ch <= '9')
    {
        return ch - '0';
    }
    if (ch >= 'A' && ch <= 'F')
    {
        return ch - 'A' + 10;
    }
    if (ch >= 'a' && ch <= 'f')
    {
        return ch - 'a' + 10;
    }
    return -1;

}
/* 十六进制数转换为十进制数 */
long hexToDec(char* source)
{
    long sum = 0;
    long t = 1;
    int i, len;

    len = (int)strlen(source);
    // printf("len:%d\r\n",len);
    for (i = len - 1; i >= 0; i--)
    {
        sum += t * getIndexOfSigns(*(source + i));
        t *= 16;
    }

    return sum;
}
/***********************************OID**************************************************/
void cb_oid_test()
{
    int ret,pos = -1;
	struct termios old_cfg={0}; //用于保存终端配置之前的参数
	struct termios new_cfg={0}; //用于保存终端新配置的参数
	speed_t speed = B9600;		//定义波特率为9600

    /*第一步，串口初始化*/
    for(map<const char*,int>::iterator it=pos_map.begin();it!=pos_map.end();it++)
    {
        if(strcmp(it->first,"/dev/ttyUSB1") == 0)
        {
            pos = it->second;
            break;
        }
        else
        {
            printf("cb_brt_test pos_map没找到 %d\r\n",__LINE__);
        }
    }
    
	fd[pos] = open(port[pos].c_str(),O_RDWR | O_NOCTTY | O_NDELAY);//O_NOCTTY 标志，告知系统该节点不会成为进程的控制终端
    if(fd[pos] < 0)
    {
        printf("uart device open error\n");
        return;
    }
	ret = tcgetattr(fd[pos], &old_cfg);
	if(ret < 0)
    {
		printf("tcgetattr error\n");
		close(fd[pos]);
		return;
	}
	/*第二步，配置串口参数*/
	cfmakeraw(&new_cfg);//设置为原始模式 
	new_cfg.c_cflag |= CREAD;// 使能接收 
	cfsetspeed(&new_cfg, speed);//将波特率设置为9600
	new_cfg.c_cflag &= ~CSIZE; //将数据位相关的比特位清零
	new_cfg.c_cflag |= CS8;    //将数据位数设置为8位
	new_cfg.c_cflag &= ~PARENB;
	new_cfg.c_iflag &= ~INPCK;//设置为无校验模式
	new_cfg.c_cflag &= ~CSTOPB;//将停止位设置为1位
	new_cfg.c_cc[VTIME] = 0;// 将 MIN 和 TIME 设置为 0
	new_cfg.c_cc[VMIN] = 0;
	ret = tcflush(fd[pos], TCIOFLUSH);//清空缓冲区域

    if(ret < 0)
    {
        printf("tcflush error\n");
        return;
    }
    ret = tcsetattr(fd[pos], TCSANOW, &new_cfg);//写入配置、使配置生效
    if(ret < 0)
    {
        printf("tcsetattr error\n");
        return;
    }	
    OID_Save_Data.order[0] = 0x01;
    OID_Save_Data.order[1] = 0x03;
    OID_Save_Data.order[2] = 0x00;
    OID_Save_Data.order[3] = 0x03;
    OID_Save_Data.order[4] = 0x00;
    OID_Save_Data.order[5] = 0x01;
    OID_Save_Data.order[6] = 0x74;
    OID_Save_Data.order[7] = 0x0A;
	
    while(Get_isstop(pos) != true)
    { 
        int ret = write(fd[pos],OID_Save_Data.order,8);
        if(ret < 0)
        {
                printf("write fail\r\n");
        }
        bzero(OID_Save_Data.SUDU_Buffer,sizeof(OID_Save_Data.SUDU_Buffer));
        ret = read(fd[pos],OID_Save_Data.SUDU_Buffer,10);
        if(ret < 0)
        {
                printf("read fail\r\n");
        }
        else
        {
#ifdef  __Debug_info
            for(int i = 0;i < ret;i++)
            {
                    printf("0x%x ",OID_Save_Data.SUDU_Buffer[i]);
            }
            printf("\r\n");
#endif

            for(int i = 0;i < ret;i++)
            {
                if(OID_Save_Data.SUDU_Buffer[i] == 0x03 && OID_Save_Data.SUDU_Buffer[i + 1] == 0x02)
                {
                        OID_Save_Data.pos = i + 1;
                        break;
                }
            }
            sprintf(OID_Save_Data.buf, "%x%x",OID_Save_Data.SUDU_Buffer[ OID_Save_Data.pos + 1],OID_Save_Data.SUDU_Buffer[ OID_Save_Data.pos + 2]);
            system("clear");
            OID_Save_Data.Data =  hexToDec(OID_Save_Data.buf);
            // printf("SUDU_Data: %d\r\n", OID_Save_Data.Data);
            OID_Save_Data.n =   (float)OID_Save_Data.Data*0.00915;
            printf("n: %f\r\n",OID_Save_Data.n);
        }
        usleep(100000);
    }
    tcsetattr(fd[pos], TCSANOW, &old_cfg);//恢复到之前的配置
    close(fd[pos]);

}
void cb_oid_test_singal()
{
    int loaction = -1;
    auto iter_order = dev_order.find("oid");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }

    int ret,pos = -1;
	struct termios old_cfg={0}; //用于保存终端配置之前的参数
	struct termios new_cfg={0}; //用于保存终端新配置的参数
	speed_t speed = B9600;		//定义波特率为9600

    /*第一步，串口初始化*/
    for(map<const char*,int>::iterator it=pos_map.begin();it!=pos_map.end();it++)
    {
        if(strcmp(it->first,dev_name[loaction]) == 0)
        {
            pos = it->second;
            break;
        }
        else
        {
            printf("cb_oid_test pos_map没找到 %d\r\n",__LINE__);
        }
    }
    
	fd[pos] = open(port[pos].c_str(),O_RDWR | O_NOCTTY | O_NDELAY);//O_NOCTTY 标志，告知系统该节点不会成为进程的控制终端
    if(fd[pos] < 0)
    {
            printf("uart device open error\n");
            return;
    }
	ret = tcgetattr(fd[pos], &old_cfg);
	if(ret < 0)
    {
		printf("tcgetattr error\n");
		close(fd[pos]);
		return;
	}
	/*第二步，配置串口参数*/
	cfmakeraw(&new_cfg);//设置为原始模式 
	new_cfg.c_cflag |= CREAD;// 使能接收 
	cfsetspeed(&new_cfg, speed);//将波特率设置为9600
	new_cfg.c_cflag &= ~CSIZE; //将数据位相关的比特位清零
	new_cfg.c_cflag |= CS8;    //将数据位数设置为8位
	new_cfg.c_cflag &= ~PARENB;
	new_cfg.c_iflag &= ~INPCK;//设置为无校验模式
	new_cfg.c_cflag &= ~CSTOPB;//将停止位设置为1位
	new_cfg.c_cc[VTIME] = 0;// 将 MIN 和 TIME 设置为 0
	new_cfg.c_cc[VMIN] = 0;
	ret = tcflush(fd[pos], TCIOFLUSH);//清空缓冲区域

    if(ret < 0)
    {
        printf("tcflush error\n");
        return;
    }
    ret = tcsetattr(fd[pos], TCSANOW, &new_cfg);//写入配置、使配置生效
    if(ret < 0)
    {
        printf("tcsetattr error\n");
        return;
    }	
    OID_Save_Data.order[0] = 0x01;
    OID_Save_Data.order[1] = 0x03;
    OID_Save_Data.order[2] = 0x00;
    OID_Save_Data.order[3] = 0x03;
    OID_Save_Data.order[4] = 0x00;
    OID_Save_Data.order[5] = 0x01;
    OID_Save_Data.order[6] = 0x74;
    OID_Save_Data.order[7] = 0x0A;
	
    while(Get_isstop(pos) != true)
    { 
        int ret = write(fd[pos],OID_Save_Data.order,8);
        if(ret < 0)
        {
                printf("write fail\r\n");
        }
        bzero(OID_Save_Data.SUDU_Buffer,sizeof(OID_Save_Data.SUDU_Buffer));
        ret = read(fd[pos],OID_Save_Data.SUDU_Buffer,10);
        if(ret < 0)
        {
                printf("read fail\r\n");
        }
        else
        {
#ifdef  __Debug_info
            for(int i = 0;i < ret;i++)
            {
                    printf("0x%x ",OID_Save_Data.SUDU_Buffer[i]);
            }
            printf("\r\n");
#endif

            for(int i = 0;i < ret;i++)
            {
                if(OID_Save_Data.SUDU_Buffer[i] == 0x03 && OID_Save_Data.SUDU_Buffer[i + 1] == 0x02)
                {
                        OID_Save_Data.pos = i + 1;
                        break;
                }
            }
            sprintf(OID_Save_Data.buf, "%x%x",OID_Save_Data.SUDU_Buffer[ OID_Save_Data.pos + 1],OID_Save_Data.SUDU_Buffer[ OID_Save_Data.pos + 2]);
            system("clear");
            OID_Save_Data.Data =  hexToDec(OID_Save_Data.buf);
            // printf("SUDU_Data: %d\r\n", OID_Save_Data.Data);
            OID_Save_Data.n =   (float)OID_Save_Data.Data*0.00915;
            printf("n: %f\r\n",OID_Save_Data.n);
        }
        usleep(100000);
    }
    tcsetattr(fd[pos], TCSANOW, &old_cfg);//恢复到之前的配置
    close(fd[pos]);

}
/* 返回ch字符在sign数组中的序号 */
int oid_getIndexOfSigns(char ch)
{
    if (ch >= '0' && ch <= '9')
    {
        return ch - '0';
    }
    if (ch >= 'A' && ch <= 'F')
    {
        return ch - 'A' + 10;
    }
    if (ch >= 'a' && ch <= 'f')
    {
        return ch - 'a' + 10;
    }
    return -1;

}
/* 十六进制数转换为十进制数 */
long oid_hexToDec(char* source)
{
    long sum = 0;
    long t = 1;
    int i, len;

    len = (int)strlen(source);
    printf("len:%d\r\n",len);
    for (i = len - 1; i >= 0; i--)
    {
        sum += t * getIndexOfSigns(*(source + i));
        t *= 16;
    }

    return sum;
}
/***********************************姿态**************************************************/
/***********************************WIT**************************************************/
void cb_wit_test_singal()
{
    uart_cfg_t cfg = {0};
    int loaction = -1;
    auto iter_order = dev_order.find("wit");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }
    //串口初始
    wit_uart_init(dev_name[loaction]);
    //设置串口参数 115200 8 1 无校验
    wit_uart_cfg(&cfg);
    int ret = 0;
    int pos = -1;
     /*第一步，串口初始化*/

    for(map<const char*,int>::iterator it=pos_map.begin();it!=pos_map.end();it++)
    {
        if(strcmp(it->first,dev_name[loaction]) == 0)
        {
            pos = it->second;
            break;
        }
        else
        {
            printf("cb_wit_test pos_map没找到 %d\r\n",__LINE__);
        }
    }
    while(Get_isstop(pos) != true)
    { 
        
        ret = read(fd[pos], WIT_Save_Data.WIT_Buffer, sizeof(WIT_Save_Data.WIT_Buffer)-1);
        WIT_Save_Data.WIT_Buffer[ret] = 0;
        if (ret < 0)
        {
            printf("read failed\n");
        }
        else if (ret > 0)
        {
            
           //原始数据
#ifdef __Debug_info
           for (int i = 0; i < WIT_Buffer_Length; i++)
           {
                printf("0x%x ", WIT_Save_Data.WIT_Buffer[i]);              
           } 
           printf("\r\n");         
#endif
            //查找关键数据位置

            int i = 0;
            for (i = 0; i < 200; i++)
            {
                //寻找角速度
                if(WIT_Save_Data.WIT_Buffer[i] == 0x55 && WIT_Save_Data.WIT_Buffer[i + 1] == 0x52)
                {
                WIT_Save_Data.isGet = 1;       
#ifdef __Debug_info
                printf("获取到角速度关键数据帧\r\n");  
                printf("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x \r\n",WIT_Save_Data.WIT_Buffer[i + 2],WIT_Save_Data.WIT_Buffer[i + 3],WIT_Save_Data.WIT_Buffer[i + 4],WIT_Save_Data.WIT_Buffer[i + 5],WIT_Save_Data.WIT_Buffer[i + 6],WIT_Save_Data.WIT_Buffer[i + 7]);
#endif               
               WIT_Save_Data.pos = i + 2; 
                        break;
                }
            }

            for (i = 0; i < 200; i++)
            {
                //寻找角加速度
                if(WIT_Save_Data.WIT_Buffer[i] == 0x55 && WIT_Save_Data.WIT_Buffer[i + 1] == 0x51)
                {
                WIT_Save_Data.a_isGet = 1;       
#ifdef __Debug_info
                printf("获取到角加速度关键数据帧\r\n");  
                printf("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x \r\n",WIT_Save_Data.WIT_Buffer[i + 2],WIT_Save_Data.WIT_Buffer[i + 3],WIT_Save_Data.WIT_Buffer[i + 4],WIT_Save_Data.WIT_Buffer[i + 5],WIT_Save_Data.WIT_Buffer[i + 6],WIT_Save_Data.WIT_Buffer[i + 7]);
#endif        
                WIT_Save_Data.a_pos = i + 2;       
                break; 
                }
            }

            for (i = 0; i < 200; i++)
            {
                //寻找角度
                if(WIT_Save_Data.WIT_Buffer[i] == 0x55 && WIT_Save_Data.WIT_Buffer[i + 1] == 0x53)
                {
                WIT_Save_Data.j_isGet = 1;       
#ifdef __Debug_info
                printf("获取到角度关键数据帧\r\n");  
                printf("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x \r\n",WIT_Save_Data.WIT_Buffer[i + 2],WIT_Save_Data.WIT_Buffer[i + 3],WIT_Save_Data.WIT_Buffer[i + 4],WIT_Save_Data.WIT_Buffer[i + 5],WIT_Save_Data.WIT_Buffer[i + 6],WIT_Save_Data.WIT_Buffer[i + 7]);
#endif               
                WIT_Save_Data.j_pos = i + 2;     
                break;       
                }
            }
            system("clear");
            //解析角速度
            if(WIT_Save_Data.isGet)
            {
                WIT_Save_Data.isGet = 0;  
                //计算角速度
                WIT_Save_Data.raw.x =  (uint32_t)(WIT_Save_Data.WIT_Buffer[WIT_Save_Data.pos+1] << 8)|WIT_Save_Data.WIT_Buffer[WIT_Save_Data.pos];
                WIT_Save_Data.pos = WIT_Save_Data.pos + 2;
                WIT_Save_Data.raw.y =  (uint32_t)(WIT_Save_Data.WIT_Buffer[WIT_Save_Data.pos+1] << 8)|WIT_Save_Data.WIT_Buffer[WIT_Save_Data.pos];        
                WIT_Save_Data.pos = WIT_Save_Data.pos + 2;
                WIT_Save_Data.raw.z =  (uint32_t)(WIT_Save_Data.WIT_Buffer[WIT_Save_Data.pos+1] << 8)|WIT_Save_Data.WIT_Buffer[WIT_Save_Data.pos];
                
                WIT_Save_Data.x = (float)WIT_Save_Data.raw.x / 32768 *2000;
                WIT_Save_Data.y = (float)WIT_Save_Data.raw.y / 32768 *2000;
                WIT_Save_Data.z = (float)WIT_Save_Data.raw.z / 32768 *2000;



                 
                // printf("Gx: %.02f Gy: %.02f Gz: %.02f\r\n",WIT_Save_Data.x,WIT_Save_Data.y,WIT_Save_Data.z);    
             
            }

            //解析角加速度
            if(WIT_Save_Data.a_isGet)
            {
                WIT_Save_Data.a_isGet = 0;  
                //计算角加速度

                WIT_Save_Data.raw.x =  (uint32_t)(WIT_Save_Data.WIT_Buffer[WIT_Save_Data.a_pos+1] << 8)|WIT_Save_Data.WIT_Buffer[WIT_Save_Data.a_pos];
                WIT_Save_Data.a_pos = WIT_Save_Data.a_pos + 2;
                WIT_Save_Data.raw.y =  (uint32_t)(WIT_Save_Data.WIT_Buffer[WIT_Save_Data.a_pos+1] << 8)|WIT_Save_Data.WIT_Buffer[WIT_Save_Data.a_pos];        
                WIT_Save_Data.a_pos = WIT_Save_Data.a_pos + 2;
                WIT_Save_Data.raw.z =  (uint32_t)(WIT_Save_Data.WIT_Buffer[WIT_Save_Data.a_pos+1] << 8)|WIT_Save_Data.WIT_Buffer[WIT_Save_Data.a_pos];
                

                WIT_Save_Data.a_x = (float)WIT_Save_Data.raw.x / 32768 *16*9.8f;
                WIT_Save_Data.a_y = (float)WIT_Save_Data.raw.y / 32768 *16*9.8f;
                WIT_Save_Data.a_z = (float)WIT_Save_Data.raw.z / 32768 *16*9.8f;



            
                // printf("ax: %.02f ay: %.02f az: %.02f\r\n",WIT_Save_Data.a_x,WIT_Save_Data.a_y,WIT_Save_Data.a_z);    
             
            }
    
            //解析角度
            if(WIT_Save_Data.j_isGet)
            {
                WIT_Save_Data.j_isGet = 0;  
                //计算角度

                WIT_Save_Data.raw.x =  (uint32_t)(WIT_Save_Data.WIT_Buffer[WIT_Save_Data.j_pos+1] << 8)|WIT_Save_Data.WIT_Buffer[WIT_Save_Data.j_pos];
                WIT_Save_Data.j_pos = WIT_Save_Data.j_pos + 2;
                WIT_Save_Data.raw.y =  (uint32_t)(WIT_Save_Data.WIT_Buffer[WIT_Save_Data.j_pos+1] << 8)|WIT_Save_Data.WIT_Buffer[WIT_Save_Data.j_pos];        
                WIT_Save_Data.j_pos = WIT_Save_Data.j_pos + 2;
                WIT_Save_Data.raw.z =  (uint32_t)(WIT_Save_Data.WIT_Buffer[WIT_Save_Data.j_pos+1] << 8)|WIT_Save_Data.WIT_Buffer[WIT_Save_Data.j_pos];
                
                WIT_Save_Data.roll = (float)WIT_Save_Data.raw.x / 32768 *180.0f;
                WIT_Save_Data.pitch = (float)WIT_Save_Data.raw.y / 32768 *180.0f;
                WIT_Save_Data.yaw = (float)WIT_Save_Data.raw.z / 32768 *180.0f;

                printf("Gx: %.02f  Gy: %.02f  Gz: %.02f  ax: %.02f  ay: %.02f  az: %.02f  roll: %.02f  pitch: %.02f  yaw: %.02f\r\n",\
                WIT_Save_Data.x,WIT_Save_Data.y,WIT_Save_Data.z,WIT_Save_Data.a_x,WIT_Save_Data.a_y,WIT_Save_Data.a_z,WIT_Save_Data.roll,WIT_Save_Data.pitch,WIT_Save_Data.yaw);    
             
            }
        }
        usleep(100000);
    }
    /* 退出 */
    tcsetattr(fd[pos], TCSANOW, &wit_old_cfg);   //恢复到之前的配置
    close(fd[pos]);
}
//串口初始化
static int wit_uart_init(const char *device)
{
    int pos = -1;
    int loaction = -1;
    auto iter_order = dev_order.find("wit");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }

     /*第一步，串口初始化*/
    for(map<const char*,int>::iterator it=pos_map.begin();it!=pos_map.end();it++)
    {
        if(strcmp(it->first,dev_name[loaction]) == 0)
        {
            pos = it->second;
            break;
        }
        else
        {
            printf("cb_brt_test pos_map没找到 %d\r\n",__LINE__);
        }
    }


    /* 打开串口终端  使用的标志有 可读可写，告诉系统该节点不会成为进程的控制终端，非阻塞方式，读不到数据返回-1,*/
    fd[pos] = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (0 > fd[pos]) {
        fprintf(stderr, "open error: %s: %s\n", device, strerror(errno));
        return -1;
    }

    /* 获取串口当前的配置参数 */
    if (0 > tcgetattr(fd[pos], &wit_old_cfg)) 
    {
        fprintf(stderr, "tcgetattr error: %s\n", strerror(errno));
        close(fd[pos]);
        return -1;
    }

    return 0;
}
//串口配置
static int wit_uart_cfg(const uart_cfg_t *cfg)
{
    int loaction = -1;
    auto iter_order = dev_order.find("wit");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }
    int pos = -1;
     /*第一步，串口初始化*/
    for(map<const char*,int>::iterator it=pos_map.begin();it!=pos_map.end();it++)
    {
        if(strcmp(it->first,dev_name[loaction]) == 0)
        {
            pos = it->second;
            break;
        }
        else
        {
            printf("cb_brt_test pos_map没找到 %d\r\n",__LINE__);
        }
    }

    struct termios new_cfg = {0};   //将new_cfg对象清零
    speed_t speed;

    /* 设置为原始模式 */
    cfmakeraw(&new_cfg);

    /* 使能接收 */
    new_cfg.c_cflag |= CREAD;

    /* 设置波特率 */
    switch (cfg->baudrate) {
        case 1200: speed = B1200;
            break;
        case 1800: speed = B1800;
            break;
        case 2400: speed = B2400;
            break;
        case 4800: speed = B4800;
            break;
        case 9600: speed = B9600;
            break;
        case 19200: speed = B19200;
            break;
        case 38400: speed = B38400;
            break;
        case 57600: speed = B57600;
            break;
        case 115200: speed = B115200;
            break;
        case 230400: speed = B230400;
            break;
        case 460800: speed = B460800;
            break;
        case 500000: speed = B500000;
            break;
        default:    //默认配置为115200
            speed = B9600;
            // printf("default baud rate: 9600\n");
            break;
    }

    if (0 > cfsetspeed(&new_cfg, speed)) 
    {
        fprintf(stderr, "cfsetspeed error: %s\n", strerror(errno));
        return -1;
    }

    /* 设置数据位大小 */
    new_cfg.c_cflag &= ~CSIZE;  //将数据位相关的比特位清零
    switch (cfg->dbit) {
        case 5:
            new_cfg.c_cflag |= CS5;
            break;
        case 6:
            new_cfg.c_cflag |= CS6;
            break;
        case 7:
            new_cfg.c_cflag |= CS7;
            break;
        case 8:
            new_cfg.c_cflag |= CS8;
            break;
        default:    //默认数据位大小为8
            new_cfg.c_cflag |= CS8;
            // printf("default data bit size: 8\n");
            break;
    }

    /* 设置奇偶校验 */
    switch (cfg->parity) {
        case 'N':       //无校验
            new_cfg.c_cflag &= ~PARENB;
            new_cfg.c_iflag &= ~INPCK;
            break;
        case 'O':       //奇校验
            new_cfg.c_cflag |= (PARODD | PARENB);
            new_cfg.c_iflag |= INPCK;
            break;
        case 'E':       //偶校验
            new_cfg.c_cflag |= PARENB;
            new_cfg.c_cflag &= ~PARODD; /* 清除PARODD标志，配置为偶校验 */
            new_cfg.c_iflag |= INPCK;
            break;
        default:    //默认配置为无校验
            new_cfg.c_cflag &= ~PARENB;
            new_cfg.c_iflag &= ~INPCK;
            // printf("default parity: N\n");
            break;
    }

    /* 设置停止位 */
    switch (cfg->sbit) {
        case 1:     //1个停止位
            new_cfg.c_cflag &= ~CSTOPB;
            break;
        case 2:     //2个停止位
            new_cfg.c_cflag |= CSTOPB;
            break;
        default:    //默认配置为1个停止位
            new_cfg.c_cflag &= ~CSTOPB;
            // printf("default stop bit size: 1\n");
            break;
    }

    /* 将MIN和TIME设置为0 */
    new_cfg.c_cc[VTIME] = 0;
    new_cfg.c_cc[VMIN] = 0;

    /* 清空缓冲区 */
    if (0 > tcflush(fd[pos], TCIOFLUSH)) {
        fprintf(stderr, "tcflush error: %s\n", strerror(errno));
        return -1;
    }

    /* 写入配置、使配置生效 */
    if (0 > tcsetattr(fd[pos], TCSANOW, &new_cfg)) {
        fprintf(stderr, "tcsetattr error: %s\n", strerror(errno));
        return -1;
    }

    /* 配置OK 退出 */
    return 0;
}
/***********************************JY9**************************************************/
void cb_jy9_test_singal()
{
    uart_cfg_t cfg = {0};
    int loaction = -1;
    auto iter_order = dev_order.find("jy9");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }
    //串口初始
    jy9_uart_init(dev_name[loaction]);
    //设置串口参数 9600 8 1 无校验
    jy9_uart_cfg(&cfg);
    int ret = 0;
    int pos = -1;
     /*第一步，串口初始化*/
    for(map<const char*,int>::iterator it=pos_map.begin();it!=pos_map.end();it++)
    {
        if(strcmp(it->first,dev_name[loaction]) == 0)
        {
            pos = it->second;
            break;
        }
        else
        {
            printf("cb_jy9_test pos_map没找到 %d\r\n",__LINE__);
        }
    }
    while(Get_isstop(pos) != true)
    { 
        ret = read(fd[pos], JY9_Save_Data.JY9_Buffer, sizeof(JY9_Save_Data.JY9_Buffer)-1);
        JY9_Save_Data.JY9_Buffer[ret] = 0;
        if (ret < 0)
        {
            printf("read failed\n");
        }
        else if (ret > 0)
        {
            
        //原始数据
#ifdef __Debug_info
           for (int i = 0; i < WIT_Buffer_Length; i++)
           {
                printf("0x%x ", JY9_Save_Data.JY9_Buffer[i]);              
           } 
           printf("\r\n");         
#endif
            //查找关键数据位置

            int i = 0;
            for (i = 0; i < 200; i++)
            {
                //寻找角速度
                if(JY9_Save_Data.JY9_Buffer[i] == 0x55 && JY9_Save_Data.JY9_Buffer[i + 1] == 0x52)
                {
                JY9_Save_Data.isGet = 1;       
#ifdef __Debug_info
                printf("获取到角速度关键数据帧\r\n");  
                printf("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x \r\n",JY9_Save_Data.JY9_Buffer[i + 2],JY9_Save_Data.JY9_Buffer[i + 3],JY9_Save_Data.JY9_Buffer[i + 4],JY9_Save_Data.JY9_Buffer[i + 5],JY9_Save_Data.JY9_Buffer[i + 6],JY9_Save_Data.JY9_Buffer[i + 7]);
#endif               
               JY9_Save_Data.pos = i + 2; 
                        break;
                }
            }

            for (i = 0; i < 200; i++)
            {
                //寻找角加速度
                if(JY9_Save_Data.JY9_Buffer[i] == 0x55 && JY9_Save_Data.JY9_Buffer[i + 1] == 0x51)
                {
                JY9_Save_Data.a_isGet = 1;       
#ifdef __Debug_info
                printf("获取到角加速度关键数据帧\r\n");  
                printf("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x \r\n",JY9_Save_Data.JY9_Buffer[i + 2],JY9_Save_Data.JY9_Buffer[i + 3],JY9_Save_Data.JY9_Buffer[i + 4],JY9_Save_Data.JY9_Buffer[i + 5],JY9_Save_Data.JY9_Buffer[i + 6],JY9_Save_Data.JY9_Buffer[i + 7]);
#endif        
                JY9_Save_Data.a_pos = i + 2;       
                break; 
                }
            }

            for (i = 0; i < 200; i++)
            {
                //寻找角度
                if(JY9_Save_Data.JY9_Buffer[i] == 0x55 && JY9_Save_Data.JY9_Buffer[i + 1] == 0x53)
                {
                JY9_Save_Data.j_isGet = 1;       
#ifdef __Debug_info
                printf("获取到角度关键数据帧\r\n");  
                printf("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x \r\n",JY9_Save_Data.JY9_Buffer[i + 2],JY9_Save_Data.JY9_Buffer[i + 3],JY9_Save_Data.JY9_Buffer[i + 4],JY9_Save_Data.JY9_Buffer[i + 5],JY9_Save_Data.JY9_Buffer[i + 6],JY9_Save_Data.JY9_Buffer[i + 7]);
#endif               
                JY9_Save_Data.j_pos = i + 2;     
                break;       
                }
            }
            system("clear");
            //解析角速度
            if(JY9_Save_Data.isGet)
            {
                JY9_Save_Data.isGet = 0;  
                //计算角速度
                JY9_Save_Data.raw.x =  (uint32_t)(JY9_Save_Data.JY9_Buffer[JY9_Save_Data.pos+1] << 8)|JY9_Save_Data.JY9_Buffer[JY9_Save_Data.pos];
                JY9_Save_Data.pos = JY9_Save_Data.pos + 2;
                JY9_Save_Data.raw.y =  (uint32_t)(JY9_Save_Data.JY9_Buffer[JY9_Save_Data.pos+1] << 8)|JY9_Save_Data.JY9_Buffer[JY9_Save_Data.pos];        
                JY9_Save_Data.pos = JY9_Save_Data.pos + 2;
                JY9_Save_Data.raw.z =  (uint32_t)(JY9_Save_Data.JY9_Buffer[JY9_Save_Data.pos+1] << 8)|JY9_Save_Data.JY9_Buffer[JY9_Save_Data.pos];
                
                JY9_Save_Data.x = (float)JY9_Save_Data.raw.x / 32768 *2000;
                JY9_Save_Data.y = (float)JY9_Save_Data.raw.y / 32768 *2000;
                JY9_Save_Data.z = (float)JY9_Save_Data.raw.z / 32768 *2000;



                 
                // printf("Gx: %.02f Gy: %.02f Gz: %.02f\r\n",JY9_Save_Data.x,JY9_Save_Data.y,JY9_Save_Data.z);    
             
            }

            //解析角加速度
            if(JY9_Save_Data.a_isGet)
            {
                JY9_Save_Data.a_isGet = 0;  
                //计算角加速度

                JY9_Save_Data.raw.x =  (uint32_t)(JY9_Save_Data.JY9_Buffer[JY9_Save_Data.a_pos+1] << 8)|JY9_Save_Data.JY9_Buffer[JY9_Save_Data.a_pos];
                JY9_Save_Data.a_pos = JY9_Save_Data.a_pos + 2;
                JY9_Save_Data.raw.y =  (uint32_t)(JY9_Save_Data.JY9_Buffer[JY9_Save_Data.a_pos+1] << 8)|JY9_Save_Data.JY9_Buffer[JY9_Save_Data.a_pos];        
                JY9_Save_Data.a_pos = JY9_Save_Data.a_pos + 2;
                JY9_Save_Data.raw.z =  (uint32_t)(JY9_Save_Data.JY9_Buffer[JY9_Save_Data.a_pos+1] << 8)|JY9_Save_Data.JY9_Buffer[JY9_Save_Data.a_pos];
                

                JY9_Save_Data.a_x = (float)JY9_Save_Data.raw.x / 32768 *16*9.8f;
                JY9_Save_Data.a_y = (float)JY9_Save_Data.raw.y / 32768 *16*9.8f;
                JY9_Save_Data.a_z = (float)JY9_Save_Data.raw.z / 32768 *16*9.8f;



            
                // printf("ax: %.02f ay: %.02f az: %.02f\r\n",JY9_Save_Data.a_x,JY9_Save_Data.a_y,JY9_Save_Data.a_z);    
             
            }
    
            //解析角度
            if(JY9_Save_Data.j_isGet)
            {
                JY9_Save_Data.j_isGet = 0;  
                //计算角度

                JY9_Save_Data.raw.x =  (uint32_t)(JY9_Save_Data.JY9_Buffer[JY9_Save_Data.j_pos+1] << 8)|JY9_Save_Data.JY9_Buffer[JY9_Save_Data.j_pos];
                JY9_Save_Data.j_pos = JY9_Save_Data.j_pos + 2;
                JY9_Save_Data.raw.y =  (uint32_t)(JY9_Save_Data.JY9_Buffer[JY9_Save_Data.j_pos+1] << 8)|JY9_Save_Data.JY9_Buffer[JY9_Save_Data.j_pos];        
                JY9_Save_Data.j_pos = JY9_Save_Data.j_pos + 2;
                JY9_Save_Data.raw.z =  (uint32_t)(JY9_Save_Data.JY9_Buffer[JY9_Save_Data.j_pos+1] << 8)|JY9_Save_Data.JY9_Buffer[JY9_Save_Data.j_pos];
                
                JY9_Save_Data.roll = (float)JY9_Save_Data.raw.x / 32768 *180.0f;
                JY9_Save_Data.pitch = (float)JY9_Save_Data.raw.y / 32768 *180.0f;
                JY9_Save_Data.yaw = (float)JY9_Save_Data.raw.z / 32768 *180.0f;

                printf("Gx: %.02f  Gy: %.02f  Gz: %.02f  ax: %.02f  ay: %.02f  az: %.02f  roll: %.02f  pitch: %.02f  yaw: %.02f\r\n",\
                JY9_Save_Data.x,JY9_Save_Data.y,JY9_Save_Data.z,JY9_Save_Data.a_x,JY9_Save_Data.a_y,JY9_Save_Data.a_z,JY9_Save_Data.roll,JY9_Save_Data.pitch,JY9_Save_Data.yaw);    
             
            }
        }
        usleep(100000);
    }
    /* 退出 */
    tcsetattr(fd[pos], TCSANOW, &JY9_old_cfg);   //恢复到之前的配置
    close(fd[pos]);
}
//串口初始化
static int jy9_uart_init(const char *device)
{
    int loaction = -1;
    auto iter_order = dev_order.find("jy9");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }

    int pos = -1;
     /*第一步，串口初始化*/
    for(map<const char*,int>::iterator it=pos_map.begin();it!=pos_map.end();it++)
    {
        if(strcmp(it->first,dev_name[loaction]) == 0)
        {
            pos = it->second;
            break;
        }
        else
        {
            printf("cb_jy9_test pos_map没找到 %d\r\n",__LINE__);
        }
    }


    /* 打开串口终端  使用的标志有 可读可写，告诉系统该节点不会成为进程的控制终端，非阻塞方式，读不到数据返回-1,*/
    fd[pos] = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (0 > fd[pos]) {
        fprintf(stderr, "open error: %s: %s\n", device, strerror(errno));
        return -1;
    }

    /* 获取串口当前的配置参数 */
    if (0 > tcgetattr(fd[pos], &JY9_old_cfg)) 
    {
        fprintf(stderr, "tcgetattr error: %s\n", strerror(errno));
        close(fd[pos]);
        return -1;
    }

    return 0;
}
//串口配置
static int jy9_uart_cfg(const uart_cfg_t *cfg)
{
    int loaction = -1;
    auto iter_order = dev_order.find("jy9");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }

    int pos = -1;
     /*第一步，串口初始化*/
    for(map<const char*,int>::iterator it=pos_map.begin();it!=pos_map.end();it++)
    {
        if(strcmp(it->first,dev_name[loaction]) == 0)
        {
            pos = it->second;
            break;
        }
        else
        {
            printf("cb_jy9_test pos_map没找到 %d\r\n",__LINE__);
        }
    }

    struct termios new_cfg = {0};   //将new_cfg对象清零
    speed_t speed;

    /* 设置为原始模式 */
    cfmakeraw(&new_cfg);

    /* 使能接收 */
    new_cfg.c_cflag |= CREAD;

    /* 设置波特率 */
    switch (cfg->baudrate) {
        case 1200: speed = B1200;
            break;
        case 1800: speed = B1800;
            break;
        case 2400: speed = B2400;
            break;
        case 4800: speed = B4800;
            break;
        case 9600: speed = B9600;
            break;
        case 19200: speed = B19200;
            break;
        case 38400: speed = B38400;
            break;
        case 57600: speed = B57600;
            break;
        case 115200: speed = B115200;
            break;
        case 230400: speed = B230400;
            break;
        case 460800: speed = B460800;
            break;
        case 500000: speed = B500000;
            break;
        default:    //默认配置为115200
            speed = B9600;
            // printf("default baud rate: 9600\n");
            break;
    }

    if (0 > cfsetspeed(&new_cfg, speed)) 
    {
        fprintf(stderr, "cfsetspeed error: %s\n", strerror(errno));
        return -1;
    }

    /* 设置数据位大小 */
    new_cfg.c_cflag &= ~CSIZE;  //将数据位相关的比特位清零
    switch (cfg->dbit) {
        case 5:
            new_cfg.c_cflag |= CS5;
            break;
        case 6:
            new_cfg.c_cflag |= CS6;
            break;
        case 7:
            new_cfg.c_cflag |= CS7;
            break;
        case 8:
            new_cfg.c_cflag |= CS8;
            break;
        default:    //默认数据位大小为8
            new_cfg.c_cflag |= CS8;
            // printf("default data bit size: 8\n");
            break;
    }

    /* 设置奇偶校验 */
    switch (cfg->parity) {
        case 'N':       //无校验
            new_cfg.c_cflag &= ~PARENB;
            new_cfg.c_iflag &= ~INPCK;
            break;
        case 'O':       //奇校验
            new_cfg.c_cflag |= (PARODD | PARENB);
            new_cfg.c_iflag |= INPCK;
            break;
        case 'E':       //偶校验
            new_cfg.c_cflag |= PARENB;
            new_cfg.c_cflag &= ~PARODD; /* 清除PARODD标志，配置为偶校验 */
            new_cfg.c_iflag |= INPCK;
            break;
        default:    //默认配置为无校验
            new_cfg.c_cflag &= ~PARENB;
            new_cfg.c_iflag &= ~INPCK;
            // printf("default parity: N\n");
            break;
    }

    /* 设置停止位 */
    switch (cfg->sbit) {
        case 1:     //1个停止位
            new_cfg.c_cflag &= ~CSTOPB;
            break;
        case 2:     //2个停止位
            new_cfg.c_cflag |= CSTOPB;
            break;
        default:    //默认配置为1个停止位
            new_cfg.c_cflag &= ~CSTOPB;
            // printf("default stop bit size: 1\n");
            break;
    }

    /* 将MIN和TIME设置为0 */
    new_cfg.c_cc[VTIME] = 0;
    new_cfg.c_cc[VMIN] = 0;

    /* 清空缓冲区 */
    if (0 > tcflush(fd[pos], TCIOFLUSH)) {
        fprintf(stderr, "tcflush error: %s\n", strerror(errno));
        return -1;
    }

    /* 写入配置、使配置生效 */
    if (0 > tcsetattr(fd[pos], TCSANOW, &new_cfg)) {
        fprintf(stderr, "tcsetattr error: %s\n", strerror(errno));
        return -1;
    }

    /* 配置OK 退出 */
    return 0;
}
/***********************************位置**************************************************/
/***********************************ATGM336H*********************************************/
void cb_gps_test_singal()
{
    int loaction = -1;
    auto iter_order = dev_order.find("atgm");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }

    /* 清零操作 */
    bzero(GPS_Save_Data.Data,sizeof(GPS_Save_Data.Data));
    bzero(GPS_Save_Data.GNRMC,sizeof(GPS_Save_Data.GNRMC));
    bzero(GPS_Save_Data.GNGGA,sizeof(GPS_Save_Data.GNGGA));
    bzero(GPS_Save_Data.GNVTG,sizeof(GPS_Save_Data.GNVTG));
    bzero(GPS_Save_Data.GPS_Buffer,sizeof(GPS_Save_Data.GPS_Buffer));   
    
    //初始化帧GNRMC 帧数据
    GNRMC.pos = 0;
    GNRMC.start = nullptr;
    GNRMC.end = nullptr;
    GNRMC.find = false;
    GNRMC.Give_up = false;

    //初始化帧GNGGA 帧数据
    GNGGA.pos = 0;
    GNGGA.start = nullptr;
    GNGGA.end = nullptr;
    GNGGA.find = false;
    GNGGA.Give_up = false;

    //初始化帧GNVTG 帧数据
    GNVTG.pos = 0;
    GNVTG.start = nullptr;
    GNVTG.end = nullptr;
    GNVTG.find = false;
    GNVTG.Give_up = false;

    int re_value = 0;//每次获取的一帧数据
    int flag = 1;//获取数据和处理数据互斥标志位
    int count = 0;//获取填充数据的有效数据次数
    GPS_Save_Data.m_read_idx = 0;//用以限制获取的数据量
    char * buf = GPS_Save_Data.GPS_Buffer;

    uart_cfg_t cfg = {0};
    //串口初始
    gps_uart_init(dev_name[loaction]);
    //设置串口参数 115200 8 1 无校验
    gps_uart_cfg(&cfg);
    int ret = 0;
    int pos = -1;
     /*第一步，串口初始化*/
    for(map<const char*,int>::iterator it=pos_map.begin();it!=pos_map.end();it++)
    {
        if(strcmp(it->first,dev_name[loaction]) == 0)
        {
            pos = it->second;
            break;
        }
        else
        {
            printf("cb_brt_test pos_map没找到 %d\r\n",__LINE__);
        }
    }
    while(Get_isstop(pos) != true)
    {

        if(flag == 1)
        {
            re_value = read(fd[pos], GPS_Save_Data.Data, GPS_Buffer_Length_2);
            // printf("re_value:%d\r\n",re_value);                         
            if(re_value > 0)
            {
                
                if( GPS_Save_Data.m_read_idx + re_value >= GPS_Buffer_Length_4)
                {
                    GPS_Save_Data.m_read_idx = GPS_Buffer_Length_4;
                    flag = 0;
                }
                else
                {
                    count++;
                    GPS_Save_Data.m_read_idx +=  re_value;
                }
                if( GPS_Save_Data.m_read_idx < GPS_Buffer_Length_4)
                {
                    strncpy(buf,GPS_Save_Data.Data,re_value);
                    buf += re_value;
//打印读取一帧的数据
#if 0
                    for(int i = 0;i < re_value;i++)
                    {
                        printf("%c",GPS_Save_Data.Data[i]);
                    }
#endif
                }                
            }
        }
        else
        {
            //判断是否接收到了原始数据集
            if(GPS_Save_Data.m_read_idx == GPS_Buffer_Length_4)
            {
#if 0        
                //打印原始数据
                for(int i = 0;i < GPS_Buffer_Length_4;i++)
                {
                    printf("%c",GPS_Save_Data.GPS_Buffer[i]);
                }
#endif
#ifdef __Debug_info
                printf("\r\n");
#endif
                //寻找&GNRMC  获取经纬度信息
                for (int i = 0; i < GPS_Buffer_Length_4; i++)
                {
                    if (GPS_Save_Data.GPS_Buffer[i] == '$' && GPS_Save_Data.GPS_Buffer[i + 1] == 'G' && GPS_Save_Data.GPS_Buffer[i + 2] == 'N' && GPS_Save_Data.GPS_Buffer[i + 3] == 'R' && GPS_Save_Data.GPS_Buffer[i + 4] == 'M' && GPS_Save_Data.GPS_Buffer[i + 5] == 'C')
                    {
                        GNRMC.pos = i;
                        GNRMC.find = true;//标志找到数据帧
#ifdef __Debug_info
                        printf(" GNRMC pos = %d\r\n",GNRMC.pos);
                        printf("\r\n");
#endif
                        GNRMC.start = &GPS_Save_Data.GPS_Buffer[GNRMC.pos];
                        GNRMC.end = strstr(GNRMC.start + 1,"$");
                        if(GNRMC.end == nullptr)
                        {
                            GNRMC.Give_up = true;
                            printf("放弃\r\n");
                        }
                        else
                        {
                            memcpy(GPS_Save_Data.GNRMC,GNRMC.start,GNRMC.end - GNRMC.start);
                        }
#ifdef __Debug_info
                        for(int i = 0;i < GPS_Buffer_Length_3;i++)
                        {

                            printf("%c",GPS_Save_Data.GNRMC[i]);

                        }
#endif        

#ifdef __Debug_info
                        printf("\r\n");
#endif
                        break;
                    }
                }               
                //寻找&GNGGA  获取海拔，卫星数量，精度因子
                for (int i = 0; i < GPS_Buffer_Length_4; i++)
                {
                    if (GPS_Save_Data.GPS_Buffer[i] == '$' && GPS_Save_Data.GPS_Buffer[i + 1] == 'G' && GPS_Save_Data.GPS_Buffer[i + 2] == 'N' && GPS_Save_Data.GPS_Buffer[i + 3] == 'G' && GPS_Save_Data.GPS_Buffer[i + 4] == 'G' && GPS_Save_Data.GPS_Buffer[i + 5] == 'A')
                    {
                        GNGGA.pos = i;
                        GNGGA.find = true;//标志找到数据帧
#ifdef __Debug_info
                        printf("GNGGA pos = %d\r\n",GNGGA.pos);
                        printf("\r\n");
#endif

                        GNGGA.start = &GPS_Save_Data.GPS_Buffer[GNGGA.pos];
                        GNGGA.end = strstr(GNGGA.start + 1,"$");
                        if(GNGGA.end == nullptr)
                        {
                            GNGGA.Give_up = true;
                            printf("放弃\r\n");
                        }
                        else
                        {
                            memcpy(GPS_Save_Data.GNGGA,GNGGA.start,GNGGA.end - GNGGA.start);
                        }

#ifdef __Debug_info                     
                        for(int i = 0;i < GPS_Buffer_Length_3;i++)
                        {

                            printf("%c",GPS_Save_Data.GNGGA[i]);

                        }
#endif                        
#ifdef __Debug_info
                        printf("\r\n");
#endif
                        break;
                    }
                }
                //寻找&GNVTG  获取航向和速率
                for (int i = 0; i < GPS_Buffer_Length_4; i++)
                {
                    if (GPS_Save_Data.GPS_Buffer[i] == '$' && GPS_Save_Data.GPS_Buffer[i + 1] == 'G' && GPS_Save_Data.GPS_Buffer[i + 2] == 'N' && GPS_Save_Data.GPS_Buffer[i + 3] == 'V' && GPS_Save_Data.GPS_Buffer[i + 4] == 'T' && GPS_Save_Data.GPS_Buffer[i + 5] == 'G')
                    {
                        GNVTG.pos = i;
                        GNVTG.find = true;//标志找到数据帧
#ifdef __Debug_info
                        printf("GNVTG pos = %d\r\n",GNVTG.pos);
                        printf("\r\n");
#endif

                        GNVTG.start = &GPS_Save_Data.GPS_Buffer[GNVTG.pos];
                        GNVTG.end = strstr(GNVTG.start + 1,"$");
                        if(GNVTG.end == nullptr)
                        {
                            GNVTG.Give_up = true;
                            printf("放弃\r\n");

                        }
                        else
                        {
                            memcpy(GPS_Save_Data.GNVTG,GNVTG.start,GNVTG.end - GNVTG.start);
                        }

#ifdef __Debug_info                       
                        for(int i = 0;i < GPS_Buffer_Length_3;i++)
                        {

                            printf("%c",GPS_Save_Data.GNVTG[i]);

                        }
#endif                        
#ifdef __Debug_info
                        printf("\r\n");
#endif
                        break;
                    }
                }
          
                //判断是否找到GNRMC目标帧
                if(GNRMC.find == true)
                {
                    GNRMC.find = false;
                    //判断是否放弃当前帧，为最后一帧则放弃                                   
                    if(GNRMC.Give_up != true)
                    {   
                        //判断当前帧是否完整
                        char* ptr_01 = strstr(GPS_Save_Data.GNRMC,"E");
                        char* ptr_02 = strstr(GPS_Save_Data.GNRMC,"N");
                        char* ptr_03 = strstr(GPS_Save_Data.GNRMC,"A");
                        if((ptr_01!= nullptr) && (ptr_02 != nullptr) && (ptr_03 != nullptr))
                        {
                            if((*ptr_01 == 'E')&&(*ptr_02 == 'N')&&(*ptr_03 == 'A'))
                            {
#ifdef __Debug_info
                                printf("\r\nOK1\r\n");
#endif
                                GNRMC_calc_shuju();     
#ifdef __Debug_info             
                                printGpsBuffer();
#endif
                                //判断是否找到GNGGA目标帧
                                if(GNGGA.find == true)
                                {
                                    GNGGA.find == false;
                                    //判断是否放弃当前帧，为最后一帧则放弃
                                    if(GNGGA.Give_up != true)
                                    {
                                        //判断当前帧是否完整
                                        char* ptr_04 = strstr(GPS_Save_Data.GNGGA,"E");
                                        char* ptr_05 = strstr(GPS_Save_Data.GNGGA,"M");
                                        if((ptr_04 != nullptr) && (ptr_05 != nullptr))
                                        {
                                            if((*ptr_04  == 'E') && (*ptr_05  == 'M'))
                                            {
#ifdef __Debug_info
                                                    printf("\r\nOK2\r\n");
#endif
                                                    GNGGA_calc_shuju();
#ifdef __Debug_info
                                                    printf_GNGGA();           
#endif                                                                                
                                            }
                                        }
                                    }
                                    else
                                    {
                                        GNGGA.Give_up = false;
                                    }
                                }     
                                //判断是否找到GNVTG目标帧
                                if(GNVTG.find == true)
                                { 
                                    GNVTG.find = false;
                                    //判断是否放弃当前帧，为最后一帧则放弃
                                    if(GNVTG.Give_up != true)
                                    {
                                        //判断当前帧是否完整
                                        char* ptr_06 = strstr(GPS_Save_Data.GNVTG,"T");
                                        char* ptr_07 = strstr(GPS_Save_Data.GNVTG,"K");
                                        if((ptr_06 != nullptr) && (ptr_07 != nullptr))
                                        {
                                            if((*ptr_06 == 'T') && (*ptr_07  == 'K'))
                                            {
#ifdef __Debug_info
                                                printf("\r\nOK3\r\n");
#endif
                                                GNVTG_calc_shuju();
#ifdef __Debug_info
                                                printf_GNVTG();
#endif                                                                                  

                                            }
                                        }
                                    }
                                    else
                                    {
                                        GNVTG.Give_up = false;
                                    }
                                }
                                //重置
                                bzero(GPS_Save_Data.Data,sizeof(GPS_Save_Data.Data));
                                bzero(GPS_Save_Data.GNRMC,sizeof(GPS_Save_Data.GNRMC));
                                bzero(GPS_Save_Data.GNGGA,sizeof(GPS_Save_Data.GNGGA));
                                bzero(GPS_Save_Data.GNVTG,sizeof(GPS_Save_Data.GNVTG));
                                bzero(GPS_Save_Data.GPS_Buffer,sizeof(GPS_Save_Data.GPS_Buffer));
                                GPS_Save_Data.m_read_idx = 0;
                                break;
                            }  

                        }
                        else
                        {
                            printf("\r\nOK123\r\n");
                        }
                    }
                    else
                    {
                        GNRMC.Give_up = false;
                    }
                }
            }    
            flag = 1;
            GPS_Save_Data.m_read_idx = 0;
            buf = GPS_Save_Data.GPS_Buffer;
            count = 0;
        }  
        // usleep(100000);
    }
        /* 退出 */
    tcsetattr(fd[pos], TCSANOW, &wit_old_cfg);   //恢复到之前的配置
    close(fd[pos]);
}
//计算GNRMC
void GNRMC_calc_shuju()
{
    char *  pre_ptr = NULL;
    char* next_ptr = NULL;
    pre_ptr =  strstr(GPS_Save_Data.GNRMC,",");

    char usefullBuffer[2];
    for(int i = 1;i < 7;i++)
    {
        pre_ptr++;
        next_ptr =  strstr(pre_ptr,",");
        ////printf("next_ptr = %c\r\n",*(next_ptr+ 1));
        switch(i)
        {
            case 1:memcpy(GPS_Save_Data.UTCTime, pre_ptr, next_ptr - pre_ptr);break;	//获取UTC时间
            case 2:memcpy(usefullBuffer, pre_ptr, next_ptr - pre_ptr);break;	//获取UTC时间
            case 3:memcpy(GPS_Save_Data.latitude, pre_ptr, next_ptr - pre_ptr);break;	//获取纬度信息
            case 4:memcpy(GPS_Save_Data.N_S, pre_ptr, next_ptr - pre_ptr);break;	//获取N/S
            case 5:memcpy(GPS_Save_Data.longitude, pre_ptr, next_ptr - pre_ptr);break;	//获取经度信息
            case 6:memcpy(GPS_Save_Data.E_W, pre_ptr, next_ptr - pre_ptr);break;	//获取E/W

            default:break;
        }
        pre_ptr = next_ptr;     
    }
    if(usefullBuffer[0] == 'V')
    {
            GPS_Save_Data.isUsefull = false;
    }        
    else if(usefullBuffer[0] == 'A')
    {
            GPS_Save_Data.isUsefull = true;
    }
    GPS_Save_Data.isParseData = true;
      
}
//计算GNGGA
void GNGGA_calc_shuju()
{
        char *  pre_ptr = NULL;

        char buf[5] = {0};
        pre_ptr =  strstr(GPS_Save_Data.GNGGA,"E");
        pre_ptr += 4;
        //获取卫星数量
        sprintf(buf,"%c%c",*pre_ptr,*(pre_ptr + 1));
       GPS_Save_Data.raw_satellites =  atof(buf);//字符串转浮点
        //获取精度因子
        pre_ptr +=3;
        sprintf(buf,"%c%c%c",*pre_ptr,*(pre_ptr + 1),*(pre_ptr + 2));
       GPS_Save_Data.raw_hdop =  atof(buf);//字符串转浮点
        //获取海拔
          pre_ptr +=4;
        sprintf(buf,"%c%c%c%c",*pre_ptr,*(pre_ptr + 1),*(pre_ptr + 2),*(pre_ptr + 3));
       GPS_Save_Data.raw_altitude =  atof(buf);//字符串转浮点
}
//计算GNVTG
void GNVTG_calc_shuju()
{
    char *  pre_ptr = NULL;

    char buf[7] = {0};
    pre_ptr =  strstr(GPS_Save_Data.GNVTG,",");
    pre_ptr++;

    //获取航向
    sprintf(buf,"%c%c%c%c%c%c",*pre_ptr,*(pre_ptr + 1),*(pre_ptr + 2),*(pre_ptr + 3),*(pre_ptr + 4),*(pre_ptr + 5));

    GPS_Save_Data.raw_course =  atof(buf);//字符串转浮点
    //获取速率
    pre_ptr =  strstr(pre_ptr,"N");
        pre_ptr+= 2;
    sprintf(buf,"%c%c%c%c",*pre_ptr,*(pre_ptr + 1),*(pre_ptr + 2),*(pre_ptr + 3));

    GPS_Save_Data.raw_speed =  atof(buf);//字符串转浮点

}
//打印GNGGA
void printf_GNGGA()
{
        printf("\r\n卫星数量 = %0.0lf\r\n",GPS_Save_Data.raw_satellites);
        printf("\r\n精度因子 = %0.1lf\r\n",GPS_Save_Data.raw_hdop);
        printf("\r\n海拔 = %0.1lf\r\n",GPS_Save_Data.raw_altitude);
}
//打印GNVTG
void printf_GNVTG()
{
        printf("\r\n对地真北航向 = %0.2lf\r\n",GPS_Save_Data.raw_course);
        printf("\r\n速率  = %0.2lfkm/h\r\n",GPS_Save_Data.raw_speed);
}
//打印相关数据
void printGpsBuffer()
{
    double f_latitude = 0.0;
    double f_longitude = 0.0;

    if (GPS_Save_Data.isParseData)
    {
        GPS_Save_Data.isParseData = false;

        printf("GPS_Save_Data.UTCTime = ");
        printf("%s",GPS_Save_Data.UTCTime);
        printf("\r\n");

        if (GPS_Save_Data.isUsefull)
        {
            GPS_Save_Data.isUsefull = false;
            printf("GPS_Save_Data.latitude = ");
            // printf(Save_Data.latitude);
            // printf("--");
            f_latitude = Convert_to_degrees(GPS_Save_Data.latitude);
            printf("%lf%s", f_latitude, GPS_Save_Data.N_S);
            printf("\r\n");

            printf("GPS_Save_Data.N_S = ");
            printf("%s",GPS_Save_Data.N_S);
            printf("\r\n");

            printf("GPS_Save_Data.longitude = ");
            // printf(Save_Data.longitude);
            // printf("--");
            f_longitude = Convert_to_degrees(GPS_Save_Data.longitude);
            printf("%lf%s", f_longitude, GPS_Save_Data.E_W);
            printf("\r\n");

            printf("GPS_Save_Data.E_W = ");
            printf("%s",GPS_Save_Data.E_W);
            printf("\r\n");
        } else {
            printf("GPS DATA is not usefull!\r\n");
        }
        printf("\r\n");
    }
}
// GPS数据转化单位为度。
double Convert_to_degrees(char* data)
{
    double temp_data = atof(data);
    int degree = (int)(temp_data / 100);
    double f_degree = (temp_data / 100.0 - degree)*100/60.0;
    double result = degree + f_degree;
    return result;
}
//串口初始化
static int gps_uart_init(const char *device)
{
    int loaction = -1;
    auto iter_order = dev_order.find("atgm");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }
    int pos = -1;
     /*第一步，串口初始化*/
    for(map<const char*,int>::iterator it=pos_map.begin();it!=pos_map.end();it++)
    {
        if(strcmp(it->first,dev_name[loaction]) == 0)
        {
            pos = it->second;
            break;
        }
        else
        {
            printf("cb_brt_test pos_map没找到 %d\r\n",__LINE__);
        }
    }


    /* 打开串口终端  使用的标志有 可读可写，告诉系统该节点不会成为进程的控制终端，非阻塞方式，读不到数据返回-1,*/
    fd[pos] = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (0 > fd[pos]) {
        fprintf(stderr, "open error: %s: %s\n", device, strerror(errno));
        return -1;
    }

    /* 获取串口当前的配置参数 */
    if (0 > tcgetattr(fd[pos], &gps_old_cfg)) 
    {
        fprintf(stderr, "tcgetattr error: %s\n", strerror(errno));
        close(fd[pos]);
        return -1;
    }

    return 0;
}
//串口配置
static int gps_uart_cfg(const uart_cfg_t *cfg)
{
    int loaction = -1;
    auto iter_order = dev_order.find("atgm");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }
    int pos = -1;
     /*第一步，串口初始化*/
    for(map<const char*,int>::iterator it=pos_map.begin();it!=pos_map.end();it++)
    {
        if(strcmp(it->first,dev_name[loaction]) == 0)
        {
            pos = it->second;
            break;
        }
        else
        {
            printf("cb_brt_test pos_map没找到 %d\r\n",__LINE__);
        }
    }

    struct termios new_cfg = {0};   //将new_cfg对象清零
    speed_t speed;

    /* 设置为原始模式 */
    cfmakeraw(&new_cfg);

    /* 使能接收 */
    new_cfg.c_cflag |= CREAD;

    /* 设置波特率 */
    switch (cfg->baudrate) {
        case 1200: speed = B1200;
            break;
        case 1800: speed = B1800;
            break;
        case 2400: speed = B2400;
            break;
        case 4800: speed = B4800;
            break;
        case 9600: speed = B9600;
            break;
        case 19200: speed = B19200;
            break;
        case 38400: speed = B38400;
            break;
        case 57600: speed = B57600;
            break;
        case 115200: speed = B115200;
            break;
        case 230400: speed = B230400;
            break;
        case 460800: speed = B460800;
            break;
        case 500000: speed = B500000;
            break;
        default:    //默认配置为115200
            speed = B9600;
            // printf("default baud rate: 9600\n");
            break;
    }

    if (0 > cfsetspeed(&new_cfg, speed)) 
    {
        fprintf(stderr, "cfsetspeed error: %s\n", strerror(errno));
        return -1;
    }

    /* 设置数据位大小 */
    new_cfg.c_cflag &= ~CSIZE;  //将数据位相关的比特位清零
    switch (cfg->dbit) {
        case 5:
            new_cfg.c_cflag |= CS5;
            break;
        case 6:
            new_cfg.c_cflag |= CS6;
            break;
        case 7:
            new_cfg.c_cflag |= CS7;
            break;
        case 8:
            new_cfg.c_cflag |= CS8;
            break;
        default:    //默认数据位大小为8
            new_cfg.c_cflag |= CS8;
            // printf("default data bit size: 8\n");
            break;
    }

    /* 设置奇偶校验 */
    switch (cfg->parity) {
        case 'N':       //无校验
            new_cfg.c_cflag &= ~PARENB;
            new_cfg.c_iflag &= ~INPCK;
            break;
        case 'O':       //奇校验
            new_cfg.c_cflag |= (PARODD | PARENB);
            new_cfg.c_iflag |= INPCK;
            break;
        case 'E':       //偶校验
            new_cfg.c_cflag |= PARENB;
            new_cfg.c_cflag &= ~PARODD; /* 清除PARODD标志，配置为偶校验 */
            new_cfg.c_iflag |= INPCK;
            break;
        default:    //默认配置为无校验
            new_cfg.c_cflag &= ~PARENB;
            new_cfg.c_iflag &= ~INPCK;
            // printf("default parity: N\n");
            break;
    }

    /* 设置停止位 */
    switch (cfg->sbit) {
        case 1:     //1个停止位
            new_cfg.c_cflag &= ~CSTOPB;
            break;
        case 2:     //2个停止位
            new_cfg.c_cflag |= CSTOPB;
            break;
        default:    //默认配置为1个停止位
            new_cfg.c_cflag &= ~CSTOPB;
            // printf("default stop bit size: 1\n");
            break;
    }

    /* 将MIN和TIME设置为0 */
    new_cfg.c_cc[VTIME] = 0;
    new_cfg.c_cc[VMIN] = 0;

    /* 清空缓冲区 */
    if (0 > tcflush(fd[pos], TCIOFLUSH)) {
        fprintf(stderr, "tcflush error: %s\n", strerror(errno));
        return -1;
    }

    /* 写入配置、使配置生效 */
    if (0 > tcsetattr(fd[pos], TCSANOW, &new_cfg)) {
        fprintf(stderr, "tcsetattr error: %s\n", strerror(errno));
        return -1;
    }

    /* 配置OK 退出 */
    return 0;
}
/***********************************WTGPS*********************************************/
void cb_wtgps_test_singal()
{
    int loaction = -1;
    auto iter_order = dev_order.find("wtgps");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }


    /* 清零操作 */
    bzero(WTGPS_GPS_Save_Data.Data,sizeof(WTGPS_GPS_Save_Data.Data));
    bzero(WTGPS_GPS_Save_Data.GNRMC,sizeof(WTGPS_GPS_Save_Data.GNRMC));
    bzero(WTGPS_GPS_Save_Data.GNGGA,sizeof(WTGPS_GPS_Save_Data.GNGGA));
    bzero(WTGPS_GPS_Save_Data.GNVTG,sizeof(WTGPS_GPS_Save_Data.GNVTG));
    bzero(WTGPS_GPS_Save_Data.GPS_Buffer,sizeof(WTGPS_GPS_Save_Data.GPS_Buffer));   
    
    //初始化帧GNRMC 帧数据
    WTGPS_GNRMC.pos = 0;
    WTGPS_GNRMC.start = nullptr;
    WTGPS_GNRMC.end = nullptr;
    WTGPS_GNRMC.find = false;
    WTGPS_GNRMC.Give_up = false;

    //初始化帧GNGGA 帧数据
    WTGPS_GNGGA.pos = 0;
    WTGPS_GNGGA.start = nullptr;
    WTGPS_GNGGA.end = nullptr;
    WTGPS_GNGGA.find = false;
    WTGPS_GNGGA.Give_up = false;

    //初始化帧GNVTG 帧数据
    WTGPS_GNVTG.pos = 0;
    WTGPS_GNVTG.start = nullptr;
    WTGPS_GNVTG.end = nullptr;
    WTGPS_GNVTG.find = false;
    WTGPS_GNVTG.Give_up = false;

    int re_value = 0;//每次获取的一帧数据
    int flag = 1;//获取数据和处理数据互斥标志位
    int count = 0;//获取填充数据的有效数据次数
    WTGPS_GPS_Save_Data.m_read_idx = 0;//用以限制获取的数据量
    char * buf = WTGPS_GPS_Save_Data.GPS_Buffer;

    uart_cfg_t cfg = {0};
    //串口初始
    WTGPS_gps_uart_init(dev_name[loaction]);
    //设置串口参数 115200 8 1 无校验
    WTGPS_gps_uart_cfg(&cfg);
    int ret = 0;
    int pos = -1;
     /*第一步，串口初始化*/
    for(map<const char*,int>::iterator it=pos_map.begin();it!=pos_map.end();it++)
    {
        if(strcmp(it->first,dev_name[loaction]) == 0)
        {
            pos = it->second;
            break;
        }
        else
        {
            printf("cb_brt_test pos_map没找到 %d\r\n",__LINE__);
        }
    }
    while(Get_isstop(pos) != true)
    {

        if(flag == 1)
        {
            re_value = read(fd[pos], WTGPS_GPS_Save_Data.Data, WTGPS_GPS_Buffer_Length_2);
                                   
            if(re_value > 0)
            {
                
                if( WTGPS_GPS_Save_Data.m_read_idx + re_value >= WTGPS_GPS_Buffer_Length_4)
                {
                    WTGPS_GPS_Save_Data.m_read_idx = WTGPS_GPS_Buffer_Length_4;
                    flag = 0;
                }
                else
                {
                    count++;
                    WTGPS_GPS_Save_Data.m_read_idx +=  re_value;
                }
                if( WTGPS_GPS_Save_Data.m_read_idx < WTGPS_GPS_Buffer_Length_4)
                {
                    strncpy(buf,WTGPS_GPS_Save_Data.Data,re_value);
                    buf += re_value;
//打印读取一帧的数据
#if 0
                    for(int i = 0;i < re_value;i++)
                    {
                        printf("%c",WTGPS_GPS_Save_Data.Data[i]);
                    }
#endif
                }                
            }
        }
        else
        {
            //判断是否接收到了原始数据集
            if(WTGPS_GPS_Save_Data.m_read_idx == WTGPS_GPS_Buffer_Length_4)
            {
#if 0        
                //打印原始数据
                for(int i = 0;i < GPS_Buffer_Length_4;i++)
                {
                    printf("%c",WTGPS_GPS_Save_Data.GPS_Buffer[i]);
                }
#endif
#ifdef __Debug_info
                printf("\r\n");
#endif
                //寻找&GNRMC  获取经纬度信息
                for (int i = 0; i < WTGPS_GPS_Buffer_Length_4; i++)
                {
                    if (WTGPS_GPS_Save_Data.GPS_Buffer[i] == '$' && WTGPS_GPS_Save_Data.GPS_Buffer[i + 1] == 'G' && WTGPS_GPS_Save_Data.GPS_Buffer[i + 2] == 'N' && WTGPS_GPS_Save_Data.GPS_Buffer[i + 3] == 'R' && WTGPS_GPS_Save_Data.GPS_Buffer[i + 4] == 'M' && WTGPS_GPS_Save_Data.GPS_Buffer[i + 5] == 'C')
                    {
                        WTGPS_GNRMC.pos = i;
                        WTGPS_GNRMC.find = true;//标志找到数据帧
#ifdef __Debug_info
                        printf(" WTGPS_GNRMC pos = %d\r\n",WTGPS_GNRMC.pos);
                        printf("\r\n");
#endif
                        WTGPS_GNRMC.start = &WTGPS_GPS_Save_Data.GPS_Buffer[WTGPS_GNRMC.pos];
                        WTGPS_GNRMC.end = strstr(WTGPS_GNRMC.start + 1,"$");
                        if(WTGPS_GNRMC.end == nullptr)
                        {
                            WTGPS_GNRMC.Give_up = true;
                            printf("放弃\r\n");
                        }
                        else
                        {
                            memcpy(WTGPS_GPS_Save_Data.GNRMC,WTGPS_GNRMC.start,WTGPS_GNRMC.end - WTGPS_GNRMC.start);
                        }
#ifdef __Debug_info
                        for(int i = 0;i < GPS_Buffer_Length_3;i++)
                        {

                            printf("%c",WTGPS_GPS_Save_Data.GNRMC[i]);

                        }
#endif        

#ifdef __Debug_info
                        printf("\r\n");
#endif
                        break;
                    }
                }               
                //寻找&GNGGA  获取海拔，卫星数量，精度因子
                for (int i = 0; i < WTGPS_GPS_Buffer_Length_4; i++)
                {
                    if (WTGPS_GPS_Save_Data.GPS_Buffer[i] == '$' && WTGPS_GPS_Save_Data.GPS_Buffer[i + 1] == 'G' && WTGPS_GPS_Save_Data.GPS_Buffer[i + 2] == 'N' && WTGPS_GPS_Save_Data.GPS_Buffer[i + 3] == 'G' && WTGPS_GPS_Save_Data.GPS_Buffer[i + 4] == 'G' && WTGPS_GPS_Save_Data.GPS_Buffer[i + 5] == 'A')
                    {
                        WTGPS_GNGGA.pos = i;
                        WTGPS_GNGGA.find = true;//标志找到数据帧
#ifdef __Debug_info
                        printf("WTGPS_GNGGA pos = %d\r\n",WTGPS_GNGGA.pos);
                        printf("\r\n");
#endif

                        WTGPS_GNGGA.start = &WTGPS_GPS_Save_Data.GPS_Buffer[WTGPS_GNGGA.pos];
                        WTGPS_GNGGA.end = strstr(WTGPS_GNGGA.start + 1,"$");
                        if(WTGPS_GNGGA.end == nullptr)
                        {
                            WTGPS_GNGGA.Give_up = true;
                            printf("放弃\r\n");
                        }
                        else
                        {
                            memcpy(WTGPS_GPS_Save_Data.GNGGA,WTGPS_GNGGA.start,WTGPS_GNGGA.end - WTGPS_GNGGA.start);
                        }

#ifdef __Debug_info                     
                        for(int i = 0;i < GPS_Buffer_Length_3;i++)
                        {

                            printf("%c",WTGPS_GPS_Save_Data.GNGGA[i]);

                        }
#endif                        
#ifdef __Debug_info
                        printf("\r\n");
#endif
                        break;
                    }
                }
                //寻找&GNVTG  获取航向和速率
                for (int i = 0; i < WTGPS_GPS_Buffer_Length_4; i++)
                {
                    if (WTGPS_GPS_Save_Data.GPS_Buffer[i] == '$' && WTGPS_GPS_Save_Data.GPS_Buffer[i + 1] == 'G' && WTGPS_GPS_Save_Data.GPS_Buffer[i + 2] == 'N' && WTGPS_GPS_Save_Data.GPS_Buffer[i + 3] == 'V' && WTGPS_GPS_Save_Data.GPS_Buffer[i + 4] == 'T' && WTGPS_GPS_Save_Data.GPS_Buffer[i + 5] == 'G')
                    {
                        WTGPS_GNVTG.pos = i;
                        WTGPS_GNVTG.find = true;//标志找到数据帧
#ifdef __Debug_info
                        printf("WTGPS_GNVTG pos = %d\r\n",WTGPS_GNVTG.pos);
                        printf("\r\n");
#endif

                        WTGPS_GNVTG.start = &WTGPS_GPS_Save_Data.GPS_Buffer[WTGPS_GNVTG.pos];
                        WTGPS_GNVTG.end = strstr(WTGPS_GNVTG.start + 1,"$");
                        if(WTGPS_GNVTG.end == nullptr)
                        {
                            WTGPS_GNVTG.Give_up = true;
                            printf("放弃\r\n");

                        }
                        else
                        {
                            memcpy(WTGPS_GPS_Save_Data.GNVTG,WTGPS_GNVTG.start,WTGPS_GNVTG.end - WTGPS_GNVTG.start);
                        }

#ifdef __Debug_info                       
                        for(int i = 0;i < GPS_Buffer_Length_3;i++)
                        {

                            printf("%c",WTGPS_GPS_Save_Data.GNVTG[i]);

                        }
#endif                        
#ifdef __Debug_info
                        printf("\r\n");
#endif
                        break;
                    }
                }
          
                //判断是否找到GNRMC目标帧
                if(WTGPS_GNRMC.find == true)
                {
                    WTGPS_GNRMC.find = false;
                    //判断是否放弃当前帧，为最后一帧则放弃                                   
                    if(WTGPS_GNRMC.Give_up != true)
                    {   
                        //判断当前帧是否完整
                        char* ptr_01 = strstr(WTGPS_GPS_Save_Data.GNRMC,"E");
                        char* ptr_02 = strstr(WTGPS_GPS_Save_Data.GNRMC,"N");
                        char* ptr_03 = strstr(WTGPS_GPS_Save_Data.GNRMC,"A");
                        if((ptr_01!= nullptr) && (ptr_02 != nullptr) && (ptr_03 != nullptr))
                        {
                            if((*ptr_01 == 'E')&&(*ptr_02 == 'N')&&(*ptr_03 == 'A'))
                            {
#ifdef __Debug_info
                                printf("\r\nOK1\r\n");
#endif
                                WTGPS_GNRMC_calc_shuju();     
#ifdef __Debug_info             
                                WTGPS_printGpsBuffer();
#endif
                                //判断是否找到GNGGA目标帧
                                if(WTGPS_GNGGA.find == true)
                                {
                                    WTGPS_GNGGA.find == false;
                                    //判断是否放弃当前帧，为最后一帧则放弃
                                    if(WTGPS_GNGGA.Give_up != true)
                                    {
                                        //判断当前帧是否完整
                                        char* ptr_04 = strstr(WTGPS_GPS_Save_Data.GNGGA,"E");
                                        char* ptr_05 = strstr(WTGPS_GPS_Save_Data.GNGGA,"M");
                                        if((ptr_04 != nullptr) && (ptr_05 != nullptr))
                                        {
                                            if((*ptr_04  == 'E') && (*ptr_05  == 'M'))
                                            {
#ifdef __Debug_info
                                                    printf("\r\nOK2\r\n");
#endif
                                                    WTGPS_GNGGA_calc_shuju();
#ifdef __Debug_info
                                                    WTGPS_printf_GNGGA();           
#endif                                                                                
                                            }
                                        }
                                    }
                                    else
                                    {
                                        WTGPS_GNGGA.Give_up = false;
                                    }
                                }     
                                //判断是否找到GNVTG目标帧
                                if(WTGPS_GNVTG.find == true)
                                { 
                                    WTGPS_GNVTG.find = false;
                                    //判断是否放弃当前帧，为最后一帧则放弃
                                    if(WTGPS_GNVTG.Give_up != true)
                                    {
                                        //判断当前帧是否完整
                                        char* ptr_06 = strstr(WTGPS_GPS_Save_Data.GNVTG,"T");
                                        char* ptr_07 = strstr(WTGPS_GPS_Save_Data.GNVTG,"K");
                                        if((ptr_06 != nullptr) && (ptr_07 != nullptr))
                                        {
                                            if((*ptr_06 == 'T') && (*ptr_07  == 'K'))
                                            {
#ifdef __Debug_info
                                                printf("\r\nOK3\r\n");
#endif
                                                WTGPS_GNVTG_calc_shuju();
#ifdef __Debug_info
                                                WTGPS_printf_GNVTG();
#endif                                                                                  
                                            }
                                        }
                                    }
                                    else
                                    {
                                        WTGPS_GNVTG.Give_up = false;
                                    }
                                }
                                //重置
                                bzero(WTGPS_GPS_Save_Data.Data,sizeof(WTGPS_GPS_Save_Data.Data));
                                bzero(WTGPS_GPS_Save_Data.GNRMC,sizeof(WTGPS_GPS_Save_Data.GNRMC));
                                bzero(WTGPS_GPS_Save_Data.GNGGA,sizeof(WTGPS_GPS_Save_Data.GNGGA));
                                bzero(WTGPS_GPS_Save_Data.GNVTG,sizeof(WTGPS_GPS_Save_Data.GNVTG));
                                bzero(WTGPS_GPS_Save_Data.GPS_Buffer,sizeof(WTGPS_GPS_Save_Data.GPS_Buffer));
                                WTGPS_GPS_Save_Data.m_read_idx = 0;
                                break;
                            }  

                        }
                        else
                        {
                            printf("\r\nOK123\r\n");
                        }
                    }
                    else
                    {
                        WTGPS_GNRMC.Give_up = false;
                    }
                }
            }    
            flag = 1;
            WTGPS_GPS_Save_Data.m_read_idx = 0;
            buf = WTGPS_GPS_Save_Data.GPS_Buffer;
            count = 0;
        }  
        
    }
        /* 退出 */
    tcsetattr(fd[pos], TCSANOW, &WTGPS_gps_old_cfg);   //恢复到之前的配置
    close(fd[pos]);
}
//计算GNRMC
void WTGPS_GNRMC_calc_shuju()
{
    char *  pre_ptr = NULL;
    char* next_ptr = NULL;
    pre_ptr =  strstr(GPS_Save_Data.GNRMC,",");

    char usefullBuffer[2];
    for(int i = 1;i < 7;i++)
    {
        pre_ptr++;
        next_ptr =  strstr(pre_ptr,",");
        ////printf("next_ptr = %c\r\n",*(next_ptr+ 1));
        switch(i)
        {
            case 1:memcpy(GPS_Save_Data.UTCTime, pre_ptr, next_ptr - pre_ptr);break;	//获取UTC时间
            case 2:memcpy(usefullBuffer, pre_ptr, next_ptr - pre_ptr);break;	//获取UTC时间
            case 3:memcpy(GPS_Save_Data.latitude, pre_ptr, next_ptr - pre_ptr);break;	//获取纬度信息
            case 4:memcpy(GPS_Save_Data.N_S, pre_ptr, next_ptr - pre_ptr);break;	//获取N/S
            case 5:memcpy(GPS_Save_Data.longitude, pre_ptr, next_ptr - pre_ptr);break;	//获取经度信息
            case 6:memcpy(GPS_Save_Data.E_W, pre_ptr, next_ptr - pre_ptr);break;	//获取E/W

            default:break;
        }
        pre_ptr = next_ptr;     
    }
    if(usefullBuffer[0] == 'V')
    {
            GPS_Save_Data.isUsefull = false;
    }        
    else if(usefullBuffer[0] == 'A')
    {
            GPS_Save_Data.isUsefull = true;
    }
    GPS_Save_Data.isParseData = true;
      
}
//计算GNGGA
void WTGPS_GNGGA_calc_shuju()
{
        char *  pre_ptr = NULL;

        char buf[5] = {0};
        pre_ptr =  strstr(GPS_Save_Data.GNGGA,"E");
        pre_ptr += 4;
        //获取卫星数量
        sprintf(buf,"%c%c",*pre_ptr,*(pre_ptr + 1));
       GPS_Save_Data.raw_satellites =  atof(buf);//字符串转浮点
        //获取精度因子
        pre_ptr +=3;
        sprintf(buf,"%c%c%c",*pre_ptr,*(pre_ptr + 1),*(pre_ptr + 2));
       GPS_Save_Data.raw_hdop =  atof(buf);//字符串转浮点
        //获取海拔
          pre_ptr +=4;
        sprintf(buf,"%c%c%c%c",*pre_ptr,*(pre_ptr + 1),*(pre_ptr + 2),*(pre_ptr + 3));
       GPS_Save_Data.raw_altitude =  atof(buf);//字符串转浮点
}
//计算GNVTG
void WTGPS_GNVTG_calc_shuju()
{
    char *  pre_ptr = NULL;

    char buf[7] = {0};
    pre_ptr =  strstr(GPS_Save_Data.GNVTG,",");
    pre_ptr++;

    //获取航向
    sprintf(buf,"%c%c%c%c%c%c",*pre_ptr,*(pre_ptr + 1),*(pre_ptr + 2),*(pre_ptr + 3),*(pre_ptr + 4),*(pre_ptr + 5));

    GPS_Save_Data.raw_course =  atof(buf);//字符串转浮点
    //获取速率
    pre_ptr =  strstr(pre_ptr,"N");
        pre_ptr+= 2;
    sprintf(buf,"%c%c%c%c",*pre_ptr,*(pre_ptr + 1),*(pre_ptr + 2),*(pre_ptr + 3));

    GPS_Save_Data.raw_speed =  atof(buf);//字符串转浮点

}
//打印GNGGA
void WTGPS_printf_GNGGA()
{
        printf("\r\n卫星数量 = %0.0lf\r\n",GPS_Save_Data.raw_satellites);
        printf("\r\n精度因子 = %0.1lf\r\n",GPS_Save_Data.raw_hdop);
        printf("\r\n海拔 = %0.1lf\r\n",GPS_Save_Data.raw_altitude);
}
//打印GNVTG
void WTGPS_printf_GNVTG()
{
        printf("\r\n对地真北航向 = %0.2lf\r\n",GPS_Save_Data.raw_course);
        printf("\r\n速率  = %0.2lfkm/h\r\n",GPS_Save_Data.raw_speed);
}
//打印相关数据
void WTGPS_printGpsBuffer()
{
    double f_latitude = 0.0;
    double f_longitude = 0.0;

    if (GPS_Save_Data.isParseData)
    {
        GPS_Save_Data.isParseData = false;

        printf("GPS_Save_Data.UTCTime = ");
        printf("%s",GPS_Save_Data.UTCTime);
        printf("\r\n");

        if (GPS_Save_Data.isUsefull)
        {
            GPS_Save_Data.isUsefull = false;
            printf("GPS_Save_Data.latitude = ");
            // printf(Save_Data.latitude);
            // printf("--");
            f_latitude = Convert_to_degrees(GPS_Save_Data.latitude);
            printf("%lf%s", f_latitude, GPS_Save_Data.N_S);
            printf("\r\n");

            printf("GPS_Save_Data.N_S = ");
            printf("%s",GPS_Save_Data.N_S);
            printf("\r\n");

            printf("GPS_Save_Data.longitude = ");
            // printf(Save_Data.longitude);
            // printf("--");
            f_longitude = Convert_to_degrees(GPS_Save_Data.longitude);
            printf("%lf%s", f_longitude, GPS_Save_Data.E_W);
            printf("\r\n");

            printf("GPS_Save_Data.E_W = ");
            printf("%s",GPS_Save_Data.E_W);
            printf("\r\n");
        } else {
            printf("GPS DATA is not usefull!\r\n");
        }
        printf("\r\n");
    }
}
// GPS数据转化单位为度。
double WTGPS_Convert_to_degrees(char* data)
{
    double temp_data = atof(data);
    int degree = (int)(temp_data / 100);
    double f_degree = (temp_data / 100.0 - degree)*100/60.0;
    double result = degree + f_degree;
    return result;
}
//串口初始化
static int WTGPS_gps_uart_init(const char *device)
{
    int loaction = -1;
    auto iter_order = dev_order.find("wtgps");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }
    int pos = -1;
     /*第一步，串口初始化*/
    for(map<const char*,int>::iterator it=pos_map.begin();it!=pos_map.end();it++)
    {
        if(strcmp(it->first,dev_name[loaction]) == 0)
        {
            pos = it->second;
            break;
        }
        else
        {
            printf("cb_brt_test pos_map没找到 %d\r\n",__LINE__);
        }
    }


    /* 打开串口终端  使用的标志有 可读可写，告诉系统该节点不会成为进程的控制终端，非阻塞方式，读不到数据返回-1,*/
    fd[pos] = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (0 > fd[pos]) {
        fprintf(stderr, "open error: %s: %s\n", device, strerror(errno));
        return -1;
    }

    /* 获取串口当前的配置参数 */
    if (0 > tcgetattr(fd[pos], &WTGPS_gps_old_cfg)) 
    {
        fprintf(stderr, "tcgetattr error: %s\n", strerror(errno));
        close(fd[pos]);
        return -1;
    }

    return 0;
}
//串口配置
static int WTGPS_gps_uart_cfg(const uart_cfg_t *cfg)
{
    int loaction = -1;
    auto iter_order = dev_order.find("wtgps");
    if(iter_order != pos_map.end())
    {
        loaction = iter_order->second;
    }
    int pos = -1;
     /*第一步，串口初始化*/
    for(map<const char*,int>::iterator it=pos_map.begin();it!=pos_map.end();it++)
    {
        if(strcmp(it->first,dev_name[loaction]) == 0)
        {
            pos = it->second;
            break;
        }
        else
        {
            printf("cb_brt_test pos_map没找到 %d\r\n",__LINE__);
        }
    }

    struct termios new_cfg = {0};   //将new_cfg对象清零
    speed_t speed;

    /* 设置为原始模式 */
    cfmakeraw(&new_cfg);

    /* 使能接收 */
    new_cfg.c_cflag |= CREAD;

    /* 设置波特率 */
    switch (cfg->baudrate) {
        case 1200: speed = B1200;
            break;
        case 1800: speed = B1800;
            break;
        case 2400: speed = B2400;
            break;
        case 4800: speed = B4800;
            break;
        case 9600: speed = B9600;
            break;
        case 19200: speed = B19200;
            break;
        case 38400: speed = B38400;
            break;
        case 57600: speed = B57600;
            break;
        case 115200: speed = B115200;
            break;
        case 230400: speed = B230400;
            break;
        case 460800: speed = B460800;
            break;
        case 500000: speed = B500000;
            break;
        default:    //默认配置为115200
            speed = B9600;
            // printf("default baud rate: 9600\n");
            break;
    }

    if (0 > cfsetspeed(&new_cfg, speed)) 
    {
        fprintf(stderr, "cfsetspeed error: %s\n", strerror(errno));
        return -1;
    }

    /* 设置数据位大小 */
    new_cfg.c_cflag &= ~CSIZE;  //将数据位相关的比特位清零
    switch (cfg->dbit) {
        case 5:
            new_cfg.c_cflag |= CS5;
            break;
        case 6:
            new_cfg.c_cflag |= CS6;
            break;
        case 7:
            new_cfg.c_cflag |= CS7;
            break;
        case 8:
            new_cfg.c_cflag |= CS8;
            break;
        default:    //默认数据位大小为8
            new_cfg.c_cflag |= CS8;
            // printf("default data bit size: 8\n");
            break;
    }

    /* 设置奇偶校验 */
    switch (cfg->parity) {
        case 'N':       //无校验
            new_cfg.c_cflag &= ~PARENB;
            new_cfg.c_iflag &= ~INPCK;
            break;
        case 'O':       //奇校验
            new_cfg.c_cflag |= (PARODD | PARENB);
            new_cfg.c_iflag |= INPCK;
            break;
        case 'E':       //偶校验
            new_cfg.c_cflag |= PARENB;
            new_cfg.c_cflag &= ~PARODD; /* 清除PARODD标志，配置为偶校验 */
            new_cfg.c_iflag |= INPCK;
            break;
        default:    //默认配置为无校验
            new_cfg.c_cflag &= ~PARENB;
            new_cfg.c_iflag &= ~INPCK;
            // printf("default parity: N\n");
            break;
    }

    /* 设置停止位 */
    switch (cfg->sbit) {
        case 1:     //1个停止位
            new_cfg.c_cflag &= ~CSTOPB;
            break;
        case 2:     //2个停止位
            new_cfg.c_cflag |= CSTOPB;
            break;
        default:    //默认配置为1个停止位
            new_cfg.c_cflag &= ~CSTOPB;
            // printf("default stop bit size: 1\n");
            break;
    }

    /* 将MIN和TIME设置为0 */
    new_cfg.c_cc[VTIME] = 0;
    new_cfg.c_cc[VMIN] = 0;

    /* 清空缓冲区 */
    if (0 > tcflush(fd[pos], TCIOFLUSH)) {
        fprintf(stderr, "tcflush error: %s\n", strerror(errno));
        return -1;
    }

    /* 写入配置、使配置生效 */
    if (0 > tcsetattr(fd[pos], TCSANOW, &new_cfg)) {
        fprintf(stderr, "tcsetattr error: %s\n", strerror(errno));
        return -1;
    }

    /* 配置OK 退出 */
    return 0;
}

