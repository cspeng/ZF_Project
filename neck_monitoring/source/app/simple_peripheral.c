/******************************************************************************

@file  simple_peripheral.c

@brief This file contains the Simple BLE Peripheral sample application for use
with the CC2650 Bluetooth Low Energy Protocol Stack.

Group: WCS, BTS
Target Device: CC2650, CC2640, CC1350

******************************************************************************

Copyright (c) 2013-2016, Texas Instruments Incorporated
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

*  Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

*  Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

*  Neither the name of Texas Instruments Incorporated nor the names of
its contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

******************************************************************************
Release Name: ble_sdk_2_02_01_18
Release Date: 2016-10-26 15:20:04
*****************************************************************************/

/*********************************************************************
* INCLUDES
*/
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "hci_tl.h"
#include "gatt.h"
#include "linkdb.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simple_gatt_profile.h"

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include <ti/mw/display/Display.h>
#include "board_key.h"

#include "board.h"

#include "simple_peripheral.h"

#if defined( USE_FPGA ) || defined( DEBUG_SW_TRACE )
#include <driverlib/ioc.h>
#endif // USE_FPGA | DEBUG_SW_TRACE

//#include "task_uart.h"
#include "hw_gpio.h"
#include "inc/sdi_task.h"
#include "inc/sdi_tl_uart.h"
#include "oad_target.h"
#include "hw_adc.h"
#include <driverlib/aux_adc.h>
#include <driverlib/aux_wuc.h>
/*********************************************************************
* CONSTANTS
*/

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     10

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     10
#else //!FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     10

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     10
#endif // FEATURE_OAD

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          200

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               5000
#define SBP_PERIODIC_TIMER_EVT_PERIOD         200
#define SBP_PERIODIC_UPLOAD_EVT_PERIOD        14

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define SBP_TASK_PRIORITY                     1


#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   1024
#endif

// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_PERIODIC_EVT                      0x0004
#define SBP_CONN_EVT_END_EVT                  0x0008
#define SBP_TIMER_PERIODIC_EVT                0x0010
#define SBP_UPLOAD_EVT                        0x0020
/*********************************************************************
* TYPEDEFS
*/
// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
} sbpEvt_t;

typedef struct _queueRec_
{
  Queue_Elem _elem;  //queue elemt
  uint8_t *pData;  //pointer to app data
  
}queueRec_t;
//APP event passed form profiles
typedef struct
{
  uint8_t event;    //Type of event
  uint8_t *pData;    //New data
  uint8_t length;    //New status
}simpleUARTEvt_t;

/*********************************************************************
* GLOBAL VARIABLES
*/

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
* LOCAL VARIABLES
*/

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;
static Clock_Struct timerperiodicClock;
static Clock_Struct timeuploadperiodicClock;
// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;
//Queue object used for uart messages

static Queue_Struct appUARTMsg;
static Queue_Handle appUARTMsgQueue;

//static Queue_Struct uartRxMsg;
//static Queue_Handle uartRxQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

// Profile state and parameters
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[20] =
{
  // complete name
  13,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  's','e','n','s','o','r','0', ' ','n','e','c','k'
};
//static uint8_t scanRspData[] =
//{
//  // complete name
//  0x15,   // length of this data
//  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
//  'N',
//  'e',
//  'c',
//  'k',
//  '/',
//  'W',
//  'a',
//  'i',
//  's',
//  't',
//  'M',
//  'o',
//  'n',
//  'i',
//  't',
//  'o',
//  'r',
//  'i',
//  'n',
//  'g',
//  
//  // connection interval range
//  0x05,   // length of this data
//  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
//  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
//  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
//  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
//  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
//  
//  // Tx power level
//  0x02,   // length of this data
//  GAP_ADTYPE_POWER_LEVEL,
//  0       // 0dBm
//};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  
  // service UUID, to notify central devices what services are included
  // in this peripheral
#if !defined(FEATURE_OAD) || defined(FEATURE_OAD_ONCHIP)
  0x03,   // length of this data
#else //OAD for external flash
  0x05,  // lenght of this data
#endif //FEATURE_OAD
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
#ifdef FEATURE_OAD
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID),
#endif //FEATURE_OAD
#ifndef FEATURE_OAD_ONCHIP
  LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  HI_UINT16(SIMPLEPROFILE_SERV_UUID)
#endif //FEATURE_OAD_ONCHIP
};

// GAP GATT Attributes
static uint8_t attDeviceName[20] = "sensor0 neck";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;
// Profile state and parameters
static gaprole_States_t gapProfileState = GAPROLE_INIT;

//Sensor process
#define R_TO_D   (57.295780F)
#define SENSOR_DATA_LENGTH 27
#define TXBUF_LENGTH 8
#define RXBUF_LENGTH 16
#define snvBleID 0x81
#define time_address_LENGTH 11
#define UPLOADFLASHDATA_LENGTH 20
#define VOLTAGE_THRESHOLD      3.3
#define MAXNAMELENGTH          16
typedef union
{
  uint8_t u8vals[4];
  uint32_t u32val;
  float fval;
}f2int_t, int2f_t;

typedef struct
{
 uint32_t wflashCount ;      //写入数据到flash的次数
 uint16_t eraseCount ;       //  擦除flash的次数
 uint8_t startCount ;        //开始次数
 int16_t forwardErrorRange ; //前倾
 int16_t backErrorRange ;    //后仰
 uint16_t timeInterval ;     //时间间隔
 uint8_t numericalRelation ; //大于/小于
 uint8_t bleNameLength ;  //修改蓝牙名称指令长度
 uint8_t bleName[MAXNAMELENGTH]; //蓝牙名称
}snv_t;

snv_t snvData;
bool Timer_Start = 1;           //启动震动标志
extern uint8_t calibrateFlag;  //校准标志
static float eulerDatas[3];   //欧拉角数据
static uint8_t stateFlag = 0;        //开始与结束标志
static uint8_t successFlag = 1;      //是否成功获取传感器数据   
static uint8_t alarmFlag = 1;        //是否震动
static int8_t uploadEulerData[10] = {0};    //发送给软件端的数据
static uint8_t sensorData[SENSOR_DATA_LENGTH];  //存放串口数据
static uint8_t serialCompare[] = {0x3A, 0x01, 0x00, 0x09, 0x00, 0x10};   //串口数据的前5个字节

static int8_t txbuf[TXBUF_LENGTH];          //存放写入flash的数据
static int8_t rxbuf[RXBUF_LENGTH];          //存放提取的flash数据
static uint16_t tx_count = 0;               //获取到欧拉角数据的次数
static uint16_t pageErase = 0;             //  根据每一次存储的字节数来确定擦除一次flash   
static uint32_t uploadCount = 0;            //  上传flash数据的次数
static uint8_t isUpload = 1;               //  上传的标志
static int8_t uploadFlashdata[UPLOADFLASHDATA_LENGTH] = {0};    //存放上传的flash数据
static uint32_t extractflashCount = 256;             //提取flash次数
static uint8_t readFlag = 0;                //读取flash的标志
static uint8_t saveTiming = 0;                  //定时存储到flash
static uint8_t read_count = 0;                //读取FLASH次数
static uint8_t time_address [14] = {0};    //保存时间和地址
static uint32_t frontAddress = 0;           //开始地址
static uint32_t behindAddress = 0;        //结束地址
static uint16_t alarmTiming = 0;
static uint16_t alarmPeriod = 5;           //报警周期 
static float zeroValue = 0 ;             //角度清零值
static uint8_t isLedflicker = 0;           //灯是否闪烁标志
static uint8_t ledFlickerFrequency = 0;     //led闪烁周期
static uint8_t powerAlarmtime = 0;        //电量报警周期
static uint8_t powerAlarm = 0;            //电量报警标志
static uint16_t ADCVal;                  //电量的adc读取
/*********************************************************************
* LOCAL FUNCTIONS
*/
static void simple_alarm(void);
static void simple_monitor(void);
static void link_and_read_ledFlicker(void);
static void simple_role_storage_data(float *eulerData);
static void simple_role_uploadFlashdata(void);
static void Apart_eulerData(float *eulerData);
static float int2float(float *p_f, uint8_t *p_u8);
static void LPMS_parseSensorData(float *eulerData);
static uint8_t LMPS_getPacket(uint8_t len, uint8_t *pData, float *eulerData);

static void SimpleBLEPeripheral_init( void );
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);
static void simple_enqueueUARTMsg(uint8_t event, uint8_t *data, uint8_t len);
static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState);
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID);
static void SimpleBLEPeripheral_clockHandler(UArg arg);

static void SimpleBLEPeripheral_sendAttRsp(void);
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status);

static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState);
#ifndef FEATURE_OAD_ONCHIP
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID);
#endif //!FEATURE_OAD_ONCHIP
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state);

#ifdef FEATURE_OAD
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD

void Simple_Peripheral_NotiData(uint8_t *buf, uint16_t len);

static void simple_role_sendStartFrame(void);
static void simple_role_sendEndFrame(void);
static void simple_role_uploadCurveData(void);
static void ADC_Read(void);
static void ble_name_Update(void);
/*********************************************************************
* PROFILE CALLBACKS
*/

// GAP Role Callbacks
static gapRolesCBs_t SimpleBLEPeripheral_gapRoleCBs =
{
  SimpleBLEPeripheral_stateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
//static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
//{
//  NULL, // Passcode callback (not used by application)
//  NULL  // Pairing / Bonding state Callback (not used by application)
//};

// Simple GATT Profile Callbacks
#ifndef FEATURE_OAD_ONCHIP
static simpleProfileCBs_t SimpleBLEPeripheral_simpleProfileCBs =
{
  SimpleBLEPeripheral_charValueChangeCB // Characteristic value change callback
};
#endif //!FEATURE_OAD_ONCHIP

#ifdef FEATURE_OAD
static oadTargetCBs_t simpleBLEPeripheral_oadCBs =
{
  SimpleBLEPeripheral_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
* @fn      SimpleBLEPeripheral_createTask
*
* @brief   Task creation function for the Simple BLE Peripheral.
*
* @param   None.
*
* @return  None.
*/
void SimpleBLEPeripheral_createTask(void)
{
  Task_Params taskParams;
  
  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;
  
  Task_construct(&sbpTask, SimpleBLEPeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
* @fn      SimpleBLEPeripheral_init
*
* @brief   Called during initialization and contains application
*          specific initialization (ie. hardware initialization/setup,
*          table initialization, power up notification, etc), and
*          profile initialization/setup.
*
* @param   None.
*
* @return  None.
*/
static void SimpleBLEPeripheral_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);
  
  // Hard code the BD Address till CC2650 board gets its own IEEE address
  //  uint8 bdAddress[B_ADDR_LEN] = { 0x06, 0x01, 0x02, 0x03, 0x04, 0x05 };
  //  HCI_EXT_SetBDADDRCmd(bdAddress);
  
  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);
  appUARTMsgQueue = Util_constructQueue(&appUARTMsg);
  
  // Create one-shot clocks for internal periodic events.
 Util_constructClock(&timerperiodicClock, SimpleBLEPeripheral_clockHandler,
                      SBP_PERIODIC_TIMER_EVT_PERIOD, SBP_PERIODIC_TIMER_EVT_PERIOD, true, SBP_TIMER_PERIODIC_EVT);
  Util_constructClock(&timeuploadperiodicClock, SimpleBLEPeripheral_clockHandler,
                      SBP_PERIODIC_UPLOAD_EVT_PERIOD, 0, false, SBP_UPLOAD_EVT);
  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);
  
  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;
    
    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;
    
    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
    
    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);
    
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, 14,
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
    
    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }
  
  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, 12, attDeviceName);
  
  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;
    
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }
  
  // Setup the GAP Bond Manager
  //  {
  //    uint32_t passkey = 0; // passkey "000000"
  //    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
  //    uint8_t mitm = TRUE;
  //    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
  //    uint8_t bonding = TRUE;
  //
  //    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
  //                            &passkey);
  //    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
  //    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
  //    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
  //    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  //  }
  
  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service
  
#ifndef FEATURE_OAD_ONCHIP
  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
#endif //!FEATURE_OAD_ONCHIP
  
#ifdef FEATURE_OAD
  VOID OAD_addService();                 // OAD Profile
  OAD_register((oadTargetCBs_t *)&simpleBLEPeripheral_oadCBs);
  hOadQ = Util_constructQueue(&oadQ);
#endif //FEATURE_OAD
  
#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE
  
  
#ifndef FEATURE_OAD_ONCHIP
  // Setup the SimpleProfile Characteristic Values
  {
    uint8_t charValue1[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    uint8_t charValue2 = 2;
    uint8_t charValue3 = 3;
    uint8_t charValue4 = 4;
    uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR5_LEN,
                               &charValue1);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
                               &charValue2);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
                               &charValue3);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &charValue4);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                               charValue5);
  }
  
  // Register callback with SimpleGATTprofile
  SimpleProfile_RegisterAppCBs(&SimpleBLEPeripheral_simpleProfileCBs);
#endif //!FEATURE_OAD_ONCHIP
  
  // Start the Device
  VOID GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);
  
  // Start Bond Manager
  // VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);
  
  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);
  
  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);
  
  HCI_LE_ReadMaxDataLenCmd();
  
  dispHandle = Display_open(Display_Type_LCD, NULL); //初始化LCD
  
  //Register to receive UART messages
  SDITask_registerIncomingRXEventAppCB(simple_enqueueUARTMsg);
  

  HwGPIOInit();  
  
  uint8_t v = osal_snv_read(snvBleID, sizeof(snvData),&snvData);
  
  ble_name_Update();               //动态修改名称
  
  if(snvData.wflashCount == 0)
    snvData.wflashCount = 512;     //从第二页开始写入数据
  
  if(snvData.eraseCount == 0)
    snvData.eraseCount = 2;       //第二页写完后，从第三页开始清除数据
  
  alarmPeriod = (snvData.timeInterval * 1000) / 200;//定时周期转换，200:200毫秒定时
  if(alarmPeriod == 0)
  {
    alarmPeriod = 5;
  }
     //HwFlashErase(0);            //清除flash的第一页存储的数据
   //HwFlashErase(4096);           //清楚flash的第二页存储的数据
  
}

/*********************************************************************
* @fn      SimpleBLEPeripheral_taskFxn
*
* @brief   Application task entry point for the Simple BLE Peripheral.
*
* @param   a0, a1 - not used.
*
* @return  None.
*/

static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimpleBLEPeripheral_init();
  
  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);
    
    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;
      
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;
        
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;
          
          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & SBP_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              SimpleBLEPeripheral_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = SimpleBLEPeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }
        
        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }
      
      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          SimpleBLEPeripheral_processAppMsg(pMsg);
          
          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
      
      //如果串口RX接收数据队列不为空，处理串口数据 
      while (!Queue_empty(appUARTMsgQueue))                
      {
        //Get the message at the front of the queue but still keep it in the queue
        queueRec_t *pRec = Queue_head(appUARTMsgQueue);
        simpleUARTEvt_t *pMsg = (simpleUARTEvt_t *)pRec->pData;   
        
        if (pMsg)
        {
          
          bStatus_t retVal = FAILURE;
          switch(pMsg->event)
          {
          case MULTI_UART_DATA_EVT:
            {
               link_and_read_ledFlicker();
              if(!readFlag)
              {
                //将获取的串口数据转换成欧拉角
                retVal = LMPS_getPacket(pMsg->length, pMsg->pData, eulerDatas); 
                //SDITask_PrintfToUART("\r\n%f",eulerDatas[0]);             
                if(retVal!= SUCCESS)
                {
                  successFlag = 0;
                  //SDITask_PrintfToUART("failure\r\n");
                }
                else
                {
                  successFlag = 1;
                  //人工角度校准 
                  
                 eulerDatas[0] = eulerDatas[0] - zeroValue;
                 
                 if(eulerDatas[0] < -180)
                 {
                   eulerDatas[0] = eulerDatas[0] + 360;
                 }
                 else if(eulerDatas[0] > 180)
                 {
                   eulerDatas[0] = eulerDatas[0] - 360;
                 }
                  
                  if((gapProfileState == GAPROLE_CONNECTED) || (gapProfileState == GAPROLE_CONNECTED_ADV))
                  {
                    //将欧拉角数据拆成两个字节存储并上传
                    Apart_eulerData(eulerDatas);
                    Simple_Peripheral_NotiData(uploadEulerData, sizeof(uploadEulerData));
                  }
                }
              }
              
              break;
            }
          default:
            break;
          }
        }
        //remove from queue
        Util_dequeueMsg(appUARTMsgQueue);
        //Deallocate data payload beint tramsmitted
        ICall_freeMsg(pMsg->pData);
        //Free the space from the message
        ICall_free(pMsg);
        if(!Queue_empty(appUARTMsgQueue))
        {
          //Wake up the application to flush out any remaining UART data in the queue
          Semaphore_post(sem);
        }
      }
    }
    
    
    
    //200ms定时
    if(events & SBP_TIMER_PERIODIC_EVT)
    {
      events &= ~SBP_TIMER_PERIODIC_EVT;
      {
         //按下结束时停止震动
        if(!stateFlag && alarmFlag)
        {
          HwGPIOSet(Board_MOTOR, 0);
          alarmFlag = 0;
        }
        
        //是否开始监测
        if(stateFlag)
        {
          saveTiming++;
          
          simple_monitor();      //姿态监测
          if(saveTiming == 5)
          {
            simple_role_storage_data(eulerDatas);   //存储数据
            saveTiming = 0;
          }
        } 
      } 
    }
    
    //姿势报警
    if(alarmTiming == alarmPeriod)
    {
      alarmTiming = 0;
      simple_alarm();
    }
 
    //上传Flash数据
    if(readFlag)
    {
      Util_startClock(&timeuploadperiodicClock);   //开启定时器
      simple_role_uploadFlashdata();  
      delay_ms(11);
      if (events & SBP_UPLOAD_EVT)
      {
        events &= ~SBP_UPLOAD_EVT;
        {
          
        }
      }
    }
  }
}


/************************************************
* @fn    simple_role_monitor
*
* @brief monitor angle
*
* @param void
* 
* @return None
*/
static void simple_monitor(void)
{
  
  if(snvData.numericalRelation == 0x0B)
  {
    if(!(snvData.forwardErrorRange | snvData.backErrorRange))
    {
      return;
    }
    //取x轴的欧拉角与阈值相比较
    else if(eulerDatas[0] > snvData.forwardErrorRange || eulerDatas[0] < snvData.backErrorRange)
    {
      //大于或小于所设定的值时，超过一定次数时就进行报警
        alarmTiming++;  
        if(alarmTiming == 2)
        {
          HwGPIOSet(Board_MOTOR, 0);
        }
    }
    else
    { 
        alarmTiming = 0;
        HwGPIOSet(Board_MOTOR, 0);
    }
  }
//  else if(numericalRelation == 0x0C)
//  {
//    if(eulerDatas[0] < forwardErrorRange && eulerDatas[0] > backErrorRange)
//    {  
//        alarmTiming++;
//    }
//    else
//    {
//      //Stop clock
//      Util_stopClock(&periodicClock);
//      Timer_Start = 1;
//      //Motor stop
//      HwGPIOSet(Board_MOTOR, 0);
//    }
//  }
}

/***************************************************
* @fn     simple_role_alarm
*
* @brief  motor alarm
*
* @param void
*
* @return None
*/
static void simple_alarm(void)
{
  if(successFlag)
  {
    //Motor start
    HwGPIOSet(Board_MOTOR, 1);
    alarmFlag = 1;
  }
}
/*********************************************************************
* @fn      SimpleBLEPeripheral_processStackMsg
*
* @brief   Process an incoming stack message.
*
* @param   pMsg - message to process
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;
  
  switch (pMsg->event)
  {
  case GATT_MSG_EVENT:
    // Process GATT message
    safeToDealloc = SimpleBLEPeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
    break;
    
  case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch(pMsg->status)
      {
      case HCI_COMMAND_COMPLETE_EVENT_CODE:
        // Process HCI Command Complete Event
        break;
        
      default:
        break;
      }
    }
    break;
    
  default:
    // do nothing
    break;
  }
  
  return (safeToDealloc);
}

/*********************************************************************
* @fn      SimpleBLEPeripheral_processGATTMsg
*
* @brief   Process GATT messages and events.
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      SimpleBLEPeripheral_freeAttRsp(FAILURE);
      
      // Hold on to the response message for retransmission
      pAttRsp = pMsg;
      
      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.
    
    // Display the opcode of the message that caused the violation.
    //Display_print1(dispHandle, 5, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    ////Display_print1(dispHandle, 5, 0, "MTU Size: $d", pMsg->msg.mtuEvt.MTU);
    //SDITask_PrintfToUART("\r\nMTU Size: %d",pMsg->msg.mtuEvt.MTU);
  }
  
  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
  
  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
* @fn      SimpleBLEPeripheral_sendAttRsp
*
* @brief   Send a pending ATT response message.
*
* @param   none
*
* @return  none
*/
static void SimpleBLEPeripheral_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;
    
    // Increment retransmission count
    rspTxRetry++;
    
    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);
      
      // We're done with the response message
      SimpleBLEPeripheral_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      //Display_print1(dispHandle, 5, 0, "Rsp send retry: %d", rspTxRetry);
    }
  }
}

/*********************************************************************
* @fn      SimpleBLEPeripheral_freeAttRsp
*
* @brief   Free ATT response message.
*
* @param   status - response transmit status
*
* @return  none
*/
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      //Display_print1(dispHandle, 5, 0, "Rsp sent retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
      
      //Display_print1(dispHandle, 5, 0, "Rsp retry failed: %d", rspTxRetry);
    }
    
    // Free response message
    ICall_freeMsg(pAttRsp);
    
    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
* @fn      SimpleBLEPeripheral_processAppMsg
*
* @brief   Process an incoming callback from a profile.
*
* @param   pMsg - message to process
*
* @return  None.
*/
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
  case SBP_STATE_CHANGE_EVT:
    SimpleBLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                              hdr.state);
    break;
    
  case SBP_CHAR_CHANGE_EVT:
    SimpleBLEPeripheral_processCharValueChangeEvt(pMsg->hdr.state);
    break;
    
  default:
    // Do nothing.
    break;
  }
}

/*********************************************************************
* @fn      SimpleBLEPeripheral_stateChangeCB
*
* @brief   Callback from GAP Role indicating a role state change.
*
* @param   newState - new state
*
* @return  None.
*/
static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState);
}

/*********************************************************************
* @fn      SimpleBLEPeripheral_processStateChangeEvt
*
* @brief   Process a pending GAP Role state change event.
*
* @param   newState - new state
*
* @return  None.
*/
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER
  
  switch ( newState )
  {
  case GAPROLE_STARTED:
    {
      uint8_t ownAddress[B_ADDR_LEN];
      uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];
      
      GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
      
      // use 6 bytes of device address for 8 bytes of system ID value
      systemId[0] = ownAddress[0];
      systemId[1] = ownAddress[1];
      systemId[2] = ownAddress[2];
      
      // set middle bytes to zero
      systemId[4] = 0x00;
      systemId[3] = 0x00;
      
      // shift three bytes up
      systemId[7] = ownAddress[5];
      systemId[6] = ownAddress[4];
      systemId[5] = ownAddress[3];
      
      DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
      
      // Display device address
      //Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(ownAddress));
      //Display_print0(dispHandle, 2, 0, "Initialized");
    }
    break;
    
  case GAPROLE_ADVERTISING:
    //Display_print0(dispHandle, 2, 0, "Advertising");  
    HwGPIOSet(Board_DK_LED3, 0);
    isLedflicker = 0;
    break;
    
#ifdef PLUS_BROADCASTER
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
    * sending non-connectable advertisements and shall sending this change of
    * state to the application.  These are then disabled here so that sending
    * connectable advertisements can resume.
    */
  case GAPROLE_ADVERTISING_NONCONN:
    {
      uint8_t advertEnabled = FALSE;
      
      // Disable non-connectable advertising.
      GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);
      
      advertEnabled = TRUE;
      
      // Enabled connectable advertising.
      GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                           &advertEnabled);
      
      // Reset flag for next connection.
      firstConnFlag = false;
      
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
    }
    break;
#endif //PLUS_BROADCASTER
    
  case GAPROLE_CONNECTED:
    {
      linkDBInfo_t linkInfo;
      uint8_t numActive = 0;
      
      //Util_startClock(&periodicClock);
      
      numActive = linkDB_NumActive();
      

      // Use numActive to determine the connection handle of the last
      // connection
      if ( linkDB_GetInfo( numActive - 1, &linkInfo ) == SUCCESS )
      {
        //Display_print1(dispHandle, 2, 0, "Num Conns: %d", (uint16_t)numActive);
        //Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(linkInfo.addr));
        //HwGPIOSet(Board_DK_LED3, 1);
        isLedflicker = 0x01;
        ledFlickerFrequency = 0;
      }
      else
      {
        uint8_t peerAddress[B_ADDR_LEN];
        
        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);
        
        //Display_print0(dispHandle, 2, 0, "Connected");
        //Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddress));
        
      }
      
#ifdef PLUS_BROADCASTER
      // Only turn advertising on for this state when we first connect
      // otherwise, when we go from connected_advertising back to this state
      // we will be turning advertising back on.
      if (firstConnFlag == false)
      {
        uint8_t advertEnabled = FALSE; // Turn on Advertising
        
        // Disable connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);
        
        // Set to true for non-connectabel advertising.
        advertEnabled = TRUE;
        
        // Enable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                             &advertEnabled);
        firstConnFlag = true;
      }
#endif // PLUS_BROADCASTER
    }
    break;
    
  case GAPROLE_CONNECTED_ADV:
    //Display_print0(dispHandle, 2, 0, "Connected Advertising");
    break;
    
  case GAPROLE_WAITING:
    // Util_stopClock(&periodicClock);
    SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
    
    //Display_print0(dispHandle, 2, 0, "Disconnected");
    
    // Clear remaining lines
    Display_clearLines(dispHandle, 3, 5);
    break;
    
  case GAPROLE_WAITING_AFTER_TIMEOUT:
    SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
    
    //Display_print0(dispHandle, 2, 0, "Timed Out");
    
    // Clear remaining lines
    Display_clearLines(dispHandle, 3, 5);
    
#ifdef PLUS_BROADCASTER
    // Reset flag for next connection.
    firstConnFlag = false;
#endif //#ifdef (PLUS_BROADCASTER)
    break;
    
  case GAPROLE_ERROR:
    //Display_print0(dispHandle, 2, 0, "Error");
    break;
    
  default:
    Display_clearLine(dispHandle, 2);
    break;
  }
  
  // Update the state
  gapProfileState = newState;
}

#ifndef FEATURE_OAD_ONCHIP
/*********************************************************************
* @fn      SimpleBLEPeripheral_charValueChangeCB
*
* @brief   Callback from Simple Profile indicating a characteristic
*          value change.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID);
}
#endif //!FEATURE_OAD_ONCHIP

/*********************************************************************
* @fn      SimpleBLEPeripheral_processCharValueChangeEvt
*
* @brief   Process a pending Simple Profile characteristic value change
*          event.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID)
{
  
  uint8_t bleBuf[83];
  
  for(uint8_t i = 0; i < 3; i++)
     {
       HwGPIOSet(Board_DK_LED3,1);
       delay_ms(100);
       HwGPIOSet(Board_DK_LED3,0);
       delay_ms(100);
     }
  
  switch(paramID)
  {
  case SIMPLEPROFILE_CHAR1:
    
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, bleBuf);
    
    //获取软件端所设置的参数
    if(bleBuf[1] == 0x3D && bleBuf[2] == 0x01&& bleBuf[3] == 0x01 && bleBuf[12] == 0x0D && bleBuf[13] == 0x0A)
    {
       snvData.forwardErrorRange = bleBuf[6] | ( bleBuf[5] << 8 );//前倾阈值
       snvData.backErrorRange = bleBuf[8] | (bleBuf[7] << 8); //后仰阈值
       snvData.timeInterval = bleBuf[10] | (bleBuf[9] << 8); //时间间隔
       snvData.numericalRelation = bleBuf[11] ;                  //大于(小于)
      
      alarmPeriod = (snvData.timeInterval * 1000) / 200;
      alarmTiming = 0;

      //save receivedata
      osal_snv_write(snvBleID, sizeof(snvData), &snvData);
    }
    //获取开始与结束指令
    else if(bleBuf[1] == 0x3C && bleBuf[2] == 0x01)
    {
      readFlag = 0;
      stateFlag = bleBuf[3];

       if(stateFlag == 1)
      {
        
     
         //存储开始地址
        time_address[0] = (uint8_t)((TXBUF_LENGTH * snvData.wflashCount) & 0xFF);
        time_address[1] = (uint8_t)(((TXBUF_LENGTH * snvData.wflashCount) >> 8) & 0xFF);
        time_address[2] = (uint8_t)(((TXBUF_LENGTH * snvData.wflashCount) >> 16) & 0xFF);
        //存储开始时间
        for(uint8_t i = 3; i < 11; i++)
        {
          time_address[i] = bleBuf[1 + i];
        }
        HwFlashWrite(time_address_LENGTH * snvData.startCount, time_address, time_address_LENGTH);
        //存储开始次数
        snvData.startCount++;
        //snvData[4] = startCount;
        osal_snv_write(snvBleID, sizeof(snvData), &snvData);
        
      }
    }
    //获取读取指令
    else if(bleBuf[1] == 0x3B && bleBuf[2] == 0x01 && bleBuf[4] == 0x0D && bleBuf[5] == 0x0A)
    {
      if(snvData.startCount == 0)
      {
        readFlag = 0;
        snvData.startCount = 1;
        simple_role_sendStartFrame();
        simple_role_sendEndFrame();
        snvData.startCount = 0;
        return;
      }
      
      osal_snv_read(snvBleID, sizeof(snvData),&snvData);

      readFlag = bleBuf[3];
      isUpload = 1;
      stateFlag = 0;
      isLedflicker = 0x02;
      ledFlickerFrequency = 0;
    }
    else if(bleBuf[1] == 0x3E)
    {
      
      //清零指令
      if(bleBuf[2] == 0x01)
      zeroValue = eulerDatas[0] + zeroValue;
      //修改名称指令
      else if(bleBuf[2] == 0x06)
      {
        
        if(bleBuf[0] == 4 || bleBuf[0] == 0)
        {
          return;
        }
        else 
        {
          
          snvData.bleNameLength = bleBuf[0] - 4;        //去掉帧头和帧尾
          memcpy(&(snvData.bleName), &bleBuf[3], snvData.bleNameLength);
          osal_snv_write(snvBleID, sizeof(snvData), &snvData);
          ble_name_Update();  
        }
        
      }
        
    }
    
    break;  
  default:
    // should not reach here!
    break;
  }
}
/*********************************************************************
* @fn      Simple_Peripheral_NotiData
*
* @brief   Sends ATT notifications in a tight while loop to demo
*          throughput
*
* @param   none
*
* @return  none
*/
void Simple_Peripheral_NotiData(uint8_t *buf, uint16_t len)
{
  // Subtract the total packet overhead of ATT and L2CAP layer from notification payload
  bStatus_t status;
  attHandleValueNoti_t noti;
  noti.handle = 0x27;
  noti.len = len>80 ? 80 : len;
  
  noti.pValue = (uint8 *)GATT_bm_alloc(0, ATT_HANDLE_VALUE_NOTI, 100, NULL);
  if ( noti.pValue != NULL )
  {
    memcpy(noti.pValue, buf, len);
    status = GATT_Notification(0, &noti, FALSE);
    if(status != SUCCESS)
    {
      GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
    }
  }
}

#ifdef FEATURE_OAD
/*********************************************************************
* @fn      SimpleBLEPeripheral_processOadWriteCB
*
* @brief   Process a write request to the OAD profile.
*
* @param   event      - event type:
*                       OAD_WRITE_IDENTIFY_REQ
*                       OAD_WRITE_BLOCK_REQ
* @param   connHandle - the connection Handle this request is from.
* @param   pData      - pointer to data for processing and/or storing.
*
* @return  None.
*/
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
    sizeof(uint8_t) * OAD_PACKET_SIZE);
  
  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;
    
    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);
    
    Queue_put(hOadQ, (Queue_Elem *)oadWriteEvt);
    
    // Post the application's semaphore.
    Semaphore_post(sem);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD

/*********************************************************************
* @fn      SimpleBLEPeripheral_clockHandler
*

* @par* @brief   Handler function for clock timeouts.
*am   arg - event type
*
* @return  None.
*/
static void SimpleBLEPeripheral_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;
  
  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
* @fn      SimpleBLEPeripheral_enqueueMsg
*
* @brief   Creates a message and puts the message in RTOS queue.
*
* @param   event - message event.
* @param   state - message state.
*
* @return  None.
*/
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state)
{
  sbpEvt_t *pMsg;
  
  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    
    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}

/*********************************************************************
*********************************************************************/

static void simple_enqueueUARTMsg(uint8_t event, uint8_t *data, uint8_t len)
{
  simpleUARTEvt_t *pMsg;
  //  if((gapProfileState == GAPROLE_CONNECTED) || (gapProfileState == GAPROLE_CONNECTED_ADV))
  //  {
  //Enqueue message only in a connected state
  if(pMsg = ICall_malloc(sizeof(simpleUARTEvt_t)))
  {
    
    pMsg->event = event;
    pMsg->pData = (uint8 *)ICall_allocMsg(len);
    if(pMsg->pData)
    {
      //payload
      memcpy(pMsg->pData, data, len);
    }
    pMsg->length = len;
    //Enqueue the message
    Util_enqueueMsg(appUARTMsgQueue, sem, (uint8 *)pMsg);
  }
  //  }
}

/*********************************************
* @fn int2float
* 
* @brief int to float
*
* @param 
*
* @return rad data
*/
static float  int2float(float *p_f, uint8_t *p_u8)
{
  int2f_t i2f;
  
  for(uint8_t i = 0; i < 4; i++)
  {
    i2f.u8vals[i] = p_u8[i];
  }
  
  *p_f = i2f.fval;
  
  return i2f.fval;
  
}
/**********************************************************
* @fn LMPS_getPacket
*
* @brief
* 
* @param
* 
* @return success -if the data intergrity
*/
static uint8_t LMPS_getPacket(uint8_t len, uint8_t *pData, float *eulerData)
{
  int ret;
  
  VOID memcpy(sensorData, pData, len);
  ret = memcmp(serialCompare, sensorData, sizeof(serialCompare));
  if(!ret)
  {
    LPMS_parseSensorData(eulerData);
    //SDITask_PrintfToUART("z = %f\r\n", eulerData[2]);
  }
  else 
    return FAILURE;
  return SUCCESS;
}
/************************************************
* @fn LPMS_parseSensorData
*
* @brief get the sensor data
* 
* param rad data
*
* return None
*/
static void LPMS_parseSensorData(float *eulerData)
{
  uint8_t len = 11;
  
  for(uint8_t i = 0; i < 3; i++)
  {
    int2float(&eulerData[i], &sensorData[len]);
    eulerData[i] *= R_TO_D;
    len += 4;
  }
}

/****************************************************
* @fn Apart_eulerData
*
* @brief   apart variable
*
* @param   eulerData - sensordata
*
* @return None
*/
static void Apart_eulerData(float *eulerData)
{
  int16_t temp_x, temp_y, temp_z;
  temp_x = eulerData[0] * 10;
  temp_y = eulerData[1] * 10;
  temp_z = eulerData[2] * 10;
  uploadEulerData[0] = 0x3A;
  uploadEulerData[1] = 0x01;
  uploadEulerData[2] = (int8_t)(temp_x & 0x7F);
  uploadEulerData[3] = (int8_t)((temp_x >> 7) & 0xFF);
  uploadEulerData[4] = (int8_t)(temp_y & 0x7F);
  uploadEulerData[5] = (int8_t)((temp_y >> 7) & 0xFF);
  uploadEulerData[6] = (int8_t)(temp_z & 0x7F);
  uploadEulerData[7] = (int8_t)((temp_z >> 7) & 0xFF);
  uploadEulerData[8] = 0x0D;
  uploadEulerData[9] = 0x0A;
}

/********************************************
* @fn Storage_Data
*
* @brief save sensor data
*
* @param eulerData - sensor data
*
* @return None
*/
/*flash最小擦除字节数：4096个字节*/
static void simple_role_storage_data(float *eulerData)
{
  int16_t tempx ;
  
  tempx = eulerData[0] * 10;    
  txbuf[tx_count] = (int8_t)(tempx & 0x7F);
  tx_count++;
  txbuf[tx_count] = (int8_t)((tempx >> 7) & 0xFF);
  tx_count++;
  
  //每TXBUF_LENGTH个字节存储一次数据
  if(tx_count == TXBUF_LENGTH)
  {
    HwFlashWrite(TXBUF_LENGTH * snvData.wflashCount, txbuf, TXBUF_LENGTH);
    snvData.wflashCount ++;
    pageErase ++;
    tx_count = 0;
    osal_snv_write(snvBleID, sizeof(snvData), &snvData);

    //SDITask_PrintfToUART("\r\npagecount =%d=",wflashCount);
  }
  //每4096个字节擦除一次
  if(pageErase == 512)
  {
    HwFlashErase(4096 * snvData.eraseCount);
    snvData.eraseCount++;
    pageErase = 0;
    // SDITask_PrintfToUART("\r\neraseCount =%d=",eraseCount);
    if(snvData.eraseCount == 2048)
    {
      snvData.wflashCount = 512;
      HwFlashErase(4096);
      snvData.eraseCount = 2;
    }
  }
}

/**********************************************************
* @fn simple_role_uploadFlashdata
*
* @brief upload the falsh data
*
* @param falsh data
*
* @return None
*/

static void simple_role_uploadFlashdata(void)
{
  if(isUpload)
  {  
     //提取保存的数据和地址
      HwFlashRead(11 * read_count, time_address, 14);
      frontAddress = time_address[0] | time_address[1] << 8;
      frontAddress = frontAddress | time_address[2] << 16;
      //如果只开始一次，算出总共存储的数据
      if(snvData.startCount == 1)
      {
        uploadCount = (snvData.wflashCount * TXBUF_LENGTH -  frontAddress) / RXBUF_LENGTH;
        //uploadCount = 1350;
      }
      //算出开始记录不为1时某个时间段内的数据
      else if(snvData.startCount > 1)
      {
        behindAddress = time_address[11] | time_address[12] << 8;
        behindAddress = behindAddress | time_address[13] << 16;
        uploadCount = (behindAddress - frontAddress) / RXBUF_LENGTH;
      }
      
      simple_role_sendStartFrame();// 发送开始标志
  }
  
  if(uploadCount!=0)
  {
    simple_role_uploadCurveData();
    //SDITask_PrintfToUART("\r\n uploadcount = %d ",uploadCount);  
    
  }
  else if(uploadCount == 0 )
  {
      //发送完成的结束标志
      simple_role_sendEndFrame();
      if(snvData.startCount == 1)
      {
        readFlag = 0;
        snvData.startCount = 0;
        snvData.eraseCount = 2;
        read_count = 0;
        snvData.wflashCount = 512;
        extractflashCount = 256;
        osal_snv_write(snvBleID, sizeof(snvData), &snvData);
        HwFlashErase(0);
        HwFlashErase(4096);
        isLedflicker = 0x01;
        ledFlickerFrequency = 0;
      }
      else
      {
        snvData.startCount--;
        isUpload = 1;
        read_count++;
      }
  }
}

//发送数据传送开始帧
static void simple_role_sendStartFrame(void)
{
  uploadFlashdata[0] = 0x3B;
  uploadFlashdata[1] = 0x01;
  uploadFlashdata[2] = (int8_t)(3610 & 0x7F);
  uploadFlashdata[3] = (int8_t)((3610 >> 7) & 0xFF);
  for(uint8_t i = 0; i < 8; i++)
  {
    uploadFlashdata[4 + i] = time_address[3 + i];
  }
  uploadFlashdata[12] = snvData.startCount;
  uploadFlashdata[13] = (int8_t)(1000 & 0xff);
  uploadFlashdata[14] = (int8_t)((1000 >> 8) & 0xff);
  uploadFlashdata[18] = 0x0D;
  uploadFlashdata[19] = 0x0A;
  Simple_Peripheral_NotiData(uploadFlashdata, UPLOADFLASHDATA_LENGTH);
  isUpload = 0;
}
//发送完成的结束标志
static void simple_role_sendEndFrame(void)
{
  uploadFlashdata[0] = 0x3B;
  uploadFlashdata[1] = 0x01;
  uploadFlashdata[2] = 0;
  uploadFlashdata[3] = 0;
  uploadFlashdata[16] = (int8_t)(3610 & 0x7F);
  uploadFlashdata[17] = (int8_t)((3610 >> 7) & 0xFF);
  uploadFlashdata[18] = 0x0D;
  uploadFlashdata[19] = 0x0A;
  Simple_Peripheral_NotiData(uploadFlashdata, UPLOADFLASHDATA_LENGTH);
}

static void simple_role_uploadCurveData(void)
{
  //每一次提取RXBUF_LENGTH个字节
  HwFlashRead(RXBUF_LENGTH * extractflashCount, rxbuf, RXBUF_LENGTH);
  uploadFlashdata[0] = 0x3B;
  uploadFlashdata[1] = 0x01;
  for(uint16_t i = 2, j = 0; i < 18; i++,j++)
  {
    uploadFlashdata[i] = rxbuf[j];
    i++;
    j++;
    uploadFlashdata[i] = rxbuf[j];
  }    
  uploadFlashdata[18] = 0x0D;
  uploadFlashdata[19] = 0x0A;
  
  Simple_Peripheral_NotiData(uploadFlashdata, UPLOADFLASHDATA_LENGTH);
  uploadCount--;
  extractflashCount++;
}
//电量的ADC读取
static void ADC_Read(void)
{
    AUXWUCClockEnable(AUX_WUC_MODCLKEN0_ANAIF_M|AUX_WUC_MODCLKEN0_AUX_ADI4_M);
    AUXADCSelectInput(ADC_COMPB_IN_AUXIO7);
    AUXADCEnableSync(AUXADC_REF_FIXED,AUXADC_SAMPLE_TIME_10P9_MS, AUXADC_TRIGGER_MANUAL);
    AUXADCGenManualTrigger();
    ADCVal = AUXADCReadFifo();
    AUXADCDisable(); 
}
//灯的闪烁
static void link_and_read_ledFlicker(void)
{
  switch(isLedflicker)
  {
  case 0x01:
    {
      //手机连接led灯时的闪烁频率
      ledFlickerFrequency++;
      if(ledFlickerFrequency == 10)
      {
        HwGPIOSet(Board_DK_LED3, 1);
      }
      else if(ledFlickerFrequency == 20)
      {
        ledFlickerFrequency = 0;
        HwGPIOSet(Board_DK_LED3, 0);
      }
      break;
    }
  case 0x02:
    {
      //读取flash数据时led的闪烁
      ledFlickerFrequency++;
      if(ledFlickerFrequency == 2)
      {
        HwGPIOSet(Board_DK_LED3, 1);
      }
      else if(ledFlickerFrequency == 4)
      {
        ledFlickerFrequency = 0;
        HwGPIOSet(Board_DK_LED3, 0);
      }
      break;
    }
  default:
    break;
  }
}

static void ble_name_Update(void)
{
  //如果没有获取到数据，则返回(指令包括帧头和帧尾共四个字节)
  if(snvData.bleNameLength == 0)
  {
    return;
  }
  //扫描响应的第一个字节存储的是从机名的长度+1，这个1是GAP_ADTYPE_LOCAL_NAME_COMPLETE
  scanRspData[0] = snvData.bleNameLength + 1;
  scanRspData[1] = GAP_ADTYPE_LOCAL_NAME_COMPLETE;
  memcpy(&scanRspData[2], &(snvData.bleName), snvData.bleNameLength);
  memcpy(attDeviceName, &(snvData.bleName), snvData.bleNameLength);
  GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, snvData.bleNameLength + 2,
                         scanRspData);
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, snvData.bleNameLength, attDeviceName);
}