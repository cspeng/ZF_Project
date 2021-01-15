/*
* Filename: multi_role.c
*
* Description: This file contains the multi_role sample application for use
* with the CC2650 Bluetooth Low Energy Protocol Stack.
*
*
* Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

/*********************************************************************
* INCLUDES
*/
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "bcomdef.h"
#include "hci_tl.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simple_gatt_profile.h"

#include "multi.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"
#include "iotboard_key.h"
#include <ti/mw/display/Display.h>
#include "board.h"

#include "multi_role.h"
#include "inc/sdi_task.h"
#include "inc/sdi_tl_uart.h"
//#include "task_uart.h"
#include "hw_gpio.h"
#include "matrix.h"
#include <driverlib/aux_adc.h>
#include <driverlib/aux_wuc.h>
#include "oad_target.h"
/*********************************************************************
* CONSTANTS
*/
// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Connection parameters if automatic  parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     16
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     16
#define DEFAULT_DESIRED_SLAVE_LATENCY         0
#define DEFAULT_DESIRED_CONN_TIMEOUT          100

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

//connection parameters
#define DEFAULT_CONN_INT                      45
#define DEFAULT_CONN_TIMEOUT                  400
#define DEFAULT_CONN_LATENCY                  0

//How often to perform periodic event(in msec)
#define SBP_PERIODIC_EVT_PERIOD               5000
#define SBP_PERIODIC_TIMER_EVT_PERIOD         200
#define SBP_PERIODIC_UPLOAD_EVT_PERIOD        9

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// Scan parameters
#define DEFAULT_SCAN_DURATION                 4000
#define DEFAULT_SCAN_WIND                     80
#define DEFAULT_SCAN_INT                      80

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  12

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
#if !defined(BOARD_DISPLAY_EXCLUDE_LCD)
#define SBC_DISPLAY_TYPE Display_Type_LCD
#elif !defined (BOARD_DISPLAY_EXCLUDE_UART)
#define SBC_DISPLAY_TYPE Display_Type_UART
#else // BOARD_DISPLAY_EXCLUDE_LCD && BOARD_DISPLAY_EXCLUDE_UART
#define SBC_DISPLAY_TYPE 0 // Option not supported
#endif // BOARD_DISPLAY_EXCLUDE_LCD
#else // Display_DISABLE_ALL
#define SBC_DISPLAY_TYPE 0 // No Display
#endif // Display_DISABLE_ALL

// Task configuration
#define MR_TASK_PRIORITY                     1
#ifndef MR_TASK_STACK_SIZE
#define MR_TASK_STACK_SIZE                   1000
#endif

// Internal Events for RTOS application
#define MR_STATE_CHANGE_EVT                  0x0001
#define MR_CHAR_CHANGE_EVT                   0x0002
#define MR_CONN_EVT_END_EVT                  0x0004
#define MR_KEY_CHANGE_EVT                    0x0008
#define MR_PAIRING_STATE_EVT                 0x0010
#define MR_PASSCODE_NEEDED_EVT               0x0020
#define SBP_PERIODIC_EVT                     0x0004
#define SBP_TIMER_PERIODIC_EVT               0x0010
#define SBP_UPLOAD_EVT                       0x0040
// Discovery states
typedef enum {
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
} discState_t;

// Key states for connections
typedef enum {
  SCAN,                    // Scan for devices
  CONNECT,                 // Establish a connection to a discovered device
  GATT_RW,                 // Perform GATT Read/Write
  CONN_UPDATE,             // Send Connection Parameter Update
  DISCONNECT,              // Disconnect
  ADVERTISE,               // Turn advertising on / off
} keyPressConnOpt_t;

// Key states for connections
typedef enum {
  MAIN_MENU,               // top level menu
  DISC_MENU,               // Choose discovered device to connect to
  CONN_MENU                // Choose connected device to perform operation on               
} menuLevel_t;



// pointer to allocate the connection handle map
static uint16_t *connHandleMap;

/*********************************************************************
* TYPEDEFS
*/

// App event passed from profiles.
typedef struct
{
  uint16_t event;  // event type
  uint8_t *pData; // event data pointer
} mrEvt_t;

// pairing callback event
typedef struct
{
  uint16 connectionHandle; //!< connection Handle
  uint8 state;             //!< state returned from GAPBondMgr
  uint8 status;            //!< status of state
} gapPairStateEvent_t;

// discovery information
typedef struct
{
  discState_t discState;   //discovery state
  uint16_t svcStartHdl;    //service start handle     
  uint16_t svcEndHdl;      //service end handle
  uint16_t charHdl;        //characteristic handle
} discInfo_t;
typedef struct _queueRec_
{
  Queue_Elem _elem;
  uint8_t *pData;
}queueRec_t;
typedef struct 
{
  uint8_t event;
  uint8_t *pData;
  uint8_t length;
}multiUARTEvt_t;


/*********************************************************************
* GLOBAL VARIABLES
*/

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
* LOCAL VARIABLES
*/

/*********************************************************************
* LOCAL VARIABLES
*/

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

//Clock instances for internal periodic events.
static Clock_Struct periodicClock;
static Clock_Struct timeperiodicClock;
static Clock_Struct timeuploadperiodicClock;
// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Queue object used for uart messages
//static Queue_Struct uartMsg;
//static Queue_Handle uartRxQueue;
static Queue_Struct appUARTMsg;
static Queue_Handle appUARTMsgQueue;

//events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct mrTask;
Char mrTaskStack[MR_TASK_STACK_SIZE];

static uint8_t scanRspData[20] =
{
  // complete name
  15,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  's', 'e', 'n', 's', 'o', 'r', '0', ' ', 'm', 'a', 's','t','e','r'
};
//static uint8_t scanRspData[] =
//{
//  // complete name
//  16,   // length of this data
//  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
//  'G', 'a', 'i', 't', ' ', 'M', 'o', 'n', 'i', 't', 'o','r','i','n','g',
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
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  HI_UINT16(SIMPLEPROFILE_SERV_UUID)
};

// GAP GATT Attributes
static uint8_t attDeviceName[20] = "sensor0 master";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// Pointer to per connection discovery info
discInfo_t *discInfo;

// Maximim PDU size (default = 27 octets)
static uint16 maxPduSize;  

// Number of scan results and scan result index for menu functionality
static uint8_t scanRes;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// connection index for mapping connection handles
static uint16_t connIndex = INVALID_CONNHANDLE;

// maximum number of connected devices
static uint8_t maxNumBleConns = MAX_NUM_BLE_CONNS;

// 主从机状态指示，主要用于串口数据无线发送时判断role
static uint8_t roleTrans = GAP_PROFILE_CENTRAL;


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
#define ADDRESSLENGTH          6

typedef union
{
  uint8_t u8vals[4];
  uint32_t u32val;
  float fval;
}f2int_t, int2f_t;

typedef struct
{
 uint32_t wflashCount ;       //写入数据到flash的次数
 uint16_t eraseCount ;        //  擦除flash的次数
 uint8_t startCount ;          //开始次数
 int16_t upperLimit ;         //步态上限
 int16_t lowerLimit ;         //步态下限
 uint16_t timeInterval ;      //报警时间间隔
 uint8_t numericalRelation ;   //大于/小于
 uint8_t bleNameLength ;   //修改蓝牙名称指令长度
 uint8_t bleName[MAXNAMELENGTH];  //修改蓝牙名称
 uint8_t slaveAddress[ADDRESSLENGTH];  //修改从设备地址
}snv_t;
snv_t snvData;
bool Timer_Start = 1;         //启动震动标志
extern uint8_t calibrateFlag; //校准标志
static uint8_t stateFlag = 0;        //开始与结束标志
static uint8_t successFlag = 1;      //转换串口数据成功标志
static uint8_t alarmFlag = 0;        //报警标志
static float euler3Data[9] = {0};      //存放夹角
static int8_t uploadEulerData[22] = {0};  //发送给软件端的数据
static uint8_t sensordev1Data[SENSOR_DATA_LENGTH];    //存放串口数据
static uint8_t sensordev2Data[SENSOR_DATA_LENGTH];  //存放传感器2的数据
static uint8_t serialCompare[] = {0x3A, 0x01, 0x00, 0x09, 0x00, 0x10};   //串口数据的前5个字节
static uint8_t motorCommand[] = {0x3C, 0x01, 0x01, 0x0D, 0x0A};         //震动指令
static uint8_t calibrateCommand[] = {0x3E, 0x01, 0x01, 0x0D, 0x0A};     //校准指令
static float T10[N][N], T20[N][N], T01[N][N], T21[N][N], Projective_xyz21[3];


static int8_t txbuf[TXBUF_LENGTH];          //存放写入flash的数据
static int8_t rxbuf[RXBUF_LENGTH];          //存放提取的flash数据
static uint16_t tx_count = 0;               //存放提取的flash数据
static float eulerData_1[3];                //存放传感器1的欧拉角
static float eulerData_2[3];                //存放传感器2的欧拉角
static uint16_t pageErase = 0;             //  //  根据每一次存储的字节数来确定擦除一次flash  
static uint32_t uploadCount = 0;            //  上传flash数据的次数
static uint8_t isUpload = 1;               //  计算上传总数的标志
static int8_t uploadFlashdata[70] = {0};    //存放上传的flash数据
static uint32_t extractflashCount = 256;             //提取flash次数
static uint8_t readFlag = 0;                //读取flash的标志
static uint8_t endFlag = 1;
static uint8_t storagePeriod = 0;                  //定时存储到flash的标志
static bStatus_t retVal_1 = FAILURE;
static uint16_t alarmTiming = 0;                   
static uint16_t alarmPeriod = 5;              //报警周期 
static uint16_t failureCount = 0;            //接收数据失败次数
static uint8_t read_count = 0;                //读取FLASH次数
static uint8_t time_address [14] = {0};    //保存时间和地址
static uint32_t frontAddress = 0;           //开始地址
static uint32_t behindAddress = 0;         //结束地址
static float zeroValue = 0 ;             //角度清零值
static uint8_t isLedflicker = 0;                //灯是否闪烁标志
static uint8_t ledFlickerFrequency = 0;        //led闪烁周期
static uint16_t volMeasuinterval = 0;         //电量测量间隔
static uint8_t powerAlarmtime = 0;            //电量报警周期
static uint8_t powerAlarm = 0;               //电量报警标志
static uint16_t ADCVal;                      //电量的adc读取
static uint8_t scanTiming = 0;               //扫描周期
static uint8_t isLinkperipheral = false;   //判断是否连接从设备
/*********************************************************************
* LOCAL FUNCTIONS
*/
static void multi_alarm(void);
static void ADC_Read(void);
static void volRead(void);
static void multi_monitor(void);
static void link_and_read_ledFlicker(void);
static void multi_storage_data(void);
static void multi_role_uploadFlashdata(void);
static void Apart_eulerData(float *eulerData);
static float int2float(float *p_f, uint8_t *p_u8);
static void LPMS_parseSensorData(float *eulerData);
static void multi_role_clockHandler(UArg arg);
static uint8_t LMPS_getPacket(uint8_t len, uint8_t *pData, float *eulerData);

static void multi_role_init( void );
static void multi_role_taskFxn(UArg a0, UArg a1);
static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg);
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg);
static void multi_role_processAppMsg(mrEvt_t *pMsg);
static void multi_role_processCharValueChangeEvt(uint8_t paramID);
static void multi_role_processRoleEvent(gapMultiRoleEvent_t *pEvent);
static void multi_role_sendAttRsp(void);
static void multi_role_freeAttRsp(uint8_t status);
static void multi_role_charValueChangeCB(uint8_t paramID);
static uint8_t multi_role_enqueueMsg(uint16_t event, uint8_t *pData);
static void multi_role_startDiscovery(uint16_t connHandle);
static void multi_role_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static bool multi_role_findSvcUuid(uint16_t uuid, uint8_t *pData, uint8_t dataLen);
static void multi_role_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static void multi_role_handleKeys(uint8_t keys);
static uint8_t multi_role_eventCB(gapMultiRoleEvent_t *pEvent);
static void multi_role_sendAttRsp(void);
static void multi_role_freeAttRsp(uint8_t status);
static uint16_t multi_role_mapConnHandleToIndex(uint16_t connHandle);
static void multi_role_keyChangeHandler(uint8 keysPressed);
static uint8_t multi_role_addMappingEntry(uint16_t connHandle);
static void multi_role_processPasscode(gapPasskeyNeededEvent_t *pData);
static void multi_role_processPairState(gapPairStateEvent_t* pairingEvent);
static void multi_role_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison);
static void multi_role_pairStateCB(uint16_t connHandle, uint8_t state,
                                   uint8_t status);


void TransUartReceiveDataCallback(uint8_t *buf, uint16_t len);
void Simple_Central_WriteData(uint8_t *buf,uint8_t len);
void Simple_Peripheral_NotiData(uint8_t *buf, uint16_t len);
static void multi_enqueueUARTMsg(uint8_t event, uint8_t *data, uint8_t len);
static void multi_role_sendStartFrame(void);
static void multi_role_sendEndFrame(void);
static void multi_role_uploadCurveData(void);
static void multi_role_paramUpdateDecisionCB(gapUpdateLinkParamReq_t *pReq,
                                             gapUpdateLinkParamReqReply_t *pRsp);

static void ble_name_Update(void);

// GAP Role Callbacks
//想更新主从设备和手机的连接参数，可以将注释去掉
static gapRolesCBs_t multi_role_gapRoleCBs =
{
  multi_role_eventCB,        // events to be handled by the app are passed through the GAP Role here
  //multi_role_paramUpdateDecisionCB      // Callback for application to decide whether to accept a param update
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t multi_role_simpleProfileCBs =
{
  multi_role_charValueChangeCB // Characteristic value change callback
};

// GAP Bond Manager Callbacks
static gapBondCBs_t multi_role_BondMgrCBs =
{
  (pfnPasscodeCB_t)multi_role_passcodeCB, // Passcode callback
   multi_role_pairStateCB                  // Pairing state callback
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
* @fn      multi_role_createTask
*
* @brief   Task creation function for the Simple BLE Peripheral.
*
* @param   None.
*
* @return  None.
*/
void multi_role_createTask(void)
{
  Task_Params taskParams;
  
  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = mrTaskStack;
  taskParams.stackSize = MR_TASK_STACK_SIZE;
  taskParams.priority = MR_TASK_PRIORITY;
  
  Task_construct(&mrTask, multi_role_taskFxn, &taskParams, NULL);
}

/*********************************************************************
* @fn      multi_role_init
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
static void multi_role_init(void)
{  
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);
  
  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);
  appUARTMsgQueue = Util_constructQueue(&appUARTMsg);
  
  Util_constructClock(&timeperiodicClock, multi_role_clockHandler,
                      SBP_PERIODIC_TIMER_EVT_PERIOD, SBP_PERIODIC_TIMER_EVT_PERIOD, true, SBP_TIMER_PERIODIC_EVT);
  Util_constructClock(&timeuploadperiodicClock, multi_role_clockHandler,
                      SBP_PERIODIC_UPLOAD_EVT_PERIOD, 0, false, SBP_UPLOAD_EVT);
  //init keys and LCD
  Board_initKeys(multi_role_keyChangeHandler);
  
  // Setup the GAP
  {
    /*-------------------PERIPHERAL-------------------*/
    // set advertising interval the same for all scenarios
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);    
    GAP_SetParamValue(TGAP_CONN_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_CONN_ADV_INT_MAX, advInt);    
    /*-------------------CENTRAL-------------------*/
    // set scan duration
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
    // scan interval and window the same for all scenarios
    GAP_SetParamValue(TGAP_CONN_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_HIGH_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_GEN_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_LIM_DISC_SCAN_WIND, DEFAULT_SCAN_WIND);
    GAP_SetParamValue(TGAP_CONN_EST_SCAN_INT, DEFAULT_SCAN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_SCAN_WIND, DEFAULT_SCAN_WIND);
    // set connection parameters
    GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, DEFAULT_CONN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, DEFAULT_CONN_INT);
    GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, DEFAULT_CONN_TIMEOUT);
    GAP_SetParamValue(TGAP_CONN_EST_LATENCY, DEFAULT_CONN_LATENCY);
    
    //register to receive GAP and HCI messages
    GAP_RegisterForMsgs(selfEntity);
  }
  
  // Setup the GAP Role Profile
  {
    /*--------PERIPHERAL-------------*/
    uint8_t initialAdvertEnable = TRUE;
    uint16_t advertOffTime = 0;
    //    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    //    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    //    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    //    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
    
    // device starts advertising upon initialization
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable, NULL);
    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime, NULL);
    // set scan response data
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, 16,
                         scanRspData, NULL);
    // set advertising data
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData, NULL);
    // set connection parameters
    //    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
    //                         &desiredMinInterval, NULL);
    //    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
    //                         &desiredMaxInterval, NULL);
    //    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
    //                         &desiredSlaveLatency, NULL);
    //    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
    //                         &desiredConnTimeout, NULL);
    /*--------------CENTRAL-----------------*/
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;
    //set the max amount of scan responses
    GAPRole_SetParameter(GAPROLE_MAX_SCAN_RES, sizeof(uint8_t), 
                         &scanRes, NULL);     
    
    // Start the GAPRole and negotiate max number of connections
    VOID GAPRole_StartDevice(&multi_role_gapRoleCBs, &maxNumBleConns);
    
    //allocate memory for index to connection handle map
    if (connHandleMap = ICall_malloc(sizeof(uint16_t) * maxNumBleConns))
    {
      // init index to connection handle map to 0's
      for (uint8_t i = 0; i < maxNumBleConns; i++)
      {
        connHandleMap[i] = INVALID_CONNHANDLE;
      }  
    }
    
    //allocate memory for per connection discovery information
    if (discInfo = ICall_malloc(sizeof(discInfo_t) * maxNumBleConns))
    {
      // init index to connection handle map to 0's
      for (uint8_t i = 0; i < maxNumBleConns; i++)
      {
        discInfo[i].charHdl = 0;
        discInfo[i].discState = BLE_DISC_STATE_IDLE;
        discInfo[i].svcEndHdl = 0;
        discInfo[i].svcStartHdl = 0;
      }  
    }
  }
  
  //GATT
  {
    /*---------------------SERVER------------------------*/
    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, 14, attDeviceName);
    
    // Initialize GATT Server Services
    GGS_AddService(GATT_ALL_SERVICES);           // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
    DevInfo_AddService();                        // Device Information Service
    SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
    
    // Setup Profile Characteristic Values
    {
      uint8_t charValue1 = 1;
      //      uint8_t charValue2 = 2;
      //      uint8_t charValue3 = 3;
      uint8_t charValue4 = 4;
      //      uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
      
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
                                 &charValue1);
      //      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
      //                                 &charValue2);
      //      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
      //                                 &charValue3);
      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                                 &charValue4);
      //      SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
      //                                 charValue5);
    }
    
    // Register callback with SimpleGATTprofile
    SimpleProfile_RegisterAppCBs(&multi_role_simpleProfileCBs);
    
    /*-----------------CLIENT------------------*/
    // Initialize GATT Client
    VOID GATT_InitClient();
    
    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);
    
    // Register to receive incoming ATT Indications/Notifications
    GATT_RegisterForInd(selfEntity);    
  }
  
  // Setup the GAP Bond Manager
  //  {
  //    uint8_t pairMode = GAPBOND_PAIRING_MODE_INITIATE;
  //    uint8_t mitm = TRUE;
  //    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
  //    uint8_t bonding = TRUE;
  //    
  //    // set pairing mode
  //    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
  //    // set authentication requirements
  //    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
  //    // set i/o capabilities
  //    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
  //    // set bonding requirements
  //    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  //    
  //    // Register and start Bond Manager
  //    VOID GAPBondMgr_Register(&multi_role_BondMgrCBs);  
  //  }  
  
  
  
  SDITask_registerIncomingRXEventAppCB(multi_enqueueUARTMsg);
  HwGPIOInit();
  uint8_t v = osal_snv_read(snvBleID, sizeof(snvData),&snvData);
  ble_name_Update();                   //修改蓝牙名称

  if(snvData.wflashCount == 0)
    snvData.wflashCount = 512;         //从第二页开始写入数据
  
  if(snvData.eraseCount == 0)
    snvData.eraseCount = 2;             //第二页写完后，从第三页开始清除数据
  
  alarmPeriod = (snvData.timeInterval * 1000) / 200;  //定时周期转换，200:200毫秒定时
  if(alarmPeriod == 0)
  {
    alarmPeriod = 5;
  }
  //HwFlashErase(0);                //清除flash的第一页存储的数据
  //HwFlashErase(4096);             //清楚flash的第二页存储的数据
}

/*********************************************************************
* @fn      multi_role_taskFxn
*
* @brief   Application task entry point for the multi_role.
*
* @param   a0, a1 - not used.
*
* @return  None.
*/
static void multi_role_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  multi_role_init();
  
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
            if (pEvt->event_flag & MR_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              multi_role_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = multi_role_processStackMsg((ICall_Hdr *)pMsg);
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
        mrEvt_t *pMsg = (mrEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          multi_role_processAppMsg(pMsg);
          
          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
      
      //如果串口RX接收数据队列不为空，处理串口数据 
      while (!Queue_empty(appUARTMsgQueue))                
      {
        //Get the message at the front of the queue but still keep it in the queue
        queueRec_t *pRec = Queue_head(appUARTMsgQueue);
        multiUARTEvt_t *pMsg = (multiUARTEvt_t *)pRec->pData;
        if (pMsg)
        {
          bStatus_t retVal = FAILURE;
          
          switch(pMsg->event)
          {
          case MULTI_UART_DATA_EVT:
            {
              volMeasuinterval++;
              volRead();                  //电量检测
              link_and_read_ledFlicker(); //手机连接上主从设备时灯的闪烁
              if(!readFlag && !calibrateFlag)
              { 
                //将获取的串口数据转换成欧拉角
                retVal = LMPS_getPacket(pMsg->length,pMsg->pData,eulerData_1);
                LMPS_getPacket(27, sensordev2Data, eulerData_2);
                //获取步态夹角
                EulerToMatrix(eulerData_1, T10);
                EulerToMatrix(eulerData_2, T20);
                inv(T10, T01);
                MatrixMul(T01, T20, T21);
                MatrixToProjective(T21, Projective_xyz21);
                //MatrixToEuler(T21, Delta_xyz21);
                // MatirxDelta(T10, T20, Delta_xyz21);
                if((retVal | retVal_1) != SUCCESS)
                {
                  //如果获取传感器数据失败，将不再监测
                  failureCount++;
                  if(failureCount > 50)
                  {
                    successFlag = 0;
                    stateFlag = 0;
                  }
                  
                }
                else
                {
                  //还没有结束，则将启动开始标志
                  if(endFlag == 0)
                  {
                    stateFlag = 1;
                  }
                  failureCount = 0;
                  retVal_1 = 1;
                  successFlag = 1;
                  
                  //人工校准
                  Projective_xyz21[1] = Projective_xyz21[1] - zeroValue;
                  
                  if(Projective_xyz21[1] < -180)
                  {
                    Projective_xyz21[1] = Projective_xyz21[1] + 360;
                  }
                  else if(Projective_xyz21[1] > 180)
                  {
                    Projective_xyz21[1] = Projective_xyz21[1] - 360;
                  }
                  //手机连接上主从设备
                  if(roleTrans == GAP_PROFILE_PERIPHERAL)
                  { 
                    
                    //将欧拉角数据拆成两个字节存储并上传
                    VOID memcpy(euler3Data, eulerData_1, 12);
                    VOID memcpy(&euler3Data[3], eulerData_2, 12);
                    VOID memcpy(&euler3Data[6], Projective_xyz21,12);
                    Apart_eulerData(euler3Data);
                    
                    Simple_Peripheral_NotiData(uploadEulerData,sizeof(uploadEulerData));
                  }
                } 
              }
              //传感器2校准
              if(calibrateFlag == 2)
              {
                calibrateFlag = 0;
                
                Simple_Central_WriteData(calibrateCommand,sizeof(calibrateCommand));
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
        
        scanTiming++;
        //按下结束时停止震动
        if(!stateFlag && alarmFlag)
        {
          HwGPIOSet(Board_MOTOR, 0);
          alarmFlag = 0;
        }
        //是否开始监测
        if(stateFlag)
        {
          storagePeriod++;
          multi_monitor();        //姿态监测
          //400ms存储一次
          if(storagePeriod == 2)
          {
            multi_storage_data();  //存储数据
            storagePeriod = 0;
          }
        }
        
        //1秒后开启扫描
        if(scanTiming == 5)
        {
          if(isLinkperipheral == false)
          {
            scanRes = 0;
            //start scanning
            GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                   DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                   DEFAULT_DISCOVERY_WHITE_LIST); 
          }
        }
      }
    }
    //姿势报警
    if(alarmTiming == alarmPeriod)
    {
      alarmTiming = 0;
      multi_alarm();
    }
    
    //上传Flash数据
    if(readFlag)
    {
      Util_startClock(&timeuploadperiodicClock);  //开启定时
      multi_role_uploadFlashdata();  
      delay_ms(16);
      if (events & SBP_UPLOAD_EVT)
      {
        events &= ~SBP_UPLOAD_EVT;
        { 
        }
      }
    }
    
    //6秒后进行蓝牙连接
    if(scanTiming == 35)
    {
      scanTiming = 0;
      if(isLinkperipheral == false)
      {
        GAPRole_CancelDiscovery();
        uint8_t addrType;
        
        
        for(int i = 0; i < scanRes; i++)
        {     
          if(memcmp(snvData.slaveAddress, devList[i].addr, 6) == 0)
          {
            addrType = ADDRTYPE_PUBLIC;
            GAPRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                  DEFAULT_LINK_WHITE_LIST,
                                  addrType, snvData.slaveAddress);
          }
        }
      }
    }
  }
}


/************************************************
* @fn    multi_role_monitor
*
* @brief monitor angle
*
* @param void
* 
* @return None
*/
static void multi_monitor(void)
{
  if(snvData.numericalRelation == 0x0B)
  {
    if(!(snvData.upperLimit | snvData.lowerLimit))
    {
      return;
    }
    else if(Projective_xyz21[1] > snvData.upperLimit || Projective_xyz21[1] < snvData.lowerLimit)
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
  //  else if( numericalRelation == 0x0C)
  //  {
  //    if(Projective_xyz21[1] > upperLimit || Projective_xyz21[1] < lowerLimit)
  //    {    
  //      alarmTiming++;
  //    }
  //    else
  //    {      
  //      alarmTiming = 0;
  //    }
  //  }
}

/***************************************************
* @fn     multi_role_alarm
*
* @brief  motor alarm
*
* @param void
*
* @return None
*/
static void multi_alarm(void)
{
  if(successFlag)
  {  
    //发送报警指令给从设备
    Simple_Central_WriteData(motorCommand,sizeof(motorCommand));
    //Motor start
    HwGPIOSet(Board_MOTOR, 1);
    alarmFlag = 1;
  }
}
/*********************************************************************
* @fn      multi_role_processStackMsg
*
* @brief   Process an incoming stack message.
*
* @param   pMsg - message to process
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;
  
  switch (pMsg->event)
  {
  case GATT_MSG_EVENT:
    // Process GATT message
    safeToDealloc = multi_role_processGATTMsg((gattMsgEvent_t *)pMsg);
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
    
  case GAP_MSG_EVENT:
    multi_role_processRoleEvent((gapMultiRoleEvent_t *)pMsg);
    break;        
    
  default:
    // do nothing
    break;
  }
  
  return (safeToDealloc);
}

/*********************************************************************
* @fn      multi_role_processGATTMsg
*
* @brief   Process GATT messages and events.
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   MR_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      multi_role_freeAttRsp(FAILURE);
      
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
    //Display_print1(dispHandle, 0, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }    
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    ////Display_print1(dispHandle, 0, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    //SDITask_PrintfToUART("\r\nMTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }
  
  //messages from GATT server
  if (linkDB_NumActive() > 0)
  {
    //find index from connection handle
    connIndex = multi_role_mapConnHandleToIndex(pMsg->connHandle);
    if ((pMsg->method == ATT_READ_RSP)   ||
        ((pMsg->method == ATT_ERROR_RSP) &&
         (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {      
        //Display_print1(dispHandle, 0, 0, "Read Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful read, display the read value
        //Display_print1(dispHandle, 0, 0, "Read rsp: %d", pMsg->msg.readRsp.pValue[0]);
      }
      
    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
      
      if (pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP)
      {     
        //Display_print1(dispHandle, 0, 0, "Write Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a succesful write, display the value that was written and
        // increment value
        //Display_print1(dispHandle, 0, 0, "Write sent: %d", charVal++);
      }
    }
    else if(pMsg->method == ATT_HANDLE_VALUE_NOTI)           // 主机接收从机notify数据
    {
      //      SDITask_sendToUART(pMsg->msg.handleValueNoti.pValue,
      //                      pMsg->msg.handleValueNoti.len);
      VOID memcpy(sensordev2Data, pMsg->msg.handleValueNoti.pValue,27);
      retVal_1 = 0;
      
    }
    else if (discInfo[connIndex].discState != BLE_DISC_STATE_IDLE)
    {
      multi_role_processGATTDiscEvent(pMsg);
    }
  } // else - in case a GATT message came after a connection has dropped, ignore it.  
  
  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
  
  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
* @fn      multi_role_sendAttRsp
*
* @brief   Send a pending ATT response message.
*
* @param   none
*
* @return  none
*/
static void multi_role_sendAttRsp(void)
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
      multi_role_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      //Display_print1(dispHandle, 0, 0, "Rsp send retry:", rspTxRetry);
    }
  }
}

/*********************************************************************
* @fn      multi_role_freeAttRsp
*
* @brief   Free ATT response message.
*
* @param   status - response transmit status
*
* @return  none
*/
static void multi_role_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      //Display_print1(dispHandle, 0, 0, "Rsp sent, retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
      
      //Display_print1(dispHandle, 0, 0, "Rsp retry failed: %d", rspTxRetry);
    }
    
    // Free response message
    ICall_freeMsg(pAttRsp);
    
    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
* @fn      multi_role_processAppMsg
*
* @brief   Process an incoming callback from a profile.
*
* @param   pMsg - message to process
*
* @return  None.
*/
static void multi_role_processAppMsg(mrEvt_t *pMsg)
{
  switch (pMsg->event)
  {
  case MR_STATE_CHANGE_EVT:
    multi_role_processStackMsg((ICall_Hdr *)pMsg->pData);
    // Free the stack message
    ICall_freeMsg(pMsg->pData);
    break;
    
  case MR_CHAR_CHANGE_EVT:
    multi_role_processCharValueChangeEvt(*(pMsg->pData));
    // Free the app data
    ICall_free(pMsg->pData);
    break;
    
  case MR_KEY_CHANGE_EVT:
    multi_role_handleKeys(*(pMsg->pData));
    // Free the app data
    ICall_free(pMsg->pData);
    break;
    
  case MR_PAIRING_STATE_EVT:
    multi_role_processPairState((gapPairStateEvent_t*)pMsg->pData);
    // Free the app data
    ICall_free(pMsg->pData);
    break;
    
  case MR_PASSCODE_NEEDED_EVT:
    multi_role_processPasscode((gapPasskeyNeededEvent_t*)pMsg->pData);
    // Free the app data
    ICall_free(pMsg->pData);
    break;    
    
  default:
    // Do nothing.
    break;
  }
}

/*********************************************************************
* @fn      multi_role_eventCB
*
* @brief   Central event callback function.
*
* @param   pEvent - pointer to event structure
*
* @return  TRUE if safe to deallocate event message, FALSE otherwise.
*/
static uint8_t multi_role_eventCB(gapMultiRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (multi_role_enqueueMsg(MR_STATE_CHANGE_EVT, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }
  
  // Caller should free the event
  return TRUE;
}

/*********************************************************************
* @fn      multi_role_processRoleEvent
*
* @brief   Multi role event processing function.
*
* @param   pEvent - pointer to event structure
*
* @return  none
*/
static void multi_role_processRoleEvent(gapMultiRoleEvent_t *pEvent)
{
  switch (pEvent->gap.opcode)
  {
    // GAPRole started
  case GAP_DEVICE_INIT_DONE_EVENT:  
    {
      //store max pdu size
      maxPduSize = pEvent->initDone.dataPktLen;
      
      //Display_print0(dispHandle, 0, 0, "Connected to 0");
      //Display_print0(dispHandle, 0, 0, Util_convertBdAddr2Str(pEvent->initDone.devAddr));
      //Display_print0(dispHandle, 0, 0, "Initialized");
      
      //set device info characteristic
      DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, pEvent->initDone.devAddr);    
    }
    break;
    
    // advertising started
  case GAP_MAKE_DISCOVERABLE_DONE_EVENT:
    {
      //Display_print0(dispHandle, 0, 0, "Advertising");
      // HwGPIOSet(Board_DK_LED3,1);
    }
    break;
    
    // advertising ended
  case GAP_END_DISCOVERABLE_DONE_EVENT:
    {
      // display advertising info depending on whether there are any connections
      if (linkDB_NumActive() < maxNumBleConns)
      {
        HwGPIOSet(Board_DK_LED3,1);
        //Display_print0(dispHandle, 0, 0, "Ready to Advertise");
      }
      else
      {
        HwGPIOSet(Board_DK_LED3,0);
        //Display_print0(dispHandle, 0, 0, "Can't Adv : No links");
      }
    }
    break;      
    
    // a discovered device report
  case GAP_DEVICE_INFO_EVENT:
    {
      // if filtering device discovery results based on service UUID
      if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
      {
        //if the simpleGATTprofile was found
        if (multi_role_findSvcUuid(SIMPLEPROFILE_SERV_UUID,
                                   pEvent->deviceInfo.pEvtData,
                                   pEvent->deviceInfo.dataLen))
        {
          //store this device
          multi_role_addDeviceInfo(pEvent->deviceInfo.addr,
                                   pEvent->deviceInfo.addrType);
        }
      }
    }
    break;
    
    // end of discovery report
  case GAP_DEVICE_DISCOVERY_EVENT:
    { 
      // if not filtering device discovery results based on service UUID
      if (DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE)
      {
        // Copy all of the results
        scanRes = pEvent->discCmpl.numDevs;
        memcpy(devList, pEvent->discCmpl.pDevList,
               (sizeof(gapDevRec_t) * scanRes));
      }
      
      //Display_print1(dispHandle, 0, 0, "Devices Found %d", scanRes);
      
    }
    break;
    
    // connection has been established
  case GAP_LINK_ESTABLISHED_EVENT:
    {
      // if succesfully established
      if (pEvent->gap.hdr.status == SUCCESS)
      {
        //Display_print0(dispHandle, 0, 0, "Connected!");
        //Display_print1(dispHandle, 0, 0, "Connected to %d", linkDB_NumActive());
        
        //add index-to-connHandle mapping entry
        multi_role_addMappingEntry(pEvent->linkCmpl.connectionHandle);
        
        // 连接后绿灯点亮
        // HwGPIOSet(Board_DK_LED3,1); 
        
        //判读连接的设备是作为主设备还是从设备
        if(pEvent->linkCmpl.connRole == GAP_PROFILE_CENTRAL)
        {
          roleTrans = GAP_PROFILE_CENTRAL;
        }
        else if(pEvent->linkCmpl.connRole == GAP_PROFILE_PERIPHERAL)
        {
          roleTrans = GAP_PROFILE_PERIPHERAL;
          
          HwGPIOSet(Board_DK_LED3,1); 
          isLedflicker = 0x01;
          ledFlickerFrequency = 0;
        }
        else
        {
          ;
        }
        
        //turn off advertising if no available links
        if (linkDB_NumActive() >= maxNumBleConns)
        {
          uint8_t advertEnabled = FALSE;
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertEnabled, NULL);
          // //Display_print0(dispHandle, 0, 0, "Can't adv: no links");
        }
        
        //        if(roleTrans == GAP_PROFILE_CENTRAL)
        //        {
        //          multi_role_startDiscovery(pEvent->linkCmpl.connectionHandle);   //更新与从设备和主设备的数据传送长度
        //        }
        
        //start service discovery
        multi_role_startDiscovery(pEvent->linkCmpl.connectionHandle);
        
      }
      // if the connection was not sucesfully established
      else
      {  
        //Display_print0(dispHandle, 0, 0, "Connect Failed");
        //Display_print1(dispHandle, 0, 0, "Reason: %d", pEvent->gap.hdr.status);
      }
    }
    break;
    
    // connection has been terminated
  case GAP_LINK_TERMINATED_EVENT:
    {
      //find index from connection handle
      connIndex = multi_role_mapConnHandleToIndex(pEvent->linkTerminate.connectionHandle);
      //check to prevent buffer overrun
      if (connIndex < maxNumBleConns)
      {      
        //clear screen, reset discovery info, and return to main menu
        connHandleMap[connIndex] = INVALID_CONNHANDLE;
        discInfo[connIndex].charHdl = 0;
        
        HwGPIOSet(Board_DK_LED3,0);
        isLedflicker = 0;
        // if it is possible to advertise again
        if (linkDB_NumActive() == (maxNumBleConns-1)) 
        {
          //Display_print0(dispHandle, 0, 0, "Ready to Advertise");
          //Display_print0(dispHandle, 0, 0, "Ready to Scan");
        }      
        //reset discovery state
        discInfo[connIndex].discState= BLE_DISC_STATE_IDLE;
      }
      
      uint8_t advertEnabled = TRUE;
      GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &advertEnabled, NULL);
      //如果索引为0，则灯不再闪烁
      if(connIndex == 0)
      {
        isLinkperipheral = false;
      }
      
      roleTrans = 0;
    }
    break;
    
    //a parameter update has occurred
  case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      //Display_print1(dispHandle, 0, 0, "Param Update %d", pEvent->linkUpdate.status);
    }
    break;
    
  default:
    break;
  }
}

/*********************************************************************
* @fn      multi_role_charValueChangeCB
*
* @brief   Callback from Simple Profile indicating a characteristic
*          value change.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void multi_role_charValueChangeCB(uint8_t paramID)
{
  uint8_t *pData;
  
  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {    
    *pData = paramID;  
    
    // Queue the event.
    multi_role_enqueueMsg(MR_CHAR_CHANGE_EVT, pData);
  }
}

/*********************************************************************
* @fn      multi_role_processCharValueChangeEvt
*
* @brief   Process a pending Simple Profile characteristic value change
*          event.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void multi_role_processCharValueChangeEvt(uint8_t paramID)
{
  uint8_t bleBuf[75];
  
  for(uint8_t i = 0; i < 2; i++)
  {
    HwGPIOSet(Board_DK_LED3,0);
    delay_ms(100);
    HwGPIOSet(Board_DK_LED3,1);
    delay_ms(100);
  }
  switch(paramID)
  {
  case SIMPLEPROFILE_CHAR1:
    
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, bleBuf);
    //SDITask_sendToUART(bleBuf,sizeof(bleBuf));
    //获取软件端所设置的参数
    if(bleBuf[1] == 0x3D && bleBuf[2] == 0x01&& bleBuf[3] == 0x02 && bleBuf[11] == 0x0D && bleBuf[12] == 0x0A)
    {
      
       snvData.upperLimit = bleBuf[5] | (bleBuf[4] << 8); //Y轴上限
       snvData.lowerLimit = (bleBuf[7] | (bleBuf[6] << 8)); //Y轴下限
       snvData.timeInterval = bleBuf[9] | (bleBuf[8] << 8); //时间间隔
       snvData.numericalRelation = bleBuf[10];                    //大于(小于)
      //Redefine time
      alarmPeriod = (snvData.timeInterval * 1000) / 200;
      alarmTiming = 0;
      
      osal_snv_write(snvBleID, sizeof(snvData), &snvData);
    }
    //获取开始与结束指令
    else if(bleBuf[1] == 0x3C && bleBuf[2] == 0x01)
    {
      stateFlag = bleBuf[3];
      readFlag = 0;
      if(stateFlag == 0)
      {      
        endFlag = 1;
      }
      else if(stateFlag == 1)
      {
        //存储开始地址
        time_address[0] = (uint8_t)((TXBUF_LENGTH * snvData.wflashCount) & 0xFF);
        time_address[1] = (uint8_t)(((TXBUF_LENGTH * snvData.wflashCount) >> 8) & 0xFF);
        time_address[2] = (uint8_t)(((TXBUF_LENGTH * snvData.wflashCount) >> 16) & 0xFF);
        //存储开始时间
        for(uint8_t i = 3; i < 11; i ++)
        {
          time_address[i] = bleBuf[1 + i];
        }
        HwFlashWrite(time_address_LENGTH * snvData.startCount, time_address, time_address_LENGTH);
        //存储开始次数
        snvData.startCount++;
        osal_snv_write(snvBleID, sizeof(snvData), &snvData);
        endFlag = 0;
      }
      //SDITask_PrintfToUART("%d\r\n",stateFlag);
    }
    //获取读取指令
    else if(bleBuf[1] == 0x3B && bleBuf[2] == 0x01 && bleBuf[4] == 0x0D && bleBuf[5] == 0x0A)
    {
      if(snvData.startCount == 0)
      {
        readFlag = 0;
        snvData.startCount = 1;
        multi_role_sendStartFrame();
        multi_role_sendEndFrame();
        snvData.startCount = 0;
        return;
      }
      
      osal_snv_read(snvBleID, sizeof(snvData),&snvData);
     // startCount = snvData[4];
      readFlag = bleBuf[3];
      isUpload = 1;
      stateFlag = 0;
      isLedflicker = 0x02;
      ledFlickerFrequency = 0;
    }
    else if(bleBuf[1] == 0x3E)
    {
      if(bleBuf[2] == 0x01)
      {
        //清零指令
        if(bleBuf[3] == 0x03)
        {
          zeroValue = Projective_xyz21[1] + zeroValue;
        }
        //校准指令
        else 
        {
          calibrateFlag = bleBuf[3];
        }
      }
      //修改名称指令
      else if(bleBuf[2] == 0x06)
      {
        
        if(bleBuf[0] == 4 || bleBuf[0] == 0)
        {
          return;
        }
        else 
        {
          snvData.bleNameLength = bleBuf[0] - 4;
          memcpy(&(snvData.bleName), &bleBuf[3], snvData.bleNameLength);
          osal_snv_write(snvBleID, sizeof(snvData), &snvData);
          ble_name_Update();  
        }
        
      }
      //设置从机地址指令
      else if(bleBuf[2] == 0x05)
      {
        if(bleBuf[0] == 4 || bleBuf[0] == 0)
        {
          return;
        }
        else
        {
          for(int i = 0; i < 6; i++)
          {
           snvData.slaveAddress[i] = bleBuf[8 - i];
          }
          osal_snv_write(snvBleID, sizeof(snvData), &snvData);
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
* @fn      SimpleBLEPeripheral_clockHandler
*

* @par* @brief   Handler function for clock timeouts.
*am   arg - event type
*
* @return  None.
*/
static void multi_role_clockHandler(UArg arg)
{
  //Store the event 
  events |= arg;
  //Wake up the application
  Semaphore_post(sem);
}
/*********************************************************************
* @fn      multi_role_enqueueMsg
*
* @brief   Creates a message and puts the message in RTOS queue.
*
* @param   event - message event.
* @param   pData - pointer to data to be queued
*
* @return  None.
*/
static uint8_t multi_role_enqueueMsg(uint16_t event, uint8_t *pData)
{
  // allocate space for the message
  mrEvt_t *pMsg = ICall_malloc(sizeof(mrEvt_t));
  
  // if sucessfully allocated
  if (pMsg)
  {
    // fill up message
    pMsg->event = event;
    pMsg->pData = pData;
    
    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
  
  return FALSE;
}

/*********************************************************************
* @fn      multi_role_keyChangeHandler
*
* @brief   Key event handler function
*
* @param   a0 - ignored
*
* @return  none
*/
void multi_role_keyChangeHandler(uint8 keys)
{
  uint8_t *pData;
  
  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {    
    // store the key data
    *pData = keys;  
    
    // Queue the event.
    multi_role_enqueueMsg(MR_KEY_CHANGE_EVT, pData);
  }
}

/*********************************************************************
* @fn      multi_role_handleKeys
*
* @brief   Handles all key events for this device.
*
* @param   keys - bit field for key events. Valid entries:
*                 HAL_KEY_SW_2
*                 HAL_KEY_SW_1
*
* @return  none
*/
static void multi_role_handleKeys(uint8_t keys)
{
  if (keys & KEY_BTN1)
  {
    //    scanRes = 0;
    //    
    //    //SDITask_PrintfToUART("%s\r\n", "Discovering...");
    //    
    //    GAPRole_CancelDiscovery();
    //    //start scanning
    //    GAPRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
    //                           DEFAULT_DISCOVERY_ACTIVE_SCAN,
    //                           DEFAULT_DISCOVERY_WHITE_LIST); 
    //    delay_ms(10);
    //    GAPRole_CancelDiscovery();
    //    uint8_t addrType;
    //    uint8_t peerAddr[6];
    //    
    //    // connect to hardcoded device address i.e. 0x050403020100
    //    
    //    for(int x = 0; x < 6; x++)
    //    {
    //      peerAddr[x] = x + 1;
    //    }
    //    
    //    addrType = ADDRTYPE_PUBLIC;
    //    GAPRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
    //                          DEFAULT_LINK_WHITE_LIST,
    //                          addrType, peerAddr);
  }
  return;
}

/*********************************************************************
* @fn      multi_role_startDiscovery
*
* @brief   Start service discovery.
*
* @return  none
*/
static void multi_role_startDiscovery(uint16_t connHandle)
{
  //exchange MTU request
  attExchangeMTUReq_t req;
  
  //map connection handle to index
  connIndex = multi_role_mapConnHandleToIndex(connHandle);
  
  //check to prevent buffer overrun
  if (connIndex < maxNumBleConns)
  {
    //update discovery state of this connection
    discInfo[connIndex].discState= BLE_DISC_STATE_MTU;
    // Initialize cached handles
    discInfo[connIndex].svcStartHdl = discInfo[connIndex].svcEndHdl = 0;
  }
  
  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;
  
  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(connHandle, &req, selfEntity);
}

/*********************************************************************
* @fn      multi_role_processGATTDiscEvent
*
* @brief   Process GATT discovery event
*
* @return  none
*/
static void multi_role_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{ 
  //map connection handle to index
  connIndex = multi_role_mapConnHandleToIndex(pMsg->connHandle);
  //check to prevent buffer overrun
  if (connIndex < maxNumBleConns)
  {
    //MTU update
    if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {   
      // MTU size updated
      //Display_print1(dispHandle, 0, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }
    //if we've updated the MTU size
    else if (discInfo[connIndex].discState== BLE_DISC_STATE_MTU)
    {
      // MTU size response received, discover simple BLE service
      if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
      {
        uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
        HI_UINT16(SIMPLEPROFILE_SERV_UUID) };        
        //advanec state
        discInfo[connIndex].discState= BLE_DISC_STATE_SVC;
        
        // Discovery simple BLE service
        VOID GATT_DiscPrimaryServiceByUUID(pMsg->connHandle, uuid, ATT_BT_UUID_SIZE,
                                           selfEntity);
      }
    }
    //if we're performing service discovery
    else if (discInfo[connIndex].discState== BLE_DISC_STATE_SVC)
    {
      // Service found, store handles
      if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
          pMsg->msg.findByTypeValueRsp.numInfo > 0)
      {
        discInfo[connIndex].svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        discInfo[connIndex].svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      }
      
      // If procedure complete
      if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) && 
           (pMsg->hdr.status == bleProcedureComplete))  ||
          (pMsg->method == ATT_ERROR_RSP))
      {
        //if we've discovered the service
        if (discInfo[connIndex].svcStartHdl != 0)
        {
          attReadByTypeReq_t req;
          
          // Discover characteristic
          discInfo[connIndex].discState= BLE_DISC_STATE_CHAR;
          req.startHandle = discInfo[connIndex].svcStartHdl;
          req.endHandle = discInfo[connIndex].svcEndHdl;
          req.type.len = ATT_BT_UUID_SIZE;
          req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
          req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);
          
          //send characteristic discovery request
          VOID GATT_ReadUsingCharUUID(pMsg->connHandle, &req, selfEntity);
        }
      }
    }
    //if we're discovering characteristics
    else if (discInfo[connIndex].discState == BLE_DISC_STATE_CHAR)
    {
      // Characteristic found
      if ((pMsg->method == ATT_READ_BY_TYPE_RSP) && 
          (pMsg->msg.readByTypeRsp.numPairs > 0))
      {
        //store handle
        discInfo[connIndex].charHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                                                   pMsg->msg.readByTypeRsp.pDataList[1]);
        
        //Display_print0(dispHandle, 0, 0, "Simple Svc Found");
        HwGPIOSet(Board_DK_LED3, 0);
        isLinkperipheral = true;
        
      }
    }    
  }
}

/*********************************************************************
* @fn      multi_role_findSvcUuid
*
* @brief   Find a given UUID in an advertiser's service UUID list.
*
* @return  TRUE if service UUID found
*/
static bool multi_role_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                   uint8_t dataLen)
{
  uint8_t adLen;
  uint8_t adType;
  uint8_t *pEnd;
  
  //find the end of data
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while (pData < pEnd)
  {
    // Get length of next AD item
    adLen = *pData++;
    if (adLen > 0)
    {
      adType = *pData;
      
      // If AD type is for 16-bit service UUID
      if ((adType == GAP_ADTYPE_16BIT_MORE) || 
          (adType == GAP_ADTYPE_16BIT_COMPLETE))
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while (adLen >= 2 && pData < pEnd)
        {
          // Check for match
          if ((pData[0] == LO_UINT16(uuid)) && (pData[1] == HI_UINT16(uuid)))
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if (adLen == 1)
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }
  
  // Match not found
  return FALSE;
}

/*********************************************************************
* @fn      multi_role_addDeviceInfo
*
* @brief   Add a device to the device discovery result list
*
* @return  none
*/
static void multi_role_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
{
  uint8_t i;
  
  // If result count not at max
  if (scanRes < DEFAULT_MAX_SCAN_RES)
  {
    // Check if device is already in scan results
    for (i = 0; i < scanRes; i++)
    {
      if (memcmp(pAddr, devList[i].addr , B_ADDR_LEN) == 0)
      {
        return;
      }
    }
    
    // Add addr to scan result list
    memcpy(devList[scanRes].addr, pAddr, B_ADDR_LEN);
    devList[scanRes].addrType = addrType;
    
    //SDITask_PrintfToUART( "%d,%s\r\n", scanRes, Util_convertBdAddr2Str(devList[i].addr));
    
    // Increment scan result count
    scanRes++;
  }
}

/*********************************************************************
* @fn      gapRoleFindIndex
*
* @brief   to translate connection handle to index
*
* @param   connHandle (connection handle)
*
* @return  none
*/
static uint16_t multi_role_mapConnHandleToIndex(uint16_t connHandle)
{
  uint16_t index;
  //loop through connection
  for (index = 0; index < maxNumBleConns; index ++)
  {
    //if matching connection handle found
    if (connHandleMap[index] == connHandle)
    {
      return index;
    }
  }
  //not found if we got here
  return INVALID_CONNHANDLE;
}

/************************************************************************
* @fn      multi_role_pairStateCB
*
* @brief   Pairing state callback.
*
* @return  none
*/
static void multi_role_pairStateCB(uint16_t connHandle, uint8_t state,
                                   uint8_t status)
{
  gapPairStateEvent_t *pData;
  
  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(gapPairStateEvent_t))))
  {    
    pData->connectionHandle = connHandle;    
    pData->state = state;
    pData->status = status;
    
    // Enqueue the event.
    multi_role_enqueueMsg(MR_PAIRING_STATE_EVT, (uint8_t *) pData);
  }
}

/*********************************************************************
* @fn      multi_role_passcodeCB
*
* @brief   Passcode callback.
*
* @return  none
*/
static void multi_role_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison)
{
  gapPasskeyNeededEvent_t *pData;
  
  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(gapPasskeyNeededEvent_t))))
  {
    memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
    pData->connectionHandle = connHandle;    
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;
    
    // Enqueue the event.
    multi_role_enqueueMsg(MR_PASSCODE_NEEDED_EVT, (uint8_t *) pData);
  }
}

/*********************************************************************
* @fn      multi_role_processPairState
*
* @brief   Process the new paring state.
*
* @return  none
*/
static void multi_role_processPairState(gapPairStateEvent_t* pairingEvent)
{
  //if we've started pairing
  if (pairingEvent->state == GAPBOND_PAIRING_STATE_STARTED)
  {
    //Display_print1(dispHandle, 0, 0,"Cxn %d pairing started", pairingEvent->connectionHandle);
  }
  //if pairing is finished
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (pairingEvent->status == SUCCESS)
    {
      //Display_print1(dispHandle, 0, 0,"Cxn %d pairing success", pairingEvent->connectionHandle);
    }
    else
    {
      //Display_print2(dispHandle, 0, 0, "Cxn %d pairing fail: %d", pairingEvent->connectionHandle, pairingEvent->status);
    }
  }
  //if a bond has happened
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (pairingEvent->status == SUCCESS)
    {
      //Display_print1(dispHandle, 0, 0, "Cxn %d bonding success", pairingEvent->connectionHandle);
    }
  }
  //if a bond has been saved
  else if (pairingEvent->state == GAPBOND_PAIRING_STATE_BOND_SAVED)
  {
    if (pairingEvent->status == SUCCESS)
    {
      //Display_print1(dispHandle, 0, 0, "Cxn %d bond save success", pairingEvent->connectionHandle);
    }
    else
    {
      //Display_print2(dispHandle, 0, 0, "Cxn %d bond save failed: %d", pairingEvent->connectionHandle, pairingEvent->status);
    }
  }
}

/*********************************************************************
* @fn      multi_role_processPasscode
*
* @brief   Process the Passcode request.
*
* @return  none
*/
static void multi_role_processPasscode(gapPasskeyNeededEvent_t *pData)
{
  //use static passcode
  uint32_t passcode = 123456;
  //Display_print1(dispHandle, 0, 0, "Passcode: %d", passcode);
  //send passcode to GAPBondMgr
  GAPBondMgr_PasscodeRsp(pData->connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
* @fn      gapRoleFindIndex
*
* @brief   to translate connection handle to index
*
* @param   connHandle (connection handle)
*
* @return  none
*/
static uint8_t multi_role_addMappingEntry(uint16_t connHandle)
{
  uint16_t index;
  // loop though connections
  for (index = 0; index < maxNumBleConns; index ++)
  {
    //if there is an open connection
    if (connHandleMap[index] == INVALID_CONNHANDLE)
    {
      //store mapping
      connHandleMap[index] = connHandle;
      return index;
    }
  }
  //no room if we get here
  return bleNoResources;
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
  
  noti.pValue = (uint8 *)GATT_bm_alloc(1, ATT_HANDLE_VALUE_NOTI, 100, NULL);
  if ( noti.pValue != NULL )
  {
    memcpy(noti.pValue, buf, len);
    status = GATT_Notification(1, &noti, FALSE);
    if(status != SUCCESS)
    {
      GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
    }
  }
}

/*********************************************************************
* @fn      Simple_Central_WriteData
*
* @brief   .
*
* @param   buf - .
* @param   len - .
*
* @return  none.
*/
void Simple_Central_WriteData(uint8_t *buf,uint8_t len)
{
  bStatus_t status;
  attWriteReq_t req; 
  req.handle = 0x1E;
  req.len = len>80 ? 80 : len;
  req.sig = 0;
  req.cmd = TRUE;
  
  req.pValue = GATT_bm_alloc(0, ATT_WRITE_REQ, 100, NULL);
  
  if ( req.pValue != NULL )
  {
    memcpy(req.pValue,buf,len);
    status = GATT_WriteNoRsp(0, &req);
    if ( status != SUCCESS )
    {
      GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
    }
  }
}

static void multi_enqueueUARTMsg(uint8_t event, uint8_t *data, uint8_t len)
{
  multiUARTEvt_t *pMsg;
  //  if((gapProfileState == GAPROLE_CONNECTED) || (gapProfileState == GAPROLE_CONNECTED_ADV))
  //  {
  //Enqueue message only in a connected state
  if(pMsg = ICall_malloc(sizeof(multiUARTEvt_t)))
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
  
  VOID memcpy(sensordev1Data, pData, len);
  ret = memcmp(serialCompare, sensordev1Data, sizeof(serialCompare));
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
    int2float(&eulerData[i], &sensordev1Data[len]);
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
  int j = 0, z =2;
  for(int i = 0; i < 3; i++)
  {
    temp_x = eulerData[j] * 10;
    temp_y = eulerData[j + 1] * 10;
    temp_z = eulerData[j + 2] * 10;
    
    uploadEulerData[z] = (int8_t)(temp_x & 0x7F);
    uploadEulerData[z + 1] = (int8_t)((temp_x >> 7) & 0xFF);
    uploadEulerData[z + 2] = (int8_t)(temp_y & 0x7F);
    uploadEulerData[z + 3] = (int8_t)((temp_y >> 7) & 0xFF);
    uploadEulerData[z + 4] = (int8_t)(temp_z & 0x7F);
    uploadEulerData[z + 5] = (int8_t)((temp_z >> 7) & 0xFF);
    z = z + 6;
    j = j+ 3;
  }
  uploadEulerData[20] = 0x0D;
  uploadEulerData[21] = 0x0A;
  
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

static void multi_storage_data(void)
{
  int16_t tempx ;
  
  
  tempx = Projective_xyz21[1] * 10;    
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
    osal_snv_write(snvBleID,sizeof(snvData),&snvData);
  }
  //每4096个字节擦除一次
  if(pageErase == 512)
  {
    HwFlashErase(4096 * snvData.eraseCount);
    snvData.eraseCount++;
    pageErase = 0;
    if(snvData.eraseCount == 2048)
    {
      snvData.wflashCount = 512;
      HwFlashErase(4096);
      snvData.eraseCount = 2;
    }
    // SDITask_PrintfToUART("\r\eraseCount =%d=",eraseCount);
  }
}


/**********************************************************
* @fn multi_role_uploadFlashdata
*
* @brief upload the falsh data
*
* @param falsh data
*
* @return None
*/

static void multi_role_uploadFlashdata(void)
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
      uploadCount = (behindAddress - frontAddress) / RXBUF_LENGTH;//算出没有超出flash时要提取flash的次数
    }
    
    multi_role_sendStartFrame();// 发送开始标志
    
  }
  
  
  if(uploadCount!=0)
  {
    multi_role_uploadCurveData();
    //SDITask_PrintfToUART("\r\n uploadcount = %d ",uploadCount);  
    
  }
  else if(uploadCount == 0 )
  {
    
    //发送完成的结束标志
    multi_role_sendEndFrame();
    if(snvData.startCount == 1)
    {
      readFlag = 0;
      snvData.startCount = 0;
      snvData.eraseCount = 2;
      read_count = 0;
      snvData.wflashCount = 512;
      extractflashCount = 256;
      //更新snv保存的数据
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
static void multi_role_sendStartFrame(void)
{
  uploadFlashdata [0] = 0x3B;
  uploadFlashdata [1] = 0x01;
  uploadFlashdata [2] = (int8_t)(3610 & 0x7F);
  uploadFlashdata [3] = (int8_t)((3610 >> 7) & 0xFF);
  for(uint8_t i = 0; i < 8; i++)
  {
    uploadFlashdata[4 + i] = time_address[(3 + i)];
  }
  uploadFlashdata[12] = snvData.startCount;
  uploadFlashdata[13] = 400 & 0xff;
  uploadFlashdata[14] = (400 >> 8) & 0xff;
  uploadFlashdata [18] = 0x0D;
  uploadFlashdata [19] = 0x0A;
  Simple_Peripheral_NotiData(uploadFlashdata, UPLOADFLASHDATA_LENGTH);
  isUpload = 0;
}
//发送完成的结束标志
static void multi_role_sendEndFrame(void)
{
  uploadFlashdata [0] = 0x3B;
  uploadFlashdata [1] = 0x01;
  uploadFlashdata [2] = 0;
  uploadFlashdata [3] = 0;
  uploadFlashdata [16] = (int8_t)(3610 & 0x7F);
  uploadFlashdata [17] = (int8_t)((3610 >> 7) & 0xFF);
  uploadFlashdata [18] = 0x0D;
  uploadFlashdata [19] = 0x0A;
  Simple_Peripheral_NotiData(uploadFlashdata, UPLOADFLASHDATA_LENGTH);
}

static void multi_role_uploadCurveData(void)
{
  //每一次提取RXBUF_LENGTH个字节
  HwFlashRead(RXBUF_LENGTH * extractflashCount, rxbuf, RXBUF_LENGTH);
  
  uploadFlashdata [0] = 0x3B;
  uploadFlashdata [1] = 0x01;
  for(uint16_t i = 2, j = 0; i < 18; i++,j++)
  {
    uploadFlashdata [i] = rxbuf [j];
    i++;
    j++;
    uploadFlashdata [i] = rxbuf [j];
  }    
  uploadFlashdata [18] = 0x0D;
  uploadFlashdata [19] = 0x0A;
  
  Simple_Peripheral_NotiData(uploadFlashdata, UPLOADFLASHDATA_LENGTH);
  uploadCount--;
  extractflashCount++;
}
/*********************************************************************
* @fn      multi_role_paramUpdateDecisionCB
*
* @brief   Callback for application to decide whether or not to accept
*          a parameter update request and, if accepted, what parameters
*          to use
*
* @param   pReq - pointer to param update request
* @param   pRsp - pointer to param update response
*
* @return  none
*/
static void multi_role_paramUpdateDecisionCB(gapUpdateLinkParamReq_t *pReq,
                                             gapUpdateLinkParamReqReply_t *pRsp)
{
  // Make some decision based on desired parameters. Here is an example
  // where only parameter update requests with 0 slave latency are accepted
  if (pReq->connLatency == 0)
  {
    // Accept and respond with remote's desired parameters
    pRsp->accepted = TRUE;
    pRsp->connLatency = pReq->connLatency;
    pRsp->connTimeout = 300;
    pRsp->intervalMax = 40;
    pRsp->intervalMin = 40;
    
  }
  
  // Don't accept param update requests with slave latency other than 0
  else
  {
    pRsp->accepted = FALSE;
  }
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

//电量检测
static void volRead(void)
{
  //电量检测
  if(volMeasuinterval == 600)
  {
    float batt;
    volMeasuinterval = 0;
    ADC_Read();
    batt = 4.3 * ADCVal / 4095  * 2;
    //SDITask_PrintfToUART("\r\n %.2fV",batt);
    if(batt < VOLTAGE_THRESHOLD)
    {
      powerAlarm = 1;
    }
    else
    {
      powerAlarm = 0;
      HwGPIOSet(Board_DK_LED3, 0);
    }
  }
  
  //低电量报警
  if(powerAlarm == 1)
  {
    powerAlarmtime++;
    if(powerAlarmtime == 3)
    {
      HwGPIOSet(Board_DK_LED3, 1);
    }
    else if(powerAlarmtime == 5)
    {
      powerAlarmtime = 1;
      HwGPIOSet(Board_DK_LED3, 0);
    }
  }
}

/* 蓝牙名称更新 */
static void ble_name_Update(void)
{
  //如果没有获取到数据，则返回(指令包括帧头和帧尾共四个字节)
  if(snvData.bleNameLength == 0)
  {
    return;
  }
  
  scanRspData[0] = snvData.bleNameLength + 1;         //扫描响应的第一个字节存储的是从机名的长度+1，这个1是GAP_ADTYPE_LOCAL_NAME_COMPLETE
  scanRspData[1] = GAP_ADTYPE_LOCAL_NAME_COMPLETE;         
  memcpy(&scanRspData[2], &(snvData.bleName), snvData.bleNameLength);
  memcpy(attDeviceName, &(snvData.bleName), snvData.bleNameLength);
  GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, snvData.bleNameLength + 2,
                         scanRspData, NULL);
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, snvData.bleNameLength, attDeviceName);
}
