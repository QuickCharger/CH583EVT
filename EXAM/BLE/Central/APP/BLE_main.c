/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2020/08/06
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "BLE_main.h"
#include "CONFIG.h"
#include "hal.h"
#include "central.h"
#include "HID_Main.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
__attribute__((aligned(4))) uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];

/*********************************************************************
 * @fn      Main_Circulation
 *
 * @brief   主循环
 *
 * @return  none
 */
// __HIGH_CODE
// __attribute__((noinline))
// void Main_Circulation()
// {
//     while(1)
//     {
//         TMOS_SystemProcess();
//     }
// }

/*********************************************************************
 * @fn      main
 *
 * @brief   主函数
 *
 * @return  none
 */
int BLE_main(void)
{
    PRINT("%s\n", VER_LIB);
    CH58X_BLEInit();
    HAL_Init();
    GAPRole_CentralInit();
    Central_Init();
}

/******************************** endfile @ main ******************************/





/********************************** (C) COPYRIGHT *******************************
* File Name          : central.c
* Author             : WCH
* Version            : V1.1
* Date               : 2020/08/06
* Description        : 主机例程，主动扫描周围设备，连接至给定的从机设备地址，
*                      寻找自定义服务及特征，执行读写命令，需与从机例程配合使用,
                       并将从机设备地址修改为该例程目标地址，默认为(84:C2:E4:03:02:02)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "easy.h"
#include "CONFIG.h"
#include "gattprofile.h"
#include "central.h"

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                10

// Scan duration in 0.625ms
// 扫描时长 单位0.625ms
#define DEFAULT_SCAN_DURATION               2400

// Connection min interval in 1.25ms
#define DEFAULT_MIN_CONNECTION_INTERVAL     20

// Connection max interval in 1.25ms
#define DEFAULT_MAX_CONNECTION_INTERVAL     100

// Connection supervision timeout in 10ms
#define DEFAULT_CONNECTION_TIMEOUT          600

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE              DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN       TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST        FALSE

// TRUE to use high scan duty cycle when creating link
// 高占空比扫描，扫描时长更长，扫描间隔更短，扫描功耗更大
#define DEFAULT_LINK_HIGH_DUTY_CYCLE        FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST             FALSE

// Default read RSSI period in 0.625ms
#define DEFAULT_RSSI_PERIOD                 2400

// Minimum connection interval (units of 1.25ms)
// 连接间隔就是指在一个连接事件（ Connection events ）的开始到下一个连接事件（Connection events）的开始的时间间隔。连接间隔的范围是6 ~ 3200既7.5ms ~ 4s之间。单位1.25ms
// Connection Interval 缩短，Master和Slave通信更加频繁，提高数据吞吐速度，缩短了数据发送的时间，当然也增加了功耗。
// Connection Interval 增长，通信频率降低，数据吞吐速度降低，增加了数据发送的时间，当然，这种设置降低了功耗。
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL    80

// Maximum connection interval (units of 1.25ms)
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL    160

// Slave latency to use parameter update
// 允许Slave（从设备）在没有数据要发的情况下，跳过一定数目的连接事件。在这些连接事件（Connection events）中不必回复Master（主设备）的包，这样就能更加省电
// Slave Latency减少或者设置为 0，每次从机Connection Events中都需要回复Master的包，功耗会上升，数据发送速度会提高。
// Slave Latency加长，功耗下降，数据发送速度降低。
#define DEFAULT_UPDATE_SLAVE_LATENCY        4

// Supervision timeout value (units of 10ms)
// 超过此值 连接断开 单位10ms
// 此值应大于 （1 + slave Latency ）* （ connection Interval ）
#define DEFAULT_UPDATE_CONN_TIMEOUT         600

// Default passcode
#define DEFAULT_PASSCODE                    0

// Default GAP pairing mode
// 主机选 GAPBOND_PAIRING_MODE_INITIATE 或 GAPBOND_PAIRING_MODE_NO_PAIRING, 选择不配对不会出发配对结果回调
#define DEFAULT_PAIRING_MODE                GAPBOND_PAIRING_MODE_INITIATE

// Default MITM mode (TRUE to require passcode or OOB when pairing)
// 中间人保护 通常需要用户输入passkey或配对码
#define DEFAULT_MITM_MODE                   FALSE

// Default bonding mode, TRUE to bond, max bonding 6 devices
#define DEFAULT_BONDING_MODE                TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES             GAPBOND_IO_CAP_KEYBOARD_ONLY

// Default service discovery timer delay in 0.625ms
#define DEFAULT_SVC_DISCOVERY_DELAY         1600

// Default parameter update delay in 0.625ms
#define DEFAULT_PARAM_UPDATE_DELAY          3200

// Default read or write timer delay in 0.625ms
#define DEFAULT_READ_OR_WRITE_DELAY         1600

// Establish link timeout in 0.625ms
#define ESTABLISH_LINK_TIMEOUT              3200

// Application states
enum
{
	BLE_STATE_IDLE,
	BLE_STATE_CONNECTED,
};

enum EDeviceType
{
	eUnknown,
	eMouse,
	eKeyboard,
};

////////////////////////////////////////////// UUID BEGIN

// Discovery states 根据UUID查找结果
enum
{
	BLE_DISC_STATE_IDLE, // Idle
	BLE_DISC_STATE_ALL_SVC,  // 正在获取所有SVC
	BLE_DISC_STATE_SVC,  // 正在根据ServiceUUID查找handle。发送 GATT_DiscPrimaryServiceByUUID 后置此状态
	BLE_DISC_STATE_CHAR_ALL,  // 正在根据 handle 查找 charUUID。发送 GATT_DiscAllChars 后置此状态
	BLE_DISC_STATE_CHAR, // 正在根据CharacteristicUUID查找结果。发送 GATT_ReadUsingCharUUID 后置此状态
	BLE_DISC_STATE_CCCD  // 正在根据CCCD(client characteristic configuration discovery)查找结果。 发送 GATT_ReadUsingCharUUID 后置此状态。与上一个的不同之处在于发送的UUID不同
};

/**
 * GAP 的消息走 centralRoleCB centralBondCB 的回调
 * GATT的消息走 gattCentralMsg
 * 	发送消息、执行函数					回调消息
 * 	GATT_DiscAllPrimaryServices		ATT_READ_BY_GRP_TYPE_RSP
 *  GATT_DiscPrimaryServiceByUUID	ATT_FIND_BY_TYPE_VALUE_RSP
 *  GATT_DiscAllCharDescs			ATT_READ_BY_TYPE_RSP
 *  GATT_ReadUsingCharUUID			ATT_READ_BY_TYPE_RSP
 *  GATT_ReadUsingCharUUID			ATT_READ_BY_TYPE_RSP
*/

/**
 * BLE的属性类型是有限的，有四个大类：
 * Primary Service（首要服务项）
 * Secondary Service（次要服务项）
 * Include（包含服务项）
 * Characteristic（特征值）
 * 0x1800 – 0x26FF ：服务项类型
 * 0x2700 – 0x27FF ：单位
 * 0x2800 – 0x28FF ：属性类型
 * 0x2900 – 0x29FF ：描述符类型
 * 0x2A00 – 0x7FFF ：特征值类型
*/
uint16_t UUID_SRV_INFO = 0x180A;	// 设备信息
uint16_t UUID_SRV_HID = 0x1812;		// HID设备
uint16_t UUID_SRV_BTT = 0x180F;		// 电池数据
uint16_t UUID_char_MOUSE_IN = 0x2A33;	// HID鼠标输入
uint16_t UUID_char_BTT = 0x2A19;	// 电池数据

////////////////////////////////////////////// UUID END


// Task ID for internal task/event processing
static uint8_t centralTaskId;

// Scan result list
static gapDevRec_t centralDevList[DEFAULT_MAX_SCAN_RES];

// Peer device address
// Device 4 - Addr 4e a1 86 52 e4 c3, RSSI -26 dBm
// static uint8_t PeerAddrDef[B_ADDR_LEN] = {0x0, 0xa1, 0x86, 0x52, 0xe4, 0xc3};

// 自动连接的最小RSSI
const static int8_t requireRSSI = -40;
// 自动连接的设备信息
static gapDevRec_t peerDev;

// Parameter update state
static uint8_t centralParamUpdate = TRUE;

// Connection handle of current connection
// 设备句柄，GAP成功连接后置值
static uint16_t centralConnHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8_t centralState = BLE_STATE_IDLE;

// Discovery state
static uint8_t centralDiscState = BLE_DISC_STATE_IDLE;

// Discovered characteristic handle
static uint16_t centralCharHdl = 0;

// Value to write
static uint8_t centralCharVal = 0x5A;

// Value read/write toggle
static uint8_t centralDoWrite = TRUE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void gattCentralMsg(gattMsgEvent_t *pMsg);
static void gapCentralRssiCB(uint16_t connHandle, int8_t rssi);
static void gapCentralEventCB(gapRoleEvent_t *pEvent);
static void gapCentralHciMTUChangeCB(uint16_t connHandle, uint16_t maxTxOctets, uint16_t maxRxOctets);
static void gapCentralPasscodeCB(uint8_t *deviceAddr, uint16_t connectionHandle, uint8_t uiInputs, uint8_t uiOutputs);
static void gapCentralPairStateCB(uint16_t connHandle, uint8_t state, uint8_t status);
static void centralAddDeviceInfo(uint8_t *pAddr, uint8_t addrType, int8_t rssi);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
// GAP 相关的回调
static gapCentralRoleCB_t centralRoleCB = {
	gapCentralRssiCB,        // RSSI callback
	gapCentralEventCB,       // Event callback
	gapCentralHciMTUChangeCB // MTU change callback
};

// Bond Manager Callbacks
// 配对 和 绑定 相关的回调
static gapBondCBs_t centralBondCB = {
	gapCentralPasscodeCB,
	gapCentralPairStateCB
};

/// @GATTINFO begin /////////////////////////////////////////////////////////////////
#define MAX_GATT_INFO 10
struct GATTService
{
	uint16_t svcStartHandle;
	uint16_t svcEndHandle;
	uint16_t uuid;
	// struct GATTCharacteristic
	// {
	// 	uint16_t declareHandle;		// metaHandle
	// 	uint8_t readWrite;
	// 	uint16_t handle;
	// 	uint16_t uuid ;
	// }chars[10];
}gattInfo[MAX_GATT_INFO];

uint8_t queryServiceIndex = 0;
int8_t QueryServiceInfo(uint16_t centralConnHandle, uint8_t centralTaskId, uint8_t next)
{
	if(queryServiceIndex >= MAX_GATT_INFO)
	{
		LOG("查询服务结束 over max\r\n");
		return -1;
	}
	queryServiceIndex += next;
	struct GATTService *s = &gattInfo[queryServiceIndex];
	if(s->uuid == 0)
	{
		LOG("查询服务结束\r\n");
		return -2;
	}
	GATT_DiscAllChars(centralConnHandle, s->svcStartHandle, s->svcEndHandle, centralTaskId);
	return 0;
}

void clearAllGATTSvc()
{
	for(uint8_t i = 0; i < MAX_GATT_INFO; ++i)
	{
		gattInfo[i].svcStartHandle = 0;
		gattInfo[i].svcEndHandle = 0;
		gattInfo[i].uuid = 0;
	}
}

struct GATTService* getBlankGATTSvc()
{
	for(uint8_t i = 0; i < MAX_GATT_INFO; ++i)
	{
		if(gattInfo[i].uuid == 0)
			return &gattInfo[i];
	}
	return NULL;
}

uint16_t guessUUID=0x2A33;	// 2A33 鼠标输入 2A4D 鼠标输出
/// @GATTINFO end //////////////////////////////////////////////////////////////////////

// 自定义函数 begin //////////////////////////////////////////////////////////////////////////
uint8_t parseDeviceInfo(gapDeviceInfoEvent_t* info)
{
	uint8_t ret = eUnknown;
	uint8_t *p = info->pEvtData;
	uint8_t l = info->dataLen;
	for(uint8_t i = 0; i < l; i)
	{
		uint8_t siz = p[i];
		uint8_t type = p[i+1];
		uint8_t* pVal = p + i + 2;
		if(type == GAP_ADTYPE_FLAGS)
		{
			if(*pVal == GAP_ADTYPE_FLAGS_LIMITED)
			{
				LOG("    广播类型 0x01 LE Limited Discoverable Mode\r\n");
			}
			else if(*pVal == GAP_ADTYPE_FLAGS_GENERAL)
			{
				LOG("    广播类型 0x02 LE General Discoverable Mode\r\n");
			}
			else if(*pVal == GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED)
			{
				LOG("    广播类型 0x04 BR/EDR Not Supported\r\n");
			}
			else
			{
				LOG("    广播类型 0x%02X 未知\r\n", *pVal);
			}
		}
		else if(type == GAP_ADTYPE_16BIT_COMPLETE)
		{
			LOG("    UUID 0x%04X\r\n", BUILD_UINT16(*(pVal+0), *(pVal+1)));
		}
		else if(type == GAP_ADTYPE_LOCAL_NAME_COMPLETE)
		{
			uint8_t v[siz];
			memcpy(v, pVal, siz - 1);
			v[siz-1] = '\0';
			LOG("    设备名称 \"%s\"\r\n", v);
		}
		else if(type == GAP_ADTYPE_POWER_LEVEL)
		{
			LOG("    电量 %d%%\r\n", *pVal);
		}
		else if(type == GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE)
		{
			uint16_t min = BUILD_UINT16(*(pVal+0), *(pVal+1));
			uint16_t max = BUILD_UINT16(*(pVal+2), *(pVal+3));
			LOG("    连接间隔范围 %d*1.25ms - %d*1.25ms\r\n", min, max);
		}
		else if(type == GAP_ADTYPE_APPEARANCE)
		{
			// https://www.bluetooth.com/wp-content/uploads/Files/Specification/Assigned_Numbers.html#bookmark49
			uint16_t v = BUILD_UINT16(*(pVal+0), *(pVal+1));
			if(v == 0x03c1)
			{
				ret = eKeyboard;
				LOG("    Appearance 0x03c1 Keyboard\r\n");
			}
			else if (v == 0x03c2)
			{
				ret = eMouse;
				LOG("    Appearance 0x03c2 Mouse\r\n");
			}
			else
			{
				ret = eUnknown;
				LOG("    Appearance 0x%04X 未知\r\n", v);
			}
		}
		else
		{
			LOG("    未知数据类型 type 0x%X", type);
			Print_Memory(pVal, siz - 1, 1);
		}
		i += 1 + siz;
	}
	return ret;
}

void clearPeerDev(gapDevRec_t* peerDev)
{
	memset(peerDev, 0, sizeof(gapDevRec_t));
}

#define MAX_READ_HANDLE 20
static uint16_t lReadHandle[MAX_READ_HANDLE] = {0};

void addReadHandle(uint16_t handle)
{
	for(uint8_t i = 0; i < MAX_READ_HANDLE; ++i)
	{
		if(lReadHandle[i] == 0)
		{
			lReadHandle[i] = handle;
			break;
		}
	}
}

uint16_t getReadHandle(uint8_t remove)
{
	for(uint8_t i = 0; i < MAX_READ_HANDLE; ++i)
	{
		if(lReadHandle[i] != 0)
		{
			uint16_t hdl = lReadHandle[i];
			if(remove > 0)
				lReadHandle[i] = 0;
			return hdl;
		}
	}
	return 0;
}

#define MAX_JOB 100
typedef struct {
    uint8_t type;
	uint8_t v8;
	uint8_t v8_2;
	uint16_t v16;
	uint16_t v16_2;
} Node;
Node job_tmp;
static Node jobs[MAX_JOB];
void job_clear_at(uint8_t index)
{
	memset(&jobs[index], 0, sizeof(Node));
}
int8_t job_rearray()
{
	int8_t ret = 0;
	for(uint8_t i = 0; i < MAX_JOB; ++i)
	{
		if(jobs[i].type == 0)
		{
			for(uint8_t j = i + 1; j < MAX_JOB; ++j)
			{
				if(jobs[j].type != 0)
				{
					memcpy(&jobs[i], &jobs[j], sizeof(Node));
					memset(&jobs[j], 0, sizeof(Node));
					ret ++;
					break;
				}
			}
		}
	}
	return ret;
}
int8_t job_add(uint8_t type, uint8_t v8, uint8_t v8_2, uint16_t v16, uint16_t v16_2)
{
	if(jobs[MAX_JOB - 1].type != 0)
	{
		int8_t r = job_rearray();
		if(r == 0) 
		{
			LOG("任务队列已满\r\n");
			return -1;
		}
	}
	uint8_t i = MAX_JOB;
	for(uint8_t j = MAX_JOB - 1; j >= 0; --j)
	{
		if(jobs[j].type == 0)
		{
			i = j;
			break;
		}
	}
	if(i == MAX_JOB)
	{
		return -2;		// 理论应该不会出现
	}
	jobs[i].type = type;
	jobs[i].v8 = v8;
	jobs[i].v8_2 = v8_2;
	jobs[i].v16 = v16;
	jobs[i].v16_2 = v16_2;
	return 0;
}

Node* job_pop()
{
	for(uint8_t i = 0; i < MAX_JOB; ++i)
	{
		if(jobs[i].type != 0)
		{
			memcpy(&job_tmp, &jobs[i], sizeof(Node));
			job_clear_at(i);
			return &job_tmp;
		}
	}
	return NULL;
}
// 自定义函数 end ////////////////////////////////////////////////////////////////////////////

// CCCD 相关 begin //////////////////////////////////////////////////////////////////////////
uint16_t CCCD_SvcUUID=0x1812;	// 需要使能的CCCD的svc， 0x1812 HID设备
static uint16_t CCCD_SvcStartHdl = 0;
static uint16_t CCCD_SvcEndHdl = 0;
static uint16_t CCCD_Hdl = 0;

int8_t enableCCCD(uint16_t cccd)
{
	int8_t ret = 0;
	attWriteReq_t req;
	req.cmd = FALSE;
	req.sig = FALSE;
	req.handle = cccd;
	req.len = 2;
	req.pValue = GATT_bm_alloc(centralConnHandle, ATT_WRITE_REQ, req.len, NULL, 0);
	if(req.pValue != NULL)
	{
		req.pValue[0] = 1;
		req.pValue[1] = 0;
		if(!GATT_WriteCharValue(centralConnHandle, &req, centralTaskId) == SUCCESS)
		{
			ret = -1;
		}
		GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
		return ret;
	}
}
// CCCD 相关 end ////////////////////////////////////////////////////////////////////////////


/*********************************************************************
 * @fn      Central_Init
 *
 * @brief   Initialization function for the Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by TMOS.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void Central_Init()
{
	centralTaskId = TMOS_ProcessEventRegister(Central_ProcessEvent);

	clearAllGATTSvc();

	clearPeerDev(&peerDev);
	for(uint8_t i = 0; i < DEFAULT_MAX_SCAN_RES; ++i)
		clearPeerDev(&centralDevList[i]);

	for(uint8_t i = 0; i < MAX_READ_HANDLE; ++i)
		lReadHandle[i] = 0;
	
	for(uint8_t i = 0; i < MAX_JOB; ++i)
		job_clear_at(i);

	// Setup GAP
	GAP_SetParamValue(TGAP_DISC_SCAN, DEFAULT_SCAN_DURATION);	// 单次扫描时长
	GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, DEFAULT_MIN_CONNECTION_INTERVAL);
	GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, DEFAULT_MAX_CONNECTION_INTERVAL);
	GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, DEFAULT_CONNECTION_TIMEOUT);	// 连接超时时长
	// Setup the GAP Bond Manager
	{
		uint32_t passkey = DEFAULT_PASSCODE;
		uint8_t  pairMode = DEFAULT_PAIRING_MODE;
		uint8_t  mitm = DEFAULT_MITM_MODE;
		uint8_t  ioCap = DEFAULT_IO_CAPABILITIES;
		uint8_t  bonding = DEFAULT_BONDING_MODE; 
		GAPBondMgr_SetParameter(GAPBOND_CENT_DEFAULT_PASSCODE, sizeof(uint32_t), &passkey);
		GAPBondMgr_SetParameter(GAPBOND_CENT_PAIRING_MODE, sizeof(uint8_t), &pairMode);
		GAPBondMgr_SetParameter(GAPBOND_CENT_MITM_PROTECTION, sizeof(uint8_t), &mitm);
		GAPBondMgr_SetParameter(GAPBOND_CENT_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
		GAPBondMgr_SetParameter(GAPBOND_CENT_BONDING_ENABLED, sizeof(uint8_t), &bonding);
	}
	// Initialize GATT Client
	GATT_InitClient();
	// Register to receive incoming ATT Indications/Notifications
	GATT_RegisterForInd(centralTaskId);
	// Setup a delayed profile startup
	tmos_set_event(centralTaskId, START_DEVICE_EVT);
}

/*********************************************************************
 * @fn      Central_ProcessEvent
 * 主回调
 *
 * @brief   Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The TMOS assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16_t Central_ProcessEvent(uint8_t task_id, uint16_t events)
{
	if(events & SYS_EVENT_MSG)
	{
		uint8_t *pMsg;
		// LOG("主回调 SYS_EVENT_MSG\r\n");
		if((pMsg = tmos_msg_receive(centralTaskId)) != NULL)
		{
			uint8_t e = ((gattMsgEvent_t *)pMsg)->hdr.event;
			if(e == GATT_MSG_EVENT)
			{
				// GATT 消息
				gattCentralMsg((gattMsgEvent_t *)pMsg);
			}
			else
			{
				LOG("  收到消息 未知消息类型 unknown %d\r\n", e);
			}
			tmos_msg_deallocate(pMsg);
		}
		return (events ^ SYS_EVENT_MSG);
	}
	if(events & START_DEVICE_EVT)
	{
		LOG("主回调 START_DEVICE_EVT 设备初始化完成\r\n");
		GAPRole_CentralStartDevice(centralTaskId, &centralBondCB, &centralRoleCB);
		return (events ^ START_DEVICE_EVT);
	}
	if(events & ESTABLISH_LINK_TIMEOUT_EVT)
	{
		LOG("主回调 ESTABLISH_LINK_TIMEOUT_EVT\r\n");
		GAPRole_TerminateLink(INVALID_CONNHANDLE);
		return (events ^ ESTABLISH_LINK_TIMEOUT_EVT);
	}	
	if(events & START_SVC_DISCOVERY_EVT)
	{
		// start service discovery
		LOG("主回调 START_SVC_DISCOVERY_EVT 搜索枚举服务任务 开始\r\n");
		{
			centralDiscState = BLE_DISC_STATE_ALL_SVC;
			if(DEFAULT_PAIRING_MODE == GAPBOND_PAIRING_MODE_INITIATE)
			{
				bStatus_t r = GAPBondMgr_PeriSecurityReq(centralConnHandle);
				if(r == SUCCESS)
				{
					LOG("主回调 START_SVC_DISCOVERY_EVT 请求配对成功\r\n");
				}
				else if (r == bleNotConnected)
				{
					LOG("主回调 START_SVC_DISCOVERY_EVT 请求配对失败 bleNotConnected\r\n");
				}
				else if(r == bleAlreadyInRequestedMode)
				{
					LOG("主回调 START_SVC_DISCOVERY_EVT 请求配对失败 bleAlreadyInRequestedMode\r\n");
				}
				else if(r == bleIncorrectMode)
				{
					LOG("主回调 START_SVC_DISCOVERY_EVT 请求配对失败 bleIncorrectMode\r\n");
				}
				else if(r == bleMemAllocError)
				{
					LOG("主回调 START_SVC_DISCOVERY_EVT 请求配对失败 bleMemAllocError\r\n");
				}
				else if(r == bleInvalidRange)
				{
					LOG("主回调 START_SVC_DISCOVERY_EVT 请求配对失败 bleInvalidRange\r\n");
				}
				else if(r == bleAlreadyInRequestedMode)
				{
					LOG("主回调 START_SVC_DISCOVERY_EVT 请求配对失败 bleAlreadyInRequestedMode\r\n");
				}
				else
				{
					LOG("主回调 START_SVC_DISCOVERY_EVT 请求配对失败 %d\r\n", r);
				}
			}
			GATT_DiscAllPrimaryServices(centralConnHandle, centralTaskId);
		}
		return (events ^ START_SVC_DISCOVERY_EVT);
	}	
	if(events & START_PARAM_UPDATE_EVT)
	{
		// start connect parameter update 
		LOG("主回调 START_PARAM_UPDATE_EVT\r\n");
		GAPRole_UpdateLink(centralConnHandle,
							DEFAULT_UPDATE_MIN_CONN_INTERVAL,
							DEFAULT_UPDATE_MAX_CONN_INTERVAL,
							DEFAULT_UPDATE_SLAVE_LATENCY,
							DEFAULT_UPDATE_CONN_TIMEOUT);
		return (events ^ START_PARAM_UPDATE_EVT);
	}	
	if(events & START_PHY_UPDATE_EVT)
	{
		// start phy update
		LOG("主回调 START_PHY_UPDATE_EVT\r\n");
		bStatus_t t = GAPRole_UpdatePHY(centralConnHandle, 0, GAP_PHY_BIT_LE_2M, GAP_PHY_BIT_LE_2M, GAP_PHY_OPTIONS_NOPRE);
		LOG("PHY Update %x...\r\n", t);
		return (events ^ START_PHY_UPDATE_EVT);
	}
	if(events & START_READ_OR_WRITE_EVT)
	{
		LOG("主回调 START_READ_OR_WRITE_EVT 主机读数据从从机，一秒一次\r\n");
		{
			// if(centralDoWrite)
			// {
			// 	// Do a write
			// 	attWriteReq_t req;
			// 	req.cmd = FALSE;
			// 	req.sig = FALSE;
			// 	req.handle = centralCharHdl;
			// 	req.len = 1;
			// 	req.pValue = GATT_bm_alloc(centralConnHandle, ATT_WRITE_REQ, req.len, NULL, 0);
			// 	if(req.pValue != NULL)
			// 	{
			// 		*req.pValue = centralCharVal;
			// 		if(GATT_WriteCharValue(centralConnHandle, &req, centralTaskId) == SUCCESS)
			// 		{
			// 			centralDoWrite = !centralDoWrite;
			// 			tmos_start_task(centralTaskId, START_READ_OR_WRITE_EVT, DEFAULT_READ_OR_WRITE_DELAY);
			// 		}
			// 		else
			// 		{
			// 			GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
			// 		}
			// 	}
			// }
			// else
			{
				// Do a read
				// LOG("0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X \r\n", jobHandle[0], jobHandle[1], jobHandle[2], jobHandle[3], jobHandle[4], jobHandle[5], jobHandle[6], jobHandle[7], jobHandle[8], jobHandle[9]);
				uint16_t hdl = getReadHandle(1);
				if(hdl != 0)
				{
					attReadReq_t req;
					req.handle = hdl;
					// 调用后收到 GATT_MSG_EVENT 事件， ATT_READ_RSP 消息
					LOG("    开启一个读写任务 handle 0x%04X\r\n", hdl);
					if(GATT_ReadCharValue(centralConnHandle, &req, centralTaskId) == SUCCESS)
					{
						centralDoWrite = !centralDoWrite;
					}
					tmos_start_task(centralTaskId, START_READ_OR_WRITE_EVT, DEFAULT_READ_OR_WRITE_DELAY);
				}
			}
		}
		return (events ^ START_READ_OR_WRITE_EVT);
	}
	if(events & START_WRITE_CCCD_EVT)
	{
		LOG("主回调 START_WRITE_CCCD_EVT 尝试使能 CCCD\r\n");
		{
			// Do a write
			attWriteReq_t req;
			req.cmd = FALSE;
			req.sig = FALSE;
			req.handle = 0;
			req.len = 2;
			req.pValue = GATT_bm_alloc(centralConnHandle, ATT_WRITE_REQ, req.len, NULL, 0);
			if(req.pValue != NULL)
			{
				req.pValue[0] = 1;
				req.pValue[1] = 0;
				if(GATT_WriteCharValue(centralConnHandle, &req, centralTaskId) == SUCCESS)
				{
					LOG("主回调 START_WRITE_CCCD_EVT 尝试使能 CCCD 2\r\n");
				}
				GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
			}
		}
		return (events ^ START_WRITE_CCCD_EVT);
	}
	if(events & START_READ_RSSI_EVT)
	{
		GAPRole_ReadRssiCmd(centralConnHandle);
		tmos_start_task(centralTaskId, START_READ_RSSI_EVT, DEFAULT_RSSI_PERIOD);
		return (events ^ START_READ_RSSI_EVT);
	}
	// Discard unknown events
	return 0;
}

/*********************************************************************
 * @fn      gattCentralMsg
 *
 * @brief   Process GATT messages
 * GATT 读取从设备发来的数据
 *
 * @return  none
 */
// 日志前加二空格
static void gattCentralMsg(gattMsgEvent_t *pMsg)
{
	{
		// PRINT("GATT 消息总体描述. "); BLE_GATT_MSG_DESC(pMsg); PRINT("\r\n");
	}
	// 如果设备当前不在已连接状态，则忽略所有GATT消息，释放消息内存并返回。
	if(centralState != BLE_STATE_CONNECTED)
	{
		LOG("  centralState != BLE_STATE_CONNECTED\r\n");
		GATT_bm_free(&pMsg->msg, pMsg->method);
		return;
	}
	if(pMsg->method == ATT_ERROR_RSP)
	{
		uint8_t reqOpcode = pMsg->msg.errorRsp.reqOpcode;
		uint8_t errCode = pMsg->msg.errorRsp.errCode;
		if(reqOpcode == ATT_EXCHANGE_MTU_REQ)
		{
			LOG("  Exchange MTU Error: %x\r\n", errCode);
		}
		else if(reqOpcode == ATT_READ_REQ)
		{
			if(errCode == ATT_ERR_INSUFFICIENT_AUTHEN)	// 需要发起安全请求
			{
				LOG("  ATT_ERROR_RSP Read Error: ATT_ERR_INSUFFICIENT_AUTHEN\r\n");
			}
			else if(errCode == ATT_ERR_INSUFFICIENT_ENCRYPT)	// 需要发起加密请求
			{
				LOG("  ATT_ERROR_RSP Read Error: ATT_ERR_INSUFFICIENT_ENCRYPT\r\n");
			}
			else
			{
				LOG("  ATT_ERROR_RSP Read Error: %x\r\n", errCode);
			}
		}
		else if(reqOpcode == ATT_WRITE_REQ)
		{
			if(errCode == ATT_ERR_INSUFFICIENT_AUTHEN)	// 需要发起安全请求
			{
				LOG("  ATT_ERROR_RSP Write Error: ATT_ERR_INSUFFICIENT_AUTHEN\r\n");
			}
			else if(errCode == ATT_ERR_INSUFFICIENT_ENCRYPT)	// 需要发起加密请求
			{
				LOG("  ATT_ERROR_RSP Write Error: ATT_ERR_INSUFFICIENT_ENCRYPT\r\n");
			}
			else
			{
				LOG("  ATT_ERROR_RSP Write Error: %x\r\n", errCode);
			}
		}
		else
		{
			LOG("  ATT ERROR: %x %x\r\n", reqOpcode, errCode);
		}
		return;
	}

	// 如果消息是MTU交换响应 或 MTU交换请求的错误响应，则处理MTU交换。
	if(pMsg->method == ATT_EXCHANGE_MTU_RSP)
	{
		// 获取实际协商的MTU大小
		uint16_t negotiatedMTU = pMsg->msg.exchangeMTUReq.clientRxMTU;
		LOG("  Negotiated MTU: %d\r\n", negotiatedMTU);
	}
	// 如果消息是MTU更新事件，则打印新的MTU值。
	if(pMsg->method == ATT_MTU_UPDATED_EVENT)
	{
		LOG("  ATT_MTU_UPDATED_EVENT MTU: %d\r\n", pMsg->msg.mtuEvt.MTU);
	}
	// 如果消息是读取响应 或 读取请求的错误响应，则处理读取结果。
	if(pMsg->method == ATT_READ_RSP)
	{
		// After a successful read, display the read value
		LOG("  ATT_READ_RSP Read rsp: %s\r\n", *pMsg->msg.readRsp.pValue);
	}
	// 如果消息是写入响应 或 写入请求的错误响应，则处理写入结果。
	else if(pMsg->method == ATT_WRITE_RSP)
	{
		// Write success
		LOG("  ATT_WRITE_RSP Write success\r\n");
	}
	// 通知，则打印接收到的通知值。
	// 鼠标会走这！！！
	else if(pMsg->method == ATT_HANDLE_VALUE_NOTI)
	{
		// 接收到 数据 通知 指示
		//LOG("  ATT_HANDLE_VALUE_NOTI Receive noti: %x 接收到从机notify数据 \r\n", *pMsg->msg.handleValueNoti.pValue);
		uint8_t* p = pMsg->msg.handleValueNoti.pValue;
		uint8_t mouse = *p;
		int8_t x = *(p + 2);
		// int8_t y = *(p + 3);
		int8_t y1 = *(p + 4);
		int8_t y2 = *(p + 3);
		int8_t y = (0xF0&(y1<<4)) + (0x0F & (y2>>4));
		int8_t wheel = 0;
    	// PRINT("y1=0x%x y2=0x%x \n", y1, y2);
    	// PRINT("y1=0x%x y2=0x%x \n", 0xF0&(y1<<4), 0x0F & (y2>>4));
    	// PRINT("x=0x%x y=0x%x \n", x, y);
		DevHIDMouseReport(mouse, x, y, wheel);
	}
	// 指示，和通知不同的是，指示需要发送确认。
	else if(pMsg->method == ATT_HANDLE_VALUE_IND)
	{
		LOG("  ATT_HANDLE_VALUE_IND Receive ind: %x 接收到从机ind数据 \r\n", *pMsg->msg.handleValueInd.pValue);
		ATT_HandleValueCfm(centralConnHandle);	// 发送确认
	}
	else if(centralDiscState != BLE_DISC_STATE_IDLE)
	{
		// 处于发送UUID查找功能的状态
		// 期间会收到多个数据返回 根据 pMsg->hdr.status == bleProcedureComplete 判断数据接收完毕
		if(centralDiscState == BLE_DISC_STATE_ALL_SVC)
		{
			LOG("    当前状态 BLE_DISC_STATE_ALL_SVC\r\n");
			if(pMsg->method == ATT_READ_BY_GRP_TYPE_RSP)
			{
				uint16_t numGrps = pMsg->msg.readByGrpTypeRsp.numGrps;
				uint16_t len = pMsg->msg.readByGrpTypeRsp.len;
				uint8_t* l = pMsg->msg.readByGrpTypeRsp.pDataList;
				if(numGrps >= 0)
				{
					BLE_UUID_DESC(l, numGrps);
					for (uint16_t i = 0; i < numGrps; i++)
					{
						uint8_t* u = l + i * 6;
						uint16_t startHandle = BUILD_UINT16(*(u+0), *(u+1));
						uint16_t endHandle = BUILD_UINT16(*(u+2), *(u+3));
						uint16_t uuid = BUILD_UINT16(*(u+4), *(u+5));
						
						struct GATTService* s = getBlankGATTSvc();
						if(s)
						{
							s->svcStartHandle = startHandle;
							s->svcEndHandle = endHandle;
							s->uuid = uuid;
						}

						if(uuid == CCCD_SvcUUID)
						{
							CCCD_SvcStartHdl = startHandle;
							CCCD_SvcEndHdl = endHandle;
							LOG("    找到HID设备, CCCD_SvcStartHdl 0x%04X CCCD_SvcEndHdl 0x%04X\r\n", CCCD_SvcStartHdl, CCCD_SvcEndHdl);
						}
					}
				}
				else if(len != 6)
				{
					LOG("    这个是128位的UUID 暂时不支持数据打印 todo\r\n");
				}
				
				if(pMsg->hdr.status == bleProcedureComplete)
				{
					centralDiscState = BLE_DISC_STATE_CHAR_ALL;
					LOG("    BLE_DISC_STATE_ALL_SVC 结束. 进入 BLE_DISC_STATE_CHAR_ALL 状态\r\n");
					QueryServiceInfo(centralConnHandle, centralTaskId, 0);
				}
			}
		}
		else if(centralDiscState == BLE_DISC_STATE_CHAR_ALL)
		{
			LOG("    当前状态 BLE_DISC_STATE_CHAR_ALL\r\n");
			if(pMsg->method == ATT_READ_BY_TYPE_RSP && pMsg->msg.readByTypeRsp.numPairs > 0)
			{
				uint16_t numPairs = pMsg->msg.readByTypeRsp.numPairs;
				uint16_t len = pMsg->msg.readByTypeRsp.len;
				uint8_t *pDataList = pMsg->msg.readByTypeRsp.pDataList;
				for(uint16_t i = 0; i < numPairs; ++i)
				{
					// 12 = GATT_PROP_READ | GATT_PROP_NOTIFY
					uint8_t* u = pDataList + len * i;
					uint16_t declareHandle = BUILD_UINT16(*(u+0), *(u+1));	// 属性声明句柄 metadata
					uint8_t readWrite = *(u + 2);		// 读写权限 GATT_PROP_WRITE
					uint16_t handle = BUILD_UINT16(*(u+3), *(u+4));
					uint16_t uuid = BUILD_UINT16(*(u+5), *(u+6));
					
					LOG("    DeclareHandle 0x%04X 权限 %s%s%s%s%s%s%s%s handle 0x%04X uuid 0x%04X %s\r\n", declareHandle, \
					(readWrite & GATT_PROP_BCAST) ? "广播,":"", \
					(readWrite & GATT_PROP_READ) ? "读,":"", \
					(readWrite & GATT_PROP_WRITE_NO_RSP) ? "写无应答,":"", \
					(readWrite & GATT_PROP_WRITE) ? "写,":"", \
					(readWrite & GATT_PROP_NOTIFY) ? "通知,":"", \
					(readWrite & GATT_PROP_INDICATE) ? "指示,":"", \
					(readWrite & GATT_PROP_AUTHEN) ? "认证,":"", \
					(readWrite & GATT_PROP_EXTENDED) ? "扩展,":"", \
					handle, uuid, BLE_UUID2str(uuid));

					if(readWrite & GATT_PROP_READ)
					{
						addReadHandle(handle);
					}
				}
			}
			// 0x2A4B 是 HID信息的UUID 需要获取数据否则解析不了数据
			if(pMsg->method == ATT_READ_BY_TYPE_RSP && pMsg->hdr.status == bleProcedureComplete)
			{
				
				int8_t r = QueryServiceInfo(centralConnHandle, centralTaskId, 1);
				if(r >= 0)
				{
					LOG("    继续状态 BLE_DISC_STATE_CHAR_ALL\r\n");
					return;
				}
				if(CCCD_SvcStartHdl != 0)
				{
					centralDiscState = BLE_DISC_STATE_CCCD;
					LOG("    BLE_DISC_STATE_CHAR_ALL 结束. 进入 BLE_DISC_STATE_CCCD 状态\r\n");
					attReadByTypeReq_t req;
					req.startHandle = CCCD_SvcStartHdl;
					req.endHandle = CCCD_SvcEndHdl;
					req.type.len = ATT_BT_UUID_SIZE;
					// GATT_CLIENT_CHAR_CFG_UUID 是获取CCCD的UUID
					req.type.uuid[0] = LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID);
					req.type.uuid[1] = HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID);
					GATT_ReadUsingCharUUID(centralConnHandle, &req, centralTaskId);
				}
				else
				{
					centralDiscState = BLE_DISC_STATE_IDLE;
					LOG("    未发现HID相关的CCCD服务\r\n");
					LOG("    BLE_DISC_STATE_CHAR_ALL 结束. 进入 BLE_DISC_STATE_IDLE 状态\r\n");
				}
			}
		}
		else if(centralDiscState == BLE_DISC_STATE_CCCD)
		{
			LOG("    当前状态 BLE_DISC_STATE_CCCD\r\n");
			if(pMsg->method == ATT_READ_BY_TYPE_RSP && pMsg->msg.readByTypeRsp.numPairs > 0)
			{
				uint16_t numPairs = pMsg->msg.readByTypeRsp.numPairs;
				uint16_t len = pMsg->msg.readByTypeRsp.len;
				uint8_t *pDataList = pMsg->msg.readByTypeRsp.pDataList;
				// 此处假设只有一个CCCD！！！
				for(uint16_t i = 3; i < numPairs; i++)
				{
					uint16_t handle = BUILD_UINT16(pDataList[len * i], pDataList[len * i+1]);
					LOG("    发现 CCCD handle 0x%04X\r\n", handle);
					if(CCCD_Hdl == 0)
						CCCD_Hdl = handle;
				}
				// Start do write CCCD
				//tmos_start_task(centralTaskId, START_WRITE_CCCD_EVT, DEFAULT_WRITE_CCCD_DELAY);
			}
			if(pMsg->method == ATT_READ_BY_TYPE_RSP && pMsg->hdr.status == bleProcedureComplete)
			{
				if(CCCD_Hdl)
				{
					int8_t r = enableCCCD(CCCD_Hdl);
					if(r == 0)
						LOG("    尝试使能CCCD 0x%04X\r\n", CCCD_Hdl);
					else
						LOG("    使能CCCD失败\r\n");
				}
				centralDiscState = BLE_DISC_STATE_IDLE;
				LOG("    BLE_DISC_STATE_CCCD 结束. 进入 BLE_DISC_STATE_IDLE 状态\r\n");
			}
		}
		else
		{
			LOG("    未知错误 todo\r\n");
		}
	}
	GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      gapCentralRssiCB
 *
 * @brief   RSSI callback.
 * 这个回调函数在接收到 RSSI（接收信号强度指示）报告时被调用。
 * 应用程序可以根据 RSSI 值来调整通信策略，例如调整发射功率或者决定是否保持连接。
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void gapCentralRssiCB(uint16_t connHandle, int8_t rssi)
{
	LOG("RSSI : -%d dB \r\n", -rssi);
}

/*********************************************************************
 * @fn      gapCentralHciMTUChangeCB
 *
 * @brief   MTU changed callback.
 * 这个回调函数在 MTU（Maximum Transmission Unit，最大传输单元）大小发生变化时被调用。
 * MTU 决定了单个 BLE 数据包的最大字节数。
 * 应用程序可以根据新的 MTU 大小调整数据传输策略，以优化传输效率。
 *
 * @param   maxTxOctets - Max tx octets
 * @param   maxRxOctets - Max rx octets
 *
 * @return  none
 */
static void gapCentralHciMTUChangeCB(uint16_t connHandle, uint16_t maxTxOctets, uint16_t maxRxOctets)
{
	LOG("HCI data length changed, Tx: %d, Rx: %d\r\n", maxTxOctets, maxRxOctets);
}



/*********************************************************************
 * @fn      gapCentralEventCB
 *
 * @brief   Central event callback function.
 * 这个回调函数在发生 GAP 事件时被调用。
 * GAP 事件可能包括连接建立、连接断开、发现服务等。
 * 应用程序可以根据具体的事件类型采取相应的措施。例如，当连接建立时，可以启动数据传输；当连接断开时，可以重新启动扫描。
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void  gapCentralEventCB(gapRoleEvent_t *pEvent)
{
	switch(pEvent->gap.opcode)
	{
		// 设备初始化完成事件
		case GAP_DEVICE_INIT_DONE_EVENT:
		{
			LOG("EventCB GAP_DEVICE_INIT_DONE_EVENT 初始化结束 开始探测设备\r\n");
			// 调用后会多次触发 GAP_DEVICE_INFO_EVENT 事件
			// 扫描结束后 DEFAULT_SCAN_DURATION时长, 会触发 GAP_DEVICE_DISCOVERY_EVENT 事件
			GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
										DEFAULT_DISCOVERY_ACTIVE_SCAN,
										DEFAULT_DISCOVERY_WHITE_LIST);
			break;
		}
		// 蓝牙设备扫描过程中触发，一次扫描过程中会被触发多次。每当扫描到一个新设备或接收到一个设备的广播数据包时，都会触发这个事件。 GAPRole_CentralStartDiscovery 触发
		case GAP_DEVICE_INFO_EVENT:
		{
			if(pEvent->deviceInfo.rssi > requireRSSI)
			{
				LOG("EventCB GAP_DEVICE_INFO_EVENT 发现设备\r\n");
				int8_t rssi = pEvent->deviceInfo.rssi;
				uint8_t addrType =  pEvent->deviceInfo.addrType;
				uint8_t eventType = pEvent->deviceInfo.eventType;
				uint8_t *p = pEvent->deviceInfo.pEvtData;
				uint8_t l = pEvent->deviceInfo.dataLen;
				// 要加入的是广播包还是扫描应答包 todo
				if(eventType == GAP_ADRPT_ADV_IND)	// 广播包
				{
					LOG("  扫描广播包 ");
					Print_Memory(p, l, 1);
					uint8_t type = parseDeviceInfo(&pEvent->deviceInfo);
					if(type == eMouse || type == eKeyboard)
					{
						centralAddDeviceInfo(pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType, pEvent->deviceInfo.rssi);
					}
				}
				else if(eventType == GAP_ADRPT_SCAN_RSP)	// 扫描应答数据
				{
					LOG("  扫描应答包 ");
					Print_Memory(pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen, 1);
					parseDeviceInfo(&pEvent->deviceInfo);
				}
				else
				{
					LOG("  未知扫描包 !!!");
				}
			}
			break;
		}
		// 蓝牙设备扫描结束触发 GAPRole_CentralStartDiscovery 触发
		case GAP_DEVICE_DISCOVERY_EVENT:
		{
			LOG("EventCB GAP_DEVICE_DISCOVERY_EVENT 探测设备 结束\r\n");
			if(peerDev.rssi == 0)
			{
				LOG("  探测设备 未找到设备 开始重新探测设备\r\n");
				GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
											DEFAULT_DISCOVERY_ACTIVE_SCAN,
											DEFAULT_DISCOVERY_WHITE_LIST);
			}
			else
			{
				LOG("  找到设备 MAC %x-%x-%x-%x-%x-%x. rssi %d. 尝试连接\r\n", peerDev.addr[0], peerDev.addr[1], peerDev.addr[2], peerDev.addr[3], peerDev.addr[4], peerDev.addr[5], peerDev.rssi);
				GAPRole_CentralEstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
											DEFAULT_LINK_WHITE_LIST,
											peerDev.addrType,
											peerDev.addr);
				tmos_start_task(centralTaskId, ESTABLISH_LINK_TIMEOUT_EVT, ESTABLISH_LINK_TIMEOUT);
			}
			break;
		}
		case GAP_LINK_ESTABLISHED_EVENT:
		{
			LOG("EventCB GAP_LINK_ESTABLISHED_EVENT\r\n");
			tmos_stop_task(centralTaskId, ESTABLISH_LINK_TIMEOUT_EVT);
			if(pEvent->gap.hdr.status == SUCCESS)
			{
				LOG("  MAC %x-%x-%x-%x-%x-%x 连接成功\r\n", peerDev.addr[0], peerDev.addr[1], peerDev.addr[2], peerDev.addr[3], peerDev.addr[4], peerDev.addr[5]);
				centralState = BLE_STATE_CONNECTED;
				centralConnHandle = pEvent->linkCmpl.connectionHandle;

				// Update MTU
				attExchangeMTUReq_t req = {
					.clientRxMTU = BLE_BUFF_MAX_LEN - 4,
				};

				GATT_ExchangeMTU(centralConnHandle, &req, centralTaskId);

				// Initiate service discovery
				// 开启枚举服务任务
				tmos_start_task(centralTaskId, START_SVC_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY);

				// See if initiate connect parameter update
				// 开启更新连接参数任务
				if(centralParamUpdate)
				{
					tmos_start_task(centralTaskId, START_PARAM_UPDATE_EVT, DEFAULT_PARAM_UPDATE_DELAY);
				}
			}
			else
			{
				LOG("  MAC %x-%x-%x-%x-%x-%x 连接失败 Reason:%X. 开始探测设备\r\n", peerDev.addr[0], peerDev.addr[1], peerDev.addr[2], peerDev.addr[3], peerDev.addr[4], peerDev.addr[5],pEvent->gap.hdr.status);
				GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
											DEFAULT_DISCOVERY_ACTIVE_SCAN,
											DEFAULT_DISCOVERY_WHITE_LIST);
											
				clearPeerDev(&peerDev);
			}
			break;
		}
		case GAP_LINK_TERMINATED_EVENT:
		{
			LOG("EventCB GAP_LINK_TERMINATED_EVENT\r\n");
			centralState = BLE_STATE_IDLE;
			centralConnHandle = GAP_CONNHANDLE_INIT;
			centralDiscState = BLE_DISC_STATE_IDLE;
			centralCharHdl = 0;
			tmos_stop_task(centralTaskId, START_READ_RSSI_EVT);
			// https://blog.csdn.net/xiaoshideyuxiang/article/details/115075257
			// 0x13 remote user terminated connection
			LOG("  Disconnected...Reason:0x%x\r\n", pEvent->linkTerminate.reason);
			LOG("  探测设备 开始\r\n");
			GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
										DEFAULT_DISCOVERY_ACTIVE_SCAN,
										DEFAULT_DISCOVERY_WHITE_LIST);
			break;
		}
		case GAP_LINK_PARAM_UPDATE_EVENT:
		{
			LOG("EventCB GAP_LINK_PARAM_UPDATE_EVENT 更新成功\r\n");
			break;
		}
		case GAP_PHY_UPDATE_EVENT:
		{
			LOG("EventCB GAP_PHY_UPDATE_EVENT PHY Update...\r\n");
			break;
		}
		// case GAP_EXT_ADV_DEVICE_INFO_EVENT:
		// {
		// 	LOG("EventCB GAP_EXT_ADV_DEVICE_INFO_EVENT Recv ext adv \r\n");
		// 	// Add device to list
		// 	centralAddDeviceInfo(pEvent->deviceExtAdvInfo.addr, pEvent->deviceExtAdvInfo.addrType, pEvent->deviceExtAdvInfo.rssi);
		// 	break;
		// }
		// case GAP_DIRECT_DEVICE_INFO_EVENT:
		// {
		// 	LOG("EventCB GAP_DIRECT_DEVICE_INFO_EVENT\r\n");
		// 	// Display device addr
		// 	LOG("  Recv direct adv \r\n");
		// 	// Add device to list
		// 	centralAddDeviceInfo(pEvent->deviceDirectInfo.addr, pEvent->deviceDirectInfo.addrType, pEvent->deviceExtAdvInfo.rssi);
		// 	break;
		// }
		default:
			LOG("EventCB 未知事件 id %02x\r\n", pEvent->gap.opcode);
			break;
	}
}

/*********************************************************************
 * @fn      gapCentralPairStateCB
 *
 * @brief   Pairing state callback.
 * 当配对过程的状态发生变化时，会调用这个回调函数。
 * 这个回调函数允许应用程序处理不同的配对状态，例如配对成功或失败。
 * 应用程序可以根据配对状态和结果采取相应的措施，例如显示配对状态或执行后续操作。
 * GAPBondMgr_SlaveReqSecurity 会触发这个回调函数
 * 
 * status
 * 0x00	成功（SUCCESS）
 * 0x01	通用失败（FAILURE）
 * 0x02	无效参数（INVALID_PARAM）
 * 0x03	内存不足（OUT_OF_MEMORY）
 * 0x04	不支持的操作（NOT_SUPPORTED）
 * 0x05	认证失败（AUTH_FAILURE）
 * 0x06	无效句柄（INVALID_HANDLE）
 * 0x07	操作未完成（OPERATION_NOT_COMPLETE）
 * 0x08	超时（TIMEOUT）
 * 0x09	连接已存在（CONNECTION_ALREADY_EXISTS）
 * 0x0A	10 连接不存在（CONNECTION_NOT_EXISTS）
 * 0x0B	11 资源不足（RESOURCE_UNAVAILABLE）
 * 0x0C	12 协议错误（PROTOCOL_ERROR）
 * 0x0D	13 加密失败（ENCRYPTION_FAILURE）
 * 0x0E	14 配对失败（PAIRING_FAILURE）
 * 0x0F	15 配对已存在（PAIRING_ALREADY_EXISTS）
 * 0x10	16 配对被拒绝（PAIRING_REJECTED）
 * 0x11	17 配对超时（PAIRING_TIMEOUT）
 * 0x12	18 配对参数无效（PAIRING_INVALID_PARAM）
 * 0x13	19 配对未完成（PAIRING_NOT_COMPLETE）
 * 0x14	20 配对已取消（PAIRING_CANCELED）
 * 0x15	21 配对已禁用（PAIRING_DISABLED）
 * 0x16	22 配对已启用（PAIRING_ENABLED）
 * 0x17	23 配对已重置（PAIRING_RESET）
 * 0x18	24 配对已保存（PAIRING_SAVED）
 * 0x19	25 配对已删除（PAIRING_DELETED）
 * 0x1A	26 配对已恢复（PAIRING_RESTORED）
 * 0x1B	27 配对已更新（PAIRING_UPDATED）
 * 0x1C	28 配对已过期（PAIRING_EXPIRED）
 * 0x1D	29 配对已锁定（PAIRING_LOCKED）
 * @return  none
 */
static void gapCentralPairStateCB(uint16_t connHandle, uint8_t state, uint8_t status)
{
	LOG("gapCentralPairStateCB %d %d\r\n", state, status);
	if(state == GAPBOND_PAIRING_STATE_STARTED)
	{
		LOG("Pairing started:%d\r\n", status);
	}
	else if(state == GAPBOND_PAIRING_STATE_COMPLETE)
	{
		if(status == SUCCESS)
		{
			LOG("Pairing success\r\n");
		}
		else
		{
			LOG("Pairing fail\r\n");
		}
	}
	else if(state == GAPBOND_PAIRING_STATE_BONDED)
	{
		if(status == SUCCESS)
		{
			LOG("Bonding success\r\n");
		}
	}
	else if(state == GAPBOND_PAIRING_STATE_BOND_SAVED)
	{
		if(status == SUCCESS)
		{
			LOG("Bond save success\r\n");
		}
		else
		{
			LOG("Bond save failed: %d\r\n", status);
		}
	}
}

/*********************************************************************
 * @fn      gapCentralPasscodeCB
 *
 * @brief   Passcode callback.
 * 当设备要求输入配对码时，会调用这个回调函数。
 * 这个回调函数允许应用程序提供配对码，以便继续配对过程。
 * 应用程序可以根据输入和输出能力显示或请求用户输入配对码。
 *
 * @return  none
 */
static void gapCentralPasscodeCB(uint8_t *deviceAddr, uint16_t connectionHandle, uint8_t uiInputs, uint8_t uiOutputs)
{
	LOG("gapCentralPasscodeCB\r\n");
	uint32_t passcode = 0;
	// passcode = tmos_rand();
	// passcode %= 1000000;
	LOG("  send Passcode:%06d\r\n", (int)passcode);
	// Send passcode response
	GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      centralAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void centralAddDeviceInfo(uint8_t *pAddr, uint8_t addrType, int8_t rssi)
{
	if(rssi > requireRSSI)
	{
		LOG("  发现新设备 MAC %x-%x-%x-%x-%x-%x, RSSI %d dBm\r\n", pAddr[0], pAddr[1], pAddr[2], pAddr[3], pAddr[4], pAddr[5], rssi);
		if(peerDev.rssi == 0 || peerDev.rssi < rssi)
		{
			LOG("  发现新设备 记录 MAC %x-%x-%x-%x-%x-%x, RSSI %d dBm\r\n", pAddr[0], pAddr[1], pAddr[2], pAddr[3], pAddr[4], pAddr[5], rssi);
			memcpy(peerDev.addr, pAddr, B_ADDR_LEN);
			peerDev.addrType = addrType;
			peerDev.rssi = rssi;
		}
		else
		{
			LOG("信号强度弱于之前的设备 忽略\r\n");
		}
	}
}

/************************ endfile @ central **************************/
