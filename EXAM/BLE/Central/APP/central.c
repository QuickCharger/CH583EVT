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

/*********************************************************************
 * INCLUDES
 */
#include "CONFIG.h"
#include "gattprofile.h"
#include "central.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                      15

/*********************************************************************
 * CONSTANTS
 */
// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                10

// Scan duration in 0.625ms
#define DEFAULT_SCAN_DURATION               2400

// Connection min interval in 1.25ms
#define DEFAULT_MIN_CONNECTION_INTERVAL     20

// Connection max interval in 1.25ms
#define DEFAULT_MAX_CONNECTION_INTERVAL     100

// Connection supervision timeout in 10ms
#define DEFAULT_CONNECTION_TIMEOUT          100

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE              DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN       TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST        FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE        FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST             FALSE

// Default read RSSI period in 0.625ms
#define DEFAULT_RSSI_PERIOD                 2400

// Minimum connection interval (units of 1.25ms)
// 连接间隔就是指在一个连接事件（ Connection events ）的开始到下一个连接事件（Connection events）的开始的时间间隔。连接间隔的范围是6 ~ 3200既7.5ms ~ 4s之间。单位1.25ms
// Connection Interval 缩短，Master和Slave通信更加频繁，提高数据吞吐速度，缩短了数据发送的时间，当然也增加了功耗。
// Connection Interval 增长，通信频率降低，数据吞吐速度降低，增加了数据发送的时间，当然，这种设置降低了功耗。
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL    20

// Maximum connection interval (units of 1.25ms)
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL    100

// Slave latency to use parameter update
// 允许Slave（从设备）在没有数据要发的情况下，跳过一定数目的连接事件。在这些连接事件（Connection events）中不必回复Master（主设备）的包，这样就能更加省电
// Slave Latency减少或者设置为 0，每次从机Connection Events中都需要回复Master的包，功耗会上升，数据发送速度会提高。
// Slave Latency加长，功耗下降，数据发送速度降低。
#define DEFAULT_UPDATE_SLAVE_LATENCY        0

// Supervision timeout value (units of 10ms)
// 超过此值 连接断开 单位10ms
// 此值应大于 （1 + slave Latency ）* （ connection Interval ）
#define DEFAULT_UPDATE_CONN_TIMEOUT         600

// Default passcode
#define DEFAULT_PASSCODE                    0

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                   TRUE

// Default bonding mode, TRUE to bond, max bonding 6 devices
#define DEFAULT_BONDING_MODE                TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES             GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

// Default service discovery timer delay in 0.625ms
#define DEFAULT_SVC_DISCOVERY_DELAY         1600

// Default parameter update delay in 0.625ms
#define DEFAULT_PARAM_UPDATE_DELAY          3200

// Default phy update delay in 0.625ms
#define DEFAULT_PHY_UPDATE_DELAY            2400

// Default read or write timer delay in 0.625ms
#define DEFAULT_READ_OR_WRITE_DELAY         1600

// Default write CCCD delay in 0.625ms
#define DEFAULT_WRITE_CCCD_DELAY            1600

// Establish link timeout in 0.625ms
#define ESTABLISH_LINK_TIMEOUT              3200

// Application states
enum
{
	BLE_STATE_IDLE,
	BLE_STATE_CONNECTING,
	BLE_STATE_CONNECTED,
	BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
	BLE_DISC_STATE_IDLE, // Idle
	BLE_DISC_STATE_SVC,  // Service discovery
	BLE_DISC_STATE_CHAR, // Characteristic discovery
	BLE_DISC_STATE_CCCD  // client characteristic configuration discovery
};
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for internal task/event processing
static uint8_t centralTaskId;

// Number of scan results
static uint8_t centralScanRes;

// Scan result list
static gapDevRec_t centralDevList[DEFAULT_MAX_SCAN_RES];

// Peer device address
// Device 4 - Addr 4e a1 86 52 e4 c3, RSSI -26 dBm
static uint8_t PeerAddrDef[B_ADDR_LEN] = {0x0, 0xa1, 0x86, 0x52, 0xe4, 0xc3};

// RSSI polling state
static uint8_t centralRssi = TRUE;

// Parameter update state
static uint8_t centralParamUpdate = TRUE;

// Phy update state
static uint8_t centralPhyUpdate = FALSE;

// Connection handle of current connection
static uint16_t centralConnHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8_t centralState = BLE_STATE_IDLE;

// Discovery state
static uint8_t centralDiscState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16_t centralSvcStartHdl = 0;
static uint16_t centralSvcEndHdl = 0;

// Discovered characteristic handle
static uint16_t centralCharHdl = 0;

// Discovered Client Characteristic Configuration handle
static uint16_t centralCCCDHdl = 0;

// Value to write
static uint8_t centralCharVal = 0x5A;

// Value read/write toggle
static uint8_t centralDoWrite = TRUE;

// GATT read/write procedure state
static uint8_t centralProcedureInProgress = FALSE;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void centralProcessGATTMsg(gattMsgEvent_t *pMsg);
static void centralRssiCB(uint16_t connHandle, int8_t rssi);
static void centralEventCB(gapRoleEvent_t *pEvent);
static void centralHciMTUChangeCB(uint16_t connHandle, uint16_t maxTxOctets, uint16_t maxRxOctets);
static void centralPasscodeCB(uint8_t *deviceAddr, uint16_t connectionHandle, uint8_t uiInputs, uint8_t uiOutputs);
static void centralPairStateCB(uint16_t connHandle, uint8_t state, uint8_t status);
static void central_ProcessTMOSMsg(tmos_event_hdr_t *pMsg);
static void centralGATTDiscoveryEvent(gattMsgEvent_t *pMsg);
static void centralAddDeviceInfo(uint8_t *pAddr, uint8_t addrType, int8_t rssi);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
// GAP 相关的回调
static gapCentralRoleCB_t centralRoleCB = {
	centralRssiCB,        // RSSI callback
	centralEventCB,       // Event callback
	centralHciMTUChangeCB // MTU change callback
};

// Bond Manager Callbacks
// 配对 和 绑定 相关的回调
static gapBondCBs_t centralBondCB = {
	centralPasscodeCB,
	centralPairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

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

	// Setup GAP
	GAP_SetParamValue(TGAP_DISC_SCAN, DEFAULT_SCAN_DURATION);
	GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, DEFAULT_MIN_CONNECTION_INTERVAL);
	GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, DEFAULT_MAX_CONNECTION_INTERVAL);
	GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, DEFAULT_CONNECTION_TIMEOUT);	
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
		PRINT("主回调 SYS_EVENT_MSG\r\n");
		if((pMsg = tmos_msg_receive(centralTaskId)) != NULL)
		{
			central_ProcessTMOSMsg((tmos_event_hdr_t *)pMsg);
			tmos_msg_deallocate(pMsg);
		}
		return (events ^ SYS_EVENT_MSG);
	}
	if(events & START_DEVICE_EVT)
	{
		// Start the Device
		PRINT("主回调 START_DEVICE_EVT\r\n");
		GAPRole_CentralStartDevice(centralTaskId, &centralBondCB, &centralRoleCB);
		return (events ^ START_DEVICE_EVT);
	}
	if(events & ESTABLISH_LINK_TIMEOUT_EVT)
	{
		PRINT("主回调 ESTABLISH_LINK_TIMEOUT_EVT\r\n");
		GAPRole_TerminateLink(INVALID_CONNHANDLE);
		return (events ^ ESTABLISH_LINK_TIMEOUT_EVT);
	}	
	if(events & START_SVC_DISCOVERY_EVT)
	{
		// start service discovery
		PRINT("主回调 搜索枚举服务任务 开始\r\n");
		{
			uint8_t uuid[ATT_BT_UUID_SIZE] = {LO_UINT16(SIMPLEPROFILE_SERV_UUID), HI_UINT16(SIMPLEPROFILE_SERV_UUID)};
			// Initialize cached handles
			centralSvcStartHdl = centralSvcEndHdl = centralCharHdl = 0;
			centralDiscState = BLE_DISC_STATE_SVC;
			// Discovery simple BLE service
			// 搜索此UUID的服务
			GATT_DiscPrimaryServiceByUUID(centralConnHandle, uuid, ATT_BT_UUID_SIZE, centralTaskId);
		}
		return (events ^ START_SVC_DISCOVERY_EVT);
	}	
	if(events & START_PARAM_UPDATE_EVT)
	{
		// start connect parameter update
		PRINT("主回调 START_PARAM_UPDATE_EVT\r\n");
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
		PRINT("主回调 START_PHY_UPDATE_EVT\r\n");
		PRINT("PHY Update %x...\r\n", GAPRole_UpdatePHY(centralConnHandle, 0, GAP_PHY_BIT_LE_2M, GAP_PHY_BIT_LE_2M, GAP_PHY_OPTIONS_NOPRE));
		return (events ^ START_PHY_UPDATE_EVT);
	}
	if(events & START_READ_OR_WRITE_EVT)
	{
		PRINT("主回调 START_READ_OR_WRITE_EVT\r\n");
		if(centralProcedureInProgress == FALSE)
		{
			if(centralDoWrite)
			{
				// Do a write
				attWriteReq_t req;
				req.cmd = FALSE;
				req.sig = FALSE;
				req.handle = centralCharHdl;
				req.len = 1;
				req.pValue = GATT_bm_alloc(centralConnHandle, ATT_WRITE_REQ, req.len, NULL, 0);
				if(req.pValue != NULL)
				{
					*req.pValue = centralCharVal;
					if(GATT_WriteCharValue(centralConnHandle, &req, centralTaskId) == SUCCESS)
					{
						centralProcedureInProgress = TRUE;
						centralDoWrite = !centralDoWrite;
						tmos_start_task(centralTaskId, START_READ_OR_WRITE_EVT, DEFAULT_READ_OR_WRITE_DELAY);
					}
					else
					{
						GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
					}
				}
			}
			else
			{
				// Do a read
				attReadReq_t req;
				req.handle = centralCharHdl;
				if(GATT_ReadCharValue(centralConnHandle, &req, centralTaskId) == SUCCESS)
				{
					centralProcedureInProgress = TRUE;
					centralDoWrite = !centralDoWrite;
				}
			}
		}
		return (events ^ START_READ_OR_WRITE_EVT);
	}
	if(events & START_WRITE_CCCD_EVT)
	{
		PRINT("主回调 START_WRITE_CCCD_EVT\r\n");
		if(centralProcedureInProgress == FALSE)
		{
			// Do a write
			attWriteReq_t req;
			req.cmd = FALSE;
			req.sig = FALSE;
			req.handle = centralCCCDHdl;
			req.len = 2;
			req.pValue = GATT_bm_alloc(centralConnHandle, ATT_WRITE_REQ, req.len, NULL, 0);
			if(req.pValue != NULL)
			{
				req.pValue[0] = 1;
				req.pValue[1] = 0;	
				if(GATT_WriteCharValue(centralConnHandle, &req, centralTaskId) == SUCCESS)
				{
					centralProcedureInProgress = TRUE;
				}
				else
				{
					GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
				}
			}
		}
		return (events ^ START_WRITE_CCCD_EVT);
	}
	if(events & START_READ_RSSI_EVT)
	{
		// PRINT("主回调 START_READ_RSSI_EVT\r\n");
		GAPRole_ReadRssiCmd(centralConnHandle);
		tmos_start_task(centralTaskId, START_READ_RSSI_EVT, DEFAULT_RSSI_PERIOD);
		return (events ^ START_READ_RSSI_EVT);
	}
	// Discard unknown events
	return 0;
}

/*********************************************************************
 * @fn      central_ProcessTMOSMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void central_ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
	switch(pMsg->event)
	{
		case GATT_MSG_EVENT:
		{
			PRINT("收到消息 GATT消息\r\n");
			centralProcessGATTMsg((gattMsgEvent_t *)pMsg);
			break;
		}
		default:
		{
			PRINT("收到消息 GATT消息 unknown %d\r\n", pMsg->event);
		}
	}
}

/*********************************************************************
 * @fn      centralProcessGATTMsg
 *
 * @brief   Process GATT messages
 * GATT 读取从设备发来的数据
 *
 * @return  none
 */
static void centralProcessGATTMsg(gattMsgEvent_t *pMsg)
{
	// 如果设备当前不在已连接状态，则忽略所有GATT消息，释放消息内存并返回。
	if(centralState != BLE_STATE_CONNECTED)
	{
		PRINT("centralProcessGATTMsg !=GATT_MSG_EVENT\r\n");
		GATT_bm_free(&pMsg->msg, pMsg->method);
		return;
	}
	// 如果消息是MTU交换响应 或 MTU交换请求的错误响应，则处理MTU交换。
	if((pMsg->method == ATT_EXCHANGE_MTU_RSP) ||
		((pMsg->method == ATT_ERROR_RSP) && (pMsg->msg.errorRsp.reqOpcode == ATT_EXCHANGE_MTU_REQ)))
	{
		PRINT("收到消息 MTU交换响应 处理MTU交换\r\n");
		// PRINT("centralProcessGATTMsg (ATT_EXCHANGE_MTU_RSP||(ATT_ERROR_RSP&&ATT_EXCHANGE_MTU_REQ))\r\n");
		if(pMsg->method == ATT_ERROR_RSP)
		{
			uint8_t status = pMsg->msg.errorRsp.errCode;
			PRINT("Exchange MTU Error: %x\r\n", status);
		}
		centralProcedureInProgress = FALSE;
	}
	// 如果消息是MTU更新事件，则打印新的MTU值。
	if(pMsg->method == ATT_MTU_UPDATED_EVENT)
	{
		PRINT("centralProcessGATTMsg ATT_MTU_UPDATED_EVENT\r\n");
		PRINT("MTU: %d\r\n", pMsg->msg.mtuEvt.MTU);
	}
	// 如果消息是读取响应 或 读取请求的错误响应，则处理读取结果。
	if((pMsg->method == ATT_READ_RSP) ||
		((pMsg->method == ATT_ERROR_RSP) && (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
	{
		if(pMsg->method == ATT_ERROR_RSP)
		{
			PRINT("centralProcessGATTMsg ATT_ERROR_RSP\r\n");
			uint8_t status = pMsg->msg.errorRsp.errCode;
			PRINT("Read Error: %x\r\n", status);
		}
		else
		{
			PRINT("centralProcessGATTMsg ATT_READ_RSP\r\n");
			// After a successful read, display the read value
			PRINT("Read rsp: %x\r\n", *pMsg->msg.readRsp.pValue);
		}
		centralProcedureInProgress = FALSE;
	}
	// 如果消息是写入响应 或 写入请求的错误响应，则处理写入结果。
	else if((pMsg->method == ATT_WRITE_RSP) ||
			((pMsg->method == ATT_ERROR_RSP) && (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
	{
		if(pMsg->method == ATT_ERROR_RSP)
		{
			PRINT("centralProcessGATTMsg ATT_ERROR_RSP2\r\n");
			uint8_t status = pMsg->msg.errorRsp.errCode;
			PRINT("Write Error: %x\r\n", status);
		}
		else
		{
			PRINT("centralProcessGATTMsg ATT_WRITE_RSP\r\n");
			// Write success
			PRINT("Write success \r\n");
		}
		centralProcedureInProgress = FALSE;
	}
	// 如果消息是特性值通知，则打印接收到的通知值。
	else if(pMsg->method == ATT_HANDLE_VALUE_NOTI)
	{
		// 接收到 数据 通知 指示
		PRINT("centralProcessGATTMsg ATT_HANDLE_VALUE_NOTI\r\n");
		PRINT("Receive noti: %x\r\n", *pMsg->msg.handleValueNoti.pValue);
	}
	// 如果设备正在进行服务或特征发现，则调用 centralGATTDiscoveryEvent 函数处理发现事件。
	else if(centralDiscState != BLE_DISC_STATE_IDLE)
	{
		PRINT("centralProcessGATTMsg != BLE_DISC_STATE_IDLE\r\n");
		centralGATTDiscoveryEvent(pMsg);
	}
	GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      centralRssiCB
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
static void centralRssiCB(uint16_t connHandle, int8_t rssi)
{
	PRINT("RSSI : -%d dB \r\n", -rssi);
}

/*********************************************************************
 * @fn      centralHciMTUChangeCB
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
static void centralHciMTUChangeCB(uint16_t connHandle, uint16_t maxTxOctets, uint16_t maxRxOctets)
{
	PRINT(" HCI data length changed, Tx: %d, Rx: %d\r\n", maxTxOctets, maxRxOctets);
	centralProcedureInProgress = TRUE;
}



/*********************************************************************
 * @fn      centralEventCB
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
static void centralEventCB(gapRoleEvent_t *pEvent)
{
	switch(pEvent->gap.opcode)
	{
		// 设备初始化完成事件
		case GAP_DEVICE_INIT_DONE_EVENT:
		{
			PRINT("探测设备 初始化结束 开始探测设备\r\n");
			GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
										DEFAULT_DISCOVERY_ACTIVE_SCAN,
										DEFAULT_DISCOVERY_WHITE_LIST);
			break;
		}
		// 蓝牙设备扫描过程中触发，一次扫描过程中会被触发多次。每当扫描到一个新设备或接收到一个设备的广播数据包时，都会触发这个事件。 GAPRole_CentralStartDiscovery 触发
		case GAP_DEVICE_INFO_EVENT:
		{
			// PRINT("GAP_DEVICE_INFO_EVENT 发现新设备\r\n");
			// Add device to list
			centralAddDeviceInfo(pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType, pEvent->deviceInfo.rssi);
			break;
		}
		// 蓝牙设备扫描结束触发 GAPRole_CentralStartDiscovery 触发
		case GAP_DEVICE_DISCOVERY_EVENT:
		{
			PRINT("探测设备 结束\r\n");
			uint8_t i;	
			for(i = 0; i < centralScanRes; i++)
			{
				if(tmos_memcmp(PeerAddrDef, centralDevList[i].addr, B_ADDR_LEN))
					break;
			}

			// Peer device not found
			if(i == centralScanRes)
			{
				// PRINT("Device not found... Discovering...\r\n");
				centralScanRes = 0;
				PRINT("探测设备 未找到设备 开始重新探测设备\r\n");
				GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
											DEFAULT_DISCOVERY_ACTIVE_SCAN,
											DEFAULT_DISCOVERY_WHITE_LIST);
			}
			// Peer device found
			else
			{
				// 连接第一步
				PRINT("找到设备 MAC %x-%x-%x-%x-%x-%x 尝试连接\r\n", PeerAddrDef[0], PeerAddrDef[1], PeerAddrDef[2], PeerAddrDef[3], PeerAddrDef[4], PeerAddrDef[5]);
				GAPRole_CentralEstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
											DEFAULT_LINK_WHITE_LIST,
											centralDevList[i].addrType,
											centralDevList[i].addr);
				tmos_start_task(centralTaskId, ESTABLISH_LINK_TIMEOUT_EVT, ESTABLISH_LINK_TIMEOUT);
			}
			break;
		}
		case GAP_LINK_ESTABLISHED_EVENT:
		{
			tmos_stop_task(centralTaskId, ESTABLISH_LINK_TIMEOUT_EVT);
			if(pEvent->gap.hdr.status == SUCCESS)
			{
				PRINT("         MAC %x-%x-%x-%x-%x-%x 连接成功\r\n", PeerAddrDef[0], PeerAddrDef[1], PeerAddrDef[2], PeerAddrDef[3], PeerAddrDef[4], PeerAddrDef[5]);
				centralState = BLE_STATE_CONNECTED;
				centralConnHandle = pEvent->linkCmpl.connectionHandle;
				centralProcedureInProgress = TRUE;

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
				// See if initiate phy update
				if(centralPhyUpdate)
				{
					tmos_start_task(centralTaskId, START_PHY_UPDATE_EVT, DEFAULT_PHY_UPDATE_DELAY);
				}
				// See if start RSSI polling
				if(centralRssi)
				{
					tmos_start_task(centralTaskId, START_READ_RSSI_EVT, DEFAULT_RSSI_PERIOD);
				}
			}
			else
			{
				PRINT("     MAC %x-%x-%x-%x-%x-%x 连接失败 Reason:%X. 开始探测设备\r\n", PeerAddrDef[0], PeerAddrDef[1], PeerAddrDef[2], PeerAddrDef[3], PeerAddrDef[4], PeerAddrDef[5],pEvent->gap.hdr.status);
				centralScanRes = 0;
				GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
											DEFAULT_DISCOVERY_ACTIVE_SCAN,
											DEFAULT_DISCOVERY_WHITE_LIST);
			}
			break;
		}
		case GAP_LINK_TERMINATED_EVENT:
		{
			PRINT("centralProcessGATTMsg GAP_LINK_TERMINATED_EVENT\r\n");
			centralState = BLE_STATE_IDLE;
			centralConnHandle = GAP_CONNHANDLE_INIT;
			centralDiscState = BLE_DISC_STATE_IDLE;
			centralCharHdl = 0;
			centralScanRes = 0;
			centralProcedureInProgress = FALSE;
			tmos_stop_task(centralTaskId, START_READ_RSSI_EVT);
			PRINT("Disconnected...Reason:%x\r\n", pEvent->linkTerminate.reason);
			// PRINT("Discovering...\r\n");
			PRINT("探测设备 开始\r\n");
			GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
										DEFAULT_DISCOVERY_ACTIVE_SCAN,
										DEFAULT_DISCOVERY_WHITE_LIST);
			break;
		}
		case GAP_LINK_PARAM_UPDATE_EVENT:
		{
			PRINT("centralProcessGATTMsg GAP_LINK_PARAM_UPDATE_EVENT\r\n");
			// 连接第三步
			PRINT("Param Update...\r\n");
			break;
		}
		case GAP_PHY_UPDATE_EVENT:
		{
			PRINT("centralProcessGATTMsg GAP_PHY_UPDATE_EVENT\r\n");
			PRINT("PHY Update...\r\n");
			break;
		}
		case GAP_EXT_ADV_DEVICE_INFO_EVENT:
		{
			PRINT("centralProcessGATTMsg GAP_EXT_ADV_DEVICE_INFO_EVENT\r\n");
			// Display device addr
			PRINT("Recv ext adv \r\n");
			// Add device to list
			centralAddDeviceInfo(pEvent->deviceExtAdvInfo.addr, pEvent->deviceExtAdvInfo.addrType, pEvent->deviceExtAdvInfo.rssi);
			break;
		}
		case GAP_DIRECT_DEVICE_INFO_EVENT:
		{
			PRINT("centralProcessGATTMsg GAP_DIRECT_DEVICE_INFO_EVENT\r\n");
			// Display device addr
			PRINT("Recv direct adv \r\n");
			// Add device to list
			centralAddDeviceInfo(pEvent->deviceDirectInfo.addr, pEvent->deviceDirectInfo.addrType, pEvent->deviceExtAdvInfo.rssi);
			break;
		}
		default:
			PRINT("centralProcessGATTMsg default\r\n");
			break;
	}
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 * 当配对过程的状态发生变化时，会调用这个回调函数。
 * 这个回调函数允许应用程序处理不同的配对状态，例如配对成功或失败。
 * 应用程序可以根据配对状态和结果采取相应的措施，例如显示配对状态或执行后续操作。
 *
 * @return  none
 */
static void centralPairStateCB(uint16_t connHandle, uint8_t state, uint8_t status)
{
	PRINT("centralPairStateCB\r\n");
	if(state == GAPBOND_PAIRING_STATE_STARTED)
	{
		PRINT("Pairing started:%d\r\n", status);
	}
	else if(state == GAPBOND_PAIRING_STATE_COMPLETE)
	{
		if(status == SUCCESS)
		{
			PRINT("Pairing success\r\n");
		}
		else
		{
			PRINT("Pairing fail\r\n");
		}
	}
	else if(state == GAPBOND_PAIRING_STATE_BONDED)
	{
		if(status == SUCCESS)
		{
			PRINT("Bonding success\r\n");
		}
	}
	else if(state == GAPBOND_PAIRING_STATE_BOND_SAVED)
	{
		if(status == SUCCESS)
		{
			PRINT("Bond save success\r\n");
		}
		else
		{
			PRINT("Bond save failed: %d\r\n", status);
		}
	}
}

/*********************************************************************
 * @fn      centralPasscodeCB
 *
 * @brief   Passcode callback.
 * 当设备要求输入配对码时，会调用这个回调函数。
 * 这个回调函数允许应用程序提供配对码，以便继续配对过程。
 * 应用程序可以根据输入和输出能力显示或请求用户输入配对码。
 *
 * @return  none
 */
static void centralPasscodeCB(uint8_t *deviceAddr, uint16_t connectionHandle, uint8_t uiInputs, uint8_t uiOutputs)
{
	PRINT("centralPasscodeCB\r\n");
	uint32_t passcode;
	// Create random passcode
	passcode = tmos_rand();
	passcode %= 1000000;
	// Display passcode to user
	if(uiOutputs != 0)
	{
		PRINT("Passcode:%06d\r\n", (int)passcode);
	}
	// Send passcode response
	GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      centralGATTDiscoveryEvent
 * centralGATTDiscoveryEvent 函数用于处理GATT（通用属性协议）发现事件。这个函数主要负责处理来自从设备的服务和特征发现过程，包括服务发现、特征发现和描述符发现。
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void centralGATTDiscoveryEvent(gattMsgEvent_t *pMsg)
{
	PRINT("centralGATTDiscoveryEvent\r\n");
	attReadByTypeReq_t req;
	if(centralDiscState == BLE_DISC_STATE_SVC)
	{
		// 如果消息是服务发现响应 ATT_FIND_BY_TYPE_VALUE_RSP, 则遍历所有发现的服务，并打印每个服务的起始句柄和结束句柄。
		PRINT("centralGATTDiscoveryEvent BLE_DISC_STATE_SVC\r\n");
		// Service found, store handles
		if(pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP && pMsg->msg.findByTypeValueRsp.numInfo > 0)
		{
			centralSvcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
			centralSvcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);	
			// Display Profile Service handle range
			PRINT("Found Profile Service handle : %x ~ %x \r\n", centralSvcStartHdl, centralSvcEndHdl);
		}
		// If procedure complete
		if((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
			pMsg->hdr.status == bleProcedureComplete) || (pMsg->method == ATT_ERROR_RSP))
		{
			if(centralSvcStartHdl != 0)
			{
				// Discover characteristic
				centralDiscState = BLE_DISC_STATE_CHAR;
				req.startHandle = centralSvcStartHdl;
				req.endHandle = centralSvcEndHdl;
				req.type.len = ATT_BT_UUID_SIZE;
				req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
				req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);
				GATT_ReadUsingCharUUID(centralConnHandle, &req, centralTaskId);
			}
		}
	}
	else if(centralDiscState == BLE_DISC_STATE_CHAR)
	{
		// 如果消息是特征发现响应 ATT_READ_BY_TYPE_RSP, 则遍历所有发现的特征，并打印每个特征的句柄和值句柄。
		PRINT("centralGATTDiscoveryEvent BLE_DISC_STATE_CHAR\r\n");
		// 特征句柄的获取
		// Characteristic found, store handle
		if(pMsg->method == ATT_READ_BY_TYPE_RSP && pMsg->msg.readByTypeRsp.numPairs > 0)
		{
			centralCharHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0], pMsg->msg.readByTypeRsp.pDataList[1]);	
			// 开启读写任务
			tmos_start_task(centralTaskId, START_READ_OR_WRITE_EVT, DEFAULT_READ_OR_WRITE_DELAY);	
			// Display Characteristic 1 handle
			PRINT("Found Characteristic 1 handle : %x \r\n", centralCharHdl);
		}
		if((pMsg->method == ATT_READ_BY_TYPE_RSP &&
			pMsg->hdr.status == bleProcedureComplete) || (pMsg->method == ATT_ERROR_RSP))
		{
			// 订阅通知：获取 CCCD 句柄
			// Discover characteristic
			centralDiscState = BLE_DISC_STATE_CCCD;
			req.startHandle = centralSvcStartHdl;
			req.endHandle = centralSvcEndHdl;
			req.type.len = ATT_BT_UUID_SIZE;
			req.type.uuid[0] = LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID);
			req.type.uuid[1] = HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID);
			GATT_ReadUsingCharUUID(centralConnHandle, &req, centralTaskId);
		}
	}
	else if(centralDiscState == BLE_DISC_STATE_CCCD)
	{
		PRINT("centralGATTDiscoveryEvent BLE_DISC_STATE_CCCD\r\n");
		// Characteristic found, store handle
		if(pMsg->method == ATT_READ_BY_TYPE_RSP && pMsg->msg.readByTypeRsp.numPairs > 0)
		{
			centralCCCDHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0], pMsg->msg.readByTypeRsp.pDataList[1]);
			centralProcedureInProgress = FALSE;
			// Start do write CCCD
			tmos_start_task(centralTaskId, START_WRITE_CCCD_EVT, DEFAULT_WRITE_CCCD_DELAY);	
			// Display Characteristic 1 handle
			PRINT("Found client characteristic configuration handle : %x \r\n", centralCCCDHdl);
		}
		centralDiscState = BLE_DISC_STATE_IDLE;
	}
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
	// PRINT("centralAddDeviceInfo\r\n");
	uint8_t i;
	// If result count not at max
	if(centralScanRes < DEFAULT_MAX_SCAN_RES)
	{
		// Check if device is already in scan results
		for(i = 0; i < centralScanRes; i++)
		{
			if(tmos_memcmp(pAddr, centralDevList[i].addr, B_ADDR_LEN))
			{
				centralDevList[i].rssi = rssi;
				return;
			}
		}
		// Add addr to scan result list
		tmos_memcpy(centralDevList[centralScanRes].addr, pAddr, B_ADDR_LEN);
		centralDevList[centralScanRes].addrType = addrType;
		centralDevList[centralScanRes].rssi = rssi;
		centralScanRes++;

		if(rssi > -65)
		{
			PRINT("发现新设备 连接到 connect to MAC %x-%x-%x-%x-%x-%x, RSSI %d dBm\r\n", pAddr[0], pAddr[1], pAddr[2], pAddr[3], pAddr[4], pAddr[5], rssi);
			memcpy(PeerAddrDef, pAddr, B_ADDR_LEN);
		}
	}
	// if((rssi > -35)
	//  && !tmos_memcmp(PeerAddrDef, pAddr, B_ADDR_LEN)
	// )
	// {
	// 	memcpy(PeerAddrDef, pAddr, B_ADDR_LEN);
	// 	PRINT("connect to MAC %x-%x-%x-%x-%x-%x, RSSI %d dBm\r\n",
	// 		pAddr, pAddr+1, pAddr+2, pAddr+3, pAddr+4, pAddr+5,
	// 		centralDevList[centralScanRes - 1].rssi);
	// }
}

/************************ endfile @ central **************************/
