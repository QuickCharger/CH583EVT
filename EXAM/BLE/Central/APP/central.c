/********************************** (C) COPYRIGHT *******************************
* File Name          : central.c
* Author             : WCH
* Version            : V1.1
* Date               : 2020/08/06
* Description        : �������̣�����ɨ����Χ�豸�������������Ĵӻ��豸��ַ��
*                      Ѱ���Զ������������ִ�ж�д�������ӻ��������ʹ��,
                       �����ӻ��豸��ַ�޸�Ϊ������Ŀ���ַ��Ĭ��Ϊ(84:C2:E4:03:02:02)
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
// ���Ӽ������ָ��һ�������¼��� Connection events ���Ŀ�ʼ����һ�������¼���Connection events���Ŀ�ʼ��ʱ���������Ӽ���ķ�Χ��6 ~ 3200��7.5ms ~ 4s֮�䡣��λ1.25ms
// Connection Interval ���̣�Master��Slaveͨ�Ÿ���Ƶ����������������ٶȣ����������ݷ��͵�ʱ�䣬��ȻҲ�����˹��ġ�
// Connection Interval ������ͨ��Ƶ�ʽ��ͣ����������ٶȽ��ͣ����������ݷ��͵�ʱ�䣬��Ȼ���������ý����˹��ġ�
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL    20

// Maximum connection interval (units of 1.25ms)
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL    100

// Slave latency to use parameter update
// ����Slave�����豸����û������Ҫ��������£�����һ����Ŀ�������¼�������Щ�����¼���Connection events���в��ػظ�Master�����豸���İ����������ܸ���ʡ��
// Slave Latency���ٻ�������Ϊ 0��ÿ�δӻ�Connection Events�ж���Ҫ�ظ�Master�İ������Ļ����������ݷ����ٶȻ���ߡ�
// Slave Latency�ӳ��������½������ݷ����ٶȽ��͡�
#define DEFAULT_UPDATE_SLAVE_LATENCY        0

// Supervision timeout value (units of 10ms)
// ������ֵ ���ӶϿ� ��λ10ms
// ��ֵӦ���� ��1 + slave Latency ��* �� connection Interval ��
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
// GAP ��صĻص�
static gapCentralRoleCB_t centralRoleCB = {
	centralRssiCB,        // RSSI callback
	centralEventCB,       // Event callback
	centralHciMTUChangeCB // MTU change callback
};

// Bond Manager Callbacks
// ��� �� �� ��صĻص�
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
 * ���ص�
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
		PRINT("���ص� SYS_EVENT_MSG\r\n");
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
		PRINT("���ص� START_DEVICE_EVT\r\n");
		GAPRole_CentralStartDevice(centralTaskId, &centralBondCB, &centralRoleCB);
		return (events ^ START_DEVICE_EVT);
	}
	if(events & ESTABLISH_LINK_TIMEOUT_EVT)
	{
		PRINT("���ص� ESTABLISH_LINK_TIMEOUT_EVT\r\n");
		GAPRole_TerminateLink(INVALID_CONNHANDLE);
		return (events ^ ESTABLISH_LINK_TIMEOUT_EVT);
	}	
	if(events & START_SVC_DISCOVERY_EVT)
	{
		// start service discovery
		PRINT("���ص� ����ö�ٷ������� ��ʼ\r\n");
		{
			uint8_t uuid[ATT_BT_UUID_SIZE] = {LO_UINT16(SIMPLEPROFILE_SERV_UUID), HI_UINT16(SIMPLEPROFILE_SERV_UUID)};
			// Initialize cached handles
			centralSvcStartHdl = centralSvcEndHdl = centralCharHdl = 0;
			centralDiscState = BLE_DISC_STATE_SVC;
			// Discovery simple BLE service
			// ������UUID�ķ���
			GATT_DiscPrimaryServiceByUUID(centralConnHandle, uuid, ATT_BT_UUID_SIZE, centralTaskId);
		}
		return (events ^ START_SVC_DISCOVERY_EVT);
	}	
	if(events & START_PARAM_UPDATE_EVT)
	{
		// start connect parameter update
		PRINT("���ص� START_PARAM_UPDATE_EVT\r\n");
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
		PRINT("���ص� START_PHY_UPDATE_EVT\r\n");
		PRINT("PHY Update %x...\r\n", GAPRole_UpdatePHY(centralConnHandle, 0, GAP_PHY_BIT_LE_2M, GAP_PHY_BIT_LE_2M, GAP_PHY_OPTIONS_NOPRE));
		return (events ^ START_PHY_UPDATE_EVT);
	}
	if(events & START_READ_OR_WRITE_EVT)
	{
		PRINT("���ص� START_READ_OR_WRITE_EVT\r\n");
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
		PRINT("���ص� START_WRITE_CCCD_EVT\r\n");
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
		// PRINT("���ص� START_READ_RSSI_EVT\r\n");
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
			PRINT("�յ���Ϣ GATT��Ϣ\r\n");
			centralProcessGATTMsg((gattMsgEvent_t *)pMsg);
			break;
		}
		default:
		{
			PRINT("�յ���Ϣ GATT��Ϣ unknown %d\r\n", pMsg->event);
		}
	}
}

/*********************************************************************
 * @fn      centralProcessGATTMsg
 *
 * @brief   Process GATT messages
 * GATT ��ȡ���豸����������
 *
 * @return  none
 */
static void centralProcessGATTMsg(gattMsgEvent_t *pMsg)
{
	// ����豸��ǰ����������״̬�����������GATT��Ϣ���ͷ���Ϣ�ڴ沢���ء�
	if(centralState != BLE_STATE_CONNECTED)
	{
		PRINT("centralProcessGATTMsg !=GATT_MSG_EVENT\r\n");
		GATT_bm_free(&pMsg->msg, pMsg->method);
		return;
	}
	// �����Ϣ��MTU������Ӧ �� MTU��������Ĵ�����Ӧ������MTU������
	if((pMsg->method == ATT_EXCHANGE_MTU_RSP) ||
		((pMsg->method == ATT_ERROR_RSP) && (pMsg->msg.errorRsp.reqOpcode == ATT_EXCHANGE_MTU_REQ)))
	{
		PRINT("�յ���Ϣ MTU������Ӧ ����MTU����\r\n");
		// PRINT("centralProcessGATTMsg (ATT_EXCHANGE_MTU_RSP||(ATT_ERROR_RSP&&ATT_EXCHANGE_MTU_REQ))\r\n");
		if(pMsg->method == ATT_ERROR_RSP)
		{
			uint8_t status = pMsg->msg.errorRsp.errCode;
			PRINT("Exchange MTU Error: %x\r\n", status);
		}
		centralProcedureInProgress = FALSE;
	}
	// �����Ϣ��MTU�����¼������ӡ�µ�MTUֵ��
	if(pMsg->method == ATT_MTU_UPDATED_EVENT)
	{
		PRINT("centralProcessGATTMsg ATT_MTU_UPDATED_EVENT\r\n");
		PRINT("MTU: %d\r\n", pMsg->msg.mtuEvt.MTU);
	}
	// �����Ϣ�Ƕ�ȡ��Ӧ �� ��ȡ����Ĵ�����Ӧ�������ȡ�����
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
	// �����Ϣ��д����Ӧ �� д������Ĵ�����Ӧ������д������
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
	// �����Ϣ������ֵ֪ͨ�����ӡ���յ���ֵ֪ͨ��
	else if(pMsg->method == ATT_HANDLE_VALUE_NOTI)
	{
		// ���յ� ���� ֪ͨ ָʾ
		PRINT("centralProcessGATTMsg ATT_HANDLE_VALUE_NOTI\r\n");
		PRINT("Receive noti: %x\r\n", *pMsg->msg.handleValueNoti.pValue);
	}
	// ����豸���ڽ��з�����������֣������ centralGATTDiscoveryEvent �����������¼���
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
 * ����ص������ڽ��յ� RSSI�������ź�ǿ��ָʾ������ʱ�����á�
 * Ӧ�ó�����Ը��� RSSI ֵ������ͨ�Ų��ԣ�����������书�ʻ��߾����Ƿ񱣳����ӡ�
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
 * ����ص������� MTU��Maximum Transmission Unit������䵥Ԫ����С�����仯ʱ�����á�
 * MTU �����˵��� BLE ���ݰ�������ֽ�����
 * Ӧ�ó�����Ը����µ� MTU ��С�������ݴ�����ԣ����Ż�����Ч�ʡ�
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
 * ����ص������ڷ��� GAP �¼�ʱ�����á�
 * GAP �¼����ܰ������ӽ��������ӶϿ������ַ���ȡ�
 * Ӧ�ó�����Ը��ݾ�����¼����Ͳ�ȡ��Ӧ�Ĵ�ʩ�����磬�����ӽ���ʱ�������������ݴ��䣻�����ӶϿ�ʱ��������������ɨ�衣
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void centralEventCB(gapRoleEvent_t *pEvent)
{
	switch(pEvent->gap.opcode)
	{
		// �豸��ʼ������¼�
		case GAP_DEVICE_INIT_DONE_EVENT:
		{
			PRINT("̽���豸 ��ʼ������ ��ʼ̽���豸\r\n");
			GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
										DEFAULT_DISCOVERY_ACTIVE_SCAN,
										DEFAULT_DISCOVERY_WHITE_LIST);
			break;
		}
		// �����豸ɨ������д�����һ��ɨ������лᱻ������Ρ�ÿ��ɨ�赽һ�����豸����յ�һ���豸�Ĺ㲥���ݰ�ʱ�����ᴥ������¼��� GAPRole_CentralStartDiscovery ����
		case GAP_DEVICE_INFO_EVENT:
		{
			// PRINT("GAP_DEVICE_INFO_EVENT �������豸\r\n");
			// Add device to list
			centralAddDeviceInfo(pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType, pEvent->deviceInfo.rssi);
			break;
		}
		// �����豸ɨ��������� GAPRole_CentralStartDiscovery ����
		case GAP_DEVICE_DISCOVERY_EVENT:
		{
			PRINT("̽���豸 ����\r\n");
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
				PRINT("̽���豸 δ�ҵ��豸 ��ʼ����̽���豸\r\n");
				GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
											DEFAULT_DISCOVERY_ACTIVE_SCAN,
											DEFAULT_DISCOVERY_WHITE_LIST);
			}
			// Peer device found
			else
			{
				// ���ӵ�һ��
				PRINT("�ҵ��豸 MAC %x-%x-%x-%x-%x-%x ��������\r\n", PeerAddrDef[0], PeerAddrDef[1], PeerAddrDef[2], PeerAddrDef[3], PeerAddrDef[4], PeerAddrDef[5]);
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
				PRINT("         MAC %x-%x-%x-%x-%x-%x ���ӳɹ�\r\n", PeerAddrDef[0], PeerAddrDef[1], PeerAddrDef[2], PeerAddrDef[3], PeerAddrDef[4], PeerAddrDef[5]);
				centralState = BLE_STATE_CONNECTED;
				centralConnHandle = pEvent->linkCmpl.connectionHandle;
				centralProcedureInProgress = TRUE;

				// Update MTU
				attExchangeMTUReq_t req = {
					.clientRxMTU = BLE_BUFF_MAX_LEN - 4,
				};

				GATT_ExchangeMTU(centralConnHandle, &req, centralTaskId);

				// Initiate service discovery
				// ����ö�ٷ�������
				tmos_start_task(centralTaskId, START_SVC_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY);

				// See if initiate connect parameter update
				// �����������Ӳ�������
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
				PRINT("     MAC %x-%x-%x-%x-%x-%x ����ʧ�� Reason:%X. ��ʼ̽���豸\r\n", PeerAddrDef[0], PeerAddrDef[1], PeerAddrDef[2], PeerAddrDef[3], PeerAddrDef[4], PeerAddrDef[5],pEvent->gap.hdr.status);
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
			PRINT("̽���豸 ��ʼ\r\n");
			GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
										DEFAULT_DISCOVERY_ACTIVE_SCAN,
										DEFAULT_DISCOVERY_WHITE_LIST);
			break;
		}
		case GAP_LINK_PARAM_UPDATE_EVENT:
		{
			PRINT("centralProcessGATTMsg GAP_LINK_PARAM_UPDATE_EVENT\r\n");
			// ���ӵ�����
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
 * ����Թ��̵�״̬�����仯ʱ�����������ص�������
 * ����ص���������Ӧ�ó�����ͬ�����״̬��������Գɹ���ʧ�ܡ�
 * Ӧ�ó�����Ը������״̬�ͽ����ȡ��Ӧ�Ĵ�ʩ��������ʾ���״̬��ִ�к���������
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
 * ���豸Ҫ�����������ʱ�����������ص�������
 * ����ص���������Ӧ�ó����ṩ����룬�Ա������Թ��̡�
 * Ӧ�ó�����Ը�����������������ʾ�������û���������롣
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
 * centralGATTDiscoveryEvent �������ڴ���GATT��ͨ������Э�飩�����¼������������Ҫ���������Դ��豸�ķ�����������ֹ��̣����������֡��������ֺ����������֡�
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
		// �����Ϣ�Ƿ�������Ӧ ATT_FIND_BY_TYPE_VALUE_RSP, ��������з��ֵķ��񣬲���ӡÿ���������ʼ����ͽ��������
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
		// �����Ϣ������������Ӧ ATT_READ_BY_TYPE_RSP, ��������з��ֵ�����������ӡÿ�������ľ����ֵ�����
		PRINT("centralGATTDiscoveryEvent BLE_DISC_STATE_CHAR\r\n");
		// ��������Ļ�ȡ
		// Characteristic found, store handle
		if(pMsg->method == ATT_READ_BY_TYPE_RSP && pMsg->msg.readByTypeRsp.numPairs > 0)
		{
			centralCharHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0], pMsg->msg.readByTypeRsp.pDataList[1]);	
			// ������д����
			tmos_start_task(centralTaskId, START_READ_OR_WRITE_EVT, DEFAULT_READ_OR_WRITE_DELAY);	
			// Display Characteristic 1 handle
			PRINT("Found Characteristic 1 handle : %x \r\n", centralCharHdl);
		}
		if((pMsg->method == ATT_READ_BY_TYPE_RSP &&
			pMsg->hdr.status == bleProcedureComplete) || (pMsg->method == ATT_ERROR_RSP))
		{
			// ����֪ͨ����ȡ CCCD ���
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
			PRINT("�������豸 ���ӵ� connect to MAC %x-%x-%x-%x-%x-%x, RSSI %d dBm\r\n", pAddr[0], pAddr[1], pAddr[2], pAddr[3], pAddr[4], pAddr[5], rssi);
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
