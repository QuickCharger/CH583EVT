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

#include "easy.h"
#include "CONFIG.h"
#include "gattprofile.h"
#include "central.h"

// Length of bd addr as a string
#define B_ADDR_STR_LEN                      15

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
	BLE_STATE_CONNECTED,
};

////////////////////////////////////////////// UUID BEGIN

// Discovery states ����UUID���ҽ��
enum
{
	BLE_DISC_STATE_IDLE, // Idle
	BLE_DISC_STATE_ALL_SVC,  // ���ڻ�ȡ����SVC
	BLE_DISC_STATE_SVC,  // ���ڸ���ServiceUUID����handle������ GATT_DiscPrimaryServiceByUUID ���ô�״̬
	BLE_DISC_STATE_CHAR_ALL,  // ���ڸ��� handle ���� charUUID������ GATT_DiscAllChars ���ô�״̬
	BLE_DISC_STATE_CHAR, // ���ڸ���CharacteristicUUID���ҽ�������� GATT_ReadUsingCharUUID ���ô�״̬
	BLE_DISC_STATE_CCCD  // ���ڸ���CCCD(client characteristic configuration discovery)���ҽ���� ���� GATT_ReadUsingCharUUID ���ô�״̬������һ���Ĳ�֮ͬ�����ڷ��͵�UUID��ͬ
};

/**
 * GAP ����Ϣ�� centralRoleCB centralBondCB �Ļص�
 * GATT����Ϣ�� gattCentralMsg
 * 	������Ϣ��ִ�к���					�ص���Ϣ
 * 	GATT_DiscAllPrimaryServices		ATT_READ_BY_GRP_TYPE_RSP
 *  GATT_DiscPrimaryServiceByUUID	ATT_FIND_BY_TYPE_VALUE_RSP
 *  GATT_DiscAllCharDescs			ATT_READ_BY_TYPE_RSP
 *  GATT_ReadUsingCharUUID			ATT_READ_BY_TYPE_RSP
 *  GATT_ReadUsingCharUUID			ATT_READ_BY_TYPE_RSP
*/
uint16_t UUID_SRV_INFO = 0x180A;	// �豸��Ϣ
uint16_t UUID_SRV_HID = 0x1812;		// HID�豸
uint16_t UUID_SRV_BTT = 0x180F;		// �������
uint16_t UUID_char_MOUSE_IN = 0x2A33;	// HID�������
uint16_t UUID_char_BTT = 0x2A19;	// �������

////////////////////////////////////////////// UUID END


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
// �豸�����GAP�ɹ����Ӻ���ֵ
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

static uint16_t jobHandle[10] = {0};
uint8_t curHandle = 0;
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
// GAP ��صĻص�
static gapCentralRoleCB_t centralRoleCB = {
	gapCentralRssiCB,        // RSSI callback
	gapCentralEventCB,       // Event callback
	gapCentralHciMTUChangeCB // MTU change callback
};

// Bond Manager Callbacks
// ��� �� �� ��صĻص�
static gapBondCBs_t centralBondCB = {
	gapCentralPasscodeCB,
	gapCentralPairStateCB
};

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

	for(uint8_t i = 0; i < 10; ++i)
		jobHandle[i] = 0;

	// Setup GAP
	GAP_SetParamValue(TGAP_DISC_SCAN, DEFAULT_SCAN_DURATION);	// ����ɨ��ʱ��
	GAP_SetParamValue(TGAP_CONN_EST_INT_MIN, DEFAULT_MIN_CONNECTION_INTERVAL);
	GAP_SetParamValue(TGAP_CONN_EST_INT_MAX, DEFAULT_MAX_CONNECTION_INTERVAL);
	GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, DEFAULT_CONNECTION_TIMEOUT);	// ���ӳ�ʱʱ��
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
		LOG("���ص� SYS_EVENT_MSG\r\n");
		if((pMsg = tmos_msg_receive(centralTaskId)) != NULL)
		{
			uint8_t e = ((gattMsgEvent_t *)pMsg)->hdr.event;
			if(e == GATT_MSG_EVENT)
			{
				gattCentralMsg((gattMsgEvent_t *)pMsg);
			}
			else
			{
				LOG("  �յ���Ϣ δ֪��Ϣ���� unknown %d\r\n", e);
			}
			tmos_msg_deallocate(pMsg);
		}
		return (events ^ SYS_EVENT_MSG);
	}
	if(events & START_DEVICE_EVT)
	{
		// Start the Device
		LOG("���ص� START_DEVICE_EVT �豸��ʼ�����\r\n");
		GAPRole_CentralStartDevice(centralTaskId, &centralBondCB, &centralRoleCB);
		return (events ^ START_DEVICE_EVT);
	}
	if(events & ESTABLISH_LINK_TIMEOUT_EVT)
	{
		LOG("���ص� ESTABLISH_LINK_TIMEOUT_EVT\r\n");
		GAPRole_TerminateLink(INVALID_CONNHANDLE);
		return (events ^ ESTABLISH_LINK_TIMEOUT_EVT);
	}	
	if(events & START_SVC_DISCOVERY_EVT)
	{
		// start service discovery
		LOG("���ص� START_SVC_DISCOVERY_EVT ����ö�ٷ������� ��ʼ\r\n");
		{
			centralDiscState = BLE_DISC_STATE_ALL_SVC;
			GATT_DiscAllPrimaryServices(centralConnHandle, centralTaskId);
		}
		return (events ^ START_SVC_DISCOVERY_EVT);
	}	
	if(events & START_PARAM_UPDATE_EVT)
	{
		// start connect parameter update 
		LOG("���ص� START_PARAM_UPDATE_EVT\r\n");
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
		LOG("���ص� START_PHY_UPDATE_EVT\r\n");
		bStatus_t t = GAPRole_UpdatePHY(centralConnHandle, 0, GAP_PHY_BIT_LE_2M, GAP_PHY_BIT_LE_2M, GAP_PHY_OPTIONS_NOPRE);
		LOG("PHY Update %x...\r\n", t);
		return (events ^ START_PHY_UPDATE_EVT);
	}
	if(events & START_READ_OR_WRITE_EVT)
	{
		LOG("���ص� START_READ_OR_WRITE_EVT ���������ݴӴӻ���һ��һ��\r\n");
		// if(centralProcedureInProgress == FALSE)
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
			// 			centralProcedureInProgress = TRUE;
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
				uint16_t hdl = 0;
				for(uint8_t i = 0; i < 10; ++i)
				{
					if(jobHandle[i] != 0)
					{
						hdl = jobHandle[i];
						jobHandle[i] = 0;
						curHandle = hdl;
						break;
					}
				}
				if(hdl != 0)
				{
					attReadReq_t req;
					req.handle = hdl;
					// ���ú��յ� GATT_MSG_EVENT �¼��� ATT_READ_RSP ��Ϣ
					LOG("    ����һ����д���� handle 0x%04X\r\n", hdl);
					if(GATT_ReadCharValue(centralConnHandle, &req, centralTaskId) == SUCCESS)
					{
						centralProcedureInProgress = TRUE;
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
		LOG("���ص� START_WRITE_CCCD_EVT ����ʹ�� CCCD\r\n");
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
 * GATT ��ȡ���豸����������
 *
 * @return  none
 */
// ��־ǰ�Ӷ��ո�
static void gattCentralMsg(gattMsgEvent_t *pMsg)
{
	{
		PRINT("GATT ��Ϣ��������. "); BLE_GATT_MSG_DESC(pMsg); PRINT("\r\n");
	}
	// ����豸��ǰ����������״̬�����������GATT��Ϣ���ͷ���Ϣ�ڴ沢���ء�
	if(centralState != BLE_STATE_CONNECTED)
	{
		LOG("  centralState != BLE_STATE_CONNECTED\r\n");
		GATT_bm_free(&pMsg->msg, pMsg->method);
		return;
	}
	// �����Ϣ��MTU������Ӧ �� MTU��������Ĵ�����Ӧ������MTU������
	if((pMsg->method == ATT_EXCHANGE_MTU_RSP) ||
		((pMsg->method == ATT_ERROR_RSP) && (pMsg->msg.errorRsp.reqOpcode == ATT_EXCHANGE_MTU_REQ)))
	{
		if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
		{
			// ��ȡʵ��Э�̵�MTU��С
			uint16_t negotiatedMTU = pMsg->msg.exchangeMTUReq.clientRxMTU;
			LOG("  Negotiated MTU: %d\r\n", negotiatedMTU);
		}
		else
		{
			if(pMsg->method == ATT_ERROR_RSP)
			{
				uint8_t status = pMsg->msg.errorRsp.errCode;
				LOG("  Exchange MTU Error: %x\r\n", status);
			}
			centralProcedureInProgress = FALSE;
		}
	}
	// �����Ϣ��MTU�����¼������ӡ�µ�MTUֵ��
	if(pMsg->method == ATT_MTU_UPDATED_EVENT)
	{
		LOG("  ATT_MTU_UPDATED_EVENT MTU: %d\r\n", pMsg->msg.mtuEvt.MTU);
	}
	// �����Ϣ�Ƕ�ȡ��Ӧ �� ��ȡ����Ĵ�����Ӧ�������ȡ�����
	if((pMsg->method == ATT_READ_RSP)
	|| ((pMsg->method == ATT_ERROR_RSP) && (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
	{
		if(pMsg->method == ATT_ERROR_RSP)
		{
			uint8_t status = pMsg->msg.errorRsp.errCode;
			LOG("  ATT_ERROR_RSP Read Error: %x\r\n", status);
		}
		else
		{
			// After a successful read, display the read value
			LOG("  ATT_READ_RSP Read rsp: %x\r\n", *pMsg->msg.readRsp.pValue);
		}
		centralProcedureInProgress = FALSE;
	}
	// �����Ϣ��д����Ӧ �� д������Ĵ�����Ӧ������д������
	else if((pMsg->method == ATT_WRITE_RSP)
	|| ((pMsg->method == ATT_ERROR_RSP) && (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
	{
		if(pMsg->method == ATT_ERROR_RSP)
		{
			uint8_t status = pMsg->msg.errorRsp.errCode;
			LOG("  ATT_ERROR_RSP Write Error: %x\r\n", status);
		}
		else
		{
			// Write success
			LOG("  ATT_WRITE_RSP Write success\r\n");
		}
		centralProcedureInProgress = FALSE;
	}
	// �����Ϣ������ֵ֪ͨ�����ӡ���յ���ֵ֪ͨ��
	else if(pMsg->method == ATT_HANDLE_VALUE_NOTI)
	{
		// ���յ� ���� ֪ͨ ָʾ
		LOG("  ATT_HANDLE_VALUE_NOTI Receive noti: %x ���յ��ӻ�notify���� \r\n", *pMsg->msg.handleValueNoti.pValue);
	}
	else if(centralDiscState != BLE_DISC_STATE_IDLE)
	{
		// ���ڷ���UUID���ҹ��ܵ�״̬
		// �ڼ���յ�������ݷ��� ���� pMsg->hdr.status == bleProcedureComplete �ж����ݽ������
		attReadByTypeReq_t req;
		if(pMsg->method == ATT_ERROR_RSP)
		{
			LOG("    Error response todo\r\n");
		}
		else if(centralDiscState == BLE_DISC_STATE_ALL_SVC)
		{
			LOG("    ��ǰ״̬ BLE_DISC_STATE_ALL_SVC\r\n");
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
						// if(uuid == UUID_SRV_BTT)
						if(uuid == 0x180A)
						{
							centralSvcStartHdl = startHandle;
							centralSvcEndHdl = endHandle;
						}
					}
				}
				else if(len != 6)
				{
					LOG("    �����128λ��UUID ��ʱ��֧�����ݴ�ӡ todo\r\n");
				}
				
				if(pMsg->hdr.status == bleProcedureComplete)
				{
					centralDiscState = BLE_DISC_STATE_CHAR_ALL;
					LOG("    BLE_DISC_STATE_ALL_SVC ����. ���� BLE_DISC_STATE_CHAR_ALL ״̬\r\n");
					GATT_DiscAllChars(centralConnHandle, centralSvcStartHdl, centralSvcEndHdl, centralTaskId);
				}
			}
		}
		else if(centralDiscState == BLE_DISC_STATE_CHAR_ALL)
		{
			LOG("    ��ǰ״̬ BLE_DISC_STATE_CHAR_ALL\r\n");
			if(pMsg->method == ATT_READ_BY_TYPE_RSP && pMsg->msg.readByTypeRsp.numPairs > 0)
			{
				uint16_t numPairs = pMsg->msg.readByTypeRsp.numPairs;
				uint16_t len = pMsg->msg.readByTypeRsp.len;
				uint8_t *pDataList = pMsg->msg.readByTypeRsp.pDataList;
				for(uint16_t i = 0; i < numPairs; ++i)
				{
					GATT_PROP_WRITE;
					// 12 = GATT_PROP_READ | GATT_PROP_NOTIFY
					uint8_t* u = pDataList + len * i;
					uint16_t declareHandle = BUILD_UINT16(*(u+0), *(u+1));	// �����������
					uint8_t readWrite = *(u + 2);		// ��дȨ�� GATT_PROP_WRITE
					uint16_t handle = BUILD_UINT16(*(u+3), *(u+4));
					uint16_t uuid = BUILD_UINT16(*(u+5), *(u+6));
					uint8_t uuid_len = len - 5;
					LOG("    DeclareHandle 0x%04X readWrite 0x%02X handle 0x%04X uuid 0x%04X %s\r\n", declareHandle, readWrite, handle, uuid, BLE_UUID2str(uuid));
					
					for(uint8_t i = 0; i < 10; ++i)
					{
						if(jobHandle[i] == 0)
						{
							jobHandle[i] = handle;
							LOG("    push handle 0x%04X\r\n", handle);
							break;
						}
					}

					// 0x180A
					// if(uuid == 0x2A23)	// GATT ��Ϣ��������. GATT {method:ATT_READ_RSP, {len:8, pDataList:0x00 00 00 00 00 00 00 00 }} readRsp: ""
					// if(uuid == 0x2A24)	// GATT ��Ϣ��������. GATT {method:ATT_READ_RSP, {len:12, pDataList:0x4d 6f 64 65 6c 20 4e 75 6d 62 65 72 }} readRsp: "Model Number"
					// if(uuid == 0x2A25)	// GATT ��Ϣ��������. GATT {method:ATT_READ_RSP, {len:13, pDataList:0x53 65 72 69 61 6c 20 4e 75 6d 62 65 72 }} readRsp: "Serial Number"
					// if(uuid == 0x2A26)	// GATT ��Ϣ��������. GATT {method:ATT_READ_RSP, {len:17, pDataList:0x46 69 72 6d 77 61 72 65 20 52 65 76 69 73 69 6f 6e }} readRsp: "Firmware Revision"
					// if(uuid == 0x2A27)	// GATT ��Ϣ��������. GATT {method:ATT_READ_RSP, {len:17, pDataList:0x48 61 72 64 77 61 72 65 20 52 65 76 69 73 69 6f 6e }} readRsp: "Hardware Revision"
					// if(uuid == 0x2A28)	// GATT ��Ϣ��������. GATT {method:ATT_READ_RSP, {len:17, pDataList:0x53 6f 66 74 77 61 72 65 20 52 65 76 69 73 69 6f 6e }} readRsp: "Software Revision"
					// if(uuid == 0x2A29)	// GATT ��Ϣ��������. GATT {method:ATT_READ_RSP, {len:17, pDataList:0x4d 61 6e 75 66 61 63 74 75 72 65 72 20 4e 61 6d 65 }} readRsp: "Manufacturer Name"
					// if(uuid == 0x2A2A)	// GATT ��Ϣ��������. GATT {method:ATT_READ_RSP, {len:14, pDataList:0xfe 00 65 78 70 65 72 69 6d 65 6e 74 61 6c }} readRsp: "�6�0xperimental"
					// if(uuid == 0x2A50)	// GATT ��Ϣ��������. GATT {method:ATT_READ_RSP, {len:7, pDataList:0x01 d7 07 00 00 10 01 }} readRsp: "����"

					// 0x1812
					// if(uuid == 0x2A4A)
					// {
					// 	centralCharHdl = handle;
					// 	// ������д����
					// 	LOG("    ����һ����д���� handle 0x%04X\r\n", centralCharHdl);
					// 	tmos_start_task(centralTaskId, START_READ_OR_WRITE_EVT, DEFAULT_READ_OR_WRITE_DELAY);
					// }
				}
			}
			if(pMsg->method == ATT_READ_BY_TYPE_RSP && pMsg->hdr.status == bleProcedureComplete)
			{
				centralDiscState = BLE_DISC_STATE_IDLE;
				LOG("    BLE_DISC_STATE_CHAR_ALL ����. ���� BLE_DISC_STATE_IDLE ״̬\r\n");
				// centralDiscState = BLE_DISC_STATE_CCCD;
				// LOG("    BLE_DISC_STATE_CHAR_ALL ����. ���� BLE_DISC_STATE_CCCD ״̬\r\n");
				// // ����֪ͨ����ȡ CCCD ���
				// // Discover characteristic
				// req.startHandle = centralSvcStartHdl;
				// req.endHandle = centralSvcEndHdl;
				// req.type.len = ATT_BT_UUID_SIZE;
				// req.type.uuid[0] = LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID);
				// req.type.uuid[1] = HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID);
				// GATT_ReadUsingCharUUID(centralConnHandle, &req, centralTaskId);
				LOG("    ����һ����д���� handle\r\n");
				tmos_start_task(centralTaskId, START_READ_OR_WRITE_EVT, DEFAULT_READ_OR_WRITE_DELAY);
			}
		}
		else if(centralDiscState == BLE_DISC_STATE_CCCD)
		{
			LOG("    ��ǰ״̬ BLE_DISC_STATE_CCCD\r\n");
			if(pMsg->method == ATT_READ_BY_TYPE_RSP && pMsg->msg.readByTypeRsp.numPairs > 0)
			{
				uint16_t numPairs = pMsg->msg.readByTypeRsp.numPairs;
				uint16_t len = pMsg->msg.readByTypeRsp.len;
				uint8_t *pDataList = pMsg->msg.readByTypeRsp.pDataList;
				for(uint16_t i = 0; i < numPairs; i++)
				{
					uint16_t handle = BUILD_UINT16(pDataList[len * i], pDataList[len * i+1]);
					LOG("    CCCD handle 0x%04X\r\n", handle);
					centralCCCDHdl = handle;
				}
				centralProcedureInProgress = FALSE;
				// Start do write CCCD
				tmos_start_task(centralTaskId, START_WRITE_CCCD_EVT, DEFAULT_WRITE_CCCD_DELAY);
			}
			// �˴�û�ж� pMsg->hdr.status == bleProcedureComplete �Ĵ��� todo
			centralDiscState = BLE_DISC_STATE_IDLE;
			LOG("    BLE_DISC_STATE_CCCD ����. ���� BLE_DISC_STATE_IDLE ״̬\r\n");
		}
		else
		{
			LOG("    δ֪���� todo\r\n");
		}
	}
	GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      gapCentralRssiCB
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
static void gapCentralRssiCB(uint16_t connHandle, int8_t rssi)
{
	LOG("RSSI : -%d dB \r\n", -rssi);
}

/*********************************************************************
 * @fn      gapCentralHciMTUChangeCB
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
static void gapCentralHciMTUChangeCB(uint16_t connHandle, uint16_t maxTxOctets, uint16_t maxRxOctets)
{
	LOG("HCI data length changed, Tx: %d, Rx: %d\r\n", maxTxOctets, maxRxOctets);
	centralProcedureInProgress = TRUE;
}



/*********************************************************************
 * @fn      gapCentralEventCB
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
static void gapCentralEventCB(gapRoleEvent_t *pEvent)
{
	switch(pEvent->gap.opcode)
	{
		// �豸��ʼ������¼�
		case GAP_DEVICE_INIT_DONE_EVENT:
		{
			LOG("�¼� GAP_DEVICE_INIT_DONE_EVENT ��ʼ������ ��ʼ̽���豸\r\n");
			GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
										DEFAULT_DISCOVERY_ACTIVE_SCAN,
										DEFAULT_DISCOVERY_WHITE_LIST);
			break;
		}
		// �����豸ɨ������д�����һ��ɨ������лᱻ������Ρ�ÿ��ɨ�赽һ�����豸����յ�һ���豸�Ĺ㲥���ݰ�ʱ�����ᴥ������¼��� GAPRole_CentralStartDiscovery ����
		case GAP_DEVICE_INFO_EVENT:
		{
			if(pEvent->deviceInfo.rssi > -40)
			{
				LOG("�¼� GAP_DEVICE_INFO_EVENT �����豸\r\n");
				int8_t rssi = pEvent->deviceInfo.rssi;
				uint8_t addrType =  pEvent->deviceInfo.addrType;
				uint8_t eventType = pEvent->deviceInfo.eventType;
				uint8_t *p = pEvent->deviceInfo.pEvtData;
				uint8_t l = pEvent->deviceInfo.dataLen;
				if(eventType == GAP_ADRPT_ADV_IND)	// �㲥��
				{
					LOG("ɨ��㲥�� ");
					Print_Memory(p, l, 1);
					for(uint8_t i = 0; i < l; ++i)
					{
						uint8_t siz = p[i];
						uint8_t type = p[i+1];
						uint8_t* pVal = p + i + 2;
						if(type == GAP_ADTYPE_LOCAL_NAME_COMPLETE)
						{
							uint8_t v[siz];
							memset(v, 0, siz);
							memcpy(pVal, v, siz - 1);
							LOG("�豸���� \"%s\"\r\n", v);
						}
						else if(type == GAP_ADTYPE_16BIT_COMPLETE)
						{
							LOG("UUID 0x%04X", BUILD_UINT16(*(pVal+0), *(pVal+1)));
						}
						else
						{
							LOG("δ֪�������� key 0x%02X value ");
							Print_Memory(pVal, siz - 1, 1);
						}
					}
				}
				else if(eventType == GAP_ADRPT_SCAN_RSP)	// ɨ��Ӧ������
				{
					LOG("ɨ��Ӧ��� ");
					Print_Memory(pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen, 1);
					LOG("�㲥���� %s\r\n", pEvent->deviceInfo.pEvtData);
				}
				centralAddDeviceInfo(pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType, pEvent->deviceInfo.rssi);
			}
			break;
		}
		// �����豸ɨ��������� GAPRole_CentralStartDiscovery ����
		case GAP_DEVICE_DISCOVERY_EVENT:
		{
			LOG("�¼� GAP_DEVICE_DISCOVERY_EVENT ̽���豸 ����\r\n");
			uint8_t i;
			for(i = 0; i < centralScanRes; i++)
			{
				if(tmos_memcmp(PeerAddrDef, centralDevList[i].addr, B_ADDR_LEN))
					break;
			}

			// Peer device not found
			if(i == centralScanRes)
			{
				centralScanRes = 0;
				LOG("  ̽���豸 δ�ҵ��豸 ��ʼ����̽���豸\r\n");
				GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
											DEFAULT_DISCOVERY_ACTIVE_SCAN,
											DEFAULT_DISCOVERY_WHITE_LIST);
			}
			// Peer device found
			else
			{
				// ���ӵ�һ��
				LOG("  �ҵ��豸 MAC %x-%x-%x-%x-%x-%x ��������\r\n", PeerAddrDef[0], PeerAddrDef[1], PeerAddrDef[2], PeerAddrDef[3], PeerAddrDef[4], PeerAddrDef[5]);
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
				LOG("�¼� GAP_LINK_ESTABLISHED_EVENT MAC %x-%x-%x-%x-%x-%x ���ӳɹ�\r\n", PeerAddrDef[0], PeerAddrDef[1], PeerAddrDef[2], PeerAddrDef[3], PeerAddrDef[4], PeerAddrDef[5]);
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
				LOG("�¼� GAP_LINK_ESTABLISHED_EVENT MAC %x-%x-%x-%x-%x-%x ����ʧ�� Reason:%X. ��ʼ̽���豸\r\n", PeerAddrDef[0], PeerAddrDef[1], PeerAddrDef[2], PeerAddrDef[3], PeerAddrDef[4], PeerAddrDef[5],pEvent->gap.hdr.status);
				centralScanRes = 0;
				GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
											DEFAULT_DISCOVERY_ACTIVE_SCAN,
											DEFAULT_DISCOVERY_WHITE_LIST);
			}
			break;
		}
		case GAP_LINK_TERMINATED_EVENT:
		{
			LOG("gapCentralEventCB �¼� GAP_LINK_TERMINATED_EVENT\r\n");
			centralState = BLE_STATE_IDLE;
			centralConnHandle = GAP_CONNHANDLE_INIT;
			centralDiscState = BLE_DISC_STATE_IDLE;
			centralCharHdl = 0;
			centralScanRes = 0;
			centralProcedureInProgress = FALSE;
			tmos_stop_task(centralTaskId, START_READ_RSSI_EVT);
			LOG("Disconnected...Reason:%x\r\n", pEvent->linkTerminate.reason);
			LOG("̽���豸 ��ʼ\r\n");
			GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
										DEFAULT_DISCOVERY_ACTIVE_SCAN,
										DEFAULT_DISCOVERY_WHITE_LIST);
			break;
		}
		case GAP_LINK_PARAM_UPDATE_EVENT:
		{
			LOG("�¼� GAP_LINK_PARAM_UPDATE_EVENT ���³ɹ�\r\n");
			break;
		}
		case GAP_PHY_UPDATE_EVENT:
		{
			LOG("�¼� GAP_PHY_UPDATE_EVENT PHY Update...\r\n");
			break;
		}
		case GAP_EXT_ADV_DEVICE_INFO_EVENT:
		{
			LOG("�¼� GAP_EXT_ADV_DEVICE_INFO_EVENT Recv ext adv \r\n");
			// Add device to list
			centralAddDeviceInfo(pEvent->deviceExtAdvInfo.addr, pEvent->deviceExtAdvInfo.addrType, pEvent->deviceExtAdvInfo.rssi);
			break;
		}
		case GAP_DIRECT_DEVICE_INFO_EVENT:
		{
			LOG("�¼� GAP_DIRECT_DEVICE_INFO_EVENT\r\n");
			// Display device addr
			LOG("Recv direct adv \r\n");
			// Add device to list
			centralAddDeviceInfo(pEvent->deviceDirectInfo.addr, pEvent->deviceDirectInfo.addrType, pEvent->deviceExtAdvInfo.rssi);
			break;
		}
		default:
			LOG("�¼� δ֪�¼� id %02x\r\n", pEvent->gap.opcode);
			break;
	}
}

/*********************************************************************
 * @fn      gapCentralPairStateCB
 *
 * @brief   Pairing state callback.
 * ����Թ��̵�״̬�����仯ʱ�����������ص�������
 * ����ص���������Ӧ�ó�����ͬ�����״̬��������Գɹ���ʧ�ܡ�
 * Ӧ�ó�����Ը������״̬�ͽ����ȡ��Ӧ�Ĵ�ʩ��������ʾ���״̬��ִ�к���������
 *
 * @return  none
 */
static void gapCentralPairStateCB(uint16_t connHandle, uint8_t state, uint8_t status)
{
	LOG("gapCentralPairStateCB\r\n");
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
 * ���豸Ҫ�����������ʱ�����������ص�������
 * ����ص���������Ӧ�ó����ṩ����룬�Ա������Թ��̡�
 * Ӧ�ó�����Ը�����������������ʾ�������û���������롣
 *
 * @return  none
 */
static void gapCentralPasscodeCB(uint8_t *deviceAddr, uint16_t connectionHandle, uint8_t uiInputs, uint8_t uiOutputs)
{
	LOG("gapCentralPasscodeCB\r\n");
	uint32_t passcode;
	passcode = tmos_rand();
	passcode %= 1000000;
	if(uiOutputs != 0)
	{
		LOG("Passcode:%06d\r\n", (int)passcode);
	}
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

		if(rssi > -40)
		{
			LOG("  �������豸 ��¼ MAC %x-%x-%x-%x-%x-%x, RSSI %d dBm\r\n", pAddr[0], pAddr[1], pAddr[2], pAddr[3], pAddr[4], pAddr[5], rssi);
			memcpy(PeerAddrDef, pAddr, B_ADDR_LEN);
		}
	}
}

/************************ endfile @ central **************************/
