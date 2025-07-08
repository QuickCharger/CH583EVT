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
/* ͷ�ļ����� */
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
 * @brief   ��ѭ��
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
 * @brief   ������
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

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                10

// Scan duration in 0.625ms
// ɨ��ʱ�� ��λ0.625ms
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
// ��ռ�ձ�ɨ�裬ɨ��ʱ��������ɨ�������̣�ɨ�蹦�ĸ���
#define DEFAULT_LINK_HIGH_DUTY_CYCLE        FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST             FALSE

// Default read RSSI period in 0.625ms
#define DEFAULT_RSSI_PERIOD                 2400

// Minimum connection interval (units of 1.25ms)
// ���Ӽ������ָ��һ�������¼��� Connection events ���Ŀ�ʼ����һ�������¼���Connection events���Ŀ�ʼ��ʱ���������Ӽ���ķ�Χ��6 ~ 3200��7.5ms ~ 4s֮�䡣��λ1.25ms
// Connection Interval ���̣�Master��Slaveͨ�Ÿ���Ƶ����������������ٶȣ����������ݷ��͵�ʱ�䣬��ȻҲ�����˹��ġ�
// Connection Interval ������ͨ��Ƶ�ʽ��ͣ����������ٶȽ��ͣ����������ݷ��͵�ʱ�䣬��Ȼ���������ý����˹��ġ�
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL    80

// Maximum connection interval (units of 1.25ms)
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL    160

// Slave latency to use parameter update
// ����Slave�����豸����û������Ҫ��������£�����һ����Ŀ�������¼�������Щ�����¼���Connection events���в��ػظ�Master�����豸���İ����������ܸ���ʡ��
// Slave Latency���ٻ�������Ϊ 0��ÿ�δӻ�Connection Events�ж���Ҫ�ظ�Master�İ������Ļ����������ݷ����ٶȻ���ߡ�
// Slave Latency�ӳ��������½������ݷ����ٶȽ��͡�
#define DEFAULT_UPDATE_SLAVE_LATENCY        4

// Supervision timeout value (units of 10ms)
// ������ֵ ���ӶϿ� ��λ10ms
// ��ֵӦ���� ��1 + slave Latency ��* �� connection Interval ��
#define DEFAULT_UPDATE_CONN_TIMEOUT         600

// Default passcode
#define DEFAULT_PASSCODE                    0

// Default GAP pairing mode
// ����ѡ GAPBOND_PAIRING_MODE_INITIATE �� GAPBOND_PAIRING_MODE_NO_PAIRING, ѡ����Բ��������Խ���ص�
#define DEFAULT_PAIRING_MODE                GAPBOND_PAIRING_MODE_INITIATE

// Default MITM mode (TRUE to require passcode or OOB when pairing)
// �м��˱��� ͨ����Ҫ�û�����passkey�������
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

/**
 * BLE���������������޵ģ����ĸ����ࣺ
 * Primary Service����Ҫ�����
 * Secondary Service����Ҫ�����
 * Include�����������
 * Characteristic������ֵ��
 * 0x1800 �C 0x26FF ������������
 * 0x2700 �C 0x27FF ����λ
 * 0x2800 �C 0x28FF ����������
 * 0x2900 �C 0x29FF ������������
 * 0x2A00 �C 0x7FFF ������ֵ����
*/
uint16_t UUID_SRV_INFO = 0x180A;	// �豸��Ϣ
uint16_t UUID_SRV_HID = 0x1812;		// HID�豸
uint16_t UUID_SRV_BTT = 0x180F;		// �������
uint16_t UUID_char_MOUSE_IN = 0x2A33;	// HID�������
uint16_t UUID_char_BTT = 0x2A19;	// �������

////////////////////////////////////////////// UUID END


// Task ID for internal task/event processing
static uint8_t centralTaskId;

// Scan result list
static gapDevRec_t centralDevList[DEFAULT_MAX_SCAN_RES];

// Peer device address
// Device 4 - Addr 4e a1 86 52 e4 c3, RSSI -26 dBm
// static uint8_t PeerAddrDef[B_ADDR_LEN] = {0x0, 0xa1, 0x86, 0x52, 0xe4, 0xc3};

// �Զ����ӵ���СRSSI
const static int8_t requireRSSI = -40;
// �Զ����ӵ��豸��Ϣ
static gapDevRec_t peerDev;

// Parameter update state
static uint8_t centralParamUpdate = TRUE;

// Connection handle of current connection
// �豸�����GAP�ɹ����Ӻ���ֵ
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
		LOG("��ѯ������� over max\r\n");
		return -1;
	}
	queryServiceIndex += next;
	struct GATTService *s = &gattInfo[queryServiceIndex];
	if(s->uuid == 0)
	{
		LOG("��ѯ�������\r\n");
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

uint16_t guessUUID=0x2A33;	// 2A33 ������� 2A4D ������
/// @GATTINFO end //////////////////////////////////////////////////////////////////////

// �Զ��庯�� begin //////////////////////////////////////////////////////////////////////////
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
				LOG("    �㲥���� 0x01 LE Limited Discoverable Mode\r\n");
			}
			else if(*pVal == GAP_ADTYPE_FLAGS_GENERAL)
			{
				LOG("    �㲥���� 0x02 LE General Discoverable Mode\r\n");
			}
			else if(*pVal == GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED)
			{
				LOG("    �㲥���� 0x04 BR/EDR Not Supported\r\n");
			}
			else
			{
				LOG("    �㲥���� 0x%02X δ֪\r\n", *pVal);
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
			LOG("    �豸���� \"%s\"\r\n", v);
		}
		else if(type == GAP_ADTYPE_POWER_LEVEL)
		{
			LOG("    ���� %d%%\r\n", *pVal);
		}
		else if(type == GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE)
		{
			uint16_t min = BUILD_UINT16(*(pVal+0), *(pVal+1));
			uint16_t max = BUILD_UINT16(*(pVal+2), *(pVal+3));
			LOG("    ���Ӽ����Χ %d*1.25ms - %d*1.25ms\r\n", min, max);
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
				LOG("    Appearance 0x%04X δ֪\r\n", v);
			}
		}
		else
		{
			LOG("    δ֪�������� type 0x%X", type);
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
			LOG("�����������\r\n");
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
		return -2;		// ����Ӧ�ò������
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
// �Զ��庯�� end ////////////////////////////////////////////////////////////////////////////

// CCCD ��� begin //////////////////////////////////////////////////////////////////////////
uint16_t CCCD_SvcUUID=0x1812;	// ��Ҫʹ�ܵ�CCCD��svc�� 0x1812 HID�豸
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
// CCCD ��� end ////////////////////////////////////////////////////////////////////////////


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
		// LOG("���ص� SYS_EVENT_MSG\r\n");
		if((pMsg = tmos_msg_receive(centralTaskId)) != NULL)
		{
			uint8_t e = ((gattMsgEvent_t *)pMsg)->hdr.event;
			if(e == GATT_MSG_EVENT)
			{
				// GATT ��Ϣ
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
			if(DEFAULT_PAIRING_MODE == GAPBOND_PAIRING_MODE_INITIATE)
			{
				bStatus_t r = GAPBondMgr_PeriSecurityReq(centralConnHandle);
				if(r == SUCCESS)
				{
					LOG("���ص� START_SVC_DISCOVERY_EVT ������Գɹ�\r\n");
				}
				else if (r == bleNotConnected)
				{
					LOG("���ص� START_SVC_DISCOVERY_EVT �������ʧ�� bleNotConnected\r\n");
				}
				else if(r == bleAlreadyInRequestedMode)
				{
					LOG("���ص� START_SVC_DISCOVERY_EVT �������ʧ�� bleAlreadyInRequestedMode\r\n");
				}
				else if(r == bleIncorrectMode)
				{
					LOG("���ص� START_SVC_DISCOVERY_EVT �������ʧ�� bleIncorrectMode\r\n");
				}
				else if(r == bleMemAllocError)
				{
					LOG("���ص� START_SVC_DISCOVERY_EVT �������ʧ�� bleMemAllocError\r\n");
				}
				else if(r == bleInvalidRange)
				{
					LOG("���ص� START_SVC_DISCOVERY_EVT �������ʧ�� bleInvalidRange\r\n");
				}
				else if(r == bleAlreadyInRequestedMode)
				{
					LOG("���ص� START_SVC_DISCOVERY_EVT �������ʧ�� bleAlreadyInRequestedMode\r\n");
				}
				else
				{
					LOG("���ص� START_SVC_DISCOVERY_EVT �������ʧ�� %d\r\n", r);
				}
			}
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
					// ���ú��յ� GATT_MSG_EVENT �¼��� ATT_READ_RSP ��Ϣ
					LOG("    ����һ����д���� handle 0x%04X\r\n", hdl);
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
		LOG("���ص� START_WRITE_CCCD_EVT ����ʹ�� CCCD\r\n");
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
					LOG("���ص� START_WRITE_CCCD_EVT ����ʹ�� CCCD 2\r\n");
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
 * GATT ��ȡ���豸����������
 *
 * @return  none
 */
// ��־ǰ�Ӷ��ո�
static void gattCentralMsg(gattMsgEvent_t *pMsg)
{
	{
		// PRINT("GATT ��Ϣ��������. "); BLE_GATT_MSG_DESC(pMsg); PRINT("\r\n");
	}
	// ����豸��ǰ����������״̬�����������GATT��Ϣ���ͷ���Ϣ�ڴ沢���ء�
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
			if(errCode == ATT_ERR_INSUFFICIENT_AUTHEN)	// ��Ҫ����ȫ����
			{
				LOG("  ATT_ERROR_RSP Read Error: ATT_ERR_INSUFFICIENT_AUTHEN\r\n");
			}
			else if(errCode == ATT_ERR_INSUFFICIENT_ENCRYPT)	// ��Ҫ�����������
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
			if(errCode == ATT_ERR_INSUFFICIENT_AUTHEN)	// ��Ҫ����ȫ����
			{
				LOG("  ATT_ERROR_RSP Write Error: ATT_ERR_INSUFFICIENT_AUTHEN\r\n");
			}
			else if(errCode == ATT_ERR_INSUFFICIENT_ENCRYPT)	// ��Ҫ�����������
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

	// �����Ϣ��MTU������Ӧ �� MTU��������Ĵ�����Ӧ������MTU������
	if(pMsg->method == ATT_EXCHANGE_MTU_RSP)
	{
		// ��ȡʵ��Э�̵�MTU��С
		uint16_t negotiatedMTU = pMsg->msg.exchangeMTUReq.clientRxMTU;
		LOG("  Negotiated MTU: %d\r\n", negotiatedMTU);
	}
	// �����Ϣ��MTU�����¼������ӡ�µ�MTUֵ��
	if(pMsg->method == ATT_MTU_UPDATED_EVENT)
	{
		LOG("  ATT_MTU_UPDATED_EVENT MTU: %d\r\n", pMsg->msg.mtuEvt.MTU);
	}
	// �����Ϣ�Ƕ�ȡ��Ӧ �� ��ȡ����Ĵ�����Ӧ�������ȡ�����
	if(pMsg->method == ATT_READ_RSP)
	{
		// After a successful read, display the read value
		LOG("  ATT_READ_RSP Read rsp: %s\r\n", *pMsg->msg.readRsp.pValue);
	}
	// �����Ϣ��д����Ӧ �� д������Ĵ�����Ӧ������д������
	else if(pMsg->method == ATT_WRITE_RSP)
	{
		// Write success
		LOG("  ATT_WRITE_RSP Write success\r\n");
	}
	// ֪ͨ�����ӡ���յ���ֵ֪ͨ��
	// �������⣡����
	else if(pMsg->method == ATT_HANDLE_VALUE_NOTI)
	{
		// ���յ� ���� ֪ͨ ָʾ
		//LOG("  ATT_HANDLE_VALUE_NOTI Receive noti: %x ���յ��ӻ�notify���� \r\n", *pMsg->msg.handleValueNoti.pValue);
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
	// ָʾ����֪ͨ��ͬ���ǣ�ָʾ��Ҫ����ȷ�ϡ�
	else if(pMsg->method == ATT_HANDLE_VALUE_IND)
	{
		LOG("  ATT_HANDLE_VALUE_IND Receive ind: %x ���յ��ӻ�ind���� \r\n", *pMsg->msg.handleValueInd.pValue);
		ATT_HandleValueCfm(centralConnHandle);	// ����ȷ��
	}
	else if(centralDiscState != BLE_DISC_STATE_IDLE)
	{
		// ���ڷ���UUID���ҹ��ܵ�״̬
		// �ڼ���յ�������ݷ��� ���� pMsg->hdr.status == bleProcedureComplete �ж����ݽ������
		if(centralDiscState == BLE_DISC_STATE_ALL_SVC)
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
							LOG("    �ҵ�HID�豸, CCCD_SvcStartHdl 0x%04X CCCD_SvcEndHdl 0x%04X\r\n", CCCD_SvcStartHdl, CCCD_SvcEndHdl);
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
					QueryServiceInfo(centralConnHandle, centralTaskId, 0);
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
					// 12 = GATT_PROP_READ | GATT_PROP_NOTIFY
					uint8_t* u = pDataList + len * i;
					uint16_t declareHandle = BUILD_UINT16(*(u+0), *(u+1));	// ����������� metadata
					uint8_t readWrite = *(u + 2);		// ��дȨ�� GATT_PROP_WRITE
					uint16_t handle = BUILD_UINT16(*(u+3), *(u+4));
					uint16_t uuid = BUILD_UINT16(*(u+5), *(u+6));
					
					LOG("    DeclareHandle 0x%04X Ȩ�� %s%s%s%s%s%s%s%s handle 0x%04X uuid 0x%04X %s\r\n", declareHandle, \
					(readWrite & GATT_PROP_BCAST) ? "�㲥,":"", \
					(readWrite & GATT_PROP_READ) ? "��,":"", \
					(readWrite & GATT_PROP_WRITE_NO_RSP) ? "д��Ӧ��,":"", \
					(readWrite & GATT_PROP_WRITE) ? "д,":"", \
					(readWrite & GATT_PROP_NOTIFY) ? "֪ͨ,":"", \
					(readWrite & GATT_PROP_INDICATE) ? "ָʾ,":"", \
					(readWrite & GATT_PROP_AUTHEN) ? "��֤,":"", \
					(readWrite & GATT_PROP_EXTENDED) ? "��չ,":"", \
					handle, uuid, BLE_UUID2str(uuid));

					if(readWrite & GATT_PROP_READ)
					{
						addReadHandle(handle);
					}
				}
			}
			// 0x2A4B �� HID��Ϣ��UUID ��Ҫ��ȡ���ݷ��������������
			if(pMsg->method == ATT_READ_BY_TYPE_RSP && pMsg->hdr.status == bleProcedureComplete)
			{
				
				int8_t r = QueryServiceInfo(centralConnHandle, centralTaskId, 1);
				if(r >= 0)
				{
					LOG("    ����״̬ BLE_DISC_STATE_CHAR_ALL\r\n");
					return;
				}
				if(CCCD_SvcStartHdl != 0)
				{
					centralDiscState = BLE_DISC_STATE_CCCD;
					LOG("    BLE_DISC_STATE_CHAR_ALL ����. ���� BLE_DISC_STATE_CCCD ״̬\r\n");
					attReadByTypeReq_t req;
					req.startHandle = CCCD_SvcStartHdl;
					req.endHandle = CCCD_SvcEndHdl;
					req.type.len = ATT_BT_UUID_SIZE;
					// GATT_CLIENT_CHAR_CFG_UUID �ǻ�ȡCCCD��UUID
					req.type.uuid[0] = LO_UINT16(GATT_CLIENT_CHAR_CFG_UUID);
					req.type.uuid[1] = HI_UINT16(GATT_CLIENT_CHAR_CFG_UUID);
					GATT_ReadUsingCharUUID(centralConnHandle, &req, centralTaskId);
				}
				else
				{
					centralDiscState = BLE_DISC_STATE_IDLE;
					LOG("    δ����HID��ص�CCCD����\r\n");
					LOG("    BLE_DISC_STATE_CHAR_ALL ����. ���� BLE_DISC_STATE_IDLE ״̬\r\n");
				}
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
				// �˴�����ֻ��һ��CCCD������
				for(uint16_t i = 3; i < numPairs; i++)
				{
					uint16_t handle = BUILD_UINT16(pDataList[len * i], pDataList[len * i+1]);
					LOG("    ���� CCCD handle 0x%04X\r\n", handle);
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
						LOG("    ����ʹ��CCCD 0x%04X\r\n", CCCD_Hdl);
					else
						LOG("    ʹ��CCCDʧ��\r\n");
				}
				centralDiscState = BLE_DISC_STATE_IDLE;
				LOG("    BLE_DISC_STATE_CCCD ����. ���� BLE_DISC_STATE_IDLE ״̬\r\n");
			}
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
static void  gapCentralEventCB(gapRoleEvent_t *pEvent)
{
	switch(pEvent->gap.opcode)
	{
		// �豸��ʼ������¼�
		case GAP_DEVICE_INIT_DONE_EVENT:
		{
			LOG("EventCB GAP_DEVICE_INIT_DONE_EVENT ��ʼ������ ��ʼ̽���豸\r\n");
			// ���ú���δ��� GAP_DEVICE_INFO_EVENT �¼�
			// ɨ������� DEFAULT_SCAN_DURATIONʱ��, �ᴥ�� GAP_DEVICE_DISCOVERY_EVENT �¼�
			GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
										DEFAULT_DISCOVERY_ACTIVE_SCAN,
										DEFAULT_DISCOVERY_WHITE_LIST);
			break;
		}
		// �����豸ɨ������д�����һ��ɨ������лᱻ������Ρ�ÿ��ɨ�赽һ�����豸����յ�һ���豸�Ĺ㲥���ݰ�ʱ�����ᴥ������¼��� GAPRole_CentralStartDiscovery ����
		case GAP_DEVICE_INFO_EVENT:
		{
			if(pEvent->deviceInfo.rssi > requireRSSI)
			{
				LOG("EventCB GAP_DEVICE_INFO_EVENT �����豸\r\n");
				int8_t rssi = pEvent->deviceInfo.rssi;
				uint8_t addrType =  pEvent->deviceInfo.addrType;
				uint8_t eventType = pEvent->deviceInfo.eventType;
				uint8_t *p = pEvent->deviceInfo.pEvtData;
				uint8_t l = pEvent->deviceInfo.dataLen;
				// Ҫ������ǹ㲥������ɨ��Ӧ��� todo
				if(eventType == GAP_ADRPT_ADV_IND)	// �㲥��
				{
					LOG("  ɨ��㲥�� ");
					Print_Memory(p, l, 1);
					uint8_t type = parseDeviceInfo(&pEvent->deviceInfo);
					if(type == eMouse || type == eKeyboard)
					{
						centralAddDeviceInfo(pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType, pEvent->deviceInfo.rssi);
					}
				}
				else if(eventType == GAP_ADRPT_SCAN_RSP)	// ɨ��Ӧ������
				{
					LOG("  ɨ��Ӧ��� ");
					Print_Memory(pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen, 1);
					parseDeviceInfo(&pEvent->deviceInfo);
				}
				else
				{
					LOG("  δ֪ɨ��� !!!");
				}
			}
			break;
		}
		// �����豸ɨ��������� GAPRole_CentralStartDiscovery ����
		case GAP_DEVICE_DISCOVERY_EVENT:
		{
			LOG("EventCB GAP_DEVICE_DISCOVERY_EVENT ̽���豸 ����\r\n");
			if(peerDev.rssi == 0)
			{
				LOG("  ̽���豸 δ�ҵ��豸 ��ʼ����̽���豸\r\n");
				GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
											DEFAULT_DISCOVERY_ACTIVE_SCAN,
											DEFAULT_DISCOVERY_WHITE_LIST);
			}
			else
			{
				LOG("  �ҵ��豸 MAC %x-%x-%x-%x-%x-%x. rssi %d. ��������\r\n", peerDev.addr[0], peerDev.addr[1], peerDev.addr[2], peerDev.addr[3], peerDev.addr[4], peerDev.addr[5], peerDev.rssi);
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
				LOG("  MAC %x-%x-%x-%x-%x-%x ���ӳɹ�\r\n", peerDev.addr[0], peerDev.addr[1], peerDev.addr[2], peerDev.addr[3], peerDev.addr[4], peerDev.addr[5]);
				centralState = BLE_STATE_CONNECTED;
				centralConnHandle = pEvent->linkCmpl.connectionHandle;

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
			}
			else
			{
				LOG("  MAC %x-%x-%x-%x-%x-%x ����ʧ�� Reason:%X. ��ʼ̽���豸\r\n", peerDev.addr[0], peerDev.addr[1], peerDev.addr[2], peerDev.addr[3], peerDev.addr[4], peerDev.addr[5],pEvent->gap.hdr.status);
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
			LOG("  ̽���豸 ��ʼ\r\n");
			GAPRole_CentralStartDiscovery(DEFAULT_DISCOVERY_MODE,
										DEFAULT_DISCOVERY_ACTIVE_SCAN,
										DEFAULT_DISCOVERY_WHITE_LIST);
			break;
		}
		case GAP_LINK_PARAM_UPDATE_EVENT:
		{
			LOG("EventCB GAP_LINK_PARAM_UPDATE_EVENT ���³ɹ�\r\n");
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
			LOG("EventCB δ֪�¼� id %02x\r\n", pEvent->gap.opcode);
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
 * GAPBondMgr_SlaveReqSecurity �ᴥ������ص�����
 * 
 * status
 * 0x00	�ɹ���SUCCESS��
 * 0x01	ͨ��ʧ�ܣ�FAILURE��
 * 0x02	��Ч������INVALID_PARAM��
 * 0x03	�ڴ治�㣨OUT_OF_MEMORY��
 * 0x04	��֧�ֵĲ�����NOT_SUPPORTED��
 * 0x05	��֤ʧ�ܣ�AUTH_FAILURE��
 * 0x06	��Ч�����INVALID_HANDLE��
 * 0x07	����δ��ɣ�OPERATION_NOT_COMPLETE��
 * 0x08	��ʱ��TIMEOUT��
 * 0x09	�����Ѵ��ڣ�CONNECTION_ALREADY_EXISTS��
 * 0x0A	10 ���Ӳ����ڣ�CONNECTION_NOT_EXISTS��
 * 0x0B	11 ��Դ���㣨RESOURCE_UNAVAILABLE��
 * 0x0C	12 Э�����PROTOCOL_ERROR��
 * 0x0D	13 ����ʧ�ܣ�ENCRYPTION_FAILURE��
 * 0x0E	14 ���ʧ�ܣ�PAIRING_FAILURE��
 * 0x0F	15 ����Ѵ��ڣ�PAIRING_ALREADY_EXISTS��
 * 0x10	16 ��Ա��ܾ���PAIRING_REJECTED��
 * 0x11	17 ��Գ�ʱ��PAIRING_TIMEOUT��
 * 0x12	18 ��Բ�����Ч��PAIRING_INVALID_PARAM��
 * 0x13	19 ���δ��ɣ�PAIRING_NOT_COMPLETE��
 * 0x14	20 �����ȡ����PAIRING_CANCELED��
 * 0x15	21 ����ѽ��ã�PAIRING_DISABLED��
 * 0x16	22 ��������ã�PAIRING_ENABLED��
 * 0x17	23 ��������ã�PAIRING_RESET��
 * 0x18	24 ����ѱ��棨PAIRING_SAVED��
 * 0x19	25 �����ɾ����PAIRING_DELETED��
 * 0x1A	26 ����ѻָ���PAIRING_RESTORED��
 * 0x1B	27 ����Ѹ��£�PAIRING_UPDATED��
 * 0x1C	28 ����ѹ��ڣ�PAIRING_EXPIRED��
 * 0x1D	29 �����������PAIRING_LOCKED��
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
 * ���豸Ҫ�����������ʱ�����������ص�������
 * ����ص���������Ӧ�ó����ṩ����룬�Ա������Թ��̡�
 * Ӧ�ó�����Ը�����������������ʾ�������û���������롣
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
		LOG("  �������豸 MAC %x-%x-%x-%x-%x-%x, RSSI %d dBm\r\n", pAddr[0], pAddr[1], pAddr[2], pAddr[3], pAddr[4], pAddr[5], rssi);
		if(peerDev.rssi == 0 || peerDev.rssi < rssi)
		{
			LOG("  �������豸 ��¼ MAC %x-%x-%x-%x-%x-%x, RSSI %d dBm\r\n", pAddr[0], pAddr[1], pAddr[2], pAddr[3], pAddr[4], pAddr[5], rssi);
			memcpy(peerDev.addr, pAddr, B_ADDR_LEN);
			peerDev.addrType = addrType;
			peerDev.rssi = rssi;
		}
		else
		{
			LOG("�ź�ǿ������֮ǰ���豸 ����\r\n");
		}
	}
}

/************************ endfile @ central **************************/
