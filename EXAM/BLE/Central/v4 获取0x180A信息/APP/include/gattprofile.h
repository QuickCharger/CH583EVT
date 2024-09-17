/********************************** (C) COPYRIGHT *******************************
 * File Name          : gattprofile.h
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/12/11
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef GATTPROFILE_H
#define GATTPROFILE_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// Profile Parameters
#define SIMPLEPROFILE_CHAR1         0           // RW uint8_t - Profile Characteristic 1 value
#define SIMPLEPROFILE_CHAR2         1           // RW uint8_t - Profile Characteristic 2 value
#define SIMPLEPROFILE_CHAR3         2           // RW uint8_t - Profile Characteristic 3 value
#define SIMPLEPROFILE_CHAR4         3           // RW uint8_t - Profile Characteristic 4 value
#define SIMPLEPROFILE_CHAR5         4           // RW uint8_t - Profile Characteristic 4 value

// Simple Profile Service UUID
#define SIMPLEPROFILE_SERV_UUID     0xFFE0

/**
 * HID 信息特性：0x2A4A 通常用于获取设备的元数据，比如设备的版本、制造商、产品 ID 等。
 * 输入报告特性：0x2A22 蓝牙鼠标主要使用这个特性向主设备发送位置信息（鼠标移动、按键点击等）。
 * 输出报告特性：0x2A23 主设备可以向 HID 设备发送这些更新信息, 可以用于控制鼠标的 LED 灯、发送特定命令或进行设备配置。进入休眠模式
 */
// Key Pressed UUID
#define SIMPLEPROFILE_CHAR1_UUID    0xFFE1
#define SIMPLEPROFILE_CHAR2_UUID    0xFFE2
#define SIMPLEPROFILE_CHAR3_UUID    0xFFE3
#define SIMPLEPROFILE_CHAR4_UUID    0xFFE4
#define SIMPLEPROFILE_CHAR5_UUID    0xFFE5

// Simple Keys Profile Services bit fields
#define SIMPLEPROFILE_SERVICE       0x00000001

// Length of characteristic in bytes ( Default MTU is 23 )
#define SIMPLEPROFILE_CHAR1_LEN     1
#define SIMPLEPROFILE_CHAR2_LEN     1
#define SIMPLEPROFILE_CHAR3_LEN     1
#define SIMPLEPROFILE_CHAR4_LEN     1
#define SIMPLEPROFILE_CHAR5_LEN     5

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*simpleProfileChange_t)(uint8_t paramID);

typedef struct
{
    simpleProfileChange_t pfnSimpleProfileChange; // Called when characteristic value changes
} simpleProfileCBs_t;

/*********************************************************************
 * API FUNCTIONS
 */

/*
 * SimpleProfile_AddService- Initializes the Simple GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t SimpleProfile_AddService(uint32_t services);

/*
 * SimpleProfile_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t SimpleProfile_RegisterAppCBs(simpleProfileCBs_t *appCallbacks);

/*
 * SimpleProfile_SetParameter - Set a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern bStatus_t SimpleProfile_SetParameter(uint8_t param, uint8_t len, void *value);

/*
 * SimpleProfile_GetParameter - Get a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern bStatus_t SimpleProfile_GetParameter(uint8_t param, void *value);

/*
 * simpleProfile_Notify - Send notification.
 *
 *    connHandle - connect handle
 *    pNoti - pointer to structure to notify.
 */
extern bStatus_t simpleProfile_Notify(uint16_t connHandle, attHandleValueNoti_t *pNoti);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif
