#ifndef __CH58x_EASY_H__
#define __CH58x_EASY_H__

#include <stdarg.h>
#include "CH58x_common.h"

#include "../../BLE/LIB/CH58xBLE_LIB.h"
#ifdef __cplusplus
extern "C" {
#endif

/**
 * ¥Ú”°
*/
extern void Print_Memory(uint8_t *p, uint16_t len, uint8_t rn);
extern void DoPRINT(const char* file, int line, const char* date, const char* time, const char* func,  const char* format, ...);
#define LOG(format, ...) DoPRINT(__FILE__, __LINE__, __DATE__, __TIME__, __func__, format, ##__VA_ARGS__)


/**
 * ¿∂—¿
*/
extern const char* BLE_Opcode2str(uint8_t m);
extern const char* BLE_UUID2str(uint16_t m);
extern void BLE_GATT_MSG_DESC(gattMsgEvent_t* m);
extern void BLE_UUID_DESC(uint8_t *pDataList, uint16_t numGrps);

#ifdef __cplusplus
}
#endif

#endif
