#include "BLE_main.h"
#include "HID_Main.h"
#include "easy.h"

void main()
{
	HID_main();
	BLE_main();
	
	while(1) {
		TMOS_SystemProcess();
	}
}
