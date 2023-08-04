#include "hid.h"
#include "xil_printf.h"


void hid_callbackHandler(void *CallBackRef, u32 Mask){
	xil_printf ("[HID] %s\r\n", "Hit class");
}
