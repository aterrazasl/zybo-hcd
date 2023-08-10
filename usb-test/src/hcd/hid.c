#include "hid.h"
#include "xil_printf.h"
#include "hcd_usbCh9.h"


static hcd_endpoint0  ep1;
static hcd_endpoint0* hid_createGetReportRequest(void);

void setLED(u8 led){
#define LEDBASE_DATA 0x41200000 + 8

	Xil_Out32(LEDBASE_DATA, Xil_In32(LEDBASE_DATA) | (1<<led));

}
void clearLED(u8 led){
#define LEDBASE_DATA 0x41200000 + 8

	Xil_Out32(LEDBASE_DATA, Xil_In32(LEDBASE_DATA) & ~(1<<led));

}


void hid_printLine(u8* data, u32 size, char* comment){
	int i;
	for(i = 0; i < size; i++){
		xil_printf("%02X", *(data+i));
	}
	xil_printf(" // %s\r", comment);



}

void hid_callbackHandler(void *CallBackRef, u32 Mask){



	hcd_t *hcdPtr = (hcd_t*)CallBackRef;
//	hid_printLine((u8*)0x114000, 0x0008, "Keyboard Report");
	hcd_enquePeriodicQH(hcdPtr,hid_createGetReportRequest());

//	xil_printf ("[HID] %s\r\n", "Hit class");

}


static hcd_endpoint0* hid_createGetReportRequest(void){

	hcd_endpoint0 *epPtr = &ep1;

	epPtr->setupData.bmRequestType = DEVICE_TO_HOST | CLASS_TYPE | INTERFACE_RECIPIENT;
	epPtr->setupData.bRequest 		= GET_REPORT;
	epPtr->setupData.wValue 		= hcd_swap_uint16(REPORT_INPUT);
	epPtr->setupData.wIndex 		= hcd_swap_uint16(0x0000);
	epPtr->setupData.wLength 		= 0x0008;


	epPtr->address = 1;
	epPtr->maxPacketSize = 64;
	epPtr->expectReply = 1;

	Xil_DCacheFlush();

	return epPtr;
}

void hid_requestReport(hcd_t *hcdPtr){

	hcd_enquePeriodicQH(hcdPtr,hid_createGetReportRequest());

//	hcd_sendSetupData(hcdPtr,hid_createGetReportRequest());

}
