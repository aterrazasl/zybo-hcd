#include "hid.h"
#include "xil_printf.h"
#include "hcd_usbCh9.h"


static void hid_printLine(u8* data, u32 size, char* comment){
	int i;
	for(i = 0; i < size; i++){
		xil_printf("%02X", *(data+i));
	}
	xil_printf(" // %s\r", comment);
}

void hid_callbackHandler(void *CallBackRef, u32 Mask){

	hcd_t *hcdPtr = (hcd_t*)CallBackRef;


	hid_printLine((u8*) hcdPtr->qTD[1]->buffer[0], 0x003b, "Keyboard Report");
//	xil_printf ("[HID] %s\r\n", "Hit class");
}


static hcd_endpoint0* hid_createGetReportRequest(void){

	hcd_endpoint0 *ep0Ptr = hcd_getEp0();

	ep0Ptr->setupData.bmRequestType = DEVICE_TO_HOST | CLASS_TYPE | INTERFACE_RECIPIENT;
	ep0Ptr->setupData.bRequest 		= GET_REPORT;
	ep0Ptr->setupData.wValue 		= hcd_swap_uint16(REPORT_INPUT);
	ep0Ptr->setupData.wIndex 		= hcd_swap_uint16(0x0000);
	ep0Ptr->setupData.wLength 		= 0x003b;


	ep0Ptr->speed = 0;
	ep0Ptr->expectReply = 1;

	Xil_DCacheFlush();

	return ep0Ptr;
}

void hid_requestReport(hcd_t *hcdPtr){

	hcd_sendSetupData(hcdPtr,hid_createGetReportRequest());

}
