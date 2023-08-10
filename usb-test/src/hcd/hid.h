
#ifndef HID_H
#define HID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "xil_types.h"
#include "xil_cache.h"
#include "hcd.h"
#include "hcd_usbCh9.h"
#include "hcd_hw.h"

//bRequest
#define GET_REPORT 0x01
#define GET_IDLE 0x02
#define GET_PROTOCOL 0x03
#define Reserved 0x04
#define SET_REPORT 0x09
#define SET_IDLE 0x0A
#define SET_PROTOCOL 0x0B

//Report types
#define REPORT_INPUT 1
#define REPORT_OUTPUT 2
#define REPORT_REATURE 3
#define REPORT_RESERVED 4


void hid_callbackHandler(void *CallBackRef, u32 Mask);
void hid_requestReport(hcd_t *hcdPtr);
void hid_printLine(u8* data, u32 size, char* comment);

#ifdef __cplusplus
}
#endif

#endif /* HID */
