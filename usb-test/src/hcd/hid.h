
#ifndef HID_H
#define HID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "xil_types.h"
#include "xil_cache.h"
#include "hcd.h"


#define GET_REPORT 0x01
#define GET_IDLE 0x02
#define GET_PROTOCOL 0x03
#define Reserved 0x04
#define SET_REPORT 0x09
#define SET_IDLE 0x0A
#define SET_PROTOCOL 0x0B


void hid_callbackHandler(void *CallBackRef, u32 Mask);

#ifdef __cplusplus
}
#endif

#endif /* HID */
