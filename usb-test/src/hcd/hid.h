
#ifndef HID_H
#define HID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "xil_types.h"
#include "xil_cache.h"




void hid_callbackHandler(void *CallBackRef, u32 Mask);

#ifdef __cplusplus
}
#endif

#endif /* HID */
