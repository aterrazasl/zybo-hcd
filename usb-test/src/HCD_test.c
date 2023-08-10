#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "hcd/hcd.h"
#include "xscugic.h"
#include "hcd/hcd_hw.h"
#include "hcd/hid.h"

static XScuGic IntcInstance;



/*****************************************************************************/
/**
* Basic test entry point for creating HCD driver and start it
*
*
* @param   none
*
* @return	errors
*
* @note     None.
*
******************************************************************************/
int main()
{
	int status;
	XScuGic* IntcPtr = &IntcInstance;	/* The instance of the IRQ Controller */

    init_platform();

    print("\r\nStarting HDC Test example...\n\r");

    hcd_t* hcdPtr = NULL;

    hcdPtr = hcd_init();   // Initialize pointers, Create memory refs and initialize HW
    if(hcdPtr == NULL) return HCD_ERROR;

    status = hcd_connectClassHandler(hcdPtr, hid_callbackHandler, hcdPtr);
	if(status == HCD_ERROR) return status;


    status = hcd_start(hcdPtr, IntcPtr);	// Start interrupts and handles the enumeration of devices registers interrupt handler
    if(status == HCD_ERROR) return status;

    while(hcdPtr->state != hcd_idle){
    	//Waits until HCD finishes enumeration
    }

    print("Device connected and enumerated...\n\r");

    hid_requestReport(hcdPtr);

	int i=10000000;
	while(1){
		i--;
		if (i == 0){
			i=10000000;
			// do something...
//			hid_requestReport(hcdPtr);
//			hid_printLine((u8*)(hcdPtr->periodicqTD[1]->buffer[0]&0xfffff000), 0x0008, "Keyboard Report");

//			hid_printLine((u8*)(hcdPtr->periodicqTD[1]->buffer[0]&0xfffff000), 0x003b, "Keyboard Report");
//			hid_requestReport(hcdPtr);
//			xil_printf("R%08X = %08X ",HCD_CMD_OFFSET,hcd_ReadReg(hcdPtr->config.BaseAddress,	HCD_CMD_OFFSET));
//			xil_printf("R%08X = %08X ",HCD_ISR_OFFSET,hcd_ReadReg(hcdPtr->config.BaseAddress,	HCD_ISR_OFFSET));
//			xil_printf("R%08X = %08X ",HCD_LISTBASE_OFFSET,hcd_ReadReg(hcdPtr->config.BaseAddress,	HCD_LISTBASE_OFFSET));
//			xil_printf("R%08X = %08X ",HCD_ASYNCLISTADDR_OFFSET,hcd_ReadReg(hcdPtr->config.BaseAddress,	HCD_ASYNCLISTADDR_OFFSET));
//			xil_printf("%s","\r");
//			Xil_Out32(LEDBASE_DATA, ~(Xil_In32(LEDBASE_DATA) | (1<<3)) & 0x00000008);

		}


    }

    status = hcd_stop(hcdPtr);	// Stops interrupts
    if(status == HCD_ERROR) return status;
    status = hcd_cleanup(hcdPtr);	// releases pointers and memory allocations
    if(status == HCD_ERROR) return status;


    cleanup_platform();
    return 0;
}



