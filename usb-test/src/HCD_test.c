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

	int i=20000000;
	while(1){
		i--;
		if (i == 0){
			i=20000000;
			// do something...
			hid_requestReport(hcdPtr);
		}


    }

    status = hcd_stop(hcdPtr);	// Stops interrupts
    if(status == HCD_ERROR) return status;
    status = hcd_cleanup(hcdPtr);	// releases pointers and memory allocations
    if(status == HCD_ERROR) return status;


    cleanup_platform();
    return 0;
}



