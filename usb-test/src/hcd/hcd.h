
#ifndef HCD_H
#define HCD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "xil_types.h"
#include "xil_cache.h"
#include "xscugic.h"
#include "hcd_usbCh9.h"
//#include "hcd_debug.h"

/* Definitions */

#define VERBOSE 1
#define BUSY 1
#define NOT_BUSY 0


#define HCD_NUM_BUFFER_PTR 5
#define HCD_MAX_QTD	4
#define HCD_MAX_QH	2


#define ALIGNMENT_CACHELINE  __attribute__ ((aligned (32)))
#define MEMORY_SIZE (64 * 1024 * 2 )
#define USBDEVICEID 0	//xilinx zynq could have 2 instances of USB host
#define HCD_ERROR -1



/******************************************************************************
 * This data type defines the callback function to be used for the general
 * interrupt handler.
 *
 * @param	CallBackRef is the Callback reference passed in by the upper
 *		layer when setting the handler, and is passed back to the upper
 *		layer when the handler is called.
 * @param	IrqMask is the Content of the interrupt status register. This
 *		value can be used by the callback function to distinguish the
 *		individual interrupt types.
 */
typedef void (*hcd_IntrHandlerFunc)(void *CallBackRef, u32 IrqMask);



typedef struct {
	u16 DeviceID;		/**< Unique ID of controller. */
	u32 BaseAddress;	/**< Core register base address. */
} hcd_config;



typedef struct {
	u32 status	: 8;
	u32 PID	: 2;
	u32 Cerr : 2;
	u32 C_Page	: 3;
	u32 IOC : 1;
	u32 Total_Bytes : 15;
	u32 DT : 1;
}hcd_qTD_token_st;

typedef union {
	u32 data;
	hcd_qTD_token_st st;
}hcd_qTD_token;



/*Used to describe the queue element transfer descriptor 3.5 of ehci*/
typedef struct{
	u32 nextqTD;
	u32 nextqTD_alt;
	u32 token;
	u32 buffer[5];
}hcd_qTD_st;

typedef struct {
	u32 S_mask	: 8;
	u32 C_mask	: 8;
	u32 hubAddress : 7;
	u32 portNumber	: 7;
	u32 mult : 2;
}hcd_QH_endpoint_word2_st;

typedef union {
	u32 data;
	hcd_QH_endpoint_word2_st st;
}hcd_QH_endpoint_word2;


typedef struct {
	u32 DevAddress	: 7;
	u32 I	: 1;
	u32 EndPt : 4;
	u32 EPS	: 2;
	u32 DTC : 1;
	u32 H	: 1;
	u32 MaxPacketLength	: 11;
	u32 c 	: 1;
	u32 RL	: 4;
}hcd_QH_endpoint_word1_st;

typedef union {
	u32 data;
	hcd_QH_endpoint_word1_st st;
}hcd_QH_endpoint_word1;


/*Used to describe the queue element transfer descriptor 3.5 of ehci*/
typedef struct{
	u32 nextQH;
	u32 endpoint_word1;
	u32 endpoint_word2;
	u32 currentqTD;
	hcd_qTD_st overlay;

}hcd_QH_st;


typedef struct {
	u32 address_complete;
	u32 configure_complete;
	u32 suspend_complete;
}hcd_device_flags_st;

typedef union {
	u32 data;
	hcd_device_flags_st state;
}hcd_device_flags;

typedef enum {
	hcd_disconnected,
	hcd_attached 	,
	hcd_powered		,
	hcd_getDeviceDescriptor,
	hcd_reset		,
	hcd_setAddress,
	hcd_getDeviceDescriptorFull,
	hcd_default		,
	hcd_getStatus	,
	hcd_getConfiguration,
	hcd_getConfigurationFull,
	hcd_address		,
	hcd_configured	,
	hcd_idle		,
	hcd_suspended	,
}hcd_state;

typedef struct{
	hcd_config config;
	u32 DMAMemPhys;
	u32 PhysAligned;
	hcd_QH_st * QH[HCD_MAX_QH];
	hcd_qTD_st * qTD[HCD_MAX_QTD];
	u8 busy;							//set to 1 when the async list is enabled

	hcd_IntrHandlerFunc	HandlerFunc;	/**< Handler function for the controller. */
	void *HandlerRef;					/**< User data reference for the handler. */
	u32 HandlerMask;					/**< User interrupt mask. Defines which interrupts will cause the callback to be called. */

	hcd_IntrHandlerFunc	ClassHandlerFunc;	/**< Handler function for the controller. */
	void *ClassHandlerRef;					/**< User data reference for the handler. */
	u32 ClassHandlerMask;					/**< User interrupt mask. Defines which interrupts will cause the callback to be called. */


	u8 deviceConnected;
	u8 speedDetected;
	hcd_state state;
	hcd_device_flags flags;
}hcd_t;


/************************** Constant Definitions ******************************/
static const hcd_QH_st empty_QH	= {	0x0003,   	//nextQH
										0x0000,		//endpoint_word1
										0x0000,		//endpoint_word2
										0x0001,		//current qTD
										{	0x0001,	//nextqTD
											0x0001,	//nextqTD_alt
											0x0000,	//token
											0x0000,	//Bufferpointer0
											0x0000,	//Bufferpointer1
											0x0000,	//Bufferpointer2
											0x0000,	//Bufferpointer3
											0x0000,	//Bufferpointer4
										}};

static const hcd_qTD_st empty_qTD = {	0x0001,	//nextqTD
											0x0001,	//nextqTD_alt
											0x0000,	//token
											0x0000,	//Bufferpointer0
											0x0000,	//Bufferpointer1
											0x0000,	//Bufferpointer2
											0x0000,	//Bufferpointer3
											0x0000,	//Bufferpointer4
									};


/****************************************************************************/
/**
* This macro sets the given bit mask in the register.
*
* @param	InstancePtr is a pointer to hcd instance of the controller.
* @param	RegOffset is the register offset to be written.
* @param	Bits is the Bits to be set in the register
*
* @return	None.
*
* @note		C-style signature:
*		void hcd_SetBits(u32 BaseAddress, u32 RegOffset, u32 Bits)
*
*****************************************************************************/
#define hcd_SetBits(InstancePtr, RegOffset, Bits) \
	hcd_WriteReg((InstancePtr)->config.BaseAddress, RegOffset,	\
		hcd_ReadReg((InstancePtr)->config.BaseAddress, 	\
					RegOffset) | (Bits));


/****************************************************************************/
/**
*
* This macro clears the given bits in the register.
*
* @param	InstancePtr is a pointer to hcd instance of the controller.
* @param	RegOffset is the register offset to be written.
* @param	Bits are the bits to be cleared in the register
*
* @return	None.
*
* @note
* 	C-style signature:
*	void hcd_ClrBits(u32 BaseAddress, u32 RegOffset, u32 Bits)
*
*****************************************************************************/
#define hcd_ClrBits(InstancePtr, RegOffset, Bits) \
	hcd_WriteReg((InstancePtr)->config.BaseAddress, RegOffset,	\
		hcd_ReadReg((InstancePtr)->config.BaseAddress, 	\
				RegOffset) & ~(Bits));


/*****************************************************************************/
/**
 * This macro starts the USB engine.
 *
 * @param	InstancePtr is a pointer to the hcd instance of the
 *		controller.
 *
 * @note	C-style signature:
 * 		void hcd_Start(hcd *InstancePtr)
 *
 ******************************************************************************/
#define hcd_Start(InstancePtr) \
	hcd_SetBits(InstancePtr, HCD_CMD_OFFSET, HCD_CMD_RS_MASK)


/*****************************************************************************/
/**
* This macro enables the interrupts defined by the bit mask.
*
* @param	InstancePtr is a pointer to hcd instance of the controller.
* @param	IntrMask is the Bit mask of interrupts to be enabled.
*
* @note		C-style signature:
*		void hcd_IntrEnable(hcd *InstancePtr, u32 IntrMask)
*
******************************************************************************/
#define hcd_IntrEnable(InstancePtr, IntrMask)	\
		hcd_SetBits(InstancePtr, HCD_IER_OFFSET, IntrMask)


#define LEDBASE_DATA 0x41200000 + 8
#define hcd_setLED(led)	\
		Xil_Out32(LEDBASE_DATA, Xil_In32(LEDBASE_DATA) | (1<<led));

#define hcd_clearLED(led)	\
		Xil_Out32(LEDBASE_DATA, Xil_In32(LEDBASE_DATA) & ~(1<<led));


/*Public methods*/
hcd_t * hcd_init(void);
int hcd_start(hcd_t *hcdPtr, XScuGic *IntcPtr);
int hcd_resetPort(hcd_t *hcdPtr);
int hcd_stop(hcd_t *hcdPtr);
int hcd_cleanup(hcd_t *hcdPtr);
void hcd_printEP0(void);
int hcd_connectClassHandler(hcd_t *hcdPtr, hcd_IntrHandlerFunc CallBackFunc,void *CallBackRef);
void hcd_sendSetupData(hcd_t *hcdPtr,hcd_endpoint0* ep0Ptr);
hcd_endpoint0* hcd_getEp0();

#ifdef __cplusplus
}
#endif

#endif /* HCD */
