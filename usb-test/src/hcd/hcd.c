#include "hcd_hw.h"
#include "hcd.h"
#include "xscugic.h"



static hcd_t hcd;
static u8 hcd_HostBuffer[MEMORY_SIZE] ALIGNMENT_CACHELINE;	//todo: replace this with Dynamic memory

static hcd_standardInterfaceDescriptor 	ep0_interfaces[4];
static hcd_standardEndpointDescriptor 	ep0_endpoints[2*4];
static hcd_HIDkeyboardDescriptor 		ep0_hidDesc[2*4];

static hcd_endpoint0 ep0;

static void hcd_HostIntrHandler(void *HandlerRef);
static void hcd_UsbHostIntrHandler(void *CallBackRef, u32 Mask);
static void hcd_enumerationStateMachine(hcd_t *hcdPtr);

hcd_endpoint0* hcd_getEp0(){
	hcd_endpoint0 *ep0Ptr = &ep0;
	return ep0Ptr;
}

static void hcd_printLine(u8* data, u32 size, char* comment){
	int i;
	for(i = 0; i < size; i++){
		xil_printf("%02X ", *(data+i));
	}
	xil_printf(" // %s\r\n", comment);
}

void hcd_printEP0(void){
	int interface = 0;
	int endpoint = 0;
	int class = 0;
	hcd_endpoint0 *ep0Ptr  = hcd_getEp0();

	hcd_printLine( (u8*)(&ep0Ptr->deviceDescriptor), sizeof(hcd_standardDeviceDescriptor), "Device Descriptor" );
	hcd_printLine( (u8*)(&ep0Ptr->configurationDescriptor.stdConfiguration), sizeof(hcd_standardConfigurationDescriptor_st), "Configuration Descriptor" );

	for(interface = 0; interface < (ep0Ptr->configurationDescriptor.stdConfiguration.bNumInterfaces); interface ++){
		hcd_printLine( (u8*)(&ep0Ptr->configurationDescriptor.interfaces[interface]->stdInterface), sizeof(hcd_standardInterfaceDescriptor_st), "Interface Descriptor");

		for(class = 0; class < 1; class++){
			hcd_printLine( (u8*)((ep0Ptr->configurationDescriptor.interfaces[interface]->HIDdescriptors[class])), sizeof(hcd_HIDkeyboardDescriptor), "HID Class Descriptor");
		}
		for(endpoint = 0; endpoint < (ep0Ptr->configurationDescriptor.interfaces[interface]->stdInterface.bNumEndpoints); endpoint++){
			hcd_printLine( (u8*)((ep0Ptr->configurationDescriptor.interfaces[interface]->endpoints[endpoint])), sizeof(hcd_standardEndpointDescriptor), "Endpoint Descriptor");
		}

	}
}

static void hcd_setQueueBusy(u8 busy){
	hcd.busy = busy;
}

static u8 hcd_checkBusy(void){
	return hcd.busy;
}

static void hcd_disableAsyncList(hcd_t *hcdPtr){
	hcd_ClrBits(hcdPtr, HCD_CMD_OFFSET,HCD_CMD_IAA_MASK | HCD_CMD_ASE_MASK);
	hcd_setQueueBusy(NOT_BUSY);
}

static void hcd_enableAsyncList(hcd_t *hcdPtr){
	hcd_setQueueBusy(BUSY);
	hcd_SetBits(hcdPtr, HCD_CMD_OFFSET,HCD_CMD_IAA_MASK | HCD_CMD_ASE_MASK);
}

static int hcd_WriteLists(hcd_t *hcdPtr){

	/* Set the Queue Head List address. */
	hcd_WriteReg(hcdPtr->config.BaseAddress, HCD_ASYNCLISTADDR_OFFSET, (u32)hcdPtr->QH[1]);

	/* Set the Queue Head List address. */
	hcd_WriteReg(hcdPtr->config.BaseAddress, HCD_LISTBASE_OFFSET,(u32)hcdPtr->QH[0]);
	return 0;
}

static int hcd_configureUSBHost(hcd_t *hcdPtr){
	/* Set the USB mode register to configure HOST mode. */
	hcd_WriteReg(hcdPtr->config.BaseAddress, HCD_MODE_OFFSET, HCD_MODE_CM_HOST_MASK);

	return 0;
}

static void hcd_Initialize_Queues(hcd_t *hcdPtr){
	int i,j;
	u8 *p;

	p = (u8 *) hcdPtr->PhysAligned;



	//Initialize the QH pointers
	for (i = 0; i < HCD_MAX_QH; i++) {
		hcdPtr->QH[i] = (hcd_QH_st *)p;
		p += HCD_dQH_ALIGN;
	}
	Xil_DCacheFlush();

	//Initialize the dTD pointers
	for (i = 0; i < HCD_MAX_QTD; i++) {
		hcdPtr->qTD[i] = (hcd_qTD_st *)p;
		p += HCD_dTD_ALIGN;
	}
	Xil_DCacheFlush();

	p = (u8*)(((u32)p + HCD_dQH_BASE_ALIGN) & ~(HCD_dQH_BASE_ALIGN -1));

	//Initialize the qTD buffer pointers
	for (i = 0; i < HCD_MAX_QTD; i++) {
		for(j = 0; j < HCD_NUM_BUFFER_PTR; j++){
			hcdPtr->qTD[i]->buffer[j] = (u32)p;
			p += HCD_dQH_BASE_ALIGN;  //each buffer pointer is 4096 bytes size
		}
	}
	Xil_DCacheFlush();
}

static void hcd_physAlign(hcd_t *hcd_Ptr){

	/* Align the buffer to a 2048 byte (HCD_dQH_BASE_ALIGN) boundary.*/
	hcd_Ptr->PhysAligned = (hcd_Ptr->DMAMemPhys + HCD_dQH_BASE_ALIGN) & ~(HCD_dQH_BASE_ALIGN -1);
}

static int hcd_qTDEnque(hcd_qTD_st *hcdPtr, hcd_qTD_st *qTD){
	int status = 0;

	memcpy(hcdPtr, qTD,sizeof(hcd_qTD_st) - (5 * sizeof(u32)));
	Xil_DCacheFlush();
//	Xil_DCacheFlushRange((unsigned int)hcdPtr, sizeof(hcd_qTD_st));

	return status;
}

static int hcd_QHEnque(hcd_QH_st *hcdPtr, hcd_QH_st *QH){
	int status = 0;
	memcpy(hcdPtr, QH,sizeof(hcd_QH_st));
	Xil_DCacheFlush();
//	Xil_DCacheFlushRange((unsigned int)hcdPtr, sizeof(hcd_QH_st));

	return status;
}

static void hcd_QHInit(hcd_t *hcdPtr){

	//Initialize the QH pointers
	for (int i = 0; i < (HCD_MAX_QH) ; i++) {
		hcd_QHEnque(hcdPtr->QH[i], (hcd_QH_st *)&empty_QH);
	}
}

static void hcd_qTDInit(hcd_t *hcdPtr){

	//Initialize the QH pointers
	for (int i = 0; i < HCD_MAX_QTD; i++) {
		hcd_qTDEnque(hcdPtr->qTD[i], (hcd_qTD_st *)&empty_qTD);
	}
}

static void hcd_configureQueues(hcd_t *hcdPtr){


	/* Align the buffer to a 2048 byte (HDC_dQH_BASE_ALIGN) boundary.*/
	hcd_physAlign(hcdPtr);

	//Initilize the QH and qTD and point it to the aligned memory
	hcd_Initialize_Queues(hcdPtr);

	//Initialize QH pointer
	hcd_QHInit(hcdPtr);
	hcd_qTDInit(hcdPtr);
}

static void hcd_initialize_memory(hcd_t *hcdPtr, u8 *MemPtr){

	memset(MemPtr,0,MEMORY_SIZE);
	Xil_DCacheFlushRange((unsigned int)MemPtr, MEMORY_SIZE);
	hcdPtr->DMAMemPhys = (u32) MemPtr;

}

static int hcd_getUSBconfig (hcd_t *hcdPtr, u16 usbID){
	hcd_config		*UsbConfigPtr;

	UsbConfigPtr = hcd_LookupConfig(usbID);
	if (NULL == UsbConfigPtr) {
		return HCD_ERROR;
	}

	hcd.config = *UsbConfigPtr;


	return 0;
}

static void hcd_resetUSB(hcd_t *hcdPtr){

	hcd_WriteReg(hcdPtr->config.BaseAddress,	HCD_CMD_OFFSET, HCD_CMD_RST_MASK); // Reset USB to default values
	int done;
	do{
		done = HCD_CMD_RST_MASK & hcd_ReadReg(hcdPtr->config.BaseAddress,	HCD_CMD_OFFSET);
//		printDebugRegs2(UsbInstancePtr);
	}while(done != 0x0);
	xil_printf("USB controller Reset Complete... \r\n");
}

static void hcd_clearFlags(hcd_t *hcdPtr){

	hcdPtr->deviceConnected = 0;
	hcdPtr->speedDetected = 0;
	hcdPtr->state  = hcd_disconnected;
	hcdPtr->flags.data =0;

}

int hcd_connectClassHandler(hcd_t *hcdPtr, hcd_IntrHandlerFunc CallBackFunc,void *CallBackRef){
	Xil_AssertNonvoid(hcdPtr != NULL);

	hcdPtr->ClassHandlerFunc	= CallBackFunc;
	hcdPtr->ClassHandlerRef		= CallBackRef;
	hcdPtr->ClassHandlerMask	= HCD_IXR_ALL;

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
* Initialize hcdPtr with the required values
*
* @param   hcdPtr
*
* @return	errors
*
* @note     None.
*
******************************************************************************/
hcd_t * hcd_init(void){

	int status;
	hcd_t * hcdPtr = &hcd;			//pass the ref to hcd

	status = hcd_getUSBconfig(hcdPtr, USBDEVICEID);
	if(status == HCD_ERROR) return NULL;

	hcd_resetUSB(hcdPtr);
	hcd_configureUSBHost(hcdPtr);		//needs first to set the USB mode to host in order to write the queues

	hcd_initialize_memory(hcdPtr, hcd_HostBuffer);

	hcd_configureQueues(hcdPtr);

	hcd_WriteLists(hcdPtr);

	hcd_clearFlags(hcdPtr);


	return hcdPtr;
}

static int hcd_UsbHostSetupIntrSystem(XScuGic *IntcInstancePtr, hcd_t *hcdPtr, u16 UsbIntrId){

	int Status;
	XScuGic_Config *IntcConfig;

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	IntcConfig = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}
	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Xil_ExceptionInit();
	/*
	 * Connect the interrupt controller interrupt handler to the hardware
	 * interrupt handling logic in the processor.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
				    (Xil_ExceptionHandler)XScuGic_InterruptHandler,
				    IntcInstancePtr);
	/*
	 * Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */
	Status = XScuGic_Connect(IntcInstancePtr, UsbIntrId,
				(Xil_ExceptionHandler)hcd_HostIntrHandler,
				(void *)hcdPtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}
	/*
	 * Enable the interrupt for the device.
	 */
	XScuGic_Enable(IntcInstancePtr, UsbIntrId);

	/*
	 * Enable interrupts in the Processor.
	 */
	Xil_ExceptionEnableMask(XIL_EXCEPTION_IRQ);

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
* This function registers the user callback handler for controller
* (non-endpoint) interrupts.
*
* @param	InstancePtr is a pointer to the hcd instance of the
*		controller.
* @param	CallBackFunc is the Callback function to register.
*		CallBackFunc may be NULL to clear the entry.
* @param	CallBackRef is the user data reference passed to the
*		callback function. CallBackRef may be NULL.
* @param	Mask is the User interrupt mask. Defines which interrupts
*		will cause the callback to be called.
*
* @return
*		- XST_SUCCESS: Callback registered successfully.
*		- XST_FAILURE: Callback could not be registered.
*
* @note		None.
*
******************************************************************************/
static int hcd_IntrSetHandler(hcd_t *hcdPtr, hcd_IntrHandlerFunc CallBackFunc,void *CallBackRef, u32 Mask)
{
	Xil_AssertNonvoid(hcdPtr != NULL);

	hcdPtr->HandlerFunc	= CallBackFunc;
	hcdPtr->HandlerRef		= CallBackRef;
	hcdPtr->HandlerMask	= Mask;

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
* Enables interruptions and starts the USB controller
*
* @param   hcdPtr
*
* @return	errors
*
* @note     None.
*
******************************************************************************/
int hcd_start(hcd_t *hcdPtr, XScuGic *IntcPtr){

	int Status;
	/* Set up the interrupt subsystem.
	 */
	Status = hcd_UsbHostSetupIntrSystem(IntcPtr, hcdPtr, XPAR_XUSBPS_0_INTR);
	if (XST_SUCCESS != Status){
		return HCD_ERROR;
	}

	/* Set the handler for receiving frames. */
	Status = hcd_IntrSetHandler(hcdPtr, hcd_UsbHostIntrHandler, hcdPtr, HCD_IXR_ALL);
	if (XST_SUCCESS != Status) {
		return HCD_ERROR;
	}

	/* Start the USB engine */
	xil_printf("Starting hcd..... \r\n");
	hcd_Start(hcdPtr);
	while((hcd_ReadReg(hcdPtr->config.BaseAddress, HCD_CMD_OFFSET) & 0x1) != 0x1){
	}

	/* Enable the interrupts. */
	xil_printf("hcd_Intr enabling Interrupts... \r\n");
	hcd_IntrEnable(hcdPtr,  HCD_IXR_UI_MASK|HCD_IXR_UE_MASK|HCD_IXR_PC_MASK|HCD_IXR_AA_MASK|HCD_IXR_ULPI_MASK|HCD_IXR_HCH_MASK|HCD_IXR_RCL_MASK|HCD_IXR_AS_MASK|HCD_IXR_UA_MASK); //0x37


	hcd_SetBits(hcdPtr, HCD_PORTSCR1_OFFSET, HCD_PORTSCR_PP_MASK);  // Enable port power enable
	//todo Document the ULPI registers
	hcd_WriteReg(hcdPtr->config.BaseAddress, HCD_ULPIVIEW_OFFSET, 0x600a0067); // configures ULPI to enable 5V in the port


	return 0;
}

int hcd_resetPort(hcd_t *hcdPtr){
	hcd_WriteReg(hcdPtr->config.BaseAddress, HCD_PORTSCR1_OFFSET, HCD_PORTSCR_PR_MASK | hcd_ReadReg(hcdPtr->config.BaseAddress, HCD_PORTSCR1_OFFSET));
	return 0;
}

int hcd_stop(hcd_t *hcdPtr){  //todo implement stop USB
	return 0;
}

int hcd_cleanup(hcd_t *hcdPtr){		//todo implement the cleanup proces
	return 0;
}

static void hcd_changeAddress(u8 address){
	ep0.address =0;
}

static hcd_endpoint0* hcd_createGetDeviceDescriptor(void){

	hcd_endpoint0 *ep0Ptr = hcd_getEp0();

	ep0Ptr->setupData.bmRequestType = DEVICE_TO_HOST | STANDARD_TYPE | DEVICE_RECIPIENT;//0b10000000; // device to host, standard type, Device
	ep0Ptr->setupData.bRequest 		= GET_DESCRIPTOR;
	ep0Ptr->setupData.wValue 		= hcd_swap_uint16(DEVICE_TYPE);
	ep0Ptr->setupData.wIndex 		= hcd_swap_uint16(0x0000);

	if(ep0Ptr->deviceDescriptor.bLength == 0){
		ep0Ptr->setupData.wLength 		= sizeof(hcd_SetupData);
	}
	else{
		ep0Ptr->setupData.wLength 		= ep0Ptr->deviceDescriptor.bLength;
	}


	if(ep0Ptr->maxPacketSize == 0){
		ep0Ptr->maxPacketSize = 8;
	}
	else{
		ep0Ptr->maxPacketSize = ep0Ptr->deviceDescriptor.bMaxPacketSize0;
	}
	ep0Ptr->speed = 0;
	ep0Ptr->expectReply = 1;

	Xil_DCacheFlush();

	return ep0Ptr;
}

static hcd_endpoint0* hcd_createSetAddress(void){

	hcd_endpoint0 *ep0Ptr = hcd_getEp0();

	ep0Ptr->setupData.bmRequestType = HOST_TO_DEVICE | STANDARD_TYPE | DEVICE_RECIPIENT; //0b00000000; // host to device, standard type, Device
	ep0Ptr->setupData.bRequest 		= SET_ADDRESS;
	ep0Ptr->setupData.wValue 		= hcd_swap_uint16(DEVICE_TYPE);
	ep0Ptr->setupData.wIndex 		= hcd_swap_uint16(0x0000);
	ep0Ptr->setupData.wLength 		= (0);

	ep0Ptr->speed = 0;
	ep0Ptr->expectReply = 0;
	return ep0Ptr;
}

static hcd_endpoint0* hcd_createGetConfiguration(void){

	hcd_endpoint0 *ep0Ptr = hcd_getEp0();

	ep0Ptr->setupData.bmRequestType	= DEVICE_TO_HOST | STANDARD_TYPE | DEVICE_RECIPIENT; //0b00000000; // host to device, standard type, Device
	ep0Ptr->setupData.bRequest 		= GET_DESCRIPTOR;
	ep0Ptr->setupData.wValue 		= hcd_swap_uint16(CONFIGURATION);
	ep0Ptr->setupData.wIndex		= hcd_swap_uint16(0x0000);

	if(ep0Ptr->configurationDescriptor.stdConfiguration.wTotalLength == 0){
		ep0Ptr->setupData.wLength 		= sizeof(hcd_standardConfigurationDescriptor_st);
	}
	else{
		ep0Ptr->setupData.wLength 		= ep0Ptr->configurationDescriptor.stdConfiguration.wTotalLength;
	}

	ep0Ptr->speed = 0;
	ep0Ptr->expectReply = 1;
	return ep0Ptr;
}

static hcd_endpoint0* hcd_createGetStatus(void){

	hcd_endpoint0 *ep0Ptr = hcd_getEp0();

	ep0Ptr->setupData.bmRequestType	= DEVICE_TO_HOST | STANDARD_TYPE | DEVICE_RECIPIENT; //0b00000000; // host to device, standard type, Device
	ep0Ptr->setupData.bRequest 		= GET_STATUS_REQUEST;
	ep0Ptr->setupData.wValue 		= hcd_swap_uint16(0x0000);
	ep0Ptr->setupData.wIndex		= hcd_swap_uint16(0x0000);
	ep0Ptr->setupData.wLength 		= 2;

	ep0Ptr->speed = 0;
	ep0Ptr->expectReply = 1;
	return ep0Ptr;
}

static hcd_endpoint0* hcd_createSetConfiguration(u16 config){

	hcd_endpoint0 *ep0Ptr = hcd_getEp0();

	ep0Ptr->setupData.bmRequestType	= HOST_TO_DEVICE | STANDARD_TYPE | DEVICE_RECIPIENT; //0b00000000; // host to device, standard type, Device
	ep0Ptr->setupData.bRequest 		= SET_CONFIGURATION;
	ep0Ptr->setupData.wValue 		= config; //hcd_swap_uint16(config);
	ep0Ptr->setupData.wIndex		= hcd_swap_uint16(0x0000);
	ep0Ptr->setupData.wLength 		= 0;

	ep0Ptr->speed = 0;
	ep0Ptr->expectReply = 1;
	return ep0Ptr;
}

void hcd_sendSetupData(hcd_t *hcdPtr,hcd_endpoint0* ep0Ptr){

	hcd_configureQueues(hcdPtr);


	hcd_QH_st QH;
	hcd_QH_endpoint_word1 w1;
	hcd_QH_endpoint_word2 w2;

	#define USB_SPEED 0x02

	/////////////////////////setup qTDs
	hcd_qTD_st qTD;
	hcd_qTD_token token;

	///// setup the next qTD
	qTD.nextqTD 				= (int)hcdPtr->qTD[1] | 0x0;
	qTD.nextqTD_alt 			= 0x1;
	token.st = (hcd_qTD_token_st){0x80,HCD_SETUP_TOKEN,0,0,0,sizeof(hcd_SetupData),0};
	qTD.token					= token.data;
	hcd_qTDEnque(hcdPtr->qTD[0], &qTD);
	memcpy((void*)(hcdPtr->qTD[0]->buffer[0]&0xfffff000), ep0Ptr, sizeof(hcd_SetupData));
	Xil_DCacheFlush();

	if(ep0Ptr->expectReply){
		///// setup the next qTD
		qTD.nextqTD 				= (int)hcdPtr->qTD[2] | 0x0;
		qTD.nextqTD_alt 			= 0x1;
		token.st 					= (hcd_qTD_token_st){0x80,HCD_IN_TOKEN,0,0,0,(ep0Ptr->setupData.wLength),1};
		qTD.token					= token.data;
		hcd_qTDEnque(hcdPtr->qTD[1], &qTD);
		Xil_DCacheFlush();

//		xil_printf("setup length = %08X\r\n",ep0Ptr->setupData.wLength);

		///// setup the next qTD
		qTD.nextqTD 				= (int)hcdPtr->qTD[3] | 0x0;
		qTD.nextqTD_alt 			= 0x1;
		token.st = (hcd_qTD_token_st){0x80,HCD_OUT_TOKEN,0,0,1,0x0,1};
		qTD.token					= token.data;
		hcd_qTDEnque(hcdPtr->qTD[2], &qTD);
		Xil_DCacheFlush();

	}
	else{
		///// setup the next qTD
		qTD.nextqTD 				= (int)hcdPtr->qTD[3] | 0x0;
		qTD.nextqTD_alt 			= 0x1;
		token.st 					= (hcd_qTD_token_st){0x80,HCD_IN_TOKEN,0,0,0,0,1};
		qTD.token					= token.data;
		hcd_qTDEnque(hcdPtr->qTD[1], &qTD);
		Xil_DCacheFlush();
	}

	///// setup the next qTD
	qTD.nextqTD 				= (int)hcdPtr->qTD[3] | 0x1;
	qTD.nextqTD_alt 			= 0x1;
	token.st = (hcd_qTD_token_st){0x00,0x0,0,0,1,0x0,0};
	qTD.token					= token.data;
	hcd_qTDEnque(hcdPtr->qTD[3], &qTD);
	Xil_DCacheFlush();



	////// setup periodic list
	memcpy((void*)hcdPtr->QH[0] , &empty_QH, sizeof(hcd_QH_st));
	Xil_DCacheFlush();


	memcpy(&QH , &empty_QH, sizeof(hcd_QH_st));
	w1.st = (hcd_QH_endpoint_word1_st){ep0Ptr->address,0,0,USB_SPEED,0,1,ep0Ptr->maxPacketSize,0,0xf};
	w2.st = (hcd_QH_endpoint_word2_st){0,0,0,0,1};
	QH.nextQH = (int)hcdPtr->QH[1] | 0x0 | 0x2;
	QH.endpoint_word1 		= w1.data;
	QH.endpoint_word2 		= w2.data;
	QH.overlay.nextqTD 		= (int)hcdPtr->qTD[0] | 0x0;
	QH.overlay.nextqTD_alt 	= 0x1;
	hcd_QHEnque(hcdPtr->QH[1], &QH);




	hcd_enableAsyncList(hcdPtr);
}

static hcd_endpoint0*  hcd_parseDeviceDescriptor(hcd_t *hcdPtr){
	hcd_endpoint0 *ep0Ptr  = hcd_getEp0();

	hcd_standardDeviceDescriptor* stdDevDesc =(hcd_standardDeviceDescriptor*)(hcdPtr->qTD[1]->buffer[0] & 0xfffff000);
	ep0Ptr->deviceDescriptor = *stdDevDesc;

	return ep0Ptr;
}

static hcd_endpoint0*  hcd_parseConfigurationDescriptor(hcd_t *hcdPtr){
	hcd_endpoint0 *ep0Ptr  = hcd_getEp0();
	void* ptr = NULL;
	static u8 descriptors[4096];	//todo: calculate based on max configs/interface/endpoints supported

	hcd_standardConfigurationDescriptor_st* confDevDesc =(hcd_standardConfigurationDescriptor_st*)(hcdPtr->qTD[1]->buffer[0] & 0xfffff000);
	ep0Ptr->configurationDescriptor.stdConfiguration = *confDevDesc;

	memcpy(&descriptors, (void*)(hcdPtr->qTD[1]->buffer[0] & 0xfffff000), confDevDesc->wTotalLength);

	int i = 0;
	int interface 	= -1;
	int endpoint 	= -1;
	int hid 		= -1;
	int interface_b 	= 0;
	int endpoint_b 		= 0;
	int hid_b 			= 0;

	do{
		ptr = &descriptors[i];
		switch (descriptors[i+1]){
		case (0x04):
				interface++;
				endpoint=-1;
				hid = -1;
				memcpy(&ep0_interfaces[interface_b].stdInterface, ptr, descriptors[i]);
				ep0Ptr->configurationDescriptor.interfaces[interface]=  &ep0_interfaces[interface_b++];
			break;
		case (0x05):
				endpoint++;
				memcpy(&ep0_endpoints[endpoint_b], ptr, descriptors[i]);
				ep0Ptr->configurationDescriptor.interfaces[interface]->endpoints[endpoint] = &ep0_endpoints[endpoint_b++];
			break;
		case (0x21):
				hid++;
				memcpy(&ep0_hidDesc[hid_b], ptr, descriptors[i]);
				ep0Ptr->configurationDescriptor.interfaces[interface]->HIDdescriptors[hid] = &ep0_hidDesc[hid_b++];//todo fix the issue with HID
			break;
		default:
			break;
		}
		i = i + descriptors[i];
		if(descriptors[i] == 0x0){
			break;
		}
	}while(i < confDevDesc->wTotalLength);


	return ep0Ptr;
}

static hcd_endpoint0*  hcd_parseStatus(hcd_t *hcdPtr){
	hcd_endpoint0 *ep0Ptr  = hcd_getEp0();

	u16* devStatus =(u16*)(hcdPtr->qTD[1]->buffer[0] & 0xfffff000);
	ep0Ptr->deviceStatus = *devStatus;

	return ep0Ptr;
}

static void hcd_enumerationStateMachine(hcd_t *hcdPtr){

	switch (hcdPtr->state){
	case(hcd_disconnected):
		break;
	case(hcd_powered):
		hcd_resetPort(hcdPtr);
		hcdPtr->state = hcd_getDeviceDescriptor;
	break;

	case(hcd_getDeviceDescriptor):			// request the first 8 bytes of the device descriptor
		if (hcd_checkBusy() == BUSY){
			hcdPtr->state = hcd_reset;
		}
		else{
			hcd_sendSetupData(hcdPtr,hcd_createGetDeviceDescriptor());
		}
	break;

	case(hcd_reset):						// reset the device
		if (hcd_checkBusy() == BUSY){
			hcdPtr->state = hcd_reset;
		}
		else{
			hcd_parseDeviceDescriptor(hcdPtr);
			hcd_resetPort(hcdPtr);
			hcdPtr->state = hcd_setAddress;
		}
	break;

	case(hcd_setAddress):
		if (hcd_checkBusy() == BUSY){
			hcdPtr->state = hcd_setAddress;
		}
		else{
			hcd_sendSetupData(hcdPtr,hcd_createSetAddress());
			hcdPtr->state = hcd_getDeviceDescriptorFull;
		}
	break;

	case(hcd_getDeviceDescriptorFull):		// request the full length of the device descriptor
		if (hcd_checkBusy() == BUSY){
			hcdPtr->state = hcd_getDeviceDescriptorFull;
		}
		else{

			hcd_changeAddress(1);
			hcd_sendSetupData(hcdPtr,hcd_createGetDeviceDescriptor());
			hcdPtr->state = hcd_getConfiguration;
		}
	break;

	case(hcd_getConfiguration):		// request the full length of the device descriptor
		if (hcd_checkBusy() == BUSY){
			hcdPtr->state = hcd_getConfiguration;
		}
		else{
			hcd_parseDeviceDescriptor(hcdPtr);
			hcd_sendSetupData(hcdPtr,hcd_createGetConfiguration());
			hcdPtr->state = hcd_getConfigurationFull;
		}
	break;

	case(hcd_getConfigurationFull):		// request the full length of the device descriptor
		if (hcd_checkBusy() == BUSY){
			hcdPtr->state = hcd_getConfigurationFull;
		}
		else{
			hcd_parseConfigurationDescriptor(hcdPtr);
			hcd_sendSetupData(hcdPtr,hcd_createGetConfiguration());
			hcdPtr->state = hcd_default;
		}
	break;

	case(hcd_default):
		if (hcd_checkBusy() == BUSY){
			hcdPtr->state = hcd_default;
		}
		else{
			hcd_parseConfigurationDescriptor(hcdPtr);
			hcd_printEP0();
			hcdPtr->state = hcd_getStatus;
			hcd_sendSetupData(hcdPtr,hcd_createGetStatus());
		}
	break;
	case(hcd_getStatus):
		if (hcd_checkBusy() == BUSY){
			hcdPtr->state = hcd_getStatus;
		}
		else{
			hcd_parseStatus(hcdPtr);
			hcd_sendSetupData(hcdPtr,hcd_createSetConfiguration(1));
			hcdPtr->state = hcd_configured;
		}
	break;

	case(hcd_configured):
		if (hcd_checkBusy() == BUSY){
			hcdPtr->state = hcd_configured;
		}
		else{
			hcdPtr->state = hcd_idle;
		}
	break;

	case(hcd_idle):
		hcdPtr->state = hcd_idle;
	break;

	default:
		print("State not defined \r\n");
		break;
	}

}

static void hcd_HostIntrHandler(void *HandlerRef)
{

	hcd_t	*hcdPtr;

	u32	IrqSts;

	Xil_AssertVoid(HandlerRef != NULL);

	hcdPtr = (hcd_t *) HandlerRef;

	/* Handle controller (non-endpoint) related interrupts. */
	IrqSts = hcd_ReadReg(hcdPtr->config.BaseAddress, HCD_ISR_OFFSET);

	/* Clear the interrupt status register. */
	hcd_WriteReg(hcdPtr->config.BaseAddress, HCD_ISR_OFFSET, IrqSts);


	/* Check if we have a user handler that needs to be called. Note that
	 * this is the handler for general interrupts. Endpoint interrupts will
	 * be handled below.
	 */
	if ((IrqSts & hcdPtr->HandlerMask) && hcdPtr->HandlerFunc) {
		(hcdPtr->HandlerFunc)(hcdPtr->HandlerRef, IrqSts);
	}

	if(hcdPtr->deviceConnected == 1){
		hcd_enumerationStateMachine(hcdPtr);
	}


/*class specific interrupt handler
 *
 */
	if(hcdPtr->state == hcd_idle){
		if ((IrqSts & hcdPtr->ClassHandlerMask) && hcdPtr->ClassHandlerFunc) {
			(hcdPtr->ClassHandlerFunc)(hcdPtr->ClassHandlerRef, IrqSts);
		}
	}

}

static void hcd_UsbHostIntrHandler(void *CallBackRef, u32 Mask)
{
//	u32 ulpi_reg= 0;
	static int HCD_IXR_HCH=0;
	static int HCD_IXR_HCH_old=0;

	static int count = 0;
	static int qTDcount = 0;
	count ++;

	hcd_t *hcdPtr = (hcd_t*)CallBackRef;

	if(Mask & HCD_IXR_ULPI_MASK ){
		xil_printf("[Interrupt] Register HCD_ULPIVIEW_OFFSET = %08X \r\n",hcd_ReadReg(hcdPtr->config.BaseAddress,	HCD_ULPIVIEW_OFFSET));
	}

	if(Mask & HCD_IXR_PC_MASK ){
		u32 portStatus = hcd_ReadReg(hcdPtr->config.BaseAddress,HCD_PORTSCR1_OFFSET);

		xil_printf("[Interrupt] Port changed... mask = %08X; count = %d\r\n",Mask,count);
		xil_printf("[Interrupt] Register HCD_PORTSCR1_OFFSET = %08X \r\n",portStatus);

		if(  (portStatus & HCD_PORTSCR_CCS_MASK) == HCD_PORTSCR_CCS_MASK ){
			if(hcdPtr->deviceConnected ==0){
				hcdPtr->deviceConnected = 1;
				hcdPtr->speedDetected = portStatus >> 26;
				hcdPtr->state  = hcd_powered;
				hcdPtr->flags.data =0;
			}
		}
		else {
			hcdPtr->deviceConnected = 0;
			hcdPtr->speedDetected = portStatus >> 26;;
			hcdPtr->state = hcd_disconnected;
		}
	}

	if(Mask & HCD_IXR_UI_MASK ){
//		xil_printf("[Interrupt] Transaction complete.. mask = %08X; count = %d\r\n",Mask,count);
//		AsyncScheduleTransactionComplete =1;
	}
	if(Mask & HCD_IXR_UE_MASK ){
		xil_printf("[Interrupt] Transaction Error.. mask = %08X; count = %d\r\n",Mask,count);
	}
	if(Mask & HCD_IXR_HCH_MASK ){
		HCD_IXR_HCH =1;
		if (HCD_IXR_HCH > HCD_IXR_HCH_old){
			xil_printf("[Interrupt] Host Controller Halted = 1.. mask = %08X; count = %d\r\n",Mask,count);
			HCD_IXR_HCH_old = 1;
		}
	}
	else{
		HCD_IXR_HCH =0;
		if (HCD_IXR_HCH < HCD_IXR_HCH_old){
			xil_printf("[Interrupt] Host Controller Halted = 0.. mask = %08X; count = %d\r\n",Mask,count);
			HCD_IXR_HCH_old = 0;
		}
	}
	hcd_clearLED(1);
	if(Mask & HCD_IXR_RCL_MASK ){
		hcd_setLED(1);
//		xil_printf("[Interrupt] Host Reclamation status = 1... mask = %08X; count = %d\r\n",Mask,count);
	}

	if(Mask & HCD_IXR_AA_MASK ){
//		xil_printf("[Interrupt] Async schedule advance. mask = %08X; count = %d\r\n",Mask,count);
//		AsyncScheduleAdvance =1;
	}
	if(Mask & HCD_IXR_SR_MASK ){
//		xil_printf("[Interrupt] Start of Frame. \r\n");
	}
	if(Mask & HCD_IXR_AS_MASK){
//			xil_printf("[Interrupt] Async Sched Enabled!!. mask = %08X; count = %d\r\n",Mask,count);
	}

	if(Mask & HCD_IXR_UA_MASK ){
		qTDcount++;
//		xil_printf("[Interrupt] USB Transaction complete!. mask = %08X; count = %d; qTDCount = %d\r\n",Mask,count,qTDcount);
		hcd_disableAsyncList(hcdPtr);

	}

}

