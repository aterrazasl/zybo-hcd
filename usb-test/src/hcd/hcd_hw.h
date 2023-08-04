
#ifndef HCD_HW_H
#define HCD_HW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hcd.h"
#include "xil_types.h"
#include <string.h>
#include "xil_io.h"


#define HCD_SETUP_TOKEN 0x02
#define HCD_IN_TOKEN 0x01
#define HCD_OUT_TOKEN 0x00


//bmRequestType
#define HOST_TO_DEVICE	(0<<7)
#define DEVICE_TO_HOST	(1<<7)
#define STANDARD_TYPE	(0<<5)
#define CLASS_TYPE		(1<<5)
#define VENDOR_TYPE		(2<<5)
#define RESERVED_TYPE	(3<<5)
#define DEVICE_RECIPIENT	(0<<0)
#define INTERFACE_RECIPIENT	(1<<0)
#define ENDPOINT_RECIPIENT	(2<<0)
#define OTHER_RECIPIENT		(3<<0)
#define RESERVED_RECIPIENT	(4<<0)


//standard request codes
#define GET_STATUS_REQUEST	0
#define CLEAR_FEATURE		1
#define RESERVED2			2
#define SET_FEATURE			3
#define RESERVED4			4
#define SET_ADDRESS			5
#define GET_DESCRIPTOR		6
#define SET_DESCRIPTOR		7
#define GET_CONFIGURATION	8
#define SET_CONFIGURATION	9
#define GET_INTERFACE		10
#define SET_INTERFACE		11
#define SYNCH_FRAME			12


//Descriptor types
#define DEVICE_TYPE			1
#define	CONFIGURATION		2
#define STRING				3
#define INTERFACE			4
#define ENDPOINT			5
#define DEVICE_QUALIFIER	6
#define OTHER				7
#define INTERFACE_POWER		8



/**< Alignment of the Device Queue Head List BASE. */
#define HCD_dQH_BASE_ALIGN		4096

/**< Alignment of a Device Queue Head structure. */
#define HCD_dQH_ALIGN		64

/**< Alignment of a Device Transfer Descriptor structure. */
#define HCD_dTD_ALIGN		32

/**< Size of one RX buffer for a OUT Transfer Descriptor. */
#define HCD_dTD_BUF_SIZE		4096

/**< Maximum size of one RX/TX buffer. */
#define HCD_dTD_BUF_MAX_SIZE	16*1024

/**< Alignment requirement for Transfer Descriptor buffers. */
#define HCD_dTD_BUF_ALIGN		4096

/** @name Operational Register offsets.
 * Reduced to only Host mode
 * @{
 */
#define HCD_CMD_OFFSET				0x00000140 /**< Configuration */
#define HCD_ISR_OFFSET				0x00000144 /**< Interrupt Status */
#define HCD_IER_OFFSET				0x00000148 /**< Interrupt Enable */
#define HCD_FRAME_OFFSET			0x0000014C /**< USB Frame Index */
#define HCD_LISTBASE_OFFSET			0x00000154 /**< H: Periodic List Base Address */
#define HCD_ASYNCLISTADDR_OFFSET	0x00000158 /**< H: Async List Address */
#define HCD_TTCTRL_OFFSET			0x0000015C /**< TT Control */
#define HCD_BURSTSIZE_OFFSET		0x00000160 /**< Burst Size */
#define HCD_TXFILL_OFFSET			0x00000164 /**< Tx Fill Tuning */
#define HCD_ULPIVIEW_OFFSET			0x00000170 /**< ULPI Viewport */
#define HCD_PORTSCR1_OFFSET			0x00000184 /**< Port Control/Status 1 */
#define HCD_MODE_OFFSET				0x000001A8 /**< USB Mode */

/** @name USB Mode Register (MODE) bit positions.
 *  @{
 */
#define HCD_MODE_CM_MASK			0x00000003 /**< Controller Mode Select */
#define HCD_MODE_CM_IDLE_MASK		0x00000000
#define HCD_MODE_CM_DEVICE_MASK		0x00000002
#define HCD_MODE_CM_HOST_MASK		0x00000003
#define HCD_MODE_ES_MASK			0x00000004 /**< USB Endian Select */
#define HCD_MODE_SLOM_MASK			0x00000008 /**< USB Setup Lockout Mode Disable */
#define HCD_MODE_SDIS_MASK			0x00000010
#define HCD_MODE_VALID_MASK			0x0000001F


/** @name USB Command Register (CR) bit positions.
 *  @{
 */
#define HCD_CMD_RS_MASK		0x00000001 /**< Run/Stop */
#define HCD_CMD_RST_MASK	0x00000002 /**< Controller RESET */
#define HCD_CMD_FS01_MASK	0x0000000C /**< Frame List Size bit 0,1 */
#define HCD_CMD_PSE_MASK	0x00000010 /**< Periodic Sched Enable */
#define HCD_CMD_ASE_MASK	0x00000020 /**< Async Sched Enable */
#define HCD_CMD_IAA_MASK	0x00000040 /**< IRQ Async Advance Doorbell */
#define HCD_CMD_ASP_MASK	0x00000300 /**< Async Sched Park Mode Cnt */
#define HCD_CMD_ASPE_MASK	0x00000800 /**< Async Sched Park Mode Enbl */
#define HCD_CMD_SUTW_MASK	0x00002000 /**< Setup TripWire */
#define HCD_CMD_ATDTW_MASK	0x00004000 /**< Add dTD TripWire */
#define HCD_CMD_FS2_MASK	0x00008000 /**< Frame List Size bit 2 */
#define HCD_CMD_ITC_MASK	0x00FF0000 /**< IRQ Threshold Control */
/* @} */

/** @name Port Status Control Register bit positions.
 *  @{
 */
#define HCD_PORTSCR_CCS_MASK  0x00000001 /**< Current Connect Status */
#define HCD_PORTSCR_CSC_MASK  0x00000002 /**< Connect Status Change */
#define HCD_PORTSCR_PE_MASK	  0x00000004 /**< Port Enable/Disable */
#define HCD_PORTSCR_PEC_MASK  0x00000008 /**< Port Enable/Disable Change */
#define HCD_PORTSCR_OCA_MASK  0x00000010 /**< Over-current Active */
#define HCD_PORTSCR_OCC_MASK  0x00000020 /**< Over-current Change */
#define HCD_PORTSCR_FPR_MASK  0x00000040 /**< Force Port Resume */
#define HCD_PORTSCR_SUSP_MASK 0x00000080 /**< Suspend */
#define HCD_PORTSCR_PR_MASK	  0x00000100 /**< Port Reset */
#define HCD_PORTSCR_HSP_MASK  0x00000200 /**< High Speed Port */
#define HCD_PORTSCR_LS_MASK	  0x00000C00 /**< Line Status */
#define HCD_PORTSCR_PP_MASK	  0x00001000 /**< Port Power */
#define HCD_PORTSCR_PO_MASK	  0x00002000 /**< Port Owner */
#define HCD_PORTSCR_PIC_MASK  0x0000C000 /**< Port Indicator Control */
#define HCD_PORTSCR_PTC_MASK  0x000F0000 /**< Port Test Control */
#define HCD_PORTSCR_WKCN_MASK 0x00100000 /**< Wake on Connect Enable */
#define HCD_PORTSCR_WKDS_MASK 0x00200000 /**< Wake on Disconnect Enable */
#define HCD_PORTSCR_WKOC_MASK 0x00400000 /**< Wake on Over-current Enable */
#define HCD_PORTSCR_PHCD_MASK 0x00800000 /**< PHY Low Power Suspend -
						* Clock Disable */
#define HCD_PORTSCR_PFSC_MASK 0x01000000 /**< Port Force Full Speed
						* Connect */
#define HCD_PORTSCR_PSPD_MASK 0x0C000000 /**< Port Speed */
/* @} */


/** @name USB Interrupt Status Register (ISR) / Interrupt Enable Register (IER)
 * bit positions.
 *  @{
 */
#define HCD_IXR_UI_MASK	0x00000001 /**< USB Transaction Complete */
#define HCD_IXR_UE_MASK	0x00000002 /**< Transaction Error */
#define HCD_IXR_PC_MASK	0x00000004 /**< Port Change Detect */
#define HCD_IXR_FRE_MASK	0x00000008 /**< Frame List Rollover */
#define HCD_IXR_AA_MASK	0x00000020 /**< Async Advance */
#define HCD_IXR_UR_MASK	0x00000040 /**< RESET Received */
#define HCD_IXR_SR_MASK	0x00000080 /**< Start of Frame */
#define HCD_IXR_SLE_MASK	0x00000100 /**< Device Controller Suspend */
#define HCD_IXR_ULPI_MASK	0x00000400 /**< ULPI IRQ */
#define HCD_IXR_HCH_MASK	0x00001000 /**< Host Controller Halted
						* Read Only */
#define HCD_IXR_RCL_MASK	0x00002000 /**< USB Reclamation  Read Only */
#define HCD_IXR_PS_MASK	0x00004000 /**< Periodic Sched Status
						* Read Only */
#define HCD_IXR_AS_MASK	0x00008000 /**< Async Sched Status Read only */
#define HCD_IXR_NAK_MASK	0x00010000 /**< NAK IRQ */
#define HCD_IXR_UA_MASK	0x00040000 /**< USB Host Async IRQ */
#define HCD_IXR_UP_MASK	0x00080000 /**< USB Host Periodic IRQ */
#define HCD_IXR_TI0_MASK	0x01000000 /**< Timer 0 Interrupt */
#define HCD_IXR_TI1_MASK	0x02000000 /**< Timer 1 Interrupt */

#define HCD_IXR_HOST_ALL			(0	| \
									 0		| \
									 HCD_IXR_PC_MASK	| \
									 HCD_IXR_FRE_MASK	| \
									 HCD_IXR_AA_MASK	| \
									 0		| \
									 0		| \
									 0	| \
									 HCD_IXR_ULPI_MASK		| \
									 0	| \
									 0	| \
									 0 | \
									 0		| \
									 0		| \
									 HCD_IXR_UA_MASK	| \
									 HCD_IXR_UP_MASK | \
									 0 | \
									 0)

#define HCD_IXR_ALL			(HCD_IXR_UI_MASK	| \
					 HCD_IXR_UE_MASK		| \
					 HCD_IXR_PC_MASK	| \
					 HCD_IXR_FRE_MASK	| \
					 HCD_IXR_AA_MASK	| \
					 HCD_IXR_UR_MASK		| \
					 HCD_IXR_SR_MASK		| \
					 HCD_IXR_SLE_MASK	| \
					 HCD_IXR_ULPI_MASK		| \
					 HCD_IXR_HCH_MASK	| \
					 HCD_IXR_RCL_MASK	| \
					 HCD_IXR_PS_MASK | \
					 HCD_IXR_AS_MASK		| \
					 HCD_IXR_NAK_MASK		| \
					 HCD_IXR_UA_MASK	| \
					 HCD_IXR_UP_MASK | \
					 HCD_IXR_TI0_MASK | \
					 HCD_IXR_TI1_MASK)
					/**< Mask for ALL IRQ types */
/* @} */


/****************************************************************************/
/**
*
* This macro reads the given register.
*
* @param	BaseAddress is the base address for the USB registers.
* @param	RegOffset is the register offset to be read.
*
* @return	The 32-bit value of the register.
*
* @note		C-style signature:
*		u32 HCD_ReadReg(u32 BaseAddress, u32 RegOffset)
*
*****************************************************************************/
#define hcd_ReadReg(BaseAddress, RegOffset) \
				Xil_In32(BaseAddress + (RegOffset))


/****************************************************************************/
/**
*
* This macro writes the given register.
*
* @param	BaseAddress is the the base address for the USB registers.
* @param	RegOffset is the register offset to be written.
* @param	Data is the the 32-bit value to write to the register.
*
* @return	None.
*
* @note		C-style signature:
*		void HCD_WriteReg(u32 BaseAddress, u32 RegOffset, u32 Data)
*
 *****************************************************************************/
#define hcd_WriteReg(BaseAddress, RegOffset, Data) \
				Xil_Out32(BaseAddress + (RegOffset), (Data))





hcd_config *hcd_LookupConfig(u16 DeviceID);
uint16_t	hcd_swap_uint16( uint16_t val );
int16_t 	hcd_swap_int16( int16_t val );
uint32_t 	hcd_swap_uint32( uint32_t val );
int32_t		hcd_swap_int32( int32_t val );


#ifdef __cplusplus
}
#endif

#endif /* HCD_HW_H */
