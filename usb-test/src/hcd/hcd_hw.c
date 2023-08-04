#include "hcd.h"
#include "xstatus.h"
#include "xparameters.h"


/************************** Variable Definitions ****************************/
/**
 * Each XUsbPs device in the system has an entry in this table.
 */

hcd_config hcd_ConfigTable[] = {
	{
            0,
	    XPAR_XUSBPS_0_BASEADDR
	},
#ifdef XPAR_XUSBPS_1_BASEADDR
	{
            1,
	    XPAR_XUSBPS_1_BASEADDR
	}
#endif
};

/****************************************************************************/
/**
*
* Looks up the controller configuration based on the unique controller ID. A
* table contains the configuration info for each controller in the system.
*
* @param	DeviceID is the ID of the controller to look up the
*		configuration for.
*
* @return
*		A pointer to the configuration found or NULL if the specified
*		controller ID was not found.
*
******************************************************************************/
hcd_config *hcd_LookupConfig(u16 DeviceID)
{
	hcd_config *CfgPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XUSBPS_NUM_INSTANCES; Index++) {
		if (hcd_ConfigTable[Index].DeviceID == DeviceID) {
			CfgPtr = &hcd_ConfigTable[Index];
			break;
		}
	}

	return CfgPtr;
}

//! Byte swap unsigned short
uint16_t hcd_swap_uint16( uint16_t val )
{
    return (val << 8) | (val >> 8 );
}

//! Byte swap short
int16_t hcd_swap_int16( int16_t val )
{
    return (val << 8) | ((val >> 8) & 0xFF);
}

//! Byte swap unsigned int
uint32_t hcd_swap_uint32( uint32_t val )
{
    val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF );
    return (val << 16) | (val >> 16);
}

//! Byte swap int
int32_t hcd_swap_int32( int32_t val )
{
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF );
    return (val << 16) | ((val >> 16) & 0xFFFF);
}


