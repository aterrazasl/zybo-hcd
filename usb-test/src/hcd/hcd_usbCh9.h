
#ifndef HCD_USBCH9_H
#define HCD_USBCH9_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hcd.h"

typedef struct {
	u8  bmRequestType;	/**< bmRequestType in setup data */
	u8  bRequest;		/**< bRequest in setup data */
	u16 wValue;		/**< wValue in setup data */
	u16 wIndex;		/**< wIndex in setup data */
	u16 wLength;		/**< wLength in setup data */
}
hcd_SetupData;

typedef struct {
	u8 bLength;
	u8 bDescriptorType;
	u16 bcdUSB;
	u8 bDeviceClass;
	u8 bDeviceSubClass;
	u8 bDeviceProtocol;
	u8 bMaxPacketSize0;
	u16 idVendor;
	u16 idProduct;
	u16 bcdDevice;
	u8 iManufacturer;
	u8 iProduct;
	u8 iSerialNumber;
	u8 bNumConfigurations;
}hcd_standardDeviceDescriptor;

typedef struct {
	u8 bLength;
	u8 bDescriptorType;
	u8 bEndpointAddress;
	u8 bmAttributes;
	u16 wMaxPacketSize;
	u8 bInterval;
}hcd_standardEndpointDescriptor;


typedef struct {
	u8 bLength;
	u8 bDescriptorType;
	u16 bcdHID;
	u8 bCountryCode;
	u8 bNumbDescriptors;
	u8 bReportType;
	u8 wDescriptorType;
	u8 wDescriptorType2;
}hcd_HIDkeyboardDescriptor;

typedef struct {
	u8 bLength;
	u8 bDescriptorType;
	u8 bInterfaceNumber;
	u8 bAlternateSetting;
	u8 bNumEndpoints;
	u8 bInterfaceClass;
	u8 bInterfaceSubClass;
	u8 bInterfaceProtocol;
	u8 iInterface;
}hcd_standardInterfaceDescriptor_st;

typedef struct {
	hcd_standardInterfaceDescriptor_st stdInterface;
	hcd_standardEndpointDescriptor * endpoints[4];
	hcd_HIDkeyboardDescriptor * HIDdescriptors[4];
}hcd_standardInterfaceDescriptor;

typedef struct {
	u8 bLength;
	u8 bDescriptorType;
	u16 wTotalLength;
	u8 bNumInterfaces;
	u8 bConfigurationValue;
	u8 iConfiguration;
	u8 bmAttributes;
	u8 bMaxPower;
}hcd_standardConfigurationDescriptor_st;

typedef struct {
	hcd_standardConfigurationDescriptor_st stdConfiguration;
	hcd_standardInterfaceDescriptor * interfaces[4];
}hcd_standardConfigurationDescriptor;

typedef struct {
	hcd_SetupData setupData;
	hcd_standardDeviceDescriptor deviceDescriptor;
	u8 maxPacketSize;
	u8 address;
	u8 speed;
	u8 expectReply;
	hcd_standardConfigurationDescriptor configurationDescriptor;
	u16 deviceStatus;

}hcd_endpoint0;


#ifdef __cplusplus
}
#endif

#endif /* HCD_USBCH9_H */
