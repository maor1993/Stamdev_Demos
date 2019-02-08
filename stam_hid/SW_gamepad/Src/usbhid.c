/*
 * usbhid.c
 *
 *  Created on: 15 Dec 2018
 *      Author: Maor
 */

#include "stm32.h"
#include "usb.h"
#include "usb_hid.h"
#include <stdint.h>

#define CDC_EP0_SIZE    0x08

#define HID_RPT_EP      0x81
#define CDC_NTF_SZ      0x08


static const struct usb_string_descriptor lang_desc     = USB_ARRAY_DESC(USB_LANGID_ENG_US);
static const struct usb_string_descriptor manuf_desc_en = USB_STRING_DESC("USB stack for STM32");
static const struct usb_string_descriptor prod_desc_en  = USB_STRING_DESC("STM32 Gamepad");
static const struct usb_string_descriptor *const dtable[] = {
    &lang_desc,
    &manuf_desc_en,
    &prod_desc_en,
};

usbd_device udev;
static uint32_t	ubuf[0x20];
//uint32_t    fpos = 0;

static uint8_t idle_rate = 500 / 4; // see HID1_11.pdf sect 7.2.4
static uint8_t protocol_version = 0; // see HID1_11.pdf sect 7.2.6

typedef struct
{
	uint8_t report_id;
	uint8_t buttons;
	int8_t lx;
	int8_t ly;
	int8_t rx;
	int8_t ry;
} gamepad_report_t;

volatile gamepad_report_t gamepad;

static const uint8_t gamepad_desc[] =
{
	    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
	    0x09, 0x05,                    // USAGE (Game Pad)
	    0xa1, 0x01,                    // COLLECTION (Application)
	    0xa1, 0x00,                    //   COLLECTION (Physical)
	    0x85, 0x01,                    //     REPORT_ID (4)
	    0x05, 0x09,                    //     USAGE_PAGE (Button)
	    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
	    0x29, 0x08,                    //     USAGE_MAXIMUM (Button 8)
	    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
	    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
	    0x95, 0x08,                    //     REPORT_COUNT (8)
	    0x75, 0x01,                    //     REPORT_SIZE (1)
	    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
	    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
	    0x09, 0x30,                    //     USAGE (X)
	    0x09, 0x31,                    //     USAGE (Y)
	    0x09, 0x32,                    //     USAGE (Z)
	    0x09, 0x33,                    //     USAGE (Rx)
	    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
	    0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
	    0x75, 0x08,                    //     REPORT_SIZE (8)
	    0x95, 0x04,                    //     REPORT_COUNT (4)
	    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
	    0xc0,                          //     END_COLLECTION
		0xc0,							// end?

};



struct hid_config {
    struct usb_config_descriptor        config;
    struct usb_interface_descriptor     comm;
    struct usb_hid_descriptor          hid_hdr;
    struct usb_endpoint_descriptor      comm_ep;
} __attribute__((packed));

static const struct usb_device_descriptor hiddevice_desc = {
    .bLength            = sizeof(struct usb_device_descriptor),
    .bDescriptorType    = USB_DTYPE_DEVICE,
    .bcdUSB             = VERSION_BCD(2,0,0),
    .bDeviceClass       = USB_CLASS_PER_INTERFACE,
    .bDeviceSubClass    = USB_SUBCLASS_NONE,
    .bDeviceProtocol    = USB_PROTO_NONE,
    .bMaxPacketSize0    = CDC_EP0_SIZE,
    .idVendor           = 0x16c0,
    .idProduct          = 0x05df,
    .bcdDevice          = VERSION_BCD(1,0,0),
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = INTSERIALNO_DESCRIPTOR,
    .bNumConfigurations = 1,
};

static const struct hid_config config_desc = {
    .config = {
        .bLength                = sizeof(struct usb_config_descriptor),
        .bDescriptorType        = USB_DTYPE_CONFIGURATION,
        .wTotalLength           = sizeof(struct hid_config),
        .bNumInterfaces         = 1,
        .bConfigurationValue    = 1,
        .iConfiguration         = NO_DESCRIPTOR,
        .bmAttributes           = USB_CFG_ATTR_RESERVED | USB_CFG_ATTR_SELFPOWERED,
        .bMaxPower              = USB_CFG_POWER_MA(500),
    },
    .comm = {
        .bLength                = sizeof(struct usb_interface_descriptor),
        .bDescriptorType        = USB_DTYPE_INTERFACE,
        .bInterfaceNumber       = 0,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 1,
        .bInterfaceClass        = USB_CLASS_HID,
        .bInterfaceSubClass     = USB_HID_SUBCLASS_NONBOOT,
        .bInterfaceProtocol     = USB_HID_PROTO_NONBOOT,
        .iInterface             = NO_DESCRIPTOR,
    },

    .hid_hdr = {
        .bLength        = sizeof(struct usb_hid_descriptor),
        .bDescriptorType        = USB_DTYPE_HID,
		.bcdHID					=VERSION_BCD(1,1,0),
		.bCountryCode			=USB_HID_COUNTRY_NONE,
		.bNumDescriptors        =1,
        .bDescriptorType0		=USB_DTYPE_HID_REPORT,
		.wDescriptorLength0     =sizeof (gamepad_desc),
    },



    .comm_ep = {
        .bLength                = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType        = USB_DTYPE_ENDPOINT,
        .bEndpointAddress       = HID_RPT_EP,
        .bmAttributes           = USB_EPTYPE_INTERRUPT,
        .wMaxPacketSize         = CDC_NTF_SZ,
        .bInterval              = 50,
    },


};



static usbd_respond hid_getdesc (usbd_ctlreq *req, void **address, uint16_t *length) {
    const uint8_t dtype = req->wValue >> 8;
    const uint8_t dnumber = req->wValue & 0xFF;
    const void* desc;
    uint16_t len = 0;
    switch (dtype) {
    case USB_DTYPE_DEVICE:
        desc = &hiddevice_desc;
        break;
    case USB_DTYPE_CONFIGURATION:
        desc = &config_desc;
        len = sizeof(config_desc);
        break;
    case USB_DTYPE_STRING:
        if (dnumber < 3) {
            desc = dtable[dnumber];
        } else {
            return usbd_fail;
        }
        break;

    default:
        return usbd_fail;
    }
    if (len == 0) {
        len = ((struct usb_header_descriptor*)desc)->bLength;
    }
    *address = (void*)desc;
    *length = len;
    return usbd_ack;
};


static usbd_respond hid_control(usbd_device *dev, usbd_ctlreq *req, usbd_rqc_callback *callback) {
    if (((USB_REQ_RECIPIENT | USB_REQ_TYPE) & req->bmRequestType) == (USB_REQ_INTERFACE | USB_REQ_CLASS)) {
		switch (req->bRequest) {
		case USB_HID_GETIDLE:
			dev->status.data_ptr = &idle_rate;
			dev->status.data_count = 1;
			return usbd_ack;
		case USB_HID_GETPROTOCOL:
			dev->status.data_ptr = &protocol_version;
			dev->status.data_count = 1;
			return usbd_ack;
		case USB_HID_SETIDLE:
			idle_rate = req->data[0];
			return usbd_ack;
		case USB_HID_SETPROTOCOL:
			protocol_version = req->data[0];
			return usbd_ack;
		case USB_HID_SETREPORT:
			return usbd_ack;

		default:

			return usbd_fail;
		}
    }

    // standard request
    else if (((USB_REQ_RECIPIENT | USB_REQ_TYPE) & req->bmRequestType) == (USB_REQ_INTERFACE | USB_REQ_STANDARD)) {
        switch (req->wValue >> 8) {
            case USB_DTYPE_HID_REPORT:
    			dev->status.data_ptr = gamepad_desc;
    			dev->status.data_count = sizeof(gamepad_desc);
               return usbd_ack;

         default:
             return usbd_fail;
        }
    }

}


void hid_tx(usbd_device *dev, uint8_t event, uint8_t ep)
{

	static uint8_t bMsgwaiting =0;

	gamepad.report_id = 1;

	gamepad.buttons++;

	if(usbd_ep_write(dev,ep,(uint8_t*)&gamepad.report_id,6)>0)
	{
		bMsgwaiting = 1;
	}
}


static usbd_respond cdc_setconf (usbd_device *dev, uint8_t cfg) {
    switch (cfg) {
    case 0:
        /* deconfiguring device */
        usbd_ep_deconfig(dev, HID_RPT_EP);
//        usbd_ep_deconfig(dev, CDC_TXD_EP);
//        usbd_ep_deconfig(dev, CDC_RXD_EP);
        usbd_reg_endpoint(dev, HID_RPT_EP, 0);
//        usbd_reg_endpoint(dev, CDC_TXD_EP, 0);
        return usbd_ack;
    case 1:
        /* configuring device */
        usbd_ep_config(dev, HID_RPT_EP, USB_EPTYPE_INTERRUPT, CDC_NTF_SZ);

        usbd_reg_endpoint(dev, HID_RPT_EP, hid_tx);
        usbd_ep_write(dev,HID_RPT_EP,0,0);
//        usbd_reg_endpoint(dev, CDC_TXD_EP, cdc_txonly);
//        usbd_ep_write(dev, CDC_TXD_EP, 0, 0);
        return usbd_ack;
    default:
        return usbd_fail;
    }
}




void hid_init_usbd(void) {
    usbd_init(&udev, &usbd_hw, CDC_EP0_SIZE, ubuf, sizeof(ubuf));
    usbd_reg_config(&udev, cdc_setconf);
    usbd_reg_control(&udev, hid_control);
    usbd_reg_descr(&udev, hid_getdesc);
}

#define USB_HANDLER     USB_IRQHandler
#define USB_NVIC_IRQ    USB_IRQn


void USB_HANDLER(void) {
    usbd_poll(&udev);
}

