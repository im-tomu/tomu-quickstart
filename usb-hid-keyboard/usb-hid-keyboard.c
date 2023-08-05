/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2018 Seb Holzapfel <schnommus@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * \addtogroup Examples
 *
 * This example implements a USB Human Interface Device (HID)
 * to demonstrate the use of the USB device stack.
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <libopencm3/efm32/wdog.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/cmu.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <toboot.h>

/* Declare support for Toboot V2 */
/* To enable this program to run when you first plug in Tomu, pass
 * TOBOOT_CONFIG_FLAG_AUTORUN to this macro.  Otherwise, leave the
 * configuration value at 0 to use the defaults.
 */
// TOBOOT_CONFIGURATION(TOBOOT_CONFIG_FLAG_AUTORUN);
TOBOOT_CONFIGURATION(0);

#define SYSTICK_FREQUENCY 100
#define AHB_FREQUENCY 14000000
#define LED_GREEN_PORT GPIOA
#define LED_GREEN_PIN  GPIO0
#define LED_RED_PORT   GPIOB
#define LED_RED_PIN    GPIO7
#define VENDOR_ID                 0x1209    /* pid.code */
#define PRODUCT_ID                0x70b1    /* Assigned to Tomu project */
#define DEVICE_VER                0x0101    /* Program version */

// Declare functions
void injkeys(char *source,uint8_t mod);

bool g_usbd_is_connected = false;
bool once=true;
usbd_device *g_usbd_dev = 0;

static const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = VENDOR_ID,
	.idProduct = PRODUCT_ID,
	.bcdDevice = DEVICE_VER,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const uint8_t hid_report_descriptor[] = {
 0x05, 0x01, // USAGE_PAGE (Generic Desktop)
        0x09, 0x06, // USAGE (Keyboard)
        0xa1, 0x01, // COLLECTION (Application)
        0x05, 0x07, // USAGE_PAGE (Keyboard)
        0x19, 0xe0, // USAGE_MINIMUM (Keyboard LeftControl)
        0x29, 0xe7, // USAGE_MAXIMUM (Keyboard Right GUI)
        0x15, 0x00, // LOGICAL_MINIMUM (0)
        0x25, 0x01, // LOGICAL_MAXIMUM (1)
        0x75, 0x01, // REPORT_SIZE (1)
        0x95, 0x08, // REPORT_COUNT (8)
        0x81, 0x02, // INPUT (Data,Var,Abs) //1 byte
        0x95, 0x01, // REPORT_COUNT (1)
        0x75, 0x08, // REPORT_SIZE (8)
        0x81, 0x03, // INPUT (Cnst,Var,Abs) //1 byte
        0x95, 0x06, // REPORT_COUNT (6)
        0x75, 0x08, // REPORT_SIZE (8)
        0x15, 0x00, // LOGICAL_MINIMUM (0)
        0x25, 0x65, // LOGICAL_MAXIMUM (101)
        0x05, 0x07, // USAGE_PAGE (Keyboard)
        0x19, 0x00, // USAGE_MINIMUM (Reserved (no event indicated))
        0x29, 0x65, // USAGE_MAXIMUM (Keyboard Application)
        0x81, 0x00, // INPUT (Data,Ary,Abs) //6 bytes
        0xc0, // END_COLLECTION
};

static const struct {
	struct usb_hid_descriptor hid_descriptor;
	struct {
		uint8_t bReportDescriptorType;
		uint16_t wDescriptorLength;
	} __attribute__((packed)) hid_report;
} __attribute__((packed)) hid_function = {
	.hid_descriptor = {
		.bLength = sizeof(hid_function),
		.bDescriptorType = USB_DT_HID,
		.bcdHID = 0x0100,
		.bCountryCode = 0,
		.bNumDescriptors = 1,
	},
	.hid_report = {
		.bReportDescriptorType = USB_DT_REPORT,
		.wDescriptorLength = sizeof(hid_report_descriptor),
	}
};

const struct usb_endpoint_descriptor hid_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 8,
	.bInterval = 0x20,
};

const struct usb_interface_descriptor hid_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_HID,
	.bInterfaceSubClass = 1, /* boot */
	.bInterfaceProtocol = 1, // 1=keyboard, 2=mouse
	.iInterface = 0,
	.endpoint = &hid_endpoint,
	.extra = &hid_function,
	.extralen = sizeof(hid_function),
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &hid_iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,
	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Tomu",
	"HID keyboard Demo",
	"DEMO",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes hid_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
			void (**complete)(usbd_device *, struct usb_setup_data *)) {
	(void)complete;
	(void)dev;

	if((req->bmRequestType != 0x81) ||
	   (req->bRequest != USB_REQ_GET_DESCRIPTOR) ||
	   (req->wValue != 0x2200))
		return 0;

	// Handle the HID report descriptor
	*buf = (uint8_t *)hid_report_descriptor;
	*len = sizeof(hid_report_descriptor);

    // Dirty way to know if we're connected
    g_usbd_is_connected = true;

	return 1;
}

static void hid_set_config(usbd_device *dev, uint16_t wValue) {
	(void)wValue;
	(void)dev;
	usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 8, NULL);
	usbd_register_control_callback(
				dev,
				USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				hid_control_request);
}

void usb_isr(void) {
    usbd_poll(g_usbd_dev);
}

void hard_fault_handler(void) {
    while(1);
}

// Modifier info: https://wiki.osdev.org/USB_Human_Interface_Devices#Report_format
void sys_tick_handler(void) {
	// mod 0 == no mod
	// mod 1 == left ctrl
	// mod 2 == left shift
	// mod 3 == left ctrl + shift 
	// mod 4 == left alt
	// mod 5 == left ctrl + alt
	// mod 6 == left shift + alt
	// mod 7 == left ctrl + shift + alt
	// mod 8 == left super 
	// mod 9 == left ctrl + super 
	// mod 10 == left shift + super 
	// mod 11 == left ctrl + shift + super
	// mod 12 == left alt + super
	// mod 13 == left ctrl + alt + super
	// mod 14 == left shift + alt + super
	// mod 15 == left ctrl + shift + alt + super
	// \r carriage return
	// \a application / commonly right click
	// \t tab
	// \v f11
	// \0 right arrow
	// \1 left arrow
	// \2 down arrow
	// \3 up arrow
	// \4 win key
	// \5 left ctrl
	// \6 left alt
	if(g_usbd_is_connected && once) {
		for(int i = 0; i != 1500000; ++i) __asm__("nop");  // wait before keys injection	
		
		// Sample to inject common keys needed for scripting (excluding carriage return)
		// injkeys("`~1!2@3#4$5%6^7&8*9(0)-_=+ abcdefghijklmnopqrstuvwxyz ABCDEFGHIJKLMNOPQRSTUVWXYZ [{]}\\|;:\'\",<.>/?",0);
		
		// Sample injecting multiple identical chars in a row 
		// injkeys("a bb ccc dddd eeeee ffffff ggggggg hhhhhhhh",0);
		
		// Sample injecting 5x carriage returns
		// injkeys("\r\r\r\r\r",0);
		
		// Sample to inject left super (winkey)
		// injkeys("\4",0);
		
		// Sample to inject hi then left arrow and inject before hi: 
		injkeys("hi\1\1before hi: ",0);
		
		// Sample to inject ech, tab complete to echo, type hi, then return 
		// injkeys("ech\t hi\r",0);
		
		// Sample to inject left super key once
		// injkeys("\4",0);

		// Sample to open powershell, run sendkey script to raise volume, exit powershell, open msedge, navigate to nyan.cat, and tab to wtf nyan cat 
		// inject left super (winkey) + "r", to open the "Run" console.
		// injkeys("r",8);
		// for(int i = 0; i != 1500000; ++i) __asm__("nop");  // wait
		// injkeys("powershell\r",0);
		// for(int i = 0; i != 20000000; ++i) __asm__("nop");  // wait
		// injkeys("\r",0);
		// for(int i = 0; i != 1500000; ++i) __asm__("nop");  // wait
		// injkeys("\r",0);
		// injkeys("$obj.SendKeys([char]175)",0);
		// for(int i = 0; i != 1500000; ++i) __asm__("nop");  // wait
		// injkeys("\r",2);
		// for(int i = 0; i != 500000; ++i) __asm__("nop");  // wait
		// injkeys("$obj = new-object -com wscript.shell\r",0);
		// injkeys("\r",0);
		// injkeys("\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r\3\r",0);
		// injkeys("exit\r",0);
		// for(int i = 0; i != 1500000; ++i) __asm__("nop");  // wait
		// injkeys("r",8);
		// for(int i = 0; i != 1500000; ++i) __asm__("nop");  // wait
		// injkeys("msedge -inprivate nyan.cat\r",0);
		// for(int i = 0; i != 10000000; ++i) __asm__("nop");  // wait
		// injkeys("\v",0);
		// injkeys("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\r",0);

		once=false;
		}
}

// https://www.freecodecamp.org/news/ascii-table-hex-to-ascii-value-character-code-chart-2/
// https://www.usb.org/sites/default/files/documents/hut1_12v2.pdf
void injkeys(char *source,uint8_t mod) {
	static uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // key pressed
	static uint8_t buf2[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Key released
	if(g_usbd_is_connected) {
		// change ascii to keyboard map
		int j = 0;
		int injstrlen = strlen(source);
		while (injstrlen > j) {
			buf[0]=mod; // Key modifier, 0=None
			if(source[j]>96&&source[j]<123){ // a-z
				buf[2]=source[j]-93;
			}
			else if(mod<1&&source[j]>64&&source[j]<91){ // A-Z
				buf[0]=2; // 2=LeftShift
				buf[2]=source[j]-61;
			}
			else if(source[j]>48&&source[j]<58){ // 1-9
				buf[2]=source[j]-19;
			}
			else if(source[j]==48){ // number 0
				buf[2]=39;
			}
			else{
				switch(source[j]){
					case 32: // spacebar
						buf[2]=44;
						break;
					case 33: // !
						buf[0]=2; // 2=LeftShift
						buf[2]=30;
						break;
					case 34: // "
						buf[0]=2; // 2=LeftShift
						buf[2]=52;
						break;
					case 35: // #
						buf[0]=2; // 2=LeftShift
						buf[2]=32;
						break;
					case 36: // $
						buf[0]=2; // 2=LeftShift
						buf[2]=33;
						break;
					case 37: // %
						buf[0]=2; // 2=LeftShift
						buf[2]=34;
						break;
					case 38: // &
						buf[0]=2; // 2=LeftShift
						buf[2]=36;
						break;
					case 39 : // '
						buf[2]=52;
						break;
					case 40 : // (
						buf[0]=2; // 2=LeftShift
						buf[2]=38;
						break;
					case 41 : // )
						buf[0]=2; // 2=LeftShift
						buf[2]=39;
						break;
					case 42 : // *
						buf[0]=2; // 2=LeftShift
						buf[2]=37;
						break;
					case 43 : // +
						buf[0]=2; // 2=LeftShift
						buf[2]=46;
						break;
					case 44 : // ,
						buf[2]=54;
						break;
					case 45 : // -
						buf[2]=45;
						break;
					case 46 : // .
						buf[2]=55;
						break;
					case 47 : // /
						buf[2]=56;
						break;
					case 58 : // :
						buf[0]=2; // 2=LeftShift
						buf[2]=51;
						break;
					case 59 : // ;
						buf[2]=51;
						break;
					case 60 : // <
						buf[0]=2; // 2=LeftShift
						buf[2]=54;
						break;
					case 61 : // =
						buf[2]=46;
						break;
					case 62 : // >
						buf[0]=2; // 2=LeftShift
						buf[2]=55;
						break;
					case 63 : // ?
						buf[0]=2; // 2=LeftShift
						buf[2]=56;
						break;
					case 64 : // @
						buf[0]=2; // 2=LeftShift
						buf[2]=31;
						break;
					case 91 : // [
						buf[2]=47;
						break;
					case 92 : // backslash
						buf[2]=49;
						break;
					case 93 : // ]
						buf[2]=48;
						break;
					case 94 : // ^
						buf[0]=2; // 2=LeftShift
						buf[2]=35;
						break;
					case 95 : // _
						buf[0]=2; // 2=LeftShift
						buf[2]=45;
						break;
					case 96 : // `
						buf[2]=53;
						break;
					case 123 : // {
						buf[0]=2; // 2=LeftShift
						buf[2]=47;
						break;
					case 124 : // |
						buf[0]=2; // 2=LeftShift
						buf[2]=49;
						break;
					case 125 : // }
						buf[0]=2; // 2=LeftShift
						buf[2]=48;
						break;
					case 126 : // ~
						buf[0]=2; // 2=LeftShift
						buf[2]=53;
						break;
					case '\r' : // carriage return / enter
						buf[2]=40;
						break;
					case '\a' : // application / right click
						buf[2]=101;
						break;
					case '\t' : // tab
						buf[2]=43;
						break;
					case '\v' : // f11
						buf[2]=68;
						break;
					case '\0' : // right arrow
						buf[2]=79;
						break;
					case '\1' : // left arrow
						buf[2]=80;
						break;
					case '\2' : // down arrow
						buf[2]=81;
						break;
					case '\3' : // up arrow
						buf[2]=82;
						break;
					case '\4' : // left win key
						buf[0]=8;
						break;
					case '\5' : // left ctrl
						buf[2]=29;
						break;
					case '\6' : // left alt
						buf[2]=56;
						break;
					default:
						buf[2]=source[j]-93; // lowercase letters
				}
			}
			usbd_ep_write_packet(g_usbd_dev, 0x81, buf, 8);
			for(int i = 0; i != 150; ++i) __asm__("nop");
			usbd_ep_write_packet(g_usbd_dev, 0x81, buf2, 8);
        		for(int i = 0; i != 150000; ++i) __asm__("nop");
			usbd_ep_write_packet(g_usbd_dev, 0x81, buf2, 8); // Be sure key is released
        		for(int i = 0; i != 150000; ++i) __asm__("nop");
			j++;
  			}
		}
}

int main(void) {
    int i;

    /* Make sure the vector table is relocated correctly (after the Tomu bootloader) */
    SCB_VTOR = 0x4000;

    /* Disable the watchdog that the bootloader started. */
    WDOG_CTRL = 0;

    /* GPIO peripheral clock is necessary for us to set up the GPIO pins as outputs */
    cmu_periph_clock_enable(CMU_GPIO);

    /* Set up both LEDs as outputs */
    gpio_mode_setup(LED_RED_PORT, GPIO_MODE_WIRED_AND, LED_RED_PIN);
    gpio_mode_setup(LED_GREEN_PORT, GPIO_MODE_WIRED_AND, LED_GREEN_PIN);

    /* Configure the USB core & stack */
	g_usbd_dev = usbd_init(&efm32hg_usb_driver, &dev_descr, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(g_usbd_dev, hid_set_config);

    /* Enable USB IRQs */
    nvic_set_priority(NVIC_USB_IRQ, 0x40);
	nvic_enable_irq(NVIC_USB_IRQ);

    /* Configure the system tick, at lower priority than USB IRQ */
    systick_set_frequency(SYSTICK_FREQUENCY, AHB_FREQUENCY);
    systick_counter_enable();
    systick_interrupt_enable();
    nvic_set_priority(NVIC_SYSTICK_IRQ, 0x10);

    gpio_set(LED_RED_PORT, LED_RED_PIN);
    while(1) {

    gpio_toggle(LED_GREEN_PORT, LED_GREEN_PIN);
    gpio_toggle(LED_RED_PORT, LED_RED_PIN);
        for(i = 0; i != 500000; ++i)
			__asm__("nop");
    }
}
