/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2015 Piotr Esden-Tempski <piotr@esden.net>
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
 * This example implements a USB CDC-ACM device (aka Virtual Serial Port)
 * to demonstrate the use of the USB device stack.
 *
 * When data is recieved, it will toggle the green LED and echo the data.
 * The red LED is toggled constantly and a string is sent over USB every
 * time the LED changes state as a heartbeat.
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/efm32/wdog.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/cmu.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

// Make this program compatible with Toboot-V2.0
#include <toboot.h>
TOBOOT_CONFIGURATION(0);

// Serial prompt
#define PROMPT          "MSG> "

#define LED_GREEN_PORT  GPIOA
#define LED_GREEN_PIN   GPIO0
#define LED_RED_PORT    GPIOB
#define LED_RED_PIN     GPIO7

#define VENDOR_ID       0x1209  // pid.code
#define PRODUCT_ID      0x70b1  // Assigned to Tomu project
#define DEVICE_VER      0x0101  // Program version

#define BIT_TIME_US     52    // Bit time in uS
                              // Baud rate for transmission = 1,000,000/BIT_TIME = 19230 bps

static volatile bool g_usbd_is_connected = false;
static usbd_device *g_usbd_dev = 0;
static uint8_t g_secret_message[1024];
static uint8_t g_new_secret_message[1024];
static volatile uint32_t g_new_secret_message_len;
static volatile bool g_should_use_new_secret_message;

static const struct usb_device_descriptor dev =
{
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = USB_CLASS_CDC,
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

/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] =
{{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x83,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] =
{{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}};

static const struct {
    struct usb_cdc_header_descriptor header;
    struct usb_cdc_call_management_descriptor call_mgmt;
    struct usb_cdc_acm_descriptor acm;
    struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors =
{
    .header = {
        .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
        .bcdCDC = 0x0110,
    },
    .call_mgmt = {
        .bFunctionLength =
            sizeof(struct usb_cdc_call_management_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
        .bmCapabilities = 0,
        .bDataInterface = 1,
    },
    .acm = {
        .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_ACM,
        .bmCapabilities = 0,
    },
    .cdc_union = {
        .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_UNION,
        .bControlInterface = 0,
        .bSubordinateInterface0 = 1,
     }
};

static const struct usb_interface_descriptor comm_iface[] =
{{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_CDC,
    .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
    .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
    .iInterface = 0,

    .endpoint = comm_endp,

    .extra = &cdcacm_functional_descriptors,
    .extralen = sizeof(cdcacm_functional_descriptors)
}};

static const struct usb_interface_descriptor data_iface[] =
{{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,

    .endpoint = data_endp,
}};

static const struct usb_interface ifaces[] =
{{
    .num_altsetting = 1,
    .altsetting = comm_iface,
}, {
    .num_altsetting = 1,
    .altsetting = data_iface,
}};

static const struct usb_config_descriptor config =
{
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 2,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,

    .interface = ifaces,
};

static const char *usb_strings[] =
{
    "Tomu",
    "OpticSpy Transmitter",
    "70mu-5py",
};

// This busywait loop is roughly accurate when running at 24 MHz.
void udelay_busy(uint32_t usecs)
{
    while (usecs --> 0) {
        /* This inner loop is 3 instructions, one of which is a branch.
         * This gives us 4 cycles total.
         * We want to sleep for 1 usec, and there are cycles per usec at 24 MHz.
         * Therefore, loop 6 times, as 6*4=24.
         */
        asm("mov   r1, #6");
        asm("retry:");
        asm("sub r1, #1");
        asm("bne retry");
        asm("nop");
    }
}

/* Buffer to be used for control requests. */
static uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
        uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
    (void)complete;
    (void)buf;
    (void)usbd_dev;

    switch(req->bRequest) {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
        g_usbd_is_connected = req->wValue & 1; /* Check RTS bit */
        if (!g_usbd_is_connected) /* Note: GPIO polarity is inverted */
            gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);
        else
            gpio_clear(LED_GREEN_PORT, LED_GREEN_PIN);
        return USBD_REQ_HANDLED;
        }
    case USB_CDC_REQ_SET_LINE_CODING: 
        if(*len < sizeof(struct usb_cdc_line_coding))
            return 0;

        return USBD_REQ_HANDLED;
    }
    return 0;
}

/* Simple callback that echoes whatever is sent */
static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;

    char buf[64];
    char output[64];
    uint32_t len = usbd_ep_read_packet(usbd_dev, 0x01, buf, sizeof(buf));

    if (len) {
        /* Look for '\r' and append '\n' */
        uint32_t i;
        uint32_t new_len = 0;
        for (i = 0; (i < len) && (new_len < sizeof(buf)); i++) {
            if (buf[i] == '\r') {
                output[new_len++] = buf[i];
                if (new_len >= sizeof(output))
                    new_len--;
                output[new_len++] = '\n';
                if (new_len >= sizeof(output))
                    new_len--;

                // Switch over to using the new message.
                g_should_use_new_secret_message = true;
            }
            // For backspace characters, go back one space, print a 'space' to overwrite
            // the character, then move the cursor back by one.
            else if ((buf[i] == 0x7f) || (buf[i] == '\b')) {
                if (g_new_secret_message_len > 0) {
                    output[new_len++] = '\b';
                    if (new_len >= sizeof(output))
                        new_len--;
                    output[new_len++] = ' ';
                    if (new_len >= sizeof(output))
                        new_len--;
                    output[new_len++] = '\b';
                    if (new_len >= sizeof(output))
                        new_len--;
                    g_new_secret_message[--g_new_secret_message_len] = '\0';
                }
                continue;
            }
            // For printable characters, put them in the new-message buffer.
            else if (isprint(buf[i])) {
                g_new_secret_message[g_new_secret_message_len++] = buf[i];
                output[new_len++] = buf[i];
                if (new_len >= sizeof(output))
                    new_len--;
            }
        }
        usbd_ep_write_packet(usbd_dev, 0x82, output, new_len);
        output[new_len] = 0;
    }
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;

    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, 0);
    usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, 0);

    usbd_register_control_callback(
                usbd_dev,
                USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
                USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                cdcacm_control_request);
}

static void usb_puts(char *s)
{
    if (g_usbd_is_connected) {
        usbd_ep_write_packet(g_usbd_dev, 0x82, s, strnlen(s, 64));
    }
}

static void opticspy_gpio_set_value(bool is_on)
{
    // Note that the LEDs use inverted logic.
    if (is_on)
        gpio_clear(LED_RED_PORT, LED_RED_PIN);
    else
        gpio_set(LED_RED_PORT, LED_RED_PIN);
}

static void opticspy_putc(char c)
{
    int i;

    // Start bit
    opticspy_gpio_set_value(1);
    udelay_busy(BIT_TIME_US);

    // Data bits
    for (i = 0; i < 8; ++i)
    {
        opticspy_gpio_set_value(!((c >> i) & 1));
        udelay_busy(BIT_TIME_US);
    }

    // Stop bit
    opticspy_gpio_set_value(0);
    udelay_busy(BIT_TIME_US);
}

static void opticspy_puts(uint8_t *s)
{
    int i;

    for (i = 0; s[i] != '\0'; ++i)
        opticspy_putc(s[i]);
}

void usb_isr(void)
{
    usbd_poll(g_usbd_dev);
}

void hard_fault_handler(void)
{
    while(1);
}

int main(void)
{
    bool line_was_connected = false;

    /* Disable the watchdog that the bootloader started. */
    WDOG_CTRL = 0;

    /* GPIO peripheral clock is necessary for us to set up the GPIO pins as outputs */
    cmu_periph_clock_enable(CMU_GPIO);

    /* Set up both LEDs as outputs */
    gpio_mode_setup(LED_RED_PORT, GPIO_MODE_WIRED_AND, LED_RED_PIN);
    gpio_mode_setup(LED_GREEN_PORT, GPIO_MODE_WIRED_AND, LED_GREEN_PIN);
    gpio_set(LED_RED_PORT, LED_RED_PIN);
    gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);

    /* Configure the USB core & stack */
    g_usbd_dev = usbd_init(&efm32hg_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
    usbd_register_set_config_callback(g_usbd_dev, cdcacm_set_config);

    /* Set the CPU Core to run from the trimmed USB clock, divided by 2.
     * This will give the CPU Core a frequency of 24 MHz +/- 1% */
    CMU_CMD = CMU_CMD_HFCLKSEL(5);
    while (! (CMU_STATUS & CMU_STATUS_USHFRCODIV2SEL))
        ;

    /* Enable USB IRQs */
    nvic_enable_irq(NVIC_USB_IRQ);

    while (1) {
        // Send the current secret message out the serial port.
        opticspy_puts(g_secret_message);

        if (line_was_connected != g_usbd_is_connected) {
            if (g_usbd_is_connected) {
                // Wait for the terminal to appear
                udelay_busy(1000);
                usb_puts("\r\nEnter message to send, followed by Return:\r\n" PROMPT);
            }
            line_was_connected = g_usbd_is_connected;
        }

        if (!g_usbd_is_connected)
            continue;

        // Replace the message, if there's a new one available.
        if (g_should_use_new_secret_message) {
            memset(g_secret_message, 0, sizeof(g_secret_message));

            // Print a different message depending on whether the new message is blank.
            if (g_new_secret_message_len) {
                memcpy(g_secret_message, g_new_secret_message, g_new_secret_message_len);
                usb_puts("Message updated.\r\n" PROMPT);
            }
            else {
                usb_puts("Message cleared.\r\n" PROMPT);
            }

            g_should_use_new_secret_message = false;
            g_new_secret_message_len = 0;
        }
    }
}
