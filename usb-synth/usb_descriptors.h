/*
 * File: usb_descriptors.h
 * Boring header to glue some USB descriptors together.
 *
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

#pragma once

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/audio.h>
#include <libopencm3/usb/midi.h>

#include "mic_descriptors.h"
#include "midi_descriptors.h"

#define VENDOR_ID            0x1209    /* pid.code */
#define PRODUCT_ID           0x70b1    /* Assigned to Tomu project */
#define DEVICE_VER           0x0101    /* Program version */

static const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,    /* was 0x0110 in Table B-1 example descriptor */
    .bDeviceClass = 0,   /* device defined at interface level */
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = VENDOR_ID,
    .idProduct = PRODUCT_ID,
    .bcdDevice = DEVICE_VER,
    .iManufacturer = 1,  /* index to string desc */
    .iProduct = 2,       /* index to string desc */
    .iSerialNumber = 3,  /* index to string desc */
    .bNumConfigurations = 1,
};

uint8_t mic_streaming_iface_cur_altsetting = 0;

static const struct usb_interface ifaces[] = {{
    .num_altsetting = 1,
    .altsetting = mic_audio_control_iface,
}, {
    .num_altsetting = 2,
    .cur_altsetting = &mic_streaming_iface_cur_altsetting,
    .altsetting = mic_audio_streaming_iface,
}, {
    .num_altsetting = 1,
    .altsetting = midi_audio_control_iface,
}, {
    .num_altsetting = 1,
    .altsetting = midi_streaming_iface,
} };

static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0, /* can be anything, it is updated automatically
                          when the usb code prepares the descriptor */
    .bNumInterfaces = 4, /* control/stream [audio] + control/stream [midi] */
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80, /* bus powered */
    .bMaxPower = 0x32,
    .interface = ifaces,
};

static char usb_serial_number[25]; /* 12 bytes of desig and a \0 */

static const char * usb_strings[] = {
    "Tomu",
    "USB Synth Demo",
    usb_serial_number
};
