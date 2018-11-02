/*
 * File: mic_descriptors.h
 * Descriptors for 8KHz audio streaming as a USB microphone.
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

static const struct {
	struct usb_audio_header_descriptor_head header_head;
	struct usb_audio_header_descriptor_body header_body;
	struct usb_audio_input_terminal_descriptor input_terminal_desc;
	struct usb_audio_feature_unit_descriptor_2ch feature_unit_desc;
	struct usb_audio_output_terminal_descriptor output_terminal_desc;
} __attribute__((packed)) mic_audio_control_functional_descriptors = {
	.header_head = {
		.bLength = sizeof(struct usb_audio_header_descriptor_head) +
		           1 * sizeof(struct usb_audio_header_descriptor_body),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_AUDIO_TYPE_HEADER,
		.bcdADC = 0x0100,
		.wTotalLength =
			   sizeof(struct usb_audio_header_descriptor_head) +
			   1 * sizeof(struct usb_audio_header_descriptor_body) +
			   sizeof(struct usb_audio_input_terminal_descriptor) +
			   sizeof(struct usb_audio_feature_unit_descriptor_2ch) +
			   sizeof(struct usb_audio_output_terminal_descriptor),
		.binCollection = 1,
	},
	.header_body = {
		.baInterfaceNr = 0x01,
	},
	.input_terminal_desc = {
		.bLength = sizeof(struct usb_audio_input_terminal_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_AUDIO_TYPE_INPUT_TERMINAL,
		.bTerminalID = 1,
		.wTerminalType = 0x0710, /* XXX: Radio receiver */
		.bAssocTerminal = 0,
		.bNrChannels = 2,
		.wChannelConfig = 0x0003, /* XXX: Left & Right channels */
		.iChannelNames = 0,
		.iTerminal = 0,
	},
	.feature_unit_desc = {
		.head = {
			.bLength = sizeof(struct usb_audio_feature_unit_descriptor_2ch),
			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
			.bDescriptorSubtype = USB_AUDIO_TYPE_FEATURE_UNIT,
			.bUnitID = 2,
			.bSourceID = 1, /* Input terminal 1 */
			.bControlSize = 2,
			.bmaControlMaster = 0x0001, /* XXX: 'Mute' is supported */
		},
		.channel_control = { {
			.bmaControl = 0x0000,
		}, {
			.bmaControl = 0x0000,
		} },
		.tail = {
			.iFeature = 0x00,
		}
	},
	.output_terminal_desc = {
		.bLength = sizeof(struct usb_audio_output_terminal_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_AUDIO_TYPE_OUTPUT_TERMINAL,
		.bTerminalID = 3,
		.wTerminalType = 0x0101, /* XXX: USB Streaming */
		.bAssocTerminal = 0,
		.bSourceID = 0x02, /* Feature unit 2 */
		.iTerminal = 0,
	}
};

static const struct usb_interface_descriptor mic_audio_control_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_CONTROL,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.extra = &mic_audio_control_functional_descriptors,
	.extralen = sizeof(mic_audio_control_functional_descriptors)
} };

static const struct {
	struct usb_audio_stream_interface_descriptor audio_cs_streaming_iface_desc;
	struct usb_audio_format_type1_descriptor_1freq audio_type1_format_desc;
} __attribute__((packed)) mic_audio_streaming_functional_descriptors = {
	.audio_cs_streaming_iface_desc = {
		.bLength = sizeof(struct usb_audio_stream_interface_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = 1, /* XXX: AS_GENERAL */
		.bTerminalLink = 3, /* Terminal 3 */
		.bDelay = 0,
		.wFormatTag = 0x0001 /* XXX: PCM Format */,
	},
	.audio_type1_format_desc = {
		.head = {
			.bLength = sizeof(struct usb_audio_format_type1_descriptor_1freq),
			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
			.bDescriptorSubtype = 2, /* XXX: FORMAT_TYPE */
			.bFormatType = 1,
			.bNrChannels = 2,
			.bSubFrameSize = 2,
			.bBitResolution = 16, /* Should be 10, but 16 is more reliable */
			.bSamFreqType = 1, /* 1 discrete sampling frequency */
		},
		.freqs = { {
			.tSamFreq = 8000,
		} },
	}
};

static const struct usb_audio_stream_audio_endpoint_descriptor audio_streaming_cs_ep_desc[] = { {
	.bLength = sizeof(struct usb_audio_stream_audio_endpoint_descriptor),
	.bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
	.bDescriptorSubtype = 1, /* XXX: EP_GENERAL */
	.bmAttributes = 0,
	.bLockDelayUnits = 0x02, /* PCM samples */
	.wLockDelay = 0x0000,
} };

static const struct usb_endpoint_descriptor isochronous_ep[] = { {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82 /* XXX: EP3 In?? */,
	.bmAttributes = USB_ENDPOINT_ATTR_ASYNC | USB_ENDPOINT_ATTR_ISOCHRONOUS,
	.wMaxPacketSize = 256,
	.bInterval = 0x01, /* 1 millisecond */

	/* XXX: not using usb_audio_stream_endpoint_descriptor??
	 * (Why? These are USBv1.0 endpoint descriptors)*/

	.extra = &audio_streaming_cs_ep_desc[0],
	.extralen = sizeof(audio_streaming_cs_ep_desc[0])
} };

static const struct usb_interface_descriptor mic_audio_streaming_iface[] = { {
	/* zerobw streaming interface (alt 0) */
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.extra = 0,
	.extralen = 0,
}, {
	/* Actual streaming interface (alt 1) */
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 1,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = isochronous_ep,

	.extra = &mic_audio_streaming_functional_descriptors,
	.extralen = sizeof(mic_audio_streaming_functional_descriptors)
} };
