#pragma once

#include <stdint.h>

/* USB-MIDI Event Packet structure
 * See: http://www.usb.org/developers/docs/devclass_docs/midi10.pdf */

typedef struct _midi_usb_event_packet_t {
    uint8_t code_index_number : 4;
    uint8_t cable_number      : 4;
    uint8_t midi0;
    uint8_t midi1;
    uint8_t midi2;
} __attribute((packed)) midi_usb_event_packet_t;

/* MIDI message structure
 * See: https://ccrma.stanford.edu/~craig/articles/linuxmidi/misc/essenmidi.html */

/* Command types (shared by usb code index numbers) */
#define MIDI_NOTE_OFF          0x8
#define MIDI_NOTE_ON           0x9
#define MIDI_AFTERTOUCH        0xA
#define MIDI_CONTINUOUS        0xB
#define MIDI_PATCH_CHANGE      0xC
#define MIDI_CHANNEL_PRESSURE  0xD
#define MIDI_PITCH_BEND        0xE
#define MIDI_SYSTEM_MESSAGE    0xF

/* Some more MIDI defines */
#define MIDI_NOTE_MAX   127
#define MIDI_NOTE_MIN   0
#define MIDI_N_NOTES    128
#define MIDI_CHANNEL_MAX 15
#define MIDI_CHANNEL_MIN 0
#define MIDI_N_CHANNELS 16

#define MIDI_CC_SUSTAIN 0x40 /* Sustain pedal CC code */
