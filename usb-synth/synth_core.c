/*
 * File: synth_core.c
 * A pretty terrible wavetablesque polyphonic synthesizer.
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

#include "synth_core.h"

#include "tomu_pins.h"

#include <string.h>
#include <math.h>

/* Maximum amount of simultaneous notes. Taking this much higher
 * will cause isochronous packets not to be emitted on time. */
#define MAX_VOICES 10

/* MIDI notes above or below these values will be ignored, either
 * because it is expensive to reproduce them, or they are annoying */
#define NOTE_IGNORE_MIN 21
#define NOTE_IGNORE_MAX 94

/* Fixed-point multiplier for skip factors on the base waveform.
 * Power of two so divide optimizes to a shift */
#define FPM 128

/* 256 time units (stereo) */
#define WAVEFORM_SAMPLES 256

/* Stores the 'base' waveform to be read at different skip lengths
 * depending on the frequency of the note being played */
int16_t waveform_data[WAVEFORM_SAMPLES] = {0};

/* The state of each note currently being played */
typedef struct {
    uint8_t channel;
    uint8_t key;
    uint8_t vel;

    uint8_t sustained;

    uint16_t cur_sample;
    uint16_t skip_factor;
} note_state_t;

/* Contains all active note states. state.key == 0 for unused slot */
note_state_t note_states[MAX_VOICES];

/* Contains fixed-point offets representing amount to skip forward in
 * waveform_data per sample according to the target note frequency */
uint16_t note_skip_table[MIDI_N_NOTES];

/* Keep track of whether sustain is down on a particular channel */
uint8_t sustain[MIDI_N_CHANNELS] = {0};

void synth_core_init(void)
{
    /* Null out all note states */
    memset(note_states, 0, sizeof(note_states));

    /* Sine wave is the 'base' waveform for now */
    for(int i = 0; i != WAVEFORM_SAMPLES; ++i) {
        waveform_data[i] =
            15 * (float)sin(3.141 * 2.0 * (float)i / (float)WAVEFORM_SAMPLES);
    }

    /* Now, initialize the note skip table by:
       1. Computing note frequency & period
       2. Converting note period to per-sample master waveform skip
       3. Convert skip ratio to a fixed-point number */
    for(int n = 0; n != MIDI_N_NOTES; ++n) {
        uint16_t skip = 0;
        if(n > NOTE_IGNORE_MIN && n < NOTE_IGNORE_MAX) {
            float freq = 27.5 * 2.0 * pow(2.0, (n-21.0)/12.0);
            float period = 1.0 / freq;
            float waveform_period = ((float)WAVEFORM_SAMPLES / 8000.0);
            skip = (uint16_t)(FPM * waveform_period / period);
        }
        note_skip_table[n] = skip;
    }
}

/* The core stream callback emits a block of samples to be transferred
 * back to the host by summing skip-traversed 'base' waveforms over all voices */
void synth_core_stream(uint16_t *out_samples, uint16_t n_samples)
{
    static uint32_t t = 0; /* 'time' */

    memset(out_samples, 0, n_samples*sizeof(uint16_t));

    for(int i = 0; i != MAX_VOICES; ++i) {
        if(note_states[i].key > 0) {
            /* For every (time) sample in the note for this block... */
            for(int s = 0; s != n_samples/2; ++s) {

                /* Compute new position in 'base' waveform */
                note_states[i].cur_sample =
                    (note_states[i].cur_sample + note_states[i].skip_factor)
                        % (WAVEFORM_SAMPLES*FPM);

                /* Get sample, add to existing so we can hear all voices */
                out_samples[s*2] += note_states[i].vel *
                                        waveform_data[note_states[i].cur_sample / FPM];

                /* Both channels the same (i.e mono) */
                out_samples[s*2+1] = out_samples[s*2];

            }

            /* Hack to slowly kill note volume */
            if(t % 10 == 0) {
                --note_states[i].vel;
                if(note_states[i].vel == 0) note_states[i].key = 0;
            }
        }
    }

    ++t;
}

static void note_on_event(uint8_t channel, uint8_t key, uint8_t vel)
{
    /* Basically, when a MIDI note arrives, we update note_states
     * by either populating an empty voice, or evicting one */

    /* Ignore drums and invalid notes */
    if(channel == 10 || key < NOTE_IGNORE_MIN || key > NOTE_IGNORE_MAX) {
        return;
    }

    /* Try and find an empty note slot */
    int target = 0;
    for(int i = 0; i != MAX_VOICES; ++i) {
        if(note_states[i].key == 0) {
            target = i;
        }
    }

    /* Otherwise just evict round-robin */
    static int evict = 0;
    if(target == 0)
        target = evict++ % MAX_VOICES;

    gpio_clear(LED_GREEN_PORT, LED_GREEN_PIN);
    gpio_set(LED_RED_PORT, LED_RED_PIN);

    /* At this point, we have a target note. Fill it up */
    note_states[target].channel = channel;
    note_states[target].key = key;
    note_states[target].vel = vel;
    note_states[target].sustained = 0;
    note_states[target].cur_sample = 0;
    note_states[target].skip_factor = note_skip_table[key];
}

static void note_off_event(uint8_t channel, uint8_t key, uint8_t vel)
{
    (void) vel; /* unused for note_off */

    gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);
    gpio_clear(LED_RED_PORT, LED_RED_PIN);

    /* Delete the target note, or switch it to 'sustained' state */
    for(int i = 0; i != MAX_VOICES; ++i) {
        if(note_states[i].channel == channel &&
           note_states[i].key == key) {
            if(sustain[channel]) {
                note_states[i].sustained = 1;
            } else {
                note_states[i].key = 0;
            }
        }
    }
}

static void midi_cc_event(uint8_t channel, uint8_t cc_id, uint8_t value)
{
    /* Only handle sustain CC messages for now */
    if(cc_id == MIDI_CC_SUSTAIN) {
        if(value >= 64) {
            /* Pedal DOWN */
            sustain[channel] = 1;
        } else {
            /* Pedal UP */
            sustain[channel] = 0;

            /* If the pedal has moved UP, cull all sustained voices */
            for(int i = 0; i != MAX_VOICES; ++i) {
                if(note_states[i].channel == channel) {
                    if(note_states[i].sustained) {
                        note_states[i].key = 0;
                    }
                }
            }
        }
    }
}

void synth_core_decode_midi_packet(midi_usb_event_packet_t p)
{
    uint8_t midi_channel = p.midi0 & 0xF;
    uint8_t midi_command = p.midi0 >> 4;

    if(midi_command != p.code_index_number) {
        return;
    }

    switch (midi_command) {
        case MIDI_NOTE_OFF: {
            note_off_event(midi_channel, p.midi1, p.midi2);
            break;
        }
        case MIDI_NOTE_ON: {
            note_on_event(midi_channel, p.midi1, p.midi2);
            break;
        }
        case MIDI_CONTINUOUS: {
            midi_cc_event(midi_channel, p.midi1, p.midi2);
        }
        default: {
            break;
        }
    }
}
