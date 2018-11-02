/*
 * File: synth_core.h
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

#pragma once

#include <stdint.h>

#include "midi_events.h"

/* Initialize the synth engine */
void synth_core_init(void);

/* Generate some audio. Call this with a valid pointer of size `n_samples`.
 * `out_samples` will be populated with the next block of samples */
void synth_core_stream(uint16_t *out_samples, uint16_t n_samples);

/* Tell the synth that a MIDI event occurred. */
void synth_core_decode_midi_packet(midi_usb_event_packet_t p);
