# README

A pretty terrible USB-MIDI synthesizer. The Tomu will enumerate as a composite MIDI device
and line-in (mic) device. Sending MIDI notes to it will emit tones out the mic device.

The green and red LEDs will toggle in time with the music as it is played

Currently, this implementation ignores all drum tracks and redirects all instruments to sine tones.
Velocity and sustain are supported, but it will cull notes if one attempts > 10 voice polyphony.

The 'synth bits' are in `synth_core.c`. Notes are culled pretty inefficiently at the moment.

# Usage

Warning: only tested on a Linux host under ALSA! Windows is picky about audio descriptors,
so it's unlikely this will work on Windows without further work.

## Capturing the output

To feed the Tomu's audio to your speakers in real-time, `alsaloop` does the job:

    $ sudo alsaloop -C hw:1,0 -P hw:0,0 --rate=8000

The first hardware ID should be your Tomu (`-C` for capture), the second (`-P` for playback) your computers' soundcard.
You can figure out these hardware IDs by running `arecord -l` and `aplay -l` - for example:

    $ arecord -l
    **** List of CAPTURE Hardware Devices ****
    card 0: PCH [HDA Intel PCH], device 0: CX8200 Analog [CX8200 Analog]
      Subdevices: 0/1
      Subdevice #0: subdevice #0
    card 1: Demo [USB Synth Demo], device 0: USB Audio [USB Audio]
      Subdevices: 1/1
      Subdevice #0: subdevice #0

This indicates that card 1, device 0 (i.e `hw:1,0`) is the Tomu capture device.
The same procedure with `aplay -l` with give you the ID of your playback device.

## Feeding music to the Tomu

Then, to play MIDI files, you can use `aplaymidi` to get the port name:

    $ aplaymidi -l
     Port    Client name                      Port name
     14:0    Midi Through                     Midi Through Port-0
     20:0    USB Synth Demo                   USB Synth Demo MIDI 1

Which can be passed to `aplaymidi` to play some music:

    $ aplaymidi --port 20:0 <your_midi_file>.mid

If all goes well, you should hear a pretty terrible rendition of your MIDI file, and the LEDs will blink in time.

## Playing the Tomu as a live MIDI instrument

If you're really keen and have a MIDI keyboard, you can hook that up to the MIDI port and play the thing live.
Use `aconnect -l` to view all your ports:

    $ aconnect -i
    client 0: 'System' [type=kernel]
        0 'Timer           '
        1 'Announce        '
    client 14: 'Midi Through' [type=kernel]
        0 'Midi Through Port-0'
    client 20: 'USB Synth Demo' [type=kernel,card=1]
        0 'USB Synth Demo MIDI 1'
    client 24: 'A-Series Keyboard' [type=kernel,card=2]
        0 'A-Series Keyboard Keyboard'

Then, to connect your keyboard output to the Tomu, for the above `aconnect -l` output:

    $ aconnect 24:0 20:0

Where the arguments are `<from client>:<id> <to client>:<id>`.

## Known issues

I experienced underruns very occasionally (i.e clicks and scratches), but these
seemed to disappear by killing and re-starting `alsaloop`. Your mileage may vary...
