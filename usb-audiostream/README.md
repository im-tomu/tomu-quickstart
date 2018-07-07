# README

This example implements a streaming USB microphone device to demonstrate
the use of the USB device stack.

The microphone device will constantly stream a stereo sawtooth wave
at a sample rate of 8KHz, using 16-bit PCM samples.

The green LED will light when audio is being recorded. The red LED blinks constantly.

# Usage

Warning: only tested on a Linux host! Windows is picky about audio descriptors,
so it's unlikely this will work on Windows without further work.

For a simple test, fire up an ordinary sound recording program (i.e Audacity)
and just record a few seconds of audio from the 'AUDIO Demo' device.

When playing with different sample rates, `arecord` is nice. To list your devices:

    $ arecord -l

To perform a recording (replace `hw:2,0` with your device as listed above):

    $ sudo arecord -D hw:2,0 -d 10 -f S16 -r 8000 -c 2 test.wav

A useful debugging test with the above is to time how long the command takes.
(just prefix it with `time`). If the command takes longer than the duration
specified, the Tomu is underrunning the sample rates specified.

If you really want to annoy people and feed the Tomu's audio to your
speakers in real-time, try:

    $ sudo alsaloop -C hw:1,0 -P hw:0,0 --rate=8000

The first hardware ID should be your Tomu, the second your computers' soundcard
