# PWM Shell LED indicator

## Overview

This program demonstrates using Timer PWM and interrupts to control the Tomu's green and red LEDs.
A shell interface over USB serial (CDC-ACM) is implemented, based on the ACM and Opticspy demos.
The interface allows setting the Tomu's LEDs to various brightness values and blink/fade patterns.
Commands can be scripted externally, so the Tomu may be used as a notification light (in the spirit of "blink(1)").

The general case for LED operation is a cyclic fade ramp, from Low to ramp Up, High, ramp Down: ```__/⁻⁻⁻⁻\__```

The PWM duty cycle values for each LED are updated individually in its concomitant timer's OF interrupt.
Millisecond timing is estimated based on the LED Timer's frequency (derived from clock, prescale, top value settings).
This timing method can achieve > 99% accuracy, which seems acceptable for a LED indicator viewed by humans.
An alternative implementation can use an additional timer or the System Tick interrupt to count milliseconds and update the LEDs.

## Usage

Input format: ``<COMMAND>[PARAMETERS] <COMMAND>...``  
Command parameters are *case-insensitive*.  
Commands are parsed when the return key is sent ("Enter").  
Several commands may be sent in a single line of input, separated by spaces if necessary.

## Supported commands

### H : Print help message

### T : Advance both LEDs to their next test mode
* Cycles through several demo configurations.

### G,R : Configure Green/Red LED settings
* Syntax: ``<G,R><T,N,X,L,U,H,D,P>``
    * T : Set LED configuration to the next test mode (for demo purposes).
    * N : Min brightness, 0-100, duty cycle percentage.
    * X : Max brightness, 0-100, duty cycle percentage.
    * L : Low (Min) value duration in millisconds, positive integer.
    * U : Ramp up duration in millisconds, positive integer.
    * H : High (Max) value duration in millisconds, positive integer.
    * D : Ramp down duration in millisconds, positive integer.
    * P : Current phase, letter ``[L,U,H,D]`` or number ``[0,1,2,3]``.

### D : Disable/Enable debug printouts
* Syntax: ``<D><0,1>``

### E : Disable/Enable Serial Echo for the shell
* Syntax: ``<E><0,1>``

### P : Print current configuration of the Green/Red LED
* Syntax: ``<P><G,R>``

### S : Sync both LED timers.
* Syntax: ``<S><[L,U,H,D][L,U,H,D]>``
* The first phase letter is for the Green LED, second for Red LED.
* The command stops the timers, resets, sets the phases, then restarts both together.


## Example commands

* Printing the configuration of the Green and Red LEDs:
    ```
    pgpr
    ```
    Same as:
    ```
    PG PR
    ```
* Set constant 100% brightness for both LEDs:
    ```
    gn100 gx100 rn100 rx100 gl0 gu0 gh1000 gd0 rl0 ru0 rh1000 rd0
    ```
* Turn off Red LED:
    ```
    rn0rx0
    ```
* Set Green LED config, fade from 0 to 50 % brightness, each phase duration 1000 ms:
    ```
    gn0 gx50 gl1000 gu1000 gh1000 gd1000
    ```
* Set both LEDs to blink once a second:
    ```
    gn0 gx100 gl1000 gu0 gh1000 gd0 rn0 rx100 rl1000 ru0 rh1000 rd0
    ```
* Synchronize the LEDs, starting from Low phase:
    ```
    sll
    ```
* Set alternating Green/Red blink, once a second:
    ```
    gn0 gx100 gl1000 gu0 gh1000 gd0 rn0 rx100 rl1000 ru0 rh1000 rd0 slh
    ```
* Set alternating Green/Red fade in and out:
    ```
    gn0 gx50 gl3500 gu1500 gh500 gd1500 rn0 rx100 rl3500 ru1500 rh500 rd1500 slu
    ```

## Accessing the serial shell

### From Windows

Use a terminal emulator program, such as [PuTTY](https://www.putty.org/) or [Tera Term](https://ttssh2.osdn.jp/index.html.en).

### From Linux

* Use a terminal program, such as [Minicom](https://en.wikipedia.org/wiki/Minicom) or [Screen](https://en.wikipedia.org/wiki/GNU_Screen), either interactively or using a pipe.
* Redirect commands to the serial device, e.g. to send the "t" (test) command:
    ```
    echo -en “t\r” > /dev/ttyACM0
    ```