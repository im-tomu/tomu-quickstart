# README

This example implements a USB CDC-ACM device (aka Virtual Serial Port)
to demonstrate the use of the USB device stack.

When a terminal is connected the green LED will light up.  When disconnected,
the green LED will turn off.

When data is recieved, it will echo the data back to the host.

The red LED is toggled constantly and a string is sent over USB every
time the LED changes state as a heartbeat.
