# Commodore BASIC Emulator

This implements a USB CDC-ACM device (aka Virtual Serial Port), which is an interface to the Commodore 64 BASIC interpreter running inside a 6502 emulator.

	   **** COMMODORE 64 BASIC V2 ****
	
	64K RAM SYSTEM  6380 BASIC BYTES FREE
	
	READY.

Connect to Tomu using a terminal emulator (like minicom) and start hacking in BASIC, like this:

    10 REM SIEVE
    20 FORI=1TO120000
    30 FORJ=2TOSQR(I)
    40 IFI/J=INT(I/J)GOTO70
    50 NEXTJ
    60 PRINTI;
    70 NEXTI
    RUN

Because Tomu only has 8 KB of RAM, only about 6 KB bytes are avaiable to BASIC. A few more hundred bytes could be squeezed out of the device, for example by reusing $0090-$00ff and $0259-$02ff for other data (e.g. the input buffer), since these areas are meant for the KERNAL and unused by BASIC.

The KERNAL interface only implements the most important calls to get BASIC running. File I/O will crash the interpreter.  [The cbmbasic project](https://github.com/mist64/cbmbasic) has a more complete KERNAL implemenation.
