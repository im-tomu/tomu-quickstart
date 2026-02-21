# Tomu USB Snake

A classic ASCII Snake game running on the Tomu board, rendered over USB serial to any terminal.

Snake moves automatically. Eat the `*`, avoid walls and yourself.  
Green LED flashes when you eat, red LED lights on game over.

## Usage

Input format: ``<COMMAND>[PARAMETERS] <COMMAND>...``  
Commands are *case-insensitive*.  
Commands are parsed when the return key is sent ("Enter").  
Several commands may be sent in a single line of input, separated by spaces if necessary.

## Supported commands

Outside Snake:
- `H`          show this help
- `D0` / `D1`  debug prints off/on
- `E0` / `E1`  serial echo off/on
- `S`          start Snake
- `F0` / `F1`  LED feedback off/on
- `L##`        Snake speed level [1-20]

Inside Snake:
- WASD         change direction instantly
- SPACE        pause / resume
- `[`          slower (+50 ms)
- `]`          faster (-50 ms)
- `X`          quit back to shell

## Example session

```
Enter command, followed by Return:
CMD> s

=== SNAKE STARTED ===
+----------------------------------------+
|                                        |
|                                        |
|                                        |
|                                        |
|                                        |
|                                        |
|                  oooooooo@             |
|                  o                     |
|                  o                     |
|                  o                     |
|                  o                     |
|                  o                     |
|                  o                     |
|                  o                     |
|      *           o                     |
|           oooooooo                     |
|                                        |
|                                        |
|                                        |
|                                        |
+----------------------------------------+
Score: 210    [ / ] speed  SPACE pause  X quit
Speed level: 18

=== GAME OVER! ===
Final Score: 210
Press any key to restart or X to quit
```

## Accessing the serial shell

### From Windows

Use a terminal emulator program, such as [PuTTY](https://www.putty.org/) or [Tera Term](https://ttssh2.osdn.jp/index.html.en).

### From Linux

Use a terminal program, such as [Minicom](https://en.wikipedia.org/wiki/Minicom) or [Screen](https://en.wikipedia.org/wiki/GNU_Screen)
