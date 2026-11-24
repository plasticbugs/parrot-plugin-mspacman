# Ms. Pac-Man

This plugin plays Midway's 1981 Ms. Pac-Man arcade game on the ColecoVision. It needs the game's ROMs from MAME, which you supply.

## Setup

1. Copy `MSPACMAN.CVP` into the `/PLUGINS` folder on the card.
2. Make a folder named `/MSPACMAN` and put MAME's `mspacman.zip` in it as it is, or the files from it loose. You can also put the zip in the card's top folder as `/MSPACMAN.ZIP`.
3. Start Ms. Pac-Man from the Plugins page in the menu.

The files can have any names, because the plugin knows each ROM by its size and contents. If a ROM is missing, the screen names it. If a file has a ROM's name but other contents, the screen says that file is wrong.

## Controls

On the ColecoVision controller:

- The joystick moves Ms. Pac-Man.
- Keypad * or # adds a coin.
- Keypad 1 starts a one-player game, and so does either fire button.
- Keypad 2 starts a two-player game.
- Press RESET on the console to go back to the menu, or hold both fire buttons and \* for 2 seconds (Select and Start on a USB pad).

On a USB pad, through a powered hub:

- The D-pad or the left stick moves Ms. Pac-Man.
- Minus adds a coin. Plus, or the bottom or right face button, starts a one-player game.

The arcade stick has four ways. A diagonal turns Ms. Pac-Man the way you just pressed, as in MAME.

## Picture and sound

The arcade screen stands on its side and is 224 by 288 pixels. The ColecoVision shows 256 by 192, so the plugin scales the maze to three quarters of its size, 168 by 186, in the middle of the screen. The scores go on the left and right, with the lives at the bottom left and the fruit at the bottom right.

Ms. Pac-Man and the ghosts' bodies are the ColecoVision's sprites. When more than four share a line, they take turns, so some may flicker.

The ColecoVision has 15 colors, so some change. Ms. Pac-Man is light yellow, Pinky is magenta, and Sue, orange in the arcade, is dark yellow. Each maze has its own colors:

- Levels 1 and 2: light red walls with a dark red edge.
- Levels 3 to 5: light blue walls with a white edge.
- Levels 6 to 9: dark yellow walls with a white edge.
- Levels 10 to 13: dark blue walls with a dark yellow edge.
- From level 14 the third and fourth mazes take turns, four levels each, as in the arcade. The third comes back with magenta walls and a light yellow edge, then in its own colors. The fourth comes back in the first maze's colors, then in its own.

The arcade's three sound voices play through the ColecoVision's sound chip as 5,962 samples a second.

High scores last until you go back to the menu, as on the arcade machine when it is switched off.

## Credits

The arcade emulation is a port of MAME 0.288's code, under the BSD-3-Clause license. Its authors are:

- Nicola Salmoria and Stephane Humbert, for the Pac-Man and Ms. Pac-Man driver.
- Nicola Salmoria and Aaron Giles, for the Namco sound chip.
- Aaron Giles, for MAME's scheduler and the watchdog.
- Juergen Buchmueller and Andrei I. Holub, for the Z80 CPU.
- AJR, for the 74LS259 latch.
- Couriersud, for the resistor network colors.

You can find MAME at https://www.mamedev.org. The zip reader follows Mark Adler's puff, from zlib. USB support comes from TinyUSB by Ha Thach, under the MIT license.

The plugin itself is by Parrot Cart, and it's free. Its source is linked next to its download.

Ms. Pac-Man is a trademark of Bandai Namco. This plugin contains none of the game.
