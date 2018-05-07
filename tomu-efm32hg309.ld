/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Uwe Hermann <uwe@hermann-uwe.de>
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

/* Linker script for Tomu board with EFM32HG309. */

/* Define memory regions. */
MEMORY
{
    rom (rx) : ORIGIN = 0x00004000, LENGTH = 0xC000
    ram (rwx)  : ORIGIN = 0x20000000, LENGTH = 0x2000
}

SECTIONS {
	.text : {
		KEEP(SORT(*)(.vectors))
	} >rom
}

/* Include the common ld script. */
INCLUDE libopencm3_efm32hg.ld
