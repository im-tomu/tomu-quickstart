/*
 * This file is part of the libopencm3 project.
 *
 * Based on code:
 * Copyright (C) 2018 Seb Holzapfel <schnommus@gmail.com>
 *
 * Copyright (C) 2020 Marc MERLIN <marc_soft@merlins.org>
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

/**
 * \addtogroup Examples
 *
 * Toggles between the Red and Green LEDs.
 *
 * Red LED controlled by PA0
 * Green LED controlled by PB7
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/efm32/wdog.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/cmu.h>

#include <stdbool.h>
#include <stdio.h>

#include <toboot.h>

/* Declare support for Toboot V2 */
/* To enable this program to run when you first plug in Tomu, pass
 * TOBOOT_CONFIG_FLAG_AUTORUN to this macro.  Otherwise, leave the
 * configuration value at 0 to use the defaults.
 */
TOBOOT_CONFIGURATION(0);

/* Systick interrupt frequency, Hz */
#define SYSTICK_FREQUENCY 100000

/* USB (core clock) frequency of Tomu board */
#define USB_CLK_FREQUENCY 24000000

#define LED_GREEN_PORT GPIOA
#define LED_GREEN_PIN  GPIO0
#define LED_RED_PORT   GPIOB
#define LED_RED_PIN    GPIO7

volatile uint32_t system_millis = 0;



#define GPIO_PBT_RD    14

int pbt_get (void)
{
  return ((GPIO_PBT->IDR >> GPIO_PBT_RD) & 1) ^ GPIO_PBT_IS_LO;
}

void sys_tick_handler(void) {
    static uint16_t tick = 0;
    static int8_t  red_pwm_change =  4;
    static int8_t  green_pwm_change = -10;
    static int16_t red_pwm = 0;
    static int16_t green_pwm = 1000;
    static bool    red_on = true;
    static bool    green_on = true;

    // unused for now
    if (tick % 10 == 0) system_millis++;

    if (red_on &&   tick >= red_pwm) {
	red_on = false;
	gpio_toggle(LED_RED_PORT,   LED_RED_PIN);
    }
    if (green_on && tick >= green_pwm) {
	green_on = false;
	gpio_toggle(LED_GREEN_PORT, LED_GREEN_PIN);
    }

    if (tick++ == 1000) {
	tick = 0;
	red_pwm   += red_pwm_change;
	green_pwm += green_pwm_change;
	if (red_pwm == 0   || red_pwm   == 1000) red_pwm_change   *= -1;
	if (green_pwm == 0 || green_pwm == 1000) green_pwm_change *= -1;
	// turn LEDs back on
	gpio_toggle(LED_RED_PORT,   LED_RED_PIN);
	gpio_toggle(LED_GREEN_PORT, LED_GREEN_PIN);
	red_on   = true;
	green_on = true;
    }
}

int main(void)
{
    /* Disable the watchdog that the bootloader started. */
    WDOG_CTRL = 0;

    /* GPIO peripheral clock is necessary for us to set up the GPIO pins as outputs */
    cmu_periph_clock_enable(CMU_GPIO);

    /* Set up both LEDs as outputs */
    gpio_mode_setup(LED_RED_PORT,   GPIO_MODE_WIRED_AND, LED_RED_PIN);
    gpio_mode_setup(LED_GREEN_PORT, GPIO_MODE_WIRED_AND, LED_GREEN_PIN);

    gpio_set(LED_RED_PORT,   LED_RED_PIN);
    gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);
    gpio_toggle(LED_RED_PORT, LED_RED_PIN);
    gpio_toggle(LED_GREEN_PORT, LED_GREEN_PIN);

    /* Configure the system tick */
    /* Set the CPU Core to run from the trimmed USB clock, divided by 2.
     * This will give the CPU Core a frequency of 24 MHz +/- 1% */
    cmu_osc_on(USHFRCO);
    cmu_wait_for_osc_ready(USHFRCO);
    CMU_USBCRCTRL = CMU_USBCRCTRL_EN;
    CMU_CMD = CMU_CMD_HFCLKSEL(5);
    while (! (CMU_STATUS & CMU_STATUS_USHFRCODIV2SEL))
        ;

    systick_set_frequency(SYSTICK_FREQUENCY, USB_CLK_FREQUENCY);
    systick_counter_enable();
    systick_interrupt_enable();

    /* Spin forever, SysTick interrupt will toggle the LEDs */
    while(1);
}
