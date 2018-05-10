#include <libopencm3/efm32/wdog.h>

int main(int argc, char **argv) {
    (void)argc; /* Indicate argc is not used */
    (void)argv; /* Indicate argv is not used, either */

    /* Disable the watchdog that the bootloader started.
     * Without this, Tomu would reboot after a short time.
     */
    WDOG_CTRL = 0;

    /* Loop forever */
    while (1)
        ;
}