#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/efm32/cmu.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/common/prs_common.h>
#include <libopencm3/efm32/timer.h>
#include <libopencm3/efm32/wdog.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "capsenseconfig.h"

#define LED_GREEN_PORT      GPIOA
#define LED_GREEN_PIN       GPIO0
#define LED_RED_PORT        GPIOB
#define LED_RED_PIN         GPIO7

#undef TIMER_CC_CTRL_INSEL
#define TIMER_CC_CTRL_INSEL            (1 << 20)

static void setup_capsense(void) {
    CMU_HFPERCLKDIV |= CMU_HFPERCLKDIV_HFPERCLKEN;
    //cmu_periph_clock_enable(CMU_HFPER);
    cmu_periph_clock_enable(CMU_TIMER0);
    cmu_periph_clock_enable(CMU_TIMER1);

    CMU_HFPERCLKEN0 |= ACMP_CAPSENSE_CLKEN;
    cmu_periph_clock_enable(CMU_PRS);

      /* Initialize TIMER0 - Prescaler 2^9, top value 10, interrupt on overflow */
    TIMER_CTRL(0) = TIMER_CTRL_PRESC_DIV512;
    timer_set_top(0, 10);
    TIMER_IEN(0)  = TIMER_IEN_OF;
    TIMER_CNT(0)  = 0;

    /* Initialize TIMER1 - Prescaler 2^10, clock source CC1, top value 0xFFFF */
    TIMER_CTRL(1) = TIMER_CTRL_PRESC_DIV1024 | TIMER_CTRL_CLKSEL_CC1;
    timer_set_top(1, 0xffff);

    /* Set up TIMER1 CC1 to trigger on PRS channel 0 */
    TIMER_CC1_CTRL(1) = TIMER_CC_CTRL_MODE_INPUTCAPTURE /* Input capture      */
                       | TIMER_CC_CTRL_PRSSEL_PRSCH0   /* PRS channel 0      */
                       | TIMER_CC_CTRL_INSEL       /* PRS input selected */
                       | TIMER_CC_CTRL_ICEVCTRL_RISING /* PRS on rising edge */
                       | TIMER_CC_CTRL_ICEDGE_BOTH;    /* PRS on rising edge */

  /*Set up PRS channel 0 to trigger on ACMP1 output*/
  PRS_CH0_CTRL = PRS_CH_CTRL_EDSEL_POSEDGE      /* Posedge triggers action */
                    | PRS_CH_CTRL_SOURCESEL_ACMP_CAPSENSE      /* PRS source */
                    | PRS_CH_CTRL_SIGSEL_ACMPOUT_CAPSENSE;     /* PRS source */
}

static void setup(void) {
    /* GPIO peripheral clock is necessary for us to set up the GPIO pins as outputs */
    cmu_periph_clock_enable(CMU_GPIO);

    /* Set up both LEDs as outputs */
    gpio_mode_setup(LED_RED_PORT, GPIO_MODE_WIRED_AND, LED_RED_PIN);
    gpio_mode_setup(LED_GREEN_PORT, GPIO_MODE_WIRED_AND, LED_GREEN_PIN);

    setup_capsense();
}

int main(int argc, char **argv) {
    setup();
}