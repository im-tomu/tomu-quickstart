/**
 * This example demonstrates using EFM32HG Timer PWM and interrupts.
 * The program implements a USB CDC-ACM device (aka Virtual Serial Port)
 * to provide a shell interface to the Tomu's LEDs.
 * The green and red LEDs can be configured individually, and synchronized, 
 * to create a vareity of fade/blink/brighness patterns.
 * The interface lends itself to scripting, so the Tomu can be used as a notification light.
 * See the README.md file for full command syntax.
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/efm32/wdog.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/cmu.h>
#include <libopencm3/efm32/timer.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// Makes this program compatible with Toboot-V2.0
#include <toboot.h>
TOBOOT_CONFIGURATION(0);
//TOBOOT_CONFIGURATION(TOBOOT_CONFIG_FLAG_AUTORUN); // Uncomment to boot directly to this app, instead of the DFU bootloader.

// Serial prompt
#define PROMPT          "CMD> "

#define LED_GREEN_PORT  GPIOA
#define LED_GREEN_PIN   GPIO0
#define LED_RED_PORT    GPIOB
#define LED_RED_PIN     GPIO7

#define VENDOR_ID       0x1209  // pid.code
#define PRODUCT_ID      0x70b1  // Assigned to Tomu project
#define DEVICE_VER      0x0BEB  // Program version

#define TIMER_INPUT_CLOCK_FREQUENCY 24000000 // 24 MHz clock
#define LED_PWM_TIMER_TOP_CCV 100
#define LED_PWM_TIMER_PRESCALER TIMER_CTRL_PRESC_DIV16 // Provides ~99% accuracy. Use PRESC_DIV2 for ~99.8% accuracy.
#define LED_PWM_TIMER_CYCLES_DENOMINATOR ((LED_PWM_TIMER_TOP_CCV + 1) * (1 << LED_PWM_TIMER_PRESCALER))
#define LED_PWM_TIMER_CYCLES_PER_MILLISECOND (uint32_t) ((((float) TIMER_INPUT_CLOCK_FREQUENCY / LED_PWM_TIMER_CYCLES_DENOMINATOR) + 500) / 1000)
#define EP_WRITE_RETRY_DELAY_USECS 50

// Duty cycle percentage
#define MAX_PWM_VALUE 100
#define MAX_PWM_VALUE_STRING "100"
#define MIN_PWM_VALUE 0
#define MIN_PWM_VALUE_STRING "0"

// Supported commands
#define CMD_HELP 'H'
#define CMD_PRINT 'P'
#define CMD_GREEN_LED_CFG 'G'
#define CMD_RED_LED_CFG 'R'
#define CMD_SYNC_TIMERS 'S'
#define CMD_DEBUG_PRINTS_FLAG 'D'
#define CMD_SERIAL_ECHO_FLAG 'E'
#define CMD_TEST_MODE 'T'

// Parameters for LED commands
#define PARAM_TEST_MODE 'T'
#define PARAM_MAX_PWM 'X'
#define PARAM_MIN_PWM 'N'
#define PARAM_LOW_MS 'L'
#define PARAM_HIGH_MS 'H'
#define PARAM_RAMPUP_MS 'U'
#define PARAM_RAMPDOWN_MS 'D'
#define PARAM_CURRENT_PHASE 'P'

#define PARAM_PHASE_LOW 'L'
#define PARAM_PHASE_HIGH 'H'
#define PARAM_PHASE_RAMPUP 'U'
#define PARAM_PHASE_RAMPDOWN 'D'

#define PARAM_GREEN_LED 'G'
#define PARAM_GREEN_LED_STRING "G"
#define PARAM_RED_LED 'R'
#define PARAM_RED_LED_STRING "R"

#define LED_TEST_MODE_MAX 4

enum led_colour {
	GREEN_LED = 0,
	RED_LED = 1
};

enum led_fade_phase {
	LED_PHASE_LOW = 0,
	LED_PHASE_RAMP_UP = 1,
	LED_PHASE_HIGH = 2,
	LED_PHASE_RAMP_DOWN = 3
};

struct led_pwm_cfg
{
	// Fade cycle durations in milliseconds
	uint32_t ramp_up_ms;
	uint32_t ramp_down_ms;
	uint32_t high_duration_ms;
	uint32_t low_duration_ms;
	// Brightness percentage values, 0-100.
	uint8_t min_brightness;
	uint8_t max_brightness;
	// Current fade cycle phase
	enum led_fade_phase current_phase;
};

struct led_pwm_cfg g_green_led_cfg, g_red_led_cfg;

static volatile bool g_usbd_is_connected = false;
static usbd_device *g_usbd_dev = 0;
static uint8_t g_current_command[1024];
static uint8_t g_new_command[1024];
static volatile uint32_t g_new_command_len;
static volatile bool g_should_use_new_command;
static volatile bool g_debug_prints_enabled = true;
static volatile bool g_serial_echo_enabled = true;
static volatile bool g_green_counters_reset = false;
static volatile bool g_red_counters_reset = false;


static const struct usb_device_descriptor dev =
{
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = VENDOR_ID,
	.idProduct = PRODUCT_ID,
	.bcdDevice = DEVICE_VER,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] =
{{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] =
{{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors =
{
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 6,
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	 }
};

static const struct usb_interface_descriptor comm_iface[] =
{{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 0,

	.endpoint = comm_endp,

	.extra = &cdcacm_functional_descriptors,
	.extralen = sizeof(cdcacm_functional_descriptors)
}};

static const struct usb_interface_descriptor data_iface[] =
{{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = data_endp,
}};

static const struct usb_interface ifaces[] =
{{
	.num_altsetting = 1,
	.altsetting = comm_iface,
}, {
	.num_altsetting = 1,
	.altsetting = data_iface,
}};

static const struct usb_config_descriptor config =
{
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] =
{
	"Tomu",
	"Serial Shell",
	"LED PWM",
};

// This busywait loop is roughly accurate when running at 24 MHz.
void udelay_busy(uint32_t usecs)
{
	while (usecs --> 0) {
		/* This inner loop is 3 instructions, one of which is a branch.
		 * This gives us 4 cycles total.
		 * We want to sleep for 1 usec, and there are cycles per usec at 24 MHz.
		 * Therefore, loop 6 times, as 6*4=24.
		 */
		asm volatile(
			"mov   r1, #6\n"
			"retry:\n"
			"sub   r1, #1\n"
			"bne   retry\n"
			"nop"
			: : : "r1"
		);
	}
}

/* Buffer to be used for control requests. */
static uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch(req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		g_usbd_is_connected = req->wValue & 1; // Check RTS bit
		return USBD_REQ_HANDLED;
	break;
	case USB_CDC_REQ_SET_LINE_CODING: 
		if(*len < sizeof(struct usb_cdc_line_coding))
			return 0;

		return USBD_REQ_HANDLED;
	break;
	}
	
	return 0;
}

// Rx callback: Echoes back the input, stores the received command.
static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;

	char buf[64];
	char output[64];
	uint32_t len = usbd_ep_read_packet(usbd_dev, 0x01, buf, sizeof(buf));

	if (len) {
		uint32_t i;
		uint32_t new_len = 0;
		for (i = 0; (i < len) && (new_len < sizeof(buf)); i++) {
			// Look for '\r' and append '\n'
			if (buf[i] == '\r') {
				output[new_len++] = buf[i];
				if (new_len >= sizeof(output))
					new_len--;
				output[new_len++] = '\n';
				if (new_len >= sizeof(output))
					new_len--;

				// Switch over to using the new message.
				g_should_use_new_command = true;
			}
			// For backspace characters, go back one space, print a 'space' to overwrite
			// the character, then move the cursor back by one.
			else if ((buf[i] == 0x7f) || (buf[i] == '\b')) {
				if (g_new_command_len > 0) {
					output[new_len++] = '\b';
					if (new_len >= sizeof(output))
						new_len--;
					output[new_len++] = ' ';
					if (new_len >= sizeof(output))
						new_len--;
					output[new_len++] = '\b';
					if (new_len >= sizeof(output))
						new_len--;
					g_new_command[--g_new_command_len] = '\0';
				}
				continue;
			}
			// For printable characters, put them in the new-message buffer.
			else if (isprint((int) buf[i])) {
				g_new_command[g_new_command_len++] = buf[i];
				output[new_len++] = buf[i];
				if (new_len >= sizeof(output))
					new_len--;
			}
		}
		if (g_serial_echo_enabled) {
			usbd_ep_write_packet(usbd_dev, 0x82, output, new_len);
		}
		output[new_len] = 0;
	}
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, 0);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, 0);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}

static void usb_puts(char *s)
{
	if (!g_usbd_is_connected) return;

	uint16_t return_value = usbd_ep_write_packet(g_usbd_dev, 0x82, s, strnlen(s, 64));
	// The endpoint might be busy transmitting, wait a little and retry.
	while (!return_value) {
		udelay_busy(EP_WRITE_RETRY_DELAY_USECS);
		return_value = usbd_ep_write_packet(g_usbd_dev, 0x82, s, strnlen(s, 64));
	}
}

void usb_isr(void)
{
	usbd_poll(g_usbd_dev);
}

void hard_fault_handler(void)
{
	while(1);
}

inline static void set_led_timer_ccvb(enum led_colour led, uint32_t value)
{
	switch (led) {
		case GREEN_LED:
			TIMER0_CC0_CCVB = value;
			return;
		break;
		case RED_LED:
			TIMER1_CC0_CCVB = value;
			return;
		break;
		default: // Should never happen
			// usb_puts("ERROR SET LED TIMER CCVB\r\n"); // Commented to simplify inline function
			return;
		break;
	}
}

// State machine for LED fade phases cycle
static void update_led_state(const enum led_colour led, uint32_t* const cycle_millis)
{
	struct led_pwm_cfg* led_cfg = NULL;
	
	switch (led) {
	case GREEN_LED:
		led_cfg = &g_green_led_cfg;
	break;
	case RED_LED:
		led_cfg = &g_red_led_cfg;
	break;
	default:
		usb_puts("\r\nERROR LED COLOUR!\r\n");
		return;
	break;
	}
	
	const uint8_t brightness_delta = led_cfg->max_brightness - led_cfg->min_brightness;

	switch (led_cfg->current_phase) {
		case LED_PHASE_LOW:
			if (*cycle_millis >= led_cfg->low_duration_ms) {
				led_cfg->current_phase = LED_PHASE_RAMP_UP;
				*cycle_millis = 0;				
			} else {
				// Not necessary, but smooths the transition when setting the current phase from outside this function.
				set_led_timer_ccvb(led, led_cfg->min_brightness);
			}
		break;
		case LED_PHASE_RAMP_UP:
			if (*cycle_millis >= led_cfg->ramp_up_ms) {
				led_cfg->current_phase = LED_PHASE_HIGH;
				set_led_timer_ccvb(led, led_cfg->max_brightness);
				*cycle_millis = 0;
			} else {
				set_led_timer_ccvb(led, led_cfg->min_brightness + (brightness_delta *  (*cycle_millis) / led_cfg->ramp_up_ms));
			}
		break;
		case LED_PHASE_HIGH:
			if (*cycle_millis >= led_cfg->high_duration_ms) {
				led_cfg->current_phase = LED_PHASE_RAMP_DOWN;
				*cycle_millis = 0;
			} else {
				// Not necessary, but smooths the transition when setting the current phase from outside this function.
				set_led_timer_ccvb(led, led_cfg->max_brightness);
			}
		break;
		case LED_PHASE_RAMP_DOWN:
			if (*cycle_millis >= led_cfg->ramp_down_ms) {
				led_cfg->current_phase = LED_PHASE_LOW;
				set_led_timer_ccvb(led, led_cfg->min_brightness);
				*cycle_millis = 0;
			} else {
				set_led_timer_ccvb(led, led_cfg->max_brightness - (brightness_delta *  (*cycle_millis) / led_cfg->ramp_down_ms));
			}
		break;
		default: // Should never happen
			usb_puts("\r\nERROR LED PHASE!\r\n");
		break;
	}
}

// Interrupt for Red LED
void timer1_isr(void)
{
	// Clear overflow interrupt flag
	TIMER1_IFC = TIMER_IFC_OF;
	
	static uint32_t timer_cycles = 0;
	static uint32_t cycle_millis = 0;

	if (g_red_counters_reset) {
		g_red_counters_reset = false;
		timer_cycles = 0;
		cycle_millis = 0;
	}

	++timer_cycles;

	if (!(timer_cycles % LED_PWM_TIMER_CYCLES_PER_MILLISECOND)) {
		++cycle_millis;
		timer_cycles = 0;
	}

	update_led_state(RED_LED, &cycle_millis);
}

// Timer1 used for Red LED PWM
static void setup_timer1(void)
{
	cmu_periph_clock_enable(CMU_TIMER1);
	timer_set_clock_prescaler(TIMER1, LED_PWM_TIMER_PRESCALER);
	timer_set_top(TIMER1, LED_PWM_TIMER_TOP_CCV);
    
	TIMER1_CC0_CCV = 0; // PWM compare value
	TIMER1_CNT = 0; // Initial counter value

    TIMER1_IEN = TIMER_IEN_OF; // Interrupt enable: Overflow
    
	TIMER1_CC0_CTRL	|= TIMER_CC_CTRL_OUTINV; // Invert the output, our LEDs are active-low
	TIMER1_CC0_CTRL |= TIMER_CC_CTRL_MODE(TIMER_CC_CTRL_MODE_PWM);	

	// Set TIM1_CC0 output to Red LED GPIO's location
	// RED LED GPIO = PB7, TIM1_CC0 LOCATION #3
	TIMER1_ROUTE |= TIMER_ROUTE_LOCATION_LOCx(TIMER_ROUTE_LOCATION_LOC3);
	TIMER1_ROUTE |= TIMER_ROUTE_CC0PEN; // Enable output CC0 for TIMER1
    
    nvic_enable_irq(NVIC_TIMER1_IRQ); // Enable TIMER1 interrupt

	timer_start(TIMER1);
}

// Interrupt for Green LED
void timer0_isr(void)
{
	TIMER0_IFC = TIMER_IFC_OF;	// Clear overflow interrupt flag

	static uint32_t timer_cycles = 0;
	static uint32_t cycle_millis = 0;

	if (g_green_counters_reset) {
		g_green_counters_reset = false;
		timer_cycles = 0;
		cycle_millis = 0;
	}

	++timer_cycles;

	if (!(timer_cycles % LED_PWM_TIMER_CYCLES_PER_MILLISECOND)) {
		++cycle_millis;
		timer_cycles = 0;
	}

	update_led_state(GREEN_LED, &cycle_millis);
}

// Timer0 used for Green LED PWM
static void setup_timer0(void)
{
	cmu_periph_clock_enable(CMU_TIMER0);
	timer_set_clock_prescaler(TIMER0, LED_PWM_TIMER_PRESCALER);
	timer_set_top(TIMER0, LED_PWM_TIMER_TOP_CCV);

	TIMER0_CC0_CCV = 0; // PWM compare value
	TIMER0_CNT = 0; // Initial counter value

    TIMER0_IEN = TIMER_IEN_OF; // Interrupt enable: Overflow

	TIMER0_CC0_CTRL	|= TIMER_CC_CTRL_OUTINV; // Invert the output, our LEDs are active-low
	TIMER0_CC0_CTRL |= TIMER_CC_CTRL_MODE(TIMER_CC_CTRL_MODE_PWM);	

	// Set TIM0_CC0 output to Green LED GPIO's location
	// GREEN LED GPIO = PA0, TIM0_CC0 LOC #0/1/4, also TIM0_CC1 LOCATION #6.
	TIMER0_ROUTE |= TIMER_ROUTE_LOCATION_LOCx(TIMER_ROUTE_LOCATION_LOC0);
	TIMER0_ROUTE |= TIMER_ROUTE_CC0PEN; // Enable output CC0 for TIMER0

	nvic_enable_irq(NVIC_TIMER0_IRQ); // Enable TIMER0 interrupt

	timer_start(TIMER0);
}


static void led_timer_stop(enum led_colour led)
{
	switch (led) {
		case GREEN_LED:
			timer_stop(TIMER0);
			TIMER0_CNT = 0;
		break;
		case RED_LED:
			timer_stop(TIMER1);
			TIMER1_CNT = 0;
		break;
		default: // Should never happen
			usb_puts("ERROR LED TIMER STOP\r\n");
			return;
		break;
	}
}

inline static void led_timer_start(enum led_colour led)
{
	switch (led) {
		case GREEN_LED:
			timer_start(TIMER0);
		break;
		case RED_LED:
			timer_start(TIMER1);
		break;
		default: // Should never happen
			//usb_puts("ERROR LED TIMER START\r\n"); // Commented to simplify inline function
			return;
		break;
	}
}

static void led_isr_counters_reset(enum led_colour led)
{
	switch (led) {
		case GREEN_LED:
			g_green_counters_reset = true;
		break;
		case RED_LED:
			g_red_counters_reset = true;
		break;
		default: // Should never happen
			usb_puts("ERROR LED COUNTERS RESET\r\n");
			return;
		break;
	}
}

static void sync_timers(const enum led_fade_phase green_phase, const enum led_fade_phase red_phase)
{
	led_timer_stop(GREEN_LED);
	led_timer_stop(RED_LED);
	led_isr_counters_reset(GREEN_LED);
	led_isr_counters_reset(RED_LED);
	g_green_led_cfg.current_phase = green_phase;
	g_red_led_cfg.current_phase = red_phase;
	udelay_busy(1000);
	timer_start(TIMER0);
	timer_start(TIMER1);
}

// Returns true iff the letter represents a LED fade phase
// If valid, sets numeric_value to the corresponding enum value.
static bool set_if_valid_phase_letter(char letter, uint32_t* numeric_value)
{
	switch (letter) {
		case PARAM_PHASE_LOW:
			*numeric_value = LED_PHASE_LOW;
			return true;
		break;
		case PARAM_PHASE_RAMPUP:
			*numeric_value = LED_PHASE_RAMP_UP;
			return true;
		break;
		case PARAM_PHASE_RAMPDOWN:
			*numeric_value = LED_PHASE_RAMP_DOWN;
			return true;
		break;
		case PARAM_PHASE_HIGH:
			*numeric_value = LED_PHASE_HIGH;
			return true;
		break;
		default:
			return false;
		break;
	}							
}

// Set the appropriate configuration field for the LED
// Returns true if successful, false otherwise.
static bool set_led_cfg_value(enum led_colour led, uint8_t field, uint32_t numeric_value)
{
	struct led_pwm_cfg* led_cfg = NULL;
	
	switch (led) {
		case GREEN_LED:
			led_cfg = &g_green_led_cfg;
		break;
		case RED_LED:
			led_cfg = &g_red_led_cfg;
		break;
		default:
			usb_puts("\r\nERROR LED COLOUR!\r\n");
			return false;
		break;
	}

	switch (field) {
		case PARAM_MAX_PWM:
			if (numeric_value > MAX_PWM_VALUE) {
				usb_puts("ERROR: PWM MAX value: "MAX_PWM_VALUE_STRING"\r\n");
				return false;
			}				
			led_cfg->max_brightness = (uint8_t) numeric_value;
		break;
		case PARAM_MIN_PWM:
			/* // As long as MIN_PWM_VALUE == 0, this comparison to unsigned is redundant
			if (numeric_value < MIN_PWM_VALUE) {
				usb_puts("ERROR: PWM MIN value: "MIN_PWM_VALUE_STRING"\r\n");
				return false;
			}
			*/			
			led_cfg->min_brightness = (uint8_t) numeric_value;
		break;
		case PARAM_LOW_MS:
			led_cfg->low_duration_ms = numeric_value;
		break;
		case PARAM_HIGH_MS:
			led_cfg->high_duration_ms = numeric_value;
		break;
		case PARAM_RAMPUP_MS:
			led_cfg->ramp_up_ms = numeric_value;
		break;
		case PARAM_RAMPDOWN_MS:
			led_cfg->ramp_down_ms = numeric_value;
		break;
		case PARAM_CURRENT_PHASE:
			switch (numeric_value) {
				case LED_PHASE_LOW:
				case LED_PHASE_RAMP_UP:
				case LED_PHASE_HIGH:
				case LED_PHASE_RAMP_DOWN:
					led_cfg->current_phase = numeric_value;
				break;
				default:
					usb_puts("\r\nERROR PWM PHASE VALUE!\r\n");
					return false;
				break;
			}
			
		break;
		default:
			usb_puts("\r\nERROR LED CFG FIELD!\r\n");
			return false;
		break;
	}

	if (g_debug_prints_enabled) {
		char message_buf[11];
		itoa(numeric_value, message_buf, 10);
		usb_puts("\r\nSet value: ");
		usb_puts(message_buf);
	}

	return true;
}

static void set_led_test_mode(struct led_pwm_cfg* led_cfg, uint32_t mode)
{
	if (led_cfg == NULL) {
		usb_puts("ERROR LED TEST\r\n");
		return;
	}

	switch (mode) {
		case 0: // LED breathing demo, low power indication
			led_cfg->min_brightness = 0;
			led_cfg->max_brightness = 100;
			led_cfg->low_duration_ms = 4000;
			led_cfg->high_duration_ms = 500;
			led_cfg->ramp_up_ms = 1000;
			led_cfg->ramp_down_ms = 1000; 
			led_cfg->current_phase = LED_PHASE_LOW;
		break;
		case 1: // LED blink demo
			led_cfg->min_brightness = 0;
			led_cfg->max_brightness = 100;
			led_cfg->low_duration_ms = 1000;
			led_cfg->high_duration_ms = 1000;
			led_cfg->ramp_up_ms = 0;
			led_cfg->ramp_down_ms = 0;
			led_cfg->current_phase = LED_PHASE_LOW;
		break;
		case 2: // LED constant ON demo
			led_cfg->min_brightness = 100;
			led_cfg->max_brightness = 100;
			led_cfg->low_duration_ms = 0;
			led_cfg->high_duration_ms = 1000;
			led_cfg->ramp_up_ms = 0;
			led_cfg->ramp_down_ms = 0;
			led_cfg->current_phase = LED_PHASE_HIGH;
		break;
		case 3: // LED constant OFF demo
			led_cfg->max_brightness = 0;
			led_cfg->min_brightness = 0;
			led_cfg->low_duration_ms = 1000;
			led_cfg->high_duration_ms = 0;
			led_cfg->ramp_up_ms = 0;
			led_cfg->ramp_down_ms = 0;
			led_cfg->current_phase = LED_PHASE_LOW;
		break;
		case 4: // LED symmetric min to max
			led_cfg->max_brightness = 100;
			led_cfg->min_brightness = 0;
			led_cfg->low_duration_ms = 1000;
			led_cfg->high_duration_ms = 1000;
			led_cfg->ramp_up_ms = 10000;
			led_cfg->ramp_down_ms = 10000;
			led_cfg->current_phase = LED_PHASE_LOW;
		break;
		default:
			usb_puts("ERROR CMD TEST\r\n");
			return;
		break;
	}

	if (g_debug_prints_enabled) {
		char message_buf[4];
		usb_puts("\r\nSet test mode: ");
		itoa(mode, message_buf, 10);
		usb_puts(message_buf);
	}
}

static void advance_led_test_mode(const enum led_colour led)
{
	static uint32_t green_led_test_mode = 0;
	static uint32_t red_led_test_mode = 0;
	uint32_t* current_test_mode = NULL;
	
	switch (led) {
	case GREEN_LED:
		set_led_test_mode(&g_green_led_cfg, green_led_test_mode);
		current_test_mode = &green_led_test_mode;
	break;
	case RED_LED:
		set_led_test_mode(&g_red_led_cfg, red_led_test_mode);
		current_test_mode = &red_led_test_mode;
	break;
	default:
		usb_puts("\r\nERROR LED COLOUR!\r\n");
		return;
	break;
	}

	if (*current_test_mode >= LED_TEST_MODE_MAX) {
		*current_test_mode = 0;
	} else {
		++(*current_test_mode);
	}
}

// Parses commands with one parameter: Either 0 or 1
// Returns: true on successful parse, false otherwise.
static bool parse_flag(uint8_t flag, uint8_t param_char)
{
	volatile bool* flag_p = NULL;

	switch (flag) {
	case CMD_DEBUG_PRINTS_FLAG:
		flag_p = &g_debug_prints_enabled;
	break;
	case CMD_SERIAL_ECHO_FLAG:
		flag_p = &g_serial_echo_enabled;
	break;
	default:
		usb_puts("ERROR FLAG TYPE\r\n"); // Should never happen
		return false;
	}

	if (flag_p == NULL) { // Should never happen
		usb_puts("ERROR FLAG POINTER\r\n");
		return false;
	}

	if (param_char == (uint8_t) '1') {
		*flag_p = true;
		return true;
	} else if (param_char == (uint8_t) '0') {
		*flag_p = false;
		return true;
	} else {
		usb_puts("ERROR PARSE FLAG\r\n");
		return false;
	}

	return false; // Shouldn't get here
}

// Input parameter buffer is not assumed to be null-terminated
// Returns: true on successful parse, false otherwise.
// If successful, numeric_value is set to the value parsed.
// If successful, the value of last_char is the index of the last element parsed in the buffer
static bool parse_numeric_value(char* buffer, uint32_t len, uint32_t* numeric_value, uint32_t* last_char)
{
	uint32_t current_value = 0;
	uint32_t num_digits = 0;

	if ((buffer == NULL) || (numeric_value == NULL) || (last_char == NULL) || (len == 0)) {
		usb_puts("ERROR PARSE NUMERIC VALUE\r\n");
		return false;
	}

	while (num_digits <= len) {
		if (isdigit((int) buffer[num_digits])) {
			current_value = (current_value * 10) + (buffer[num_digits] - '0');
			++num_digits;
		} else {
			break;
		}
	}

	if (num_digits == 0) {
		usb_puts("ERROR: Converting string to numeric value!\r\n");
		return false;
	}

	*numeric_value = current_value;    
	(*last_char)+= (num_digits - 1);

	return true;
}

static void print_led_cfg(struct led_pwm_cfg* led_cfg)
{
	if (led_cfg == NULL) {
		usb_puts("ERROR PRINT LED CFG\r\n");
		return;
	}

	char string_buffer[12];

	usb_puts("\r\n=== LED CFG ===");
	usb_puts("\r\nMin brightness %: ");
	itoa(led_cfg->min_brightness, string_buffer, 10);
	usb_puts(string_buffer);
	usb_puts("\r\nMax brightness %: ");
	itoa(led_cfg->max_brightness, string_buffer, 10);
	usb_puts(string_buffer);
	usb_puts("\r\nLow duration ms: ");
	itoa(led_cfg->low_duration_ms, string_buffer, 10);
	usb_puts(string_buffer);
	usb_puts("\r\nRamp up time ms: ");
	itoa(led_cfg->ramp_up_ms, string_buffer, 10);
	usb_puts(string_buffer);
	usb_puts("\r\nHigh duration ms: ");
	itoa(led_cfg->high_duration_ms, string_buffer, 10);
	usb_puts(string_buffer);
	usb_puts("\r\nRamp down time ms: ");
	itoa(led_cfg->ramp_down_ms, string_buffer, 10);
	usb_puts(string_buffer);
	usb_puts("\r\nCurrent Phase [Low, Ramp up, High, Ramp Down]: ");
	
	// Output the current phase as a letter
	string_buffer[1] = '\r';
	string_buffer[2] = '\n';
	string_buffer[3] = '\0';
	
	switch (led_cfg->current_phase) {
		case LED_PHASE_LOW:
			string_buffer[0] = PARAM_PHASE_LOW;
		break;
		case LED_PHASE_RAMP_UP:
			string_buffer[0] = PARAM_PHASE_RAMPUP;
		break;
		case LED_PHASE_HIGH:
			string_buffer[0] = PARAM_PHASE_HIGH;
		break;
		case LED_PHASE_RAMP_DOWN:
			string_buffer[0] = PARAM_PHASE_RAMPDOWN;
		break;
		default: // Should never happen
			strcpy(string_buffer, "ERROR");
		break;
	}
	
	usb_puts(string_buffer);
}

static void print_help()
{
	usb_puts("\r\nUsage: <COMMAND>[PARAMETERS] <COMMAND>...");
	usb_puts("\r\nSupported commands: <H,T,D,E,P,S,G,R>");
	usb_puts("\r\nPrint help message: <H>");
	usb_puts("\r\nAdvance both LEDs to their next test mode: <T>");
	usb_puts("\r\nDisable/Enable debug printout: <D><0,1>");
	usb_puts("\r\nDisable/Enable serial shell echo: <E><0,1>");
	usb_puts("\r\nPrint current configuration of the Green/Red LED: <P><G,R>");
	usb_puts("\r\nSync LEDs from phase (Green,Red): <S><[L,U,H,D][L,U,H,D]>");
	usb_puts("\r\nSet Green/Red LED config: <G,R><T,N,X,L,U,H,D,P>\r\n");
}

static void handle_command(uint8_t* cmd_buffer, uint32_t buffer_len)
{
	uint8_t current_flag;
	enum led_colour current_led;
	uint8_t current_field;
	uint32_t numeric_value;

	if ((cmd_buffer == NULL) || (buffer_len == 0)) {
		return;
	}

	uint32_t current_char = 0;
	while (current_char < buffer_len) {

		if (isspace(cmd_buffer[current_char])) {
			++current_char;
			continue;
		}

		switch (toupper(cmd_buffer[current_char])) {
			case CMD_HELP:
				print_help();
			break;
			case CMD_PRINT:
				++current_char;
				if (current_char >= buffer_len) {
					usb_puts("ERROR: Missing Print parameter: ["PARAM_GREEN_LED_STRING","PARAM_RED_LED_STRING"]\r\n");
					return; // Don't parse the rest of the input
				}

				switch (toupper(cmd_buffer[current_char]))	{
					case PARAM_GREEN_LED:
						print_led_cfg(&g_green_led_cfg);
					break;
					case PARAM_RED_LED:
						print_led_cfg(&g_red_led_cfg);
					break;
					default:
						usb_puts("ERROR: Expected Print parameter: ["PARAM_GREEN_LED_STRING","PARAM_RED_LED_STRING"]\r\n");
						return; // Don't parse the rest of the input
					break;
				}
			break;	 
			case CMD_TEST_MODE:
				advance_led_test_mode(GREEN_LED);
				advance_led_test_mode(RED_LED);
				sync_timers(LED_PHASE_LOW, LED_PHASE_LOW);	
			break;
			case CMD_DEBUG_PRINTS_FLAG:
			case CMD_SERIAL_ECHO_FLAG:
				current_flag = (uint8_t) toupper(cmd_buffer[current_char]);
				++current_char;
				if (current_char >= buffer_len) {
					usb_puts("Missing flag param: 0/1\r\n");
					return; // Don't parse the rest of the input
				}
				if (!parse_flag(current_flag, cmd_buffer[current_char])) {
					return; // Don't parse the rest of the input
				}
			break;
			case CMD_SYNC_TIMERS:
			{
				++current_char;
				if (current_char >= buffer_len) {
					usb_puts("ERROR: Missing Green LED phase!\r\n");
					return; // Don't parse the rest of the input
				}

				uint32_t green_led_phase_value;
				if (!set_if_valid_phase_letter((char) toupper(cmd_buffer[current_char]), &green_led_phase_value)) {
					usb_puts("ERROR: Expected phase letter!\r\n");
					return; // Don't parse the rest of the input
				}

				++current_char;
				if (current_char >= buffer_len) {
					usb_puts("ERROR: Missing Red LED phase!\r\n");
					return; // Don't parse the rest of the input
				}

				uint32_t red_led_phase_value;
				if (!set_if_valid_phase_letter((char) toupper(cmd_buffer[current_char]), &red_led_phase_value)) {
					usb_puts("ERROR: Expected phase letter!\r\n");
					return; // Don't parse the rest of the input
				}

				sync_timers(green_led_phase_value, red_led_phase_value);
			}
			break;
			case CMD_GREEN_LED_CFG:
			case CMD_RED_LED_CFG:

				if (toupper(cmd_buffer[current_char]) == CMD_GREEN_LED_CFG) {
					current_led = GREEN_LED;
				} else {
					current_led = RED_LED;
				}

				++current_char;
				if (current_char >= buffer_len) {
					usb_puts("ERROR: Missing LED params!\r\n");
					return; // Don't parse the rest of the input
				}

				led_timer_stop(current_led);
				
				switch (toupper(cmd_buffer[current_char]))	{
					case PARAM_TEST_MODE:
						advance_led_test_mode(current_led);
					break;
					case PARAM_MAX_PWM:
					case PARAM_MIN_PWM:
					case PARAM_LOW_MS:
					case PARAM_HIGH_MS:
					case PARAM_RAMPUP_MS:
					case PARAM_RAMPDOWN_MS:
					case PARAM_CURRENT_PHASE:

						current_field = toupper(cmd_buffer[current_char]);

						++current_char;
						if (current_char >= buffer_len) {
							usb_puts("ERROR: Missing value!\r\n");
							return; // Don't parse the rest of the input
						}

						if ((current_field == PARAM_CURRENT_PHASE) && (set_if_valid_phase_letter((char) toupper(cmd_buffer[current_char]), &numeric_value))) {
							// numeric_value was set to the appropriate phase enum value														
						} else if (!parse_numeric_value((char*) &(cmd_buffer[current_char]), buffer_len - current_char, &numeric_value, &current_char)) {
							return; // Don't parse the rest of the input
						}

						if (!set_led_cfg_value(current_led, current_field, numeric_value)) {
							return; // Don't parse the rest of the input
						}
					break;

					default:
						usb_puts("ERROR: Unknown param!\r\n");
						return; // Don't parse the rest of the input
					break;
				}

				led_isr_counters_reset(current_led);
				led_timer_start(current_led);

			break;
			default:
				usb_puts("ERROR Parsing Command\r\n");
				return; // Don't parse the rest of the input
		} // Switch

		++current_char;
	} // while()

} // handle_command

int main(void)
{
	bool line_was_connected = false;

	/* Disable the watchdog that the bootloader started. */
	WDOG_CTRL = 0;

	/* GPIO peripheral clock is necessary for us to set up the GPIO pins as outputs */
	cmu_periph_clock_enable(CMU_GPIO);

	/* Set up both LEDs as outputs */
	gpio_mode_setup(LED_RED_PORT, GPIO_MODE_WIRED_AND, LED_RED_PIN);
	gpio_mode_setup(LED_GREEN_PORT, GPIO_MODE_WIRED_AND, LED_GREEN_PIN);
	// LED GPIO polarity is reveresed, gpio_set will turn off LEDs.
	gpio_set(LED_RED_PORT, LED_RED_PIN);
	gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);

	/* Configure the USB core & stack */
	g_usbd_dev = usbd_init(&efm32hg_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(g_usbd_dev, cdcacm_set_config);

	/* Set the CPU Core to run from the trimmed USB clock, divided by 2.
	 * This will give the CPU Core a frequency of 24 MHz +/- 1% */
	CMU_CMD = CMU_CMD_HFCLKSEL(5);
	while (! (CMU_STATUS & CMU_STATUS_USHFRCODIV2SEL))
		;

	// Enable USB IRQs
	nvic_enable_irq(NVIC_USB_IRQ);

	// Initialize LED fade values to first test mode
	set_led_test_mode(&g_green_led_cfg, 0); 
	set_led_test_mode(&g_red_led_cfg, 0); 

	setup_timer0();
	setup_timer1();

	while (1) {

		if (line_was_connected != g_usbd_is_connected) {
			if (g_usbd_is_connected) {
				// Wait for the terminal to appear
				udelay_busy(1000);
				usb_puts("\r\nEnter command, followed by Return:\r\n" PROMPT);
			}
			line_was_connected = g_usbd_is_connected;
		}

		if (!g_usbd_is_connected)
			continue;

		// Handle the received command, if there's a new one available.
		if (g_should_use_new_command) {
			memset(g_current_command, 0, sizeof(g_current_command));

			// Print a different message depending on whether the new message is blank.
			if (g_new_command_len) {
				memcpy(g_current_command, g_new_command, g_new_command_len);
				handle_command(g_current_command, g_new_command_len);
			}
			else {
				usb_puts("\r\n" PROMPT);
			}

			g_should_use_new_command = false;
			g_new_command_len = 0;
		}
	}
}
