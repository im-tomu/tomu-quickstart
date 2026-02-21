/**
 * This example demonstrates using EFM32HG Timer and interrupts.
 * The program implements a USB CDC-ACM device (aka Virtual Serial Port)
 * to provide a shell interface.
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

// Consecutive calls to usb_puts are not likely to be successful.
// Without a delay between them, more often than not only the first call is executed.
// The delay allows the USB driver to finish writing to the endpoint.
#define USB_PUTS_DELAY_USEC 4000

#define TIMER_INPUT_CLOCK_FREQUENCY 24000000 // 24 MHz clock
#define TIMER_TOP_CCV 1499
#define INTERRUPT_TIMER_PRESCALER TIMER_CTRL_PRESC_DIV16 // Provides ~99% accuracy. Use PRESC_DIV2 for ~99.8% accuracy.

// Supported commands
#define CMD_HELP 'H'
#define CMD_SNAKE 'S'
#define CMD_DEBUG_PRINTS_FLAG 'D'
#define CMD_SERIAL_ECHO_FLAG 'E'
#define CMD_LED_FEEDBACK_FLAG 'F'
#define CMD_SPEED_LEVEL_FLAG 'L'

#define SNAKE_TICK_MS_DEFAULT 200
#define SNAKE_TICK_MS_MAX 1000
#define SNAKE_TICK_MS_MIN 50
#define SNAKE_TICK_MS_INCREMENT 50

#define SNAKE_BOARD_HEIGHT 20
#define SNAKE_BOARD_WIDTH  40
#define SNAKE_SCORE_INCREMENT 10
#define SNAKE_MAX_TAIL_LENGTH 100

#define GREEN_LED_FLASH_MS 150

static volatile bool g_usbd_is_connected = false;
static usbd_device *g_usbd_dev = 0;
static uint8_t g_current_command[1024];
static uint8_t g_new_command[1024];
static volatile uint32_t g_new_command_len;
static volatile bool g_should_use_new_command;
static volatile bool g_debug_prints_enabled = true;
static volatile bool g_serial_echo_enabled = true;

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

void udelay_busy(uint32_t usecs);
static void usb_puts(char *s);
static void usb_puts_full(const char *s);

int snake_tail_x[SNAKE_MAX_TAIL_LENGTH], snake_tail_y[SNAKE_MAX_TAIL_LENGTH];
int snake_tail_len;
int gameover, key, score;
int snake_head_x, snake_head_y, fruit_x, fruit_y;
int in_snake_mode = 0;
int game_over_displayed = 0;

static char screen_buf[1100];
static char pending_snake_key = 0;
int snake_need_redraw = 0;
volatile uint32_t green_flash_ms = 0; // Duration to turn green LED on
int paused = 0;

volatile uint32_t snake_ms_counter = 0;
volatile uint32_t snake_tick_ms = SNAKE_TICK_MS_DEFAULT;
volatile bool snake_do_tick = false;
volatile bool snake_led_feedback = true;

static uint32_t prng_seed = 0xA5A5A5A5UL;

static uint32_t simple_rand(uint32_t max)
{
    prng_seed = prng_seed * 1103515245UL + 12345;
    return (prng_seed >> 16) % max;
}

static void snake_setup(void)
{
    gameover = 0;
    key = 0;
    score = 0;
    snake_tail_len = 3;
    paused = 0;

    snake_head_x = SNAKE_BOARD_WIDTH / 2;
    snake_head_y = SNAKE_BOARD_HEIGHT / 2;

    prng_seed ^= (uint32_t)&snake_head_x;
    prng_seed = prng_seed * 1103515245UL + 12345;

    fruit_x = 10 + simple_rand(SNAKE_BOARD_WIDTH - 20);
    fruit_y = 5  + simple_rand(SNAKE_BOARD_HEIGHT - 10);

    for (int i = 0; i < snake_tail_len; i++) {
        snake_tail_x[i] = snake_head_x - i;
        snake_tail_y[i] = snake_head_y;
    }

	/* prevent any future garbage draw */
    for (int i = snake_tail_len; i < SNAKE_MAX_TAIL_LENGTH; i++) {
        snake_tail_x[i] = -1;
        snake_tail_y[i] = -1;
    }

    /* LEDs off at start of game */
    gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);
    gpio_set(LED_RED_PORT, LED_RED_PIN);
    green_flash_ms = 0;
}

static void snake_logic(void)
{
    if (key == 0) return;

    int prev_x = snake_head_x;
    int prev_y = snake_head_y;

    /* move head */
    switch (key) {
        case 1: snake_head_x--; break;  /* A */
        case 2: snake_head_x++; break;  /* D */
        case 3: snake_head_y--; break;  /* W */
        case 4: snake_head_y++; break;  /* S */
    }

    /* wall */
    if (snake_head_x < 0 || snake_head_x >= SNAKE_BOARD_WIDTH || snake_head_y < 0 || snake_head_y >= SNAKE_BOARD_HEIGHT) {
        gameover = 1;
		if (snake_led_feedback) {
			gpio_clear(LED_RED_PORT, LED_RED_PIN);
		}
        return;
    }

    /* eat fruit FIRST — so the next link in the body has correct coordinates */
    if (snake_head_x == fruit_x && snake_head_y == fruit_y) {
        score += SNAKE_SCORE_INCREMENT;
        snake_tail_len++;                     /* grow before shifting */
		
		if (snake_led_feedback) {
			green_flash_ms = GREEN_LED_FLASH_MS;
			gpio_clear(LED_GREEN_PORT, LED_GREEN_PIN);
		}
        
		fruit_x = 5 + simple_rand(SNAKE_BOARD_WIDTH - 10);
        fruit_y = 3 + simple_rand(SNAKE_BOARD_HEIGHT - 6);
    }

    /* now shift body (new length already includes the growth) */
    for (int i = snake_tail_len - 1; i > 0; i--) {
        snake_tail_x[i] = snake_tail_x[i-1];
        snake_tail_y[i] = snake_tail_y[i-1];
    }
    snake_tail_x[0] = prev_x;   /* old head becomes new body segment */
    snake_tail_y[0] = prev_y;

    /* self collision */
    for (int i = 0; i < snake_tail_len; i++) {
        if (snake_tail_x[i] == snake_head_x && snake_tail_y[i] == snake_head_y) {
            gameover = 1;
			if (snake_led_feedback) {
            	gpio_clear(LED_RED_PORT, LED_RED_PIN);
			}
			return;
        }
    }
}

static void snake_draw(void)
{
    char *p = screen_buf;

    *p++ = '\x1B'; *p++ = '['; *p++ = '2'; *p++ = 'J';
    *p++ = '\x1B'; *p++ = '['; *p++ = 'H';

    *p++ = '+';
    for (int i = 0; i < SNAKE_BOARD_WIDTH; i++) *p++ = '-';
    *p++ = '+'; *p++ = '\r'; *p++ = '\n';

    for (int i = 0; i < SNAKE_BOARD_HEIGHT; i++) {
        *p++ = '|';
        for (int j = 0; j < SNAKE_BOARD_WIDTH; j++) {
            if (i == snake_head_y && j == snake_head_x)
                *p++ = '@';
            else if (i == fruit_y && j == fruit_x)
                *p++ = '*';
            else {
                int is_body = 0;
                for (int k = 0; k < snake_tail_len; k++) {
                    if (snake_tail_x[k] == j && snake_tail_y[k] == i) {
                        *p++ = 'o';
                        is_body = 1;
                        break;
                    }
                }
                if (!is_body) *p++ = ' ';
            }
        }
        *p++ = '|'; *p++ = '\r'; *p++ = '\n';
    }

    *p++ = '+';
    for (int i = 0; i < SNAKE_BOARD_WIDTH; i++) *p++ = '-';
    *p++ = '+'; *p++ = '\r'; *p++ = '\n';

    const char *s = "Score: ";
    while (*s) *p++ = *s++;

    char num_string_buf[12];
	itoa(score, num_string_buf, 10);
	char *np = num_string_buf;

    while (*np) *p++ = *np++;
	*p++ = ' ';   /* extra space so footer doesn't glue to score */

	const char *footer = paused ? "   PAUSED  [ / ] speed  SPACE resume  X quit\r\n"
                                : "   [ / ] speed  SPACE pause  X quit\r\n";

	while (*footer) *p++ = *footer++;
						
    // Print speed level
	const char *level_text = "Speed level: ";
    while (*level_text) *p++ = *level_text++;

	uint32_t speed_level = ((SNAKE_TICK_MS_MAX - snake_tick_ms) / SNAKE_TICK_MS_INCREMENT) + 1;
    itoa(speed_level, num_string_buf, 10);
	np = num_string_buf;
    while (*np) *p++ = *np++;

    *p = '\0';

    usb_puts_full(screen_buf);
}

static void snake_handle_key(char c)
{
    c = toupper(c);

    if (c == 'X') {
        in_snake_mode = 0;
        g_serial_echo_enabled = 1;
        g_new_command_len = 0;
        gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);
        gpio_set(LED_RED_PORT, LED_RED_PIN);
        usb_puts("\r\n=== SNAKE QUIT ===\r\n");
        return;
    }

    if (gameover) {
        snake_setup();
        snake_need_redraw = 1;
        game_over_displayed = 0;
        gameover = 0;
        snake_ms_counter = 0;
        gpio_set(LED_RED_PORT, LED_RED_PIN);
        return;
    }

    if (c == ' ') {
        paused = !paused;
        snake_need_redraw = 1;
        return;
    }

    switch (c) {
        case 'W': key = 3; break;
        case 'A': key = 1; break;
        case 'S': key = 4; break;
        case 'D': key = 2; break;
        case '[': if (snake_tick_ms < SNAKE_TICK_MS_MAX) snake_tick_ms += SNAKE_TICK_MS_INCREMENT; snake_need_redraw = 1; break;
        case ']': if (snake_tick_ms > SNAKE_TICK_MS_MIN) snake_tick_ms -= SNAKE_TICK_MS_INCREMENT; snake_need_redraw = 1; break;
    }
}

static void snake_auto_tick(void)
{
    if (paused || gameover) return;
    snake_logic();
    snake_need_redraw = 1;
}

// This busywait loop is roughly accurate when running at 24 MHz.
void udelay_busy(uint32_t usecs)
{
	while (usecs --> 0) {
		/* This inner loop is 3 instructions, one of which is a branch.
		 * This gives us 4 cycles total.
		 * We want to sleep for 1 usec, and there are cycles per usec at 24 MHz.
		 * Therefore, loop 6 times, as 6*4=24.
		 */
		asm("mov   r1, #6");
		asm("retry:");
		asm("sub r1, #1");
		asm("bne retry");
		asm("nop");
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
		// Look for '\r' and append '\n'
		uint32_t i;
		uint32_t new_len = 0;
		for (i = 0; (i < len) && (new_len < sizeof(buf)); i++) {

			if (in_snake_mode) {
				pending_snake_key = buf[i];   // just queue the raw char
				return;  /* consume the char — never reaches shell parser */
			}

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
	if (g_usbd_is_connected) {
		usbd_ep_write_packet(g_usbd_dev, 0x82, s, strnlen(s, 64));
	}
}

static void usb_puts_full(const char *s)
{
    if (!g_usbd_is_connected) return;

    size_t len = strnlen(s, 2048);  /* safe max for our grid */
    size_t offset = 0;

    while (offset < len) {
        size_t chunk = len - offset;
        if (chunk > 64) chunk = 64;

        usbd_ep_write_packet(g_usbd_dev, 0x82, (void*)(s + offset), chunk);
        offset += chunk;

		udelay_busy(200);   /* <-- increased from 50 — this fixes PuTTY tearing */
		// Without the delay the printout won't always be handled correctly in the terminal
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

// Interrupt for Snake tick + fixed-time LED flashes (1 ms per interrupt)
void timer0_isr(void)
{
	TIMER0_IFC = TIMER_IFC_OF;	// Clear overflow interrupt flag

	// Snake game tick
	snake_ms_counter++;
	if (snake_ms_counter >= snake_tick_ms) {
		snake_do_tick = true;
		snake_ms_counter = 0;
	}

	if (green_flash_ms > 0) {
		green_flash_ms--;
		if (green_flash_ms == 0) {
			gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);
		}
	}
}

static void setup_timer0(void)
{
	cmu_periph_clock_enable(CMU_TIMER0);
	timer_set_clock_prescaler(TIMER0, INTERRUPT_TIMER_PRESCALER);
	timer_set_top(TIMER0, TIMER_TOP_CCV);   // 1 ms at (24 MHz / 16), 1.5 MHz.

	TIMER0_CNT = 0; // Initial counter value
	TIMER0_IEN = TIMER_IEN_OF;     // overflow interrupt only

	nvic_enable_irq(NVIC_TIMER0_IRQ);
	timer_start(TIMER0);
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

static void print_help()
{
	usb_puts("\r\nUsage: <COMMAND>[PARAMETERS] <COMMAND>...");
	udelay_busy(USB_PUTS_DELAY_USEC);

	usb_puts("\r\nSupported commands: <H,D,E,S,F,L>");
	udelay_busy(USB_PUTS_DELAY_USEC);

	usb_puts("\r\nPrint help message: <H>");
	udelay_busy(USB_PUTS_DELAY_USEC);

	usb_puts("\r\nDisable/Enable debug printout: <D><0,1>");
	udelay_busy(USB_PUTS_DELAY_USEC);

	usb_puts("\r\nDisable/Enable serial shell echo: <E><0,1>");
	udelay_busy(USB_PUTS_DELAY_USEC);

	usb_puts("\r\nPlay Snake: <S>");
	udelay_busy(USB_PUTS_DELAY_USEC);

	usb_puts("\r\nDisable/Enable Snake LED feedback: <F><0,1>");
	udelay_busy(USB_PUTS_DELAY_USEC);
		
	usb_puts("\r\nSnake speed level: <L><##>");
	udelay_busy(USB_PUTS_DELAY_USEC);
}

static void handle_command(uint8_t* cmd_buffer, uint32_t buffer_len)
{
	uint8_t current_flag;

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
			case CMD_LED_FEEDBACK_FLAG:
				++current_char;
				if (current_char >= buffer_len) {
					usb_puts("ERROR: Missing param (0/1)\r\n");
					return;
				}
				snake_led_feedback = (cmd_buffer[current_char] == '1');
				usb_puts(snake_led_feedback ? "\r\nLED feedback ON\r\n" : "\r\nLED feedback OFF\r\n");
			break;
			case CMD_SPEED_LEVEL_FLAG:
				++current_char;
				uint32_t speed_level;
				if (!parse_numeric_value((char*)&cmd_buffer[current_char], buffer_len - current_char, &speed_level, &current_char)) {
					return;
				}
		
				if (speed_level < 1) {
					speed_level = 1;
				}
				if (speed_level > (SNAKE_TICK_MS_MAX/SNAKE_TICK_MS_INCREMENT)) {
					speed_level = (SNAKE_TICK_MS_MAX/SNAKE_TICK_MS_INCREMENT);
				}

				snake_tick_ms = SNAKE_TICK_MS_MAX - ((speed_level - 1) * SNAKE_TICK_MS_INCREMENT);

				usb_puts("\r\nSnake speed level: ");
				udelay_busy(USB_PUTS_DELAY_USEC);
				char level_buf[4];
				itoa(speed_level, level_buf, 10);
				usb_puts(level_buf);
				udelay_busy(USB_PUTS_DELAY_USEC);
				usb_puts("\r\n");
			break;

			case CMD_SNAKE:
				in_snake_mode = 1;
				game_over_displayed = 0;
				g_serial_echo_enabled = 0;
				snake_setup();
				snake_need_redraw = 1;
				usb_puts("\r\n=== SNAKE STARTED ===\r\n");		
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

	setup_timer0();

	while (1) {

		prng_seed = prng_seed * 1103515245UL + 12345;  /* runtime entropy from loop timing + human keypress delay */

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

		/* ====================== SNAKE PHASE 2 (your preferred clean structure) ====================== */
        if (in_snake_mode) {
            if (pending_snake_key) {
                snake_handle_key(pending_snake_key);
                pending_snake_key = 0;
            }

            if (snake_do_tick) {
                snake_auto_tick();
                snake_do_tick = false;
            }

            if (snake_need_redraw) {
                snake_draw();
                snake_need_redraw = 0;
            }

            if (gameover && !game_over_displayed) {
                usb_puts_full("\x1B[2J\x1B[H");
                usb_puts_full("\r\n=== GAME OVER! ===\r\nFinal Score: ");

                char num_string_buf[12];
				itoa(score, num_string_buf, 10);                
                usb_puts_full(num_string_buf);

                usb_puts_full("\r\nPress any key to restart or X to quit\r\n");
                game_over_displayed = 1;
            }
        }

	} // while(1)
}
