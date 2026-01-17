/* Basic console example (esp_console_repl API)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "argtable3/argtable3.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "cmd_system.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_mac.h" // ESP_MAC_WIFI_STA
#include "esp_console.h"


#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_intr_alloc.h"
#include "esp_memory_utils.h"
#include "esp_vfs_fat.h"
#include "esp_timer.h"
#include "linenoise/linenoise.h"



/*
 * We warn if a secondary serial console is enabled. A secondary serial console is always output-only and
 * hence not very useful for interactive console applications. If you encounter this warning, consider disabling
 * the secondary serial console in menuconfig unless you know what you are doing.
 */
#if SOC_USB_SERIAL_JTAG_SUPPORTED
#if !CONFIG_ESP_CONSOLE_SECONDARY_NONE
#warning "A secondary serial console is not useful when using the console component. Please disable it in menuconfig."
#endif
#endif

#define PROMPT_STR CONFIG_IDF_TARGET

static char wifi_password[64] = {0};

static const char *TAG = "led";

static esp_timer_handle_t pulse_off_timer = NULL;

static void pulse_off_callback(void* arg) {
    gpio_set_level(13, 0);
}

static struct {
    struct arg_int *pin;
    struct arg_int *value;
    struct arg_int *duration;
    struct arg_end *end;
} pulse_args;

static int pulse(int argc, char **argv)
{
    ESP_LOGI(TAG, "Pulse");
     int nerrors = arg_parse(argc, argv, (void **) &pulse_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, pulse_args.end, argv[0]);
        return 1;
    }
    int pin = pulse_args.pin->ival[0];
    int value = pulse_args.value->ival[0];
    int duration = pulse_args.duration->ival[0];
    ESP_LOGI(TAG, "Pulsing pin %d to %d for %d us", pin, value, duration);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, value);
    esp_rom_delay_us(duration);
    gpio_set_level(pin, !value);
    return 0;
}

static void register_pulse(void)
{
    pulse_args.pin = arg_int1(NULL, NULL, "<pin>", "GPIO pin to pulse");
    pulse_args.value = arg_int1(NULL, NULL, "<value>", "Pulse value (0 or 1)");
    pulse_args.duration = arg_int1(NULL, NULL, "<duration>", "duration in microseconds");
    pulse_args.end = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command = "pulse",
        .help = "Pulse a GPIO",
        .hint = NULL,
        .func = &pulse,
        .argtable = &pulse_args

    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}


static struct {
    struct arg_int *pin;
    struct arg_int *value;
    struct arg_end *end;
} level_args;

static int level(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &level_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, level_args.end, argv[0]);
        return 1;
    }
    int pin = level_args.pin->ival[0];

    if (level_args.value->count > 0) {
        // Set level
        int value = level_args.value->ival[0];
        ESP_LOGI(TAG, "Setting pin %d to %d", pin, value);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, value);
    } else {
        // Get level
        gpio_set_direction(pin, GPIO_MODE_INPUT); // Re-configure to be safe
        int level = gpio_get_level(pin);
        printf("level: %d\n", level);
    }

    return 0;
}

static void register_level(void)
{
    level_args.pin = arg_int1(NULL, NULL, "<pin>", "GPIO pin");
    level_args.value = arg_int0(NULL, NULL, "[value]", "Value to set (0 or 1). Omit to get current value.");
    level_args.end = arg_end(2);
    const esp_console_cmd_t cmd = {
        .command = "level",
        .help = "Set or get GPIO level. If <value> is omitted, the current level is printed.",
        .hint = NULL,
        .func = &level,
        .argtable = &level_args

    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}


static struct {
    struct arg_int *pin;
    struct arg_int *frequency;
    struct arg_int *duty;
    struct arg_end *end;
} pwm_args;

static int pwm(int argc, char **argv)
{
    ESP_LOGI(TAG, "pwm");
     int nerrors = arg_parse(argc, argv, (void **) &pwm_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, pwm_args.end, argv[0]);
        return 1;
    }
    int pin = pwm_args.pin->ival[0];
    int frequency = pwm_args.frequency->ival[0];
    int duty = pwm_args.duty->ival[0];

    if (frequency <= 0) {
        ESP_LOGE(TAG, "Frequency must be positive");
        return 1;
    }

    if (duty < 0 || duty > 99) {
        ESP_LOGE(TAG, "Duty cycle must be between 0 and 99");
        return 1;
    }

    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = frequency,
        .clk_cfg = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
        return 1;
    }

    ledc_channel_config_t channel_conf = {
        .gpio_num = pin,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    err = ledc_channel_config(&channel_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(err));
        return 1;
    }

    uint32_t duty_val = (1023 * duty) / 99;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty_val);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

    ESP_LOGI(TAG, "Pin: %d, Frequency: %d, Duty: %d", pin, frequency, duty);
    return 0;
}

static void register_pwm(void)
{
    pwm_args.pin = arg_int1(NULL, NULL, "<pin>", "GPIO pin to pulse");
    pwm_args.frequency = arg_int1(NULL, NULL, "<frequency>", "Frequency value (1-1000000)");
    pwm_args.duty = arg_int1(NULL, NULL, "<duty>", "Duty cycle value (0-99)");
    pwm_args.end = arg_end(3);
    const esp_console_cmd_t cmd = {
        .command = "pwm",
        .help = "Set GPIO pwm",
        .hint = NULL,
        .func = &pwm,
        .argtable = &pwm_args

    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}


static struct {
    struct arg_int *pin;
    struct arg_end *end;
} stoppwm_args;

static int stoppwm(int argc, char **argv)
{
    ESP_LOGI(TAG, "stoppwm");
     int nerrors = arg_parse(argc, argv, (void **) &stoppwm_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, stoppwm_args.end, argv[0]);
        return 1;
    }
    int pin = stoppwm_args.pin->ival[0];
    ESP_LOGI(TAG, "Stopping pwm on pin: %d", pin);
    ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ESP_ERROR_CHECK(gpio_reset_pin(pin) );
    return 0;
}

static void register_stoppwm(void)
{
    stoppwm_args.pin = arg_int1(NULL, NULL, "<pin>", "GPIO pin to stop pwm on");
    stoppwm_args.end = arg_end(1);
    const esp_console_cmd_t cmd = {
        .command = "stoppwm",
        .help = "Stop GPIO pwm",
        .hint = NULL,
        .func = &stoppwm,
        .argtable = &stoppwm_args

    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}




static QueueHandle_t gpio_evt_queue = NULL;

typedef struct {
    uint32_t gpio_num;
    int level;
} gpio_event_t;

static void gpio_task_example(void* arg)
{
    gpio_event_t evt;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &evt, portMAX_DELAY)) {
            printf("GPIO[%"PRIu32"] intr, val: %d, edge: %s\n", evt.gpio_num, evt.level, evt.level ? "RISING" : "FALLING");
        }
    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    int level = gpio_get_level(gpio_num);
    gpio_event_t evt = {
        .gpio_num = gpio_num,
        .level = level
    };
    
    gpio_set_level(13, 1);
    if (pulse_off_timer) {
        esp_timer_stop(pulse_off_timer);
        esp_timer_start_once(pulse_off_timer, 1000);
    }

    xQueueSendFromISR(gpio_evt_queue, &evt, NULL);
}

static struct {
    struct arg_int *pin;
    struct arg_str *edge;
    struct arg_end *end;
} interrupt_args;

static int interrupt(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &interrupt_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, interrupt_args.end, argv[0]);
        return 1;
    }

    int pin = interrupt_args.pin->ival[0];
    const char *edge_str = interrupt_args.edge->sval[0];

    gpio_int_type_t intr_type;
    if (strcmp(edge_str, "RISING") == 0) {
        intr_type = GPIO_INTR_POSEDGE;
    } else if (strcmp(edge_str, "FALLING") == 0) {
        intr_type = GPIO_INTR_NEGEDGE;
    } else if (strcmp(edge_str, "BOTH") == 0) {
        intr_type = GPIO_INTR_ANYEDGE;
    } else {
        ESP_LOGE(TAG, "Invalid interrupt type. Use RISING, FALLING or BOTH");
        return 1;
    }

    ESP_LOGI(TAG, "Setting interrupt on pin %d, edge: %s", pin, edge_str);

    gpio_config_t io_conf;
    io_conf.intr_type = intr_type;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_isr_handler_add(pin, gpio_isr_handler, (void*) (intptr_t) pin);

    return 0;
}

static void register_interrupt(void)
{
    interrupt_args.pin = arg_int1(NULL, NULL, "<pin>", "GPIO pin for interrupt");
    interrupt_args.edge = arg_str1(NULL, NULL, "<RISING|FALLING|BOTH>", "Interrupt edge");
    interrupt_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command = "interrupt",
        .help = "Set an interrupt on a GPIO pin",
        .hint = NULL,
        .func = &interrupt,
        .argtable = &interrupt_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static struct {
    struct arg_int *pin;
    struct arg_end *end;
} stopinterrupt_args;

static int stopinterrupt(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &stopinterrupt_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, stopinterrupt_args.end, argv[0]);
        return 1;
    }

    int pin = stopinterrupt_args.pin->ival[0];
    ESP_LOGI(TAG, "Removing interrupt on pin %d", pin);

    // Remove the handler
    gpio_isr_handler_remove(pin);

    // Disable interrupt type
    gpio_set_intr_type(pin, GPIO_INTR_DISABLE);

    return 0;
}

static void register_stopinterrupt(void)
{
    stopinterrupt_args.pin = arg_int1(NULL, NULL, "<pin>", "GPIO pin to remove interrupt");
    stopinterrupt_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "stopinterrupt",
        .help = "Remove interrupt on a GPIO pin",
        .hint = NULL,
        .func = &stopinterrupt,
        .argtable = &stopinterrupt_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

#define THROB_TIMER_PERIOD_MS 20


static TimerHandle_t throb_timer = NULL;
static uint32_t throb_step = 0;
static int throb_period_s = 0;

static struct {
    struct arg_int *period;
    struct arg_int *pin1;
    struct arg_int *pin2;
    struct arg_int *pin3;
    struct arg_end *end;
} throb_args;

static void throb_timer_callback(TimerHandle_t xTimer)
{
    throb_step++;
    float time_s = (throb_step * THROB_TIMER_PERIOD_MS) / 1000.0f;

    // Pin 1
    float sin_val_1 = sinf(2 * M_PI * time_s / throb_period_s);
    uint32_t duty1 = (uint32_t)((sin_val_1 + 1.0f) * 511.5f);

    // Pin 2
    float sin_val_2 = sinf(2 * M_PI * time_s / throb_period_s + 2.0f * M_PI / 3.0f);
    uint32_t duty2 = (uint32_t)((sin_val_2 + 1.0f) * 511.5f);

    // Pin 3
    float sin_val_3 = sinf(2 * M_PI * time_s / throb_period_s + 4.0f * M_PI / 3.0f);
    uint32_t duty3 = (uint32_t)((sin_val_3 + 1.0f) * 511.5f);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, duty1);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, duty2);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4, duty3);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4);
}

static int throb(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&throb_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, throb_args.end, argv[0]);
        return 1;
    }

    ESP_LOGI(TAG, "Starting throb");

    if (throb_timer != NULL) {
        ESP_LOGW(TAG, "Throb timer already running, please stop it first");
        return 0;
    }

    throb_period_s = throb_args.period->ival[0];
    int pin1 = throb_args.pin1->ival[0];
    int pin2 = throb_args.pin2->ival[0];
    int pin3 = throb_args.pin3->ival[0];


    // Timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_1, // Use a different timer
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .freq_hz          = 5000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Channels
    ledc_channel_config_t ledc_channel[3] = {
        { .gpio_num = pin1, .speed_mode = LEDC_HIGH_SPEED_MODE, .channel = LEDC_CHANNEL_2, .timer_sel = LEDC_TIMER_1, .duty = 0, .hpoint = 0 },
        { .gpio_num = pin2, .speed_mode = LEDC_HIGH_SPEED_MODE, .channel = LEDC_CHANNEL_3, .timer_sel = LEDC_TIMER_1, .duty = 0, .hpoint = 0 },
        { .gpio_num = pin3, .speed_mode = LEDC_HIGH_SPEED_MODE, .channel = LEDC_CHANNEL_4, .timer_sel = LEDC_TIMER_1, .duty = 0, .hpoint = 0 },
    };

    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[i]));
    }

    throb_step = 0;
    throb_timer = xTimerCreate("throb_timer", pdMS_TO_TICKS(THROB_TIMER_PERIOD_MS), true, NULL, throb_timer_callback);
    if (throb_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create throb timer");
        return 1;
    }

    if (xTimerStart(throb_timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start throb timer");
        xTimerDelete(throb_timer, 0);
        throb_timer = NULL;
        return 1;
    }

    return 0;
}

static int stopthrob(int argc, char **argv)
{
    ESP_LOGI(TAG, "Stopping throb");

    if (throb_timer == NULL) {
        ESP_LOGW(TAG, "Throb timer is not running");
        return 0;
    }

    if (xTimerStop(throb_timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to stop throb timer");
        return 1;
    }
    xTimerDelete(throb_timer, 0);
    throb_timer = NULL;

    ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, 0);
    ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, 0);
    ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4, 0);

    return 0;
}


static void register_throb(void)
{
    throb_args.period = arg_int1(NULL, NULL, "<period>", "Throb period in seconds");
    throb_args.pin1 = arg_int1(NULL, NULL, "<pin1>", "First pin");
    throb_args.pin2 = arg_int1(NULL, NULL, "<pin2>", "Second pin");
    throb_args.pin3 = arg_int1(NULL, NULL, "<pin3>", "Third pin");
    throb_args.end = arg_end(4);
    const esp_console_cmd_t cmd = {
        .command = "throb",
        .help = "Start throbbing effect on 3 LEDs. Provide period in seconds and 3 pins.",
        .hint = NULL,
        .func = &throb,
        .argtable = &throb_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_stopthrob(void)
{
    const esp_console_cmd_t cmd = {
        .command = "stopthrob",
        .help = "Stop throbbing effect",
        .hint = NULL,
        .func = &stopthrob,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static TimerHandle_t repeat_timer = NULL;
static struct {
    int pin;
    int duration_us;
} repeat_data;

static void repeat_timer_callback(TimerHandle_t xTimer)
{
    gpio_set_level(repeat_data.pin, 1);
    esp_rom_delay_us(repeat_data.duration_us);
    gpio_set_level(repeat_data.pin, 0);
}

static struct {
    struct arg_int *pin;
    struct arg_int *frequency;
    struct arg_int *duration;
    struct arg_end *end;
} repeat_args;

static int repeat(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &repeat_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, repeat_args.end, argv[0]);
        return 1;
    }

    if (repeat_timer != NULL) {
        ESP_LOGW(TAG, "Repeat timer already running, stop it first");
        return 0;
    }

    int pin = repeat_args.pin->ival[0];
    int frequency = repeat_args.frequency->ival[0];
    int duration = repeat_args.duration->ival[0];

    if (frequency <= 0) {
         ESP_LOGE(TAG, "Frequency must be positive");
         return 1;
    }

    repeat_data.pin = pin;
    repeat_data.duration_us = duration;

    ESP_LOGI(TAG, "Starting repeat pulse on pin %d, freq %d Hz, dur %d us", pin, frequency, duration);

    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);

    int period_ms = 1000 / frequency;
    if (period_ms == 0) period_ms = 1; // Minimum 1ms resolution for software timers

    TickType_t period_ticks = pdMS_TO_TICKS(period_ms);
    if (period_ticks == 0) period_ticks = 1;

    repeat_timer = xTimerCreate("repeat_timer", period_ticks, pdTRUE, NULL, repeat_timer_callback);
    if (repeat_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create repeat timer");
        return 1;
    }

    if (xTimerStart(repeat_timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start repeat timer");
        xTimerDelete(repeat_timer, 0);
        repeat_timer = NULL;
        return 1;
    }

    return 0;
}

static void register_repeat(void)
{
    repeat_args.pin = arg_int1(NULL, NULL, "<pin>", "GPIO pin");
    repeat_args.frequency = arg_int1(NULL, NULL, "<frequency>", "Frequency (Hz)");
    repeat_args.duration = arg_int1(NULL, NULL, "<duration>", "Pulse duration (us)");
    repeat_args.end = arg_end(3);

    const esp_console_cmd_t cmd = {
        .command = "repeat",
        .help = "Repeat a pulse of duration at frequency",
        .hint = NULL,
        .func = &repeat,
        .argtable = &repeat_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static int stoprepeat(int argc, char **argv)
{
    ESP_LOGI(TAG, "Stopping repeat");
    if (repeat_timer == NULL) {
        ESP_LOGW(TAG, "Repeat timer is not running");
        return 0;
    }

    if (xTimerStop(repeat_timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to stop repeat timer");
        return 1;
    }
    xTimerDelete(repeat_timer, 0);
    repeat_timer = NULL;
    
    // Ensure pin is low
    gpio_set_level(repeat_data.pin, 0);

    return 0;
}

static void register_stoprepeat(void)
{
    const esp_console_cmd_t cmd = {
        .command = "stoprepeat",
        .help = "Stop the repeat command",
        .hint = NULL,
        .func = &stoprepeat,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}


static int info(int argc, char **argv)
{
    ESP_LOGI(TAG, "info");
    gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);
    return 0;

}


static void register_info(void)
{
 
    const esp_console_cmd_t cmd = {
        .command = "info",
        .help = "Display GPIO information",
        .hint = NULL,
        .func = &info,

    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}


static struct {
    struct arg_int *address;
    struct arg_end *end;
} mem_args;

static int mem_read(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &mem_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, mem_args.end, argv[0]);
        return 1;
    }
    uint32_t address = mem_args.address->ival[0];

    // A very basic check. Be careful!
    // This doesn't prevent all crashes.
    if (!esp_ptr_in_dram((void *)address) && !esp_ptr_in_iram((void *)address)) {
            printf("Warning: Address 0x%x may not be in a valid readable region (DRAM/IRAM).\n", (unsigned int)address);
    }

    uint32_t value = *(uint32_t *)address;
    printf("Value at 0x%08x: 0x%08x\n", (unsigned int)address, (unsigned int)value);
    return 0;
}

static void register_mem(void)
{
    mem_args.address = arg_int1(NULL, "address", "<address>", "Memory address to read from");
    mem_args.end = arg_end(1);
    const esp_console_cmd_t cmd = {
        .command = "mem",
        .help = "Read 4 bytes from a memory address.",
        .hint = NULL,
        .func = &mem_read,
        .argtable = &mem_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}




#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));
    ESP_LOGI(TAG, "SSID: %s\n", EXAMPLE_ESP_WIFI_SSID);
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
#ifdef CONFIG_ESP_WIFI_WPA3_COMPATIBLE_SUPPORT
            .disable_wpa3_compatible_mode = 0,
#endif
        },
    };
    strncpy((char*)wifi_config.sta.password, wifi_password, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s", EXAMPLE_ESP_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", EXAMPLE_ESP_WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}


static const char* MOUNT_POINT = "/storage";

static void mount_storage(void)
{
    ESP_LOGI(TAG, "Mounting FATFS storage...");
    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 4,
        .format_if_mount_failed = true,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
    esp_err_t err = esp_vfs_fat_spiflash_mount(MOUNT_POINT, "storage", &mount_config, &s_wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount storage (%s)", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Storage mounted at %s", MOUNT_POINT);
}

static void run_boot_script(void)
{
    char filepath[64];
    snprintf(filepath, sizeof(filepath), "%s/boot.txt", MOUNT_POINT);

    struct stat st;
    if (stat(filepath, &st) != 0) {
        ESP_LOGI(TAG, "boot.txt not found, creating default...");
        FILE *f = fopen(filepath, "w");
        if (f) {
            fprintf(f, "level 13 1\n");
            fclose(f);
        } else {
            ESP_LOGE(TAG, "Failed to create boot.txt");
            return;
        }
    }

    FILE *f = fopen(filepath, "r");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open boot.txt for reading");
        return;
    }

    char line[128];
    while (fgets(line, sizeof(line), f)) {
        // Strip newline
        size_t len = strlen(line);
        if (len > 0 && line[len-1] == '\n') {
            line[len-1] = '\0';
            len--;
        }
        if (len > 0 && line[len-1] == '\r') {
            line[len-1] = '\0';
        }

        if (strlen(line) == 0) continue;

        if (line[0] == '#') {
            printf("%s\n", line);
            continue;
        }

        printf("Executing from boot.txt: %s\n", line);
        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND) {
            printf("Unrecognized command\n");
        } else if (err == ESP_ERR_INVALID_ARG) {
            printf("Command empty\n");
        } else if (err == ESP_OK && ret != ESP_OK) {
            printf("Command returned non-zero error code: 0x%x (%d)\n", ret, ret);
        } else if (err != ESP_OK) {
            printf("Internal error: %s\n", esp_err_to_name(err));
        }
    }
    fclose(f);
}



/*
 * We warn if a secondary serial console is enabled. A secondary serial console is always output-only and
 * hence not very useful for interactive console applications. If you encounter this warning, consider disabling
 * the secondary serial console in menuconfig unless you know what you are doing.
 */
#if SOC_USB_SERIAL_JTAG_SUPPORTED
#if !CONFIG_ESP_CONSOLE_SECONDARY_NONE
#error "A secondary serial console is not useful when using the console component. Please disable it in menuconfig."
#endif
#endif

static int wifi(int argc, char **argv)
{
    uint8_t mac_addr[6] = {0};
    //Get MAC address for WiFi Station interface
    ESP_ERROR_CHECK(esp_read_mac(mac_addr, ESP_MAC_WIFI_STA));
    printf("WIFI MAC: %02x-%02x-%02x-%02x-%02x-%02x\n",
        mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

    wifi_ap_record_t ap_info;
    ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&ap_info));
    printf("Associated AP SSID: %s\n", ap_info.ssid);
    printf("Associated AP MAC: %02x-%02x-%02x-%02x-%02x-%02x\n",
        ap_info.bssid[0], ap_info.bssid[1], ap_info.bssid[2], ap_info.bssid[3], ap_info.bssid[4], ap_info.bssid[5]);
    printf("Wi-Fi channel: %d\n", ap_info.primary);
    printf("Wi-Fi RSSI: %d\n", ap_info.rssi);

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info);
    printf("IP address: " IPSTR "\n", IP2STR(&ip_info.ip));

    esp_ip6_addr_t ip6[5];
    int ip6_addrs = esp_netif_get_all_ip6(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), ip6);
    for (int j = 0; j < ip6_addrs; ++j)
    {
        esp_ip6_addr_type_t ipv6_type = esp_netif_ip6_get_addr_type(&(ip6[j]));
        printf("IPv6 address: " IPV6STR ", type: %d\n", IPV62STR(ip6[j]), ipv6_type);
    }
    return 0;
}

static void console_register_wifi(void)
{
    const esp_console_cmd_t cmd = {
        .command = "wifi",
        .help = "Print Wi-Fi information",
        .hint = NULL,
        .func = &wifi,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void get_wifi_password(void)
{
    char filepath[64];
    snprintf(filepath, sizeof(filepath), "%s/password.txt", MOUNT_POINT);

    FILE *f = fopen(filepath, "r");
    if (f) {
        if (fgets(wifi_password, sizeof(wifi_password), f)) {
            // Strip newline
            size_t len = strlen(wifi_password);
            if (len > 0 && wifi_password[len-1] == '\n') {
                wifi_password[len-1] = '\0';
                len--;
            }
            if (len > 0 && wifi_password[len-1] == '\r') {
                wifi_password[len-1] = '\0';
            }
        }
        fclose(f);
    }

    if (strlen(wifi_password) == 0) {
        printf("Wi-Fi Password not found in storage.\n");
        char *line = linenoise("Enter Wi-Fi Password: ");
        if (line) {
            strncpy(wifi_password, line, sizeof(wifi_password) - 1);
            linenoiseFree(line);
            
            f = fopen(filepath, "w");
            if (f) {
                fprintf(f, "%s\n", wifi_password);
                fclose(f);
                printf("Password saved to %s\n", filepath);
            } else {
                ESP_LOGE(TAG, "Failed to save password to storage");
            }
        }
    } else {
        ESP_LOGI(TAG, "Read Wi-Fi password from %s", filepath);
    }
}

void app_main(void)
{
    //Create a queue to handle gpio events from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(gpio_event_t));
    //Start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(0);

    // Create esp_timer for ISR pulse
    const esp_timer_create_args_t pulse_timer_args = {
            .callback = &pulse_off_callback,
            .name = "pulse_off"
    };
    ESP_ERROR_CHECK(esp_timer_create(&pulse_timer_args, &pulse_off_timer));

    //setup pin 13 for ISR debugging
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<13);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);


    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    repl_config.prompt = PROMPT_STR ">";
    repl_config.max_cmdline_length = CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH;


    /* Register commands */
    esp_console_register_help_command();
    register_system_common();

#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));

#else
#error Unsupported console type
#endif

    register_pulse();
    register_level();
    register_pwm();
    register_stoppwm();
    register_interrupt();
    register_stopinterrupt();
    register_throb();
    register_stopthrob();
    register_repeat();
    register_stoprepeat();
    register_info();
    register_mem();
    console_register_wifi();

    mount_storage();
    get_wifi_password();
    run_boot_script();

    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


    if (CONFIG_LOG_MAXIMUM_LEVEL > CONFIG_LOG_DEFAULT_LEVEL) {
        /* If you only want to open more logs in the wifi module, you need to make the max level greater than the default level,
         * and call esp_log_level_set() before esp_wifi_init() to improve the log level of the wifi module. */
        esp_log_level_set("wifi", CONFIG_LOG_MAXIMUM_LEVEL);
    }

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();


    printf(">");
    fflush(stdout);

    vTaskSuspend(NULL); // Start done.
    vTaskDelete(NULL); // Task functions should never return.
}