/* NOTE!
In timer.c in function "esp_err_t timer_get_counter_value" first 4 line are commented

    esp_err_t timer_get_counter_value(timer_group_t group_num, timer_idx_t timer_num, uint64_t *timer_val)
    {
        // TIMER_CHECK(group_num < TIMER_GROUP_MAX, TIMER_GROUP_NUM_ERROR, ESP_ERR_INVALID_ARG);
        // TIMER_CHECK(timer_num < TIMER_MAX, TIMER_NUM_ERROR, ESP_ERR_INVALID_ARG);
        // TIMER_CHECK(timer_val != NULL, TIMER_PARAM_ADDR_ERROR, ESP_ERR_INVALID_ARG);
        // TIMER_CHECK(p_timer_obj[group_num][timer_num] != NULL, TIMER_NEVER_INIT_ERROR, ESP_ERR_INVALID_ARG);
        TIMER_ENTER_CRITICAL(&timer_spinlock[group_num]);
        timer_hal_get_counter_value(&(p_timer_obj[group_num][timer_num]->hal), timer_val);
        TIMER_EXIT_CRITICAL(&timer_spinlock[group_num]);
        return ESP_OK;
    }
*/

/* Usefuld link */
// https://github.com/sankarcheppali/esp_idf_esp32_posts/tree/master/timer_group
// https://github.com/espressif/esp-idf/blob/98ad01e5fc0779d11209350d8ec31fe3d2137ed5/examples/peripherals/gpio/generic_gpio/main/gpio_example_main.c
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/gpio.h"

#define TIMER_DIVIDER         (2)  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

#define GPIO_OUTPUT_IO_0    18
#define GPIO_OUTPUT_IO_1    19
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))

#define GPIO_INPUT_IO_0     4
#define GPIO_INPUT_IO_1     5
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))

#define ESP_INTR_FLAG_DEFAULT 0


typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} example_timer_info_t;

// Global variables.
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool flag = 0;
static volatile uint64_t start_counter_value = 0;
static volatile uint64_t stop_counter_value = 0;

// Function and ISR
static void inline print_timer_counter(uint64_t counter_value)
{
    printf("Counter: 0x%08x%08x\r\n", (uint32_t) (counter_value >> 32), (uint32_t) (counter_value));
    printf("Time   : %.8f s\r\n", (double) counter_value / TIMER_SCALE);
}

static void example_tg_timer_init(int group, int timer, bool auto_reload, int timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(group, timer, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(group, timer, 0);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(group, timer, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(group, timer);

    example_timer_info_t *timer_info = calloc(1, sizeof(example_timer_info_t));
    timer_info->timer_group = group;
    timer_info->timer_idx = timer;
    timer_info->auto_reload = auto_reload;
    timer_info->alarm_interval = timer_interval_sec;

    timer_start(group, timer);
}

static void IRAM_ATTR start_isr_handler(void* arg)
{
    // For latency evaluation
    // gpio_set_level(GPIO_OUTPUT_IO_0, 1);
    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &start_counter_value);
}

static void IRAM_ATTR stop_isr_handler(void* arg)
{
    // For latency evaluation
    // gpio_set_level(GPIO_OUTPUT_IO_0, 0);
    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &stop_counter_value);
    portENTER_CRITICAL_ISR(&spinlock);
    flag = 1;
    portEXIT_CRITICAL_ISR(&spinlock);
}

void app_main(void)
{
    example_tg_timer_init(TIMER_GROUP_0, TIMER_0, true, 5);

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //install gpio isr service 0.000 003 95
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, start_isr_handler, (void*) GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, stop_isr_handler, (void*) GPIO_INPUT_IO_1);

    while (1) {
        if (flag == 1) {
            printf("-------- New event detected --------\n");
            print_timer_counter((stop_counter_value - start_counter_value));

            // ESP-IDF version of a critical section (in a task)
            portENTER_CRITICAL(&spinlock);
            flag = 0;
            portEXIT_CRITICAL(&spinlock);
        }
        vTaskDelay(300 / portTICK_RATE_MS);
    }
}
