#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/ledc.h"
#include "esp_err.h"

#define GPIO_OUTPUT_LED 2
#define GPIO_OUTPUT_IR 14
#define GPIO_OUTPUT_IR_POWER 13
#define GPIO_OUTPUT_PIN_SEL                                                    \
  ((1ULL << GPIO_OUTPUT_LED) | (1ULL << GPIO_OUTPUT_IR) |                      \
   (1ULL << GPIO_OUTPUT_IR_POWER))
#define GPIO_INPUT_IR 12
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_INPUT_IR)
#define ESP_INTR_FLAG_DEFAULT 0

#define RMT_TX_CHANNEL RMT_CHANNEL_4
#define RMT_TX_GPIO_NUM GPIO_NUM_14
#define RMT_RX_CHANNEL RMT_CHANNEL_0
#define RMT_RX_GPIO_NUM GPIO_NUM_12
#define RMT_CLK_DIV 100
#define RMT_TICK_10_US (80000000 / RMT_CLK_DIV / 100000)
#define rmt_item32_TIMEOUT_US 10000

#define MAX_SIGNAL_LEN 1024 * 8

#define LEDC_HS_TIMER LEDC_TIMER_0
#define LEDC_HS_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_CHANNEL LEDC_CHANNEL_0

#define LEDC_TEST_DUTY (2)

bool ir_use = false;
size_t received = 0;

bool signal_data[MAX_SIGNAL_LEN];
int64_t signal_time[MAX_SIGNAL_LEN];
int64_t sent_time[MAX_SIGNAL_LEN];
int signal_pointer = 0;
bool flag_received = false;
bool flag_record = false;
int64_t time_prev = 0;

static void IRAM_ATTR gpio_isr_handler(void *arg) {
  int64_t time_now;
  time_now = esp_timer_get_time();
  if (flag_record == true && flag_received == false) {
    signal_pointer = 0;
    flag_received = true;
    flag_record = false;
    time_prev = time_now;
  }
  signal_time[signal_pointer] = time_now - time_prev;
  signal_data[signal_pointer] = 1 - gpio_get_level(GPIO_INPUT_IR);
  signal_pointer++;
  time_prev = time_now;
  // ets_printf(".");
}

static void print_buffer() {
  int64_t time_sum = 0;
  for (int i = 0; i < signal_pointer; i++) {
    printf(" %d\t%d\t%d\t%d\n", (int)signal_time[i], (int)sent_time[i],
           (int)(sent_time[i] - signal_time[i]), (int)signal_data[i]);
    time_sum += signal_time[i];
  }
  printf("length=%d\n", signal_pointer);
  printf("period=%d[us]", (int)time_sum);
  printf("\n");
}

void init_ir_sensor() {
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);

  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = 1;
  gpio_config(&io_conf);

  gpio_set_level(GPIO_OUTPUT_IR_POWER, 1);

  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(GPIO_INPUT_IR, gpio_isr_handler, (void *)GPIO_INPUT_IR);
}

static ledc_timer_config_t ledc_timer = {
    .duty_resolution = LEDC_TIMER_1_BIT, // resolution of PWM duty
    .freq_hz = 38000,                    // frequency of PWM signal
    .speed_mode = LEDC_HS_MODE,          // timer mode
    .timer_num = LEDC_HS_TIMER           // timer index
};

static ledc_channel_config_t ledc_channel = {.channel = LEDC_HS_CH0_CHANNEL,
                                             .duty = 0,
                                             .gpio_num = GPIO_OUTPUT_IR,
                                             .speed_mode = LEDC_HS_MODE,
                                             .timer_sel = LEDC_HS_TIMER};

void init_ledc() {
  ledc_timer_config(&ledc_timer);
  ledc_channel_config(&ledc_channel);
}

static void send_buffer() {
  int64_t time_now;
  for (int i = 0; i < signal_pointer; i++) {
    if (i != 0) {
      ets_delay_us((int)signal_time[i] - 5);
    }
    time_now = esp_timer_get_time();
    if (i == 0) {
      time_prev = time_now;
    }
    sent_time[i] = time_now - time_prev;
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel,
                  (int)signal_data[i]);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
    time_prev = time_now;
  }
  printf("sent\n");
}

void app_main() {
  init_ir_sensor();
  init_ledc();

  while (1) {

    flag_record = true;
    printf("receive wait ir sensor\n");
    gpio_set_level(GPIO_OUTPUT_LED, 1);
    gpio_intr_enable(GPIO_INPUT_IR);
    vTaskDelay(2000 / portTICK_RATE_MS);
    gpio_intr_disable(GPIO_INPUT_IR);
    gpio_set_level(GPIO_OUTPUT_LED, 0);
    flag_received = false;

    vTaskDelay(500 / portTICK_RATE_MS);
    print_buffer();
    send_buffer();
    vTaskDelay(500 / portTICK_RATE_MS);
  }
}
