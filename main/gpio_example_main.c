#include "driver/gpio.h"
#include "driver/rmt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GPIO_OUTPUT_LED 2
#define GPIO_OUTPUT_IR 14
#define GPIO_OUTPUT_IR_POWER 13
#define GPIO_OUTPUT_PIN_SEL                                                    \
  ((1ULL << GPIO_OUTPUT_LED) | (1ULL << GPIO_OUTPUT_IR) |                      \
   (1ULL << GPIO_OUTPUT_IR_POWER))
#define GPIO_INPUT_IR 12
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_INPUT_IR)

#define RMT_TX_CHANNEL RMT_CHANNEL_4
#define RMT_TX_GPIO_NUM GPIO_NUM_14
#define RMT_RX_CHANNEL RMT_CHANNEL_0
#define RMT_RX_GPIO_NUM GPIO_NUM_12
#define RMT_CLK_DIV 100
#define RMT_TICK_10_US (80000000 / RMT_CLK_DIV / 100000)
#define rmt_item32_TIMEOUT_US 10000

#define MAX_SIGNAL_LEN 1024*16

bool ir_use = false;
size_t received = 0;
rmt_item32_t signals[MAX_SIGNAL_LEN];

void print_signals(){
  for (int i = 0; i < received; ++i) {
    printf("%d ", signals[i].level0);
    printf("%d ", signals[i].level1);
  }
  printf("\n");
}

void rmt_tx_task(void *dummy) {
  printf("Tx\n");
  if (received > 0) {
    rmt_write_items(RMT_TX_CHANNEL, signals, received, true);
    rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);
    printf("  done\n");
  } else {
    printf("  no data sent\n");
  }
  ir_use = false;
  // vTaskDelete(NULL);
  print_signals();
}

void rmt_rx_task(void *dummy) {
  printf("Rx\n");
  RingbufHandle_t rb = NULL;
  rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rb);
  rmt_rx_start(RMT_RX_CHANNEL, false);

  size_t rx_size = 0;
  printf("  wait ir signal...\n");
  rmt_item32_t *item = (rmt_item32_t *)xRingbufferReceive(rb, &rx_size, 5000);
  rmt_rx_stop(RMT_RX_CHANNEL);
  if (!item) {
    printf("  no data received\n");
    ir_use = false;
    // vTaskDelete(NULL);
    return;
  }
  printf("  received length: %d\n", rx_size);

  memcpy(signals, item, sizeof(rmt_item32_t) * rx_size);
  for (int i = 0; i < rx_size; ++i) {
    signals[i].level0 = ~signals[i].level0;
    signals[i].level1 = ~signals[i].level1;
  }
  received = rx_size;
  vRingbufferReturnItem(rb, (void *)item);

  printf("  recv done\n");

  rmt_rx_stop(RMT_RX_CHANNEL);
  print_signals();
  ir_use = false;
  // vTaskDelete(NULL);
}

void init_tx() {
  rmt_config_t rmt_tx;
  rmt_tx.rmt_mode = RMT_MODE_TX;
  rmt_tx.channel = RMT_TX_CHANNEL;
  rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
  rmt_tx.mem_block_num = 4;
  rmt_tx.clk_div = RMT_CLK_DIV;
  rmt_tx.tx_config.loop_en = false;
  rmt_tx.tx_config.carrier_duty_percent = 50;
  rmt_tx.tx_config.carrier_freq_hz = 38000;
  rmt_tx.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  rmt_tx.tx_config.carrier_en = 1;
  rmt_tx.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  rmt_tx.tx_config.idle_output_en = true;
  rmt_config(&rmt_tx);
  rmt_driver_install(rmt_tx.channel, 0, 0);
}

void init_rx() {
  rmt_config_t rmt_rx;
  rmt_rx.rmt_mode = RMT_MODE_RX;
  rmt_rx.channel = RMT_RX_CHANNEL;
  rmt_rx.clk_div = RMT_CLK_DIV;
  rmt_rx.gpio_num = RMT_RX_GPIO_NUM;
  rmt_rx.mem_block_num = 4;
  rmt_rx.rx_config.filter_en = true;
  rmt_rx.rx_config.filter_ticks_thresh = 100;
  rmt_rx.rx_config.idle_threshold =
      rmt_item32_TIMEOUT_US / 10 * (RMT_TICK_10_US);
  rmt_config(&rmt_rx);
  rmt_driver_install(rmt_rx.channel, 1000, 0);
}

void init_ir_sensor() {
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
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
}

void app_main() {
  init_ir_sensor();
  init_tx();
  init_rx();

  int cnt = 0;
  int dummy;

  ir_use = true;
  // xTaskCreate(rmt_rx_task, "rmt_rx_task", 2048, NULL, 10, NULL);
  rmt_rx_task(&dummy);

  while (1) {

    ir_use = true;
    // xTaskCreate(rmt_tx_task, "rmt_tx_task", 2048, NULL, 10, NULL);
    rmt_tx_task(&dummy);

    gpio_set_level(GPIO_OUTPUT_LED, cnt % 2);
    vTaskDelay(50 / portTICK_RATE_MS);
    gpio_set_level(GPIO_OUTPUT_LED, cnt % 2);
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}
