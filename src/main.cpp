/*
 * ESP32-D0WD 433MHz ASK/OOK 抓码 + 重放 (PlatformIO 版)
 * 硬件:
 *   SYN480R DATA -> GPIO16 (接收)
 *   SYN115F DATA -> GPIO17 (发射, VCC 必须 3.3V!)
 *   收发模块与 ESP32 共地; 天线 17.3cm
 * 用法(串口 115200):
 *   按遥控器 -> 打印 raw 时序数组
 *   输入 '1'    -> 重放上次抓到的波形(重复 8 次)
 *   输入 'r'    -> 清空缓存
 */
#include <Arduino.h>
#include "driver/rmt.h"

#define RX_PIN       16
#define TX_PIN       17
#define RMT_RX_CH    RMT_CHANNEL_1
#define RMT_TX_CH    RMT_CHANNEL_0
#define MAX_ITEMS    256
#define REPEAT_TX    8

static rmt_item32_t rx_items[MAX_ITEMS];
static uint16_t rx_len = 0;
static volatile bool got_data = false;
static xSemaphoreHandle rx_sem = NULL;
static intr_handle_t rmt_intr_handle = NULL;

// 最近一次抓到的波形(us), 供重放
static uint16_t lastUs[MAX_ITEMS];
static uint16_t lastN = 0;

// 接收完成中断
void IRAM_ATTR rmt_rx_isr(void* arg) {
  portBASE_TYPE taskAwoken = pdFALSE;
  rmt_rx_start(RMT_RX_CH, false);          // 停接收
  RMT.int_st.val = 0;                       // 清中断标志
  if (rmt_get_items(RMT_RX_CH, rx_items, MAX_ITEMS, &rx_len) == ESP_OK) {
    got_data = true;
  }
  rmt_rx_start(RMT_RX_CH, true);           // 重新开启接收
  xSemaphoreGiveFromISR(rx_sem, &taskAwoken);
  if (taskAwoken == pdTRUE) portYIELD_FROM_ISR();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  rx_sem = xSemaphoreCreateBinary();

  // ---- RMT 接收 (SYN480R) ----
  rmt_config_t rx_cfg = {};
  rx_cfg.rmt_mode = RMT_MODE_RX;
  rx_cfg.channel = RMT_RX_CH;
  rx_cfg.gpio_num = (gpio_num_t)RX_PIN;
  rx_cfg.clk_div = 80;                       // 80MHz/80 = 1MHz -> 1 tick = 1us
  rx_cfg.mem_block_num = 2;
  rx_cfg.rx_config.filter_en = true;         // 过滤掉短噪声脉冲
  rx_cfg.rx_config.filter_ticks_thresh = 30; // <30us 的毛刺丢弃
  rx_cfg.rx_config.idle_threshold = 2000;    // 2ms 空闲视为一帧结束
  ESP_ERROR_CHECK(rmt_config(&rx_cfg));
  ESP_ERROR_CHECK(rmt_driver_install(RMT_RX_CH, 4096, 0));
  ESP_ERROR_CHECK(rmt_set_rx_intr_en(RMT_RX_CH, true));
  ESP_ERROR_CHECK(rmt_isr_register(rmt_rx_isr, NULL, 0, &rmt_intr_handle));
  rmt_rx_start(RMT_RX_CH, true);

  // ---- RMT 发送 (SYN115F) ----
  // 关键: SYN115F 自己产生 433MHz 载波, 这里必须关闭 RMT 载波, 只输出 OOK 高低电平
  rmt_config_t tx_cfg = {};
  tx_cfg.rmt_mode = RMT_MODE_TX;
  tx_cfg.channel = RMT_TX_CH;
  tx_cfg.gpio_num = (gpio_num_t)TX_PIN;
  tx_cfg.clk_div = 80;                       // 1 tick = 1us
  tx_cfg.mem_block_num = 1;
  tx_cfg.tx_config.carrier_en = false;       // 关闭载波!
  tx_cfg.tx_config.loop_en = false;
  tx_cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;   // 空闲输出低电平(不发射)
  tx_cfg.tx_config.idle_output_en = true;
  ESP_ERROR_CHECK(rmt_config(&tx_cfg));
  ESP_ERROR_CHECK(rmt_driver_install(RMT_TX_CH, 0, 0));

  Serial.println();
  Serial.println("=== ESP32-D0WD 433 RAW 抓码器 ready ===");
  Serial.println("按遥控器 -> 打印 raw 时序");
  Serial.println("串口输入 1 -> 重放上次波形 (x8)");
  Serial.println("串口输入 r -> 清空缓存");
}

// rmt_item32_t 转成 us 数组(电平交替), 返回个数
static uint16_t toUs(rmt_item32_t* items, uint16_t len, uint16_t* out) {
  uint16_t n = 0;
  for (uint16_t i = 0; i < len; i++) {
    if (items[i].duration0) out[n++] = items[i].duration0;
    if (items[i].duration1) out[n++] = items[i].duration1;
  }
  return n;
}

// 用 us 数组发送(电平交替, 偶数下标为高)
static void sendUs(uint16_t* tim, uint16_t n) {
  if (n == 0 || n > MAX_ITEMS) return;
  rmt_item32_t items[n];
  for (uint16_t i = 0; i < n; i++) {
    items[i].duration0 = tim[i];
    items[i].level0 = (i % 2 == 0) ? 1 : 0;  // 交替高低
    items[i].duration1 = 0;
    items[i].level1 = 0;
  }
  for (int r = 0; r < REPEAT_TX; r++) {
    rmt_write_items(RMT_TX_CH, items, n, true);  // 阻塞发送
    delay(10);
  }
}

void loop() {
  if (got_data) {
    got_data = false;
    if (rx_len > 4) {
      lastN = toUs(rx_items, rx_len, lastUs);  // 缓存供重放
      if (lastN > 4) {
        Serial.print("raw:[");
        for (uint16_t i = 0; i < lastN; i++) {
          Serial.print(lastUs[i]);
          if (i != lastN - 1) Serial.print(",");
        }
        Serial.println("]");
        Serial.printf(">> 共 %d 个电平, 输入 1 重放\n", lastN);
      }
    }
    rx_len = 0;
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') {
      if (lastN == 0) {
        Serial.println("缓存为空, 先按一下遥控器抓码");
      } else {
        Serial.printf("重放 %d 个电平 x%d ...\n", lastN, REPEAT_TX);
        sendUs(lastUs, lastN);
        Serial.println("发送完成");
      }
    } else if (c == 'r') {
      lastN = 0;
      Serial.println("缓存已清空");
    }
  }

  delay(5);
}
