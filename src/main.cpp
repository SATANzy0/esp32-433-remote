/*
 * ESP32-D0WD 433MHz ASK/OOK 抓码 + 重放 (PlatformIO 版, IDF4.x ringbuffer API)
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
#include "freertos/ringbuf.h"

#define RX_PIN       16
#define TX_PIN       17
#define RMT_RX_CH    RMT_CHANNEL_1
#define RMT_TX_CH    RMT_CHANNEL_0
#define MAX_ITEMS    256
#define REPEAT_TX    8

static RingbufHandle_t rx_rb = NULL;

// 最近一次抓到的波形(us), 供重放
static uint16_t lastUs[MAX_ITEMS];
static uint16_t lastN = 0;

// 连续帧一致性: 真实遥控连发多帧几乎相同, 需连续3帧逐脉冲匹配才打印
static uint16_t refUs[MAX_ITEMS];
static uint16_t refLen = 0;
static uint8_t repeatCount = 0;
static bool monitor = false;   // 诊断模式: 打印所有通过基础校验的帧(不过滤重复)

void setup() {
  Serial.begin(115200);
  delay(200);

  // ---- RMT 接收 (SYN480R) ----
  rmt_config_t rx_cfg = {};
  rx_cfg.rmt_mode = RMT_MODE_RX;
  rx_cfg.channel = RMT_RX_CH;
  rx_cfg.gpio_num = (gpio_num_t)RX_PIN;
  rx_cfg.clk_div = 80;                       // 80MHz/80 = 1MHz -> 1 tick = 1us
  rx_cfg.mem_block_num = 2;
  rx_cfg.rx_config.filter_en = true;         // 过滤掉短噪声脉冲
  rx_cfg.rx_config.filter_ticks_thresh = 100; // <100us 的毛刺丢弃(SYN480底噪很强)
  rx_cfg.rx_config.idle_threshold = 2000;    // 2ms 空闲视为一帧结束
  ESP_ERROR_CHECK(rmt_config(&rx_cfg));
  ESP_ERROR_CHECK(rmt_driver_install(RMT_RX_CH, 4096, 0));  // 开 ring buffer
  ESP_ERROR_CHECK(rmt_get_ringbuf_handle(RMT_RX_CH, &rx_rb));
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
  static rmt_item32_t items[MAX_ITEMS];
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

// 判断是否为"看起来像真实遥控"的帧, 用于过滤 SYN480 空闲底噪
static bool isValidFrame(uint16_t* tim, uint16_t n) {
  if (n < 16 || n > 200) return false;          // 真实遥控一般 24~50 个电平
  uint32_t total = 0;
  uint16_t shortCnt = 0;
  for (uint16_t i = 0; i < n; i++) {
    total += tim[i];
    if (tim[i] < 150) shortCnt++;               // 统计过短(<150us)的脉冲
  }
  if (total < 3000 || total > 60000) return false;   // 帧时长 3ms~60ms
  if (shortCnt > n / 3) return false;           // 太多短脉冲 => 噪声
  return true;
}

// 逐脉冲相似: 两帧电平数接近且每个脉冲宽度几乎一致, 才认为是同一码的重复发射
// (噪声每帧脉冲随机, 逐脉冲几乎不可能匹配; 遥控重复帧脉冲宽度高度一致)
static bool pulseMatch(uint16_t* a, uint16_t n, uint16_t* b, uint16_t m) {
  if (abs((int)n - (int)m) > 1) return false;          // 电平数差太多 => 不同
  int c = n < m ? n : m;
  for (int i = 0; i < c; i++) {
    uint32_t d = a[i] > b[i] ? a[i] - b[i] : b[i] - a[i];
    if (d > a[i] / 4 + 40) return false;               // 单脉冲差 >25%+40us => 不同
  }
  return true;
}

void loop() {
  // 从 ring buffer 取一帧接收数据
  size_t rx_size = 0;
  rmt_item32_t* item = (rmt_item32_t*)xRingbufferReceive(rx_rb, &rx_size, pdMS_TO_TICKS(5));
  if (item) {
    if (rx_size >= sizeof(rmt_item32_t)) {
      uint16_t n = rx_size / sizeof(rmt_item32_t);
      lastN = toUs(item, n, lastUs);          // 缓存供重放
      if (lastN > 4) {
        if (monitor) {
          // 诊断模式: 打印所有原始帧(完全不过滤), 确认 SYN480R 到底收到什么
          Serial.print("ALL:[");
          for (uint16_t i = 0; i < lastN; i++) {
            Serial.print(lastUs[i]);
            if (i != lastN - 1) Serial.print(",");
          }
          Serial.println("]");
        } else if (isValidFrame(lastUs, lastN)) {
          // 连续3帧逐脉冲一致 => 真实遥控连发, 才打印; 随机噪声不匹配则跳过
          if (pulseMatch(lastUs, lastN, refUs, refLen)) {
            repeatCount++;
          } else {
            repeatCount = 1;
            memcpy(refUs, lastUs, lastN * sizeof(uint16_t));
            refLen = lastN;
          }
          if (repeatCount >= 3) {
            Serial.print("raw:[");
            for (uint16_t i = 0; i < lastN; i++) {
              Serial.print(lastUs[i]);
              if (i != lastN - 1) Serial.print(",");
            }
            Serial.println("]");
            Serial.printf(">> 共 %d 个电平, 输入 1 重放\n", lastN);
            repeatCount = 0;   // 避免刷屏, 重新计数
          }
        }
      }
    }
    vRingbufferReturnItem(rx_rb, item);
  }

  // 串口命令
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
    } else if (c == 't') {
      // 环回自测: SYN115F 发射一段 433 测试波形, SYN480R 应收到并打印 raw
      Serial.println("环回测试: SYN115F 发射 433 测试波形 x8...");
      static const uint16_t tw[40] = {
        300,900,300,900,300,900,300,900,300,900,
        300,900,300,900,300,900,300,900,300,900,
        300,900,300,900,300,900,300,900,300,900,
        300,900,300,900,300,900,300,900,300,900
      };
      sendUs((uint16_t*)tw, 40);
      Serial.println("发射完成, 若 SYN480R 收到会在上面打印 raw");
    } else if (c == 'm') {
      monitor = !monitor;
      Serial.printf("诊断模式 %s (打印所有基础校验通过的帧)\n", monitor ? "ON" : "OFF");
    }
  }

  delay(2);
}
