# ESP32-D0WD 433MHz 抓码 + 重放

经典 ESP32（ESP32-D0WD）433M ASK/OOK 抓取 raw 时序 + 重放。

- 接收：SYN480R DATA → GPIO16
- 发射：SYN115F DATA → GPIO17（VCC 严格 3.3V！）
- 收发模块与 ESP32 共地；天线 17.3cm

## 串口用法（115200）
- 按遥控器 → 打印 raw 时序数组
- 输入 `1` → 重放上次波形（重复 8 次）
- 输入 `r` → 清空缓存

## 云端编译（GitHub Actions）
代码推送到本仓库 main 分支后，Actions 会自动用 PlatformIO 云端编译，
并把合并好的 `merged.bin`（烧录地址 0x0）作为 artifact 上传，下载即可。

## 本地烧录
```bash
esptool.py --chip esp32 --port COMx --baud 460800 write_flash -z 0x0 merged.bin
```
