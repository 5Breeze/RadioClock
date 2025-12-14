# Arduino IDE 配置指南

## 环境要求

- Arduino IDE 1.8.13 或更高版本
- 或 Arduino IDE 2.0+（推荐）

## 安装 ESP32 开发板支持

### 方法 1：使用 Arduino IDE 板管理器（推荐）

1. 打开 Arduino IDE
2. 前往 **文件** → **首选项** (或 **Arduino IDE** → **设置**)
3. 在"附加开发板管理器网址"字段中添加以下 URL：
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. 点击确定
5. 前往 **工具** → **开发板** → **开发板管理器**
6. 搜索 "esp32" 
7. 安装 "ESP32 by Espressif Systems"（选择最新版本 2.0 或更高）
8. 等待安装完成

### 方法 2：手动安装（备选）

如果上述方法不工作，可以从以下位置手动下载：
https://github.com/espressif/arduino-esp32

## 安装必需的库

### 通过库管理器安装

1. 打开 Arduino IDE
2. 前往 **工具** → **库管理器**（或按 Ctrl+Shift+I）
3. 搜索并安装以下库：

#### 必需库：
- **ArduinoJson** - DynamicJsonDocument (v6.18 或更高)
  - 搜索: "ArduinoJson"
  - 作者: Benoit Blanchon

### 内置库（无需额外安装）
- WiFi
- WebServer
- SPIFFS
- BluetoothSerial

## 开发板选择和配置

### 对于 ESP32（经典版）

1. 打开 Arduino IDE
2. 前往 **工具** → **开发板** → **ESP32 Arduino** → **ESP32 Dev Module**
3. 配置以下设置（工具菜单中）：
   - **闪存大小**: 4MB
   - **分区方案**: Default 4MB with spiffs
   - **PSRAM**: Disabled
   - **上传速度**: 921600
   - **CPU 频率**: 80MHz
   - **闪存频率**: 80MHz
   - **闪存模式**: QIO

### 对于 ESP32-C3

1. 打开 Arduino IDE
2. 前往 **工具** → **开发板** → **ESP32 Arduino** → **ESP32-C3 Dev Module**
3. 配置以下设置：
   - **闪存大小**: 4MB
   - **分区方案**: Default 4MB with spiffs
   - **上传速度**: 921600
   - **USB CDC on Boot**: Disabled

## 编译和上传

### 编译代码

1. 将 `sketch_dec12a.ino` 文件放在名为 `sketch_dec12a` 的文件夹中
2. 用 Arduino IDE 打开该文件
3. 前往 **草图** → **验证/编译** 或按 Ctrl+R

### 上传到设备

1. 连接 ESP32/C3 到电脑（使用 USB-C 或 Micro-USB 数据线）
2. 前往 **工具** → **端口**，选择正确的 COM 端口
3. 前往 **草图** → **上传** 或按 Ctrl+U
4. 等待上传完成，会看到消息：`Hard resetting via RTS pin...`

### 监视串口输出

1. 上传完成后，前往 **工具** → **串口监视器**（按 Ctrl+Shift+M）
2. 设置波特率为 **115200**
3. 会看到设备的启动日志

## 常见问题解决

### 问题 1：找不到开发板
**解决方案：**
- 确保已通过板管理器安装了 ESP32 支持
- 重启 Arduino IDE
- 尝试清空 Arduino 缓存文件夹

### 问题 2：上传失败
**解决方案：**
- 检查 USB 线是否连接良好
- 尝试更换 USB 端口
- 对于 ESP32：按住 BOOT 按钮，然后点击 RST 按钮，释放 BOOT 按钮（进入下载模式）
- 对于 ESP32-C3：自动检测下载模式（如果失败，可尝试同上）

### 问题 3：编译错误 "WiFi.h not found"
**解决方案：**
- 确保已安装 ESP32 板支持
- 检查选择的开发板是否为 ESP32 而不是 Arduino
- 在"工具"→"开发板管理器"中重新安装 ESP32 支持

### 问题 4：SPIFFS 相关错误
**解决方案：**
- 确保分区方案包含 SPIFFS（选择 "Default 4MB with spiffs"）
- 重新上传代码并等待 SPIFFS 初始化

## 第一次启动

1. 上传代码后，打开串口监视器（波特率 115200）
2. 设备会输出：
   ```
   started...
   SPIFFS Mounted Successfully
   Loaded config: SSID=
   AP Mode started. SSID: RadioStation_XXXX, IP: 192.168.4.1
   Web server started
   ```
3. 用手机或电脑搜索 WiFi，找到 `RadioStation_XXXX` 并连接
4. 打开浏览器，访问 `http://192.168.4.1`
5. 输入你的 WiFi SSID 和密码，保存配置

## 调试技巧

### 启用详细日志

在 Arduino IDE 中：
1. 前往 **文件** → **首选项**
2. 找到"编译器警告"设置为"全部"
3. 勾选"显示详细输出"选项

### 使用 Serial.print() 调试

代码中已包含多个 Serial.print 调试点。在串口监视器中观察输出可以帮助诊断问题。

### 监视堆内存

使用以下代码片段监视可用内存：
```cpp
Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
```

## 性能优化

### 减少代码大小
- 使用 `-Os` 优化编译（Arduino IDE 默认启用）
- 删除不需要的功能

### 减少 RAM 使用
- 使用 PROGMEM 将常数放在 Flash 中
- 避免大型局部数组

### 提高稳定性
- 定期 reset watchdog timer（已在 loop() 中通过 delay(1) 完成）
- 避免长时间阻塞操作

## 参考资源

- [Espressif ESP32 官方文档](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [Arduino 官方文档](https://www.arduino.cc/en/Guide/Environment)
- [ESP32 Arduino 项目](https://github.com/espressif/arduino-esp32)

---

**最后更新**: 2024 年 12 月
