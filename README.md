# nisejjy - Enhanced Radio Station Emulator

支持 ESP32 和 ESP32-C3 的增强版无线电信号发送器，具有 Web 配置界面、WiFi 网络配置和多时间表功能。

## 功能特性

### 1. 硬件支持
- ✅ **ESP32** (使用 GPIO 26, 27, 25)
- ✅ **ESP32-C3** (使用 GPIO 3, 4, 5)
- 自动检测芯片型号并配置相应的引脚

### 2. WiFi 配置
- 🌐 **Web 配置页面** - 通过浏览器配置 WiFi
- 📡 **自动进入配网模式**：
  - 首次启动时无配置自动进入 AP 模式
  - WiFi 连接失败后自动进入 AP 模式（30 秒超时）
- 🔧 **后台设备名称** - 使用 MAC 地址生成唯一设备名

### 3. 电波功能
支持 7 种电波标准：
- JJY-E (40 KHz) - 日本福岛
- JJY-W (60 KHz) - 日本福冈
- WWVB (60 KHz) - 美国
- DCF77 (77.5 KHz) - 德国
- BSF (77.5 KHz) - 台湾
- MSF (60 KHz) - 英国
- BPC (68.5 KHz) - 中国

### 4. 时间计划
- ⏰ **24 小时时间轴** - 在时间轴上拖拽设置电波发送时段
- 📊 **多电波选择** - 同时支持多个电波时段配置
- ⏱️ **分钟级精度** - 按分钟精确控制发送时间

### 5. 蓝牙功能
- 🕐 **实时时间显示** - 蓝牙名称显示格式：`RadioStation-HHmmss`
- 每秒自动更新蓝牙名称中的时间
- ✅ **移除蓝牙串口功能** - 专注时间同步

## 硬件连接

### ESP32
```
GPIO 26 → 无线电输出（可连接 30cm 环形天线）
GPIO 27 → 蜂鸣器
GPIO 25 → LED 指示灯
```

### ESP32-C3
```
GPIO 3 → 无线电输出
GPIO 4 → 蜂鸣器
GPIO 5 → LED 指示灯
```

## 安装步骤

### 1. 安装必需库
在 Arduino IDE 中安装以下库：
- WiFi (内置)
- WebServer (内置)
- SPIFFS (内置)
- ArduinoJson (库管理器搜索安装)
- BluetoothSerial (内置)

### 2. 编译设置
**对于 ESP32：**
- 开发板：ESP32 Dev Module
- 闪存大小：4MB
- SPIFFS 大小：1.5MB

**对于 ESP32-C3：**
- 开发板：ESP32-C3 Dev Module
- 闪存大小：4MB
- SPIFFS 大小：1.5MB

### 3. 上传代码
1. 连接 ESP32/C3 到电脑
2. 在 Arduino IDE 中打开 `sketch_dec12a.ino`
3. 选择正确的开发板和端口
4. 点击上传

## 使用方法

### 首次配置
1. 设备启动时会自动进入 AP 模式
2. 用手机/电脑搜索 WiFi，连接到 `RadioStation_XXXX` (XXXX 为 MAC 地址后四位)
3. 打开浏览器访问 `http://192.168.4.1`
4. 输入你的 WiFi SSID 和密码，点击保存
5. 设备会连接到你的网络并启动 NTP 时间同步

### Web 界面功能

#### WiFi 配置
- 输入 SSID（网络名称）
- 输入密码
- 保存配置

#### 电波选择
- 显示所有可用的电波标准
- 查看每种电波的频率

#### 时间计划
- **添加计划**：点击"Add Schedule"添加新的时间段
- **设置时间范围**：使用时间滑块或输入框设置 开始时间 和 结束时间
- **选择电波**：从下拉菜单选择该时段使用的电波
- **保存**：点击"Save Schedules"保存所有计划

### 时间计划示例

```
00:00 - 06:00  → JJY-E (夜间)
06:00 - 18:00  → JJY-W (白天)
18:00 - 24:00  → DCF77 (测试)
```

## API 接口

### 获取配置
```
GET /api/config
响应: {"ssid":"network_name"}
```

### 保存 WiFi 配置
```
POST /api/config
参数: ssid, password
```

### 获取可用电波列表
```
GET /api/stations
响应: [{"id":0,"name":"JJY-E (40KHz)"}, ...]
```

### 获取当前时间计划
```
GET /api/schedules
响应: [{"station":0,"start":0,"end":1439}, ...]
```

### 保存时间计划
```
POST /api/schedules
参数: 时间计划 JSON 数组
```

### 获取实时状态
```
GET /api/status
响应: {"time":"12:34:56","station":"JJY-E (40KHz)"}
```

## 配置文件

设备会自动在 SPIFFS 中创建以下配置文件：

### `/config.json`
```
SSID
PASSWORD
```

### `/stations.json`
```json
[
  {"station":0,"start":0,"end":360},
  {"station":1,"start":360,"end":1080},
  {"station":3,"start":1080,"end":1439}
]
```

## 蓝牙连接

设备完全支持蓝牙连接，设备名称会定时更新显示当前时间：
- 格式：`RadioStation-HHmmss`
- 更新频率：每秒

## 故障排除

### 无法进入 AP 模式
- 检查 SPIFFS 是否正确挂载
- 查看串口监视器的日志信息

### Web 界面无法访问
- 确认已连接到 `RadioStation_XXXX` WiFi
- 尝试在浏览器中访问 `http://192.168.4.1`
- 检查防火墙设置

### NTP 时间同步失败
- 确保 WiFi 连接正常
- 检查 DNS 服务器设置
- 查看串口输出的错误信息

### 电波输出不稳定
- 检查天线连接
- 调整天线长度（推荐 30cm）
- 检查 GPIO 引脚连接

## 串口监视器输出示例

```
started...
SPIFFS Mounted Successfully
Loaded config: SSID=MyNetwork
Loaded 3 schedules
Bluetooth MAC 34:85:18:XX:XX:XX...
AP Mode started. SSID: RadioStation_XXXX, IP: 192.168.4.1
Web server started
BT Name: RadioStation-120000
station #0:
  freq 40.000000MHz, timer intr: 80M / (80 x 1), buzz/radio: /80
...
```

## 编码约定

- 所有时间以分钟为单位（0-1439 表示一天）
- 电波编号 0-6 对应各个电波标准
- GPIO 引脚通过宏定义在编译时自动检测

## 许可证

基于原始 nisejjy 项目进行增强

## 支持的浏览器

- Chrome/Chromium 90+
- Safari 14+
- Firefox 88+
- Edge 90+

---

**最后更新**: 2024 年 12 月
