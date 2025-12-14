# RadioClock - Radio Signal Clock Generator
**[English](README.md)|[中文](README_ZH.md)**

A radio signal clock transmitter supporting ESP32 and ESP32-C3, featuring a web configuration interface, WiFi setup, and multi-time, multi-protocol scheduling.
<img width="2468" height="1428" alt="image" src="https://github.com/user-attachments/assets/548f9941-76ea-40ca-8b33-df68f4d291de" />


## Features

### 1. Hardware Support

* ✅ **ESP32** (GPIO 26, 27, 25)
* ✅ **ESP32-C3** (GPIO 3, 4, 5)
* Automatically detects chip model and configures corresponding pins

### 2. WiFi Configuration

* 🌐 **Web Configuration Page** – Configure WiFi through a browser
* 📡 **Automatic Configuration Mode**:

  * Automatically enters AP mode if no configuration exists on first startup
  * Default AP password: `12345678`
  * Automatically enters AP mode if WiFi connection fails (30-second timeout)
* 🔧 **Device Name** – Unique name generated using MAC address

### 3. Radio Signal Support

Supports 7 radio standards:

* JJY-E (40 KHz) – Fukushima, Japan
* JJY-W (60 KHz) – Fukuoka, Japan
* WWVB (60 KHz) – USA
* DCF77 (77.5 KHz) – Germany
* BSF (77.5 KHz) – Taiwan
* MSF (60 KHz) – UK
* BPC (68.5 KHz) – China

### 4. Scheduling

* ⏰ **Multi-radio Schedules** – Set different radio transmission periods as needed

## Hardware Connections

### ESP32

```
GPIO 26 → PWM simulated RF signal output
GPIO 27 → Buzzer
GPIO 25 → LED indicator (current encoding)
```

### ESP32-C3

```
GPIO 3 → PWM simulated RF signal output
GPIO 4 → Buzzer
GPIO 5 → LED indicator (current encoding)
```

## Installation

### 1. Required Libraries

Install the following in Arduino IDE:

* WiFi (built-in)
* WebServer (built-in)
* SPIFFS (built-in)
* ArduinoJson (install via Library Manager)
* BluetoothSerial (built-in)

### 2. Board Settings

**For ESP32:**

* Board: ESP32 Dev Module
* Flash Size: 4MB
* SPIFFS Size: 1.5MB

**For ESP32-C3:**

* Board: ESP32-C3 Dev Module
* Flash Size: 4MB
* SPIFFS Size: 1.5MB

### 3. Upload Code

1. Connect ESP32/C3 to PC
2. Open code in Arduino IDE
3. Select correct board and port
4. Click Upload

## Usage

### Initial Setup

1. Device starts in AP mode automatically
2. Connect your phone/PC to WiFi `RadioStation_XXXX` (XXXX = last 4 digits of MAC)
3. Open browser at `http://192.168.4.1`
4. Enter your WiFi SSID and password, click Save
5. Device connects to network and starts NTP time sync
6. Later changes can be made by visiting the device IP

### Web Interface Features

#### WiFi Configuration

* Reset SSID
* Reset password
* Modify time zone

#### Radio Selection

* View all supported radio standards
* See frequency for each radio

#### Scheduling

* **Add Schedule** – Click "Add Schedule" to create a new period
<img width="2468" height="1428" alt="image" src="https://github.com/user-attachments/assets/1a7a58f6-e8e6-41ca-9f20-6754986ca1c2" />
<img width="1893" height="939" alt="image" src="https://github.com/user-attachments/assets/a226a92e-564a-4efa-9dbd-479df388a1c3" />

* **Set Time Range** – Enter Start and End time
<img width="1611" height="481" alt="image" src="https://github.com/user-attachments/assets/9d7059b2-c5fc-4b56-90dd-fc0715419925" />


* **Select Radio** – Choose the radio standard for the period
<img width="1878" height="908" alt="image" src="https://github.com/user-attachments/assets/659a54ce-d6a7-4952-b343-9133e6c337d0" />


* **Delete** – Click "Delete" to remove a schedule
<img width="1658" height="293" alt="image" src="https://github.com/user-attachments/assets/aaf16a7a-f7ed-454f-ab49-027b9483d1c3" />


## API Endpoints

### Get Configuration

```
GET /api/config
Response: {"ssid":"network_name"}
```

### Save WiFi Configuration

```
POST /api/config
Parameters: ssid, password
```

### Get Available Radio Stations

```
GET /api/stations
Response: [{"id":0,"name":"JJY-E (40KHz)"}, ...]
```

### Get Current Schedule

```
GET /api/schedules
Response: [{"station":0,"start":0,"end":1439}, ...]
```

### Save Schedule

```
POST /api/schedules
Parameters: JSON array of schedules
```

### Get Real-time Status

```
GET /api/status
Response: {"time":"12:34:56","station":"JJY-E (40KHz)"}
```

## Configuration Files

The device automatically creates the following files in SPIFFS:

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

## Troubleshooting

### Cannot Enter AP Mode

* Check SPIFFS mounting
* Review serial monitor logs

### Web Interface Not Accessible

* Confirm connection to `RadioStation_XXXX` WiFi
* Visit `http://192.168.4.1`
* Check firewall settings

### NTP Time Sync Fails

* Ensure WiFi connection is active
* Check DNS settings
* Review serial monitor errors

### Unstable Radio Output

* Check antenna connection
* Adjust antenna length
* Check GPIO wiring

## Serial Monitor Output Example

```
freq 68.500000kHz, timer intr: 80M / (583 x 1), buzz/radio: /137
bits60 pattern: 400000000000000000004100000000000000000042000000000000000000
second pattern: 0111111111...
encode BPC format - 
400102230030323012104101022300303230121042010223003032301210
(re)started timer...
.radio started.
...~~~~~~2025-12-14, 347(0) 16:43:52 (sched=1/1)
....~~~~~~2025-12-14, 347(0) 16:43:54 (sched=1/1)
....~~~~~~2025-12-14, 347(0) 16:43:56 (sched=1/1)
..~~~~~~~~2025-12-14, 347(0) 16:43:58 (sched=1/1)
```

## Coding Conventions

* Time is expressed in minutes (0–1439 for a full day)
* Radio IDs 0–6 correspond to the supported radio standards
* GPIO pins are automatically defined via macros at compile time

## License

Enhanced version based on the original nisejjy project:
[https://github.com/tarohs/nisejjy](https://github.com/tarohs/nisejjy)

## Supported Browsers

* Chrome/Chromium 90+
* Safari 14+
* Firefox 88+
* Edge 90+

---

**Copyright**: 5Breeze

For redistribution, adaptation, or commercial use, please credit the source.

---

If you want, I can also create a **compact README.md version in English** ready for GitHub. Do you want me to do that?
