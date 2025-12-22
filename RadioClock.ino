//
// Standard Time Radio Broadcast Station
// Using software radio on ESP32/ESP32-C3.
//
// (c) 2021 by taroh (sasaki.taroh@gmail.com) (origin)
//
// (c) 2025 by 5Breeze (bitshen@qq.com)(improve)
// Modified 2025: Added web UI, WiFi config, multi-station scheduling, station rotation
//
// Supported Radio Time Services:
// - JJY (Japan): https://ja.wikipedia.org/wiki/JJY
// - WWVB (US): https://en.wikipedia.org/wiki/WWVB
// - DCF77 (Germany): https://www.eecis.udel.edu/~mills/ntp/dcf77.html
// - BSF (Taiwan): https://en.wikipedia.org/wiki/BSF_(time_service)
// - MSF (UK): https://en.wikipedia.org/wiki/Time_from_NPL_(MSF)
// - BPC (China): https://harmonyos.51cto.com/posts/1731
//
// NOTE: BSF and BPC codes are not officially certified.

//...................................................................
// Hardware Configuration - Auto-detect ESP32 variant
  #define PIN_RADIO  (0)  // Radio signal output pin
  #define PIN_BUZZ   (6)  // Buzzer output pin
  #define PIN_LED    (7)  // LED indicator pin

// Library Includes
#include <WiFi.h>           // WiFi connectivity
#include <WebServer.h>      // HTTP web server
#include <SPIFFS.h>         // File system for configuration storage
#include <ArduinoJson.h>    // JSON parsing and generation

// Global Objects
WebServer server(80);     // Web server on port 80

// Configuration Constants
#define DEVICENAME_PREFIX "RadioStation"     // Device name prefix for WiFi AP mode
#define DEFAULT_TZ (9 * 60 * 60)             // Default timezone: JST (UTC+9)
#define CONFIG_FILE "/config.json"           // WiFi and timezone configuration file
#define STATION_CONFIG_FILE "/stations.json" // Station configuration file

// WiFi Configuration Variables
char ssid[64] = "";                          // WiFi SSID (network name)
char passwd[64] = "";                        // WiFi password
long timezone_offset = DEFAULT_TZ;           // User-configurable timezone in seconds
unsigned long wifi_connect_start = 0;        // Timestamp when WiFi connection started
#define WIFI_CONNECT_TIMEOUT 30000           // WiFi connection timeout: 30 seconds
#define WIFI_CONNECT_CHECK_INTERVAL 5000     // WiFi connection check interval: 5 seconds

//...................................................................
// Radio Station Definitions and Specifications

// Station Index Constants
#define SN_JJY_E  (0) // JJY Fukushima, Japan (40 KHz)
#define SN_JJY_W  (1) // JJY Fukuoka, Japan (60 KHz)
#define SN_WWVB (2)   // WWVB, United States (60 KHz)
#define SN_DCF77  (3) // DCF77, Germany (77.5 KHz)
#define SN_BSF  (4)   // BSF, Taiwan (77.5 KHz)
#define SN_MSF  (5)   // MSF, United Kingdom (60 KHz)
#define SN_BPC  (6)   // BPC, China (68.5 KHz)

#define SN_DEFAULT  (SN_JJY_E) // Default station
#define NUM_STATIONS 7         // Total number of supported stations

// Station Display Names (Full)
const char *station_names[] = {
  "JJY-E (40KHz)",
  "JJY-W (60KHz)",
  "WWVB (60KHz)",
  "DCF77 (77.5KHz)",
  "BSF (77.5KHz)",
  "MSF (60KHz)",
  "BPC (68.5KHz)"
};

// Station Display Names (Short)
const char *station_short[] = {
  "JJY-E",
  "JJY-W",
  "WWVB",
  "DCF77",
  "BSF",
  "MSF",
  "BPC"
};

// Interrupt Cycle for each Station (in units of 1.25 MHz, double of station frequency)
// Used to generate the correct carrier frequency for each time service
int st_cycle2[] = {
  80,  // JJY-E:  40 KHz -> 80 MHz clock cycle
  120, // JJY-W:  60 KHz -> 120 MHz clock cycle
  120, // WWVB:   60 KHz -> 120 MHz clock cycle
  155, // DCF77:  77.5 KHz -> 155 MHz clock cycle
  155, // BSF:    77.5 KHz -> 155 MHz clock cycle
  120, // MSF:    60 KHz -> 120 MHz clock cycle
  137  // BPC:    68.5 KHz -> 137 MHz clock cycle
};

// Time Schedule Structure
// Defines when a specific radio station should be active
typedef struct {
  uint8_t station;      // Station index (0-6)
  uint16_t start_min;   // Start time in minutes from midnight (0-1439)
  uint16_t end_min;     // End time in minutes from midnight (0-1439)
} TimeSchedule;

// Schedule Storage
#define MAX_SCHEDULES 24                      // Maximum number of schedules
TimeSchedule schedules[MAX_SCHEDULES];        // Array of time schedules
int schedule_count = 0;                       // Current number of active schedules

// Multi-Station Rotation Control
// Used when multiple schedules overlap in time, causing the system to rotate between stations
#define ROTATION_INTERVAL_MINUTES 5           // Switch station every N minutes during rotation
int current_schedule_index = -1;              // Current selected schedule (-1 = none active)
unsigned long last_rotation_time = 0;         // Timestamp of last rotation event
int applicable_schedules[MAX_SCHEDULES];      // Array of currently applicable schedule indices
int applicable_count = 0;                     // Number of applicable schedules at current time
int last_station = -1;                        // Last selected station (prevents redundant switches)

//...................................................................
// Timer and Interrupt Configuration

// Hardware Timer Settings
int tm0cycle;   // Timer 0 cycle count for current station frequency
#define TM0RES    (1) // Timer 0 resolution (1 = 1.25 MHz)

// Radio and Buzzer Modulation Dividers
int radiodiv;   // Divider for radio carrier frequency generation

// Time Base Constants
// Timer interrupt rate depends on frequency: 1 KHz base frequency
// AMPDIV: Amplitude update frequency (10 Hz)
// SSECDIV: Second tick frequency (1 Hz)
#define AMPDIV   (100)  // 1 KHz / 100 => 10 Hz (amplitude update every 0.1 seconds)
#define SSECDIV   (10)  // 10 Hz / 10 => 1 Hz (clock tick every 1 second)

// Bit Symbol Definitions for Time Code Encoding
#define SP_0  (0)  // Binary 0 symbol
#define SP_1  (1)  // Binary 1 symbol
#define SP_M  (2)  // Minute marker symbol
#define SP_P0 (SP_1) // MSF parity 0 (= 1)
#define SP_P1 (3)    // MSF parity 1
#define SP_2  (2)    // BSF/BPC: symbol 2
#define SP_3  (3)    // BSF/BPC: symbol 3
#define SP_M4 (4)    // BSF/BPC: minute marker 4
#define SP_MAX  (SP_M4) // Maximum symbol value

// NOTE: Symbol patterns are defined in station-specific arrays:
// bits_STATION[] => bits60[]: 60-second symbol buffer with station-specific patterns
// sp_STATION[] => secpattern[]: 0.1-second pattern for each symbol (10 patterns per symbol)
// JJY & WWVB Time Code Patterns
// NOTE: Comments describe JJY format (WWVB uses similar structure)
int8_t bits_jjy[] = {  // 60-bit transmission frame: {SP_0, SP_1, SP_M} symbols
  SP_M, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M, // (M), Minutes[3], 0, Minutes[4], (M)
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M, // 0, 0, Hours[2], 0, Hours[4], (M)
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M, // 0, 0, Day_of_Year[2], Day_of_Year[4], (M)
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M, // Day_of_Year[4], 0, 0, Parity1, Parity2, 0, (M)
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M, // 0, Year[8], (M)
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M  // Day_of_Week[3], LeapSec1, LeapSec2, 0, 0, 0, 0, (M)
};

// NOTE: For summer time, set bits 57/58 (WWVB) or bits 38/40 (JJY in future)

// JJY Amplitude Modulation Pattern (SP_x defines which amplitude for each 0.1-second subframe)
int8_t sp_jjy[] = {
  1, 1, 1, 1, 1, 1, 1, 1, 0, 0,   // SP_0: 80% modulation (8 ticks high, 2 low)
  1, 1, 1, 1, 1, 0, 0, 0, 0, 0,   // SP_1: 50% modulation (5 ticks high, 5 low)
  1, 1, 0, 0, 0, 0, 0, 0, 0, 0    // SP_M: Minute marker 20% (2 ticks high, 8 low)
};

// WWVB Amplitude Modulation Pattern (inverted compared to JJY)
int8_t sp_wwvb[] = {
  0, 0, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_0: 80% modulation
  0, 0, 0, 0, 0, 1, 1, 1, 1, 1,   // SP_1: 50% modulation
  0, 0, 0, 0, 0, 0, 0, 0, 1, 1    // SP_M: Minute marker 20%
};

// DCF77 Time Code Patterns (Germany)
// NOTE: Encoding is LSB->MSB. Bit [0] changes based on DCF/HBG time zone
int8_t bits_dcf[] = {
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // 0, Reserved[9]
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_1, SP_0, // Reserved[5], 0, 0, Time_Zone, 0
  SP_1, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // 1, Minutes[4], Minutes[3], Parity1
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // Hours[4], Hours[2], Parity2, Day[4]
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // Day[2], Day_of_Week[3], Month[4], Month[1]
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M  // Year[4], Year[4], Parity3, (Minute Marker)
};

// DCF77 Amplitude Modulation Pattern
int8_t sp_dcf[] = {
  0, 1, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_0: 100ms on, 900ms off
  0, 0, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_1: 200ms on, 800ms off
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1    // SP_M: Full amplitude (1000ms on)
};

// BSF Time Code Patterns (Taiwan) - Quad encoding
int8_t bits_bsf[] = {
  SP_M4, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,
  SP_0,  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,
  SP_0,  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,
  SP_0,  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M4,
  SP_1,  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,  // 1, Minutes[3], Hours[2.5], Parity1[.5]
  SP_0,  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M4  // Day_of_Month[2.5], DOW[2.5], Month[2], Year[3.5], Parity2[.5]
};

// BSF Amplitude Modulation Patterns (5 patterns for quad encoding)
int8_t sp_bsf[] = {
  0, 0, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_0: Low
  0, 0, 0, 0, 1, 1, 1, 1, 1, 1,   // SP_1: Lower-mid
  0, 0, 0, 0, 0, 0, 0, 0, 1, 1,   // SP_2: Mid
  0, 0, 0, 0, 0, 0, 1, 1, 1, 1,   // SP_3: Upper-mid
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1    // SP_M4: Minute marker (full amplitude)
};

// MSF Time Code Patterns (UK) - 4 patterns
int8_t bits_msf[] = {
  SP_M, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,
  SP_1, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,
  SP_0, SP_0, SP_0, SP_P0, SP_P0, SP_P0, SP_P0, SP_P0, SP_P0, SP_0
};

// MSF Amplitude Modulation Patterns (4 patterns: SP_0, SP_1/SP_P0, SP_M, SP_P1)
int8_t sp_msf[] = {
  0, 1, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_0: Low
  0, 0, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_1/SP_P0: Mid (parity 0)
  0, 0, 0, 0, 0, 1, 1, 1, 1, 1,   // SP_M: Minute marker
  0, 0, 0, 1, 1, 1, 1, 1, 1, 1    // SP_P1: High (parity 1)
};

// BPC Time Code Patterns (China) - Quad with 5 patterns
int8_t bits_bpc[] = {
  SP_M4, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // (B), Parity1, Parity2, Hours[2], Minutes[3], DOW[2]
  SP_0,  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // Parity3, Day[3], Month[2], Year[3], Parity4
  SP_M4, SP_1, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // P1: 0, 1, 2 for 00-19, -39, -59 seconds
  SP_0,  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // P2: 0, P3: AM/PM(0/2)+parity<hmDOW>
  SP_M4, SP_2, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // P4: parity<DMY> (0/1)
  SP_0,  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0
};

// BPC Amplitude Modulation Patterns (5 patterns)
int8_t sp_bpc[] = {
  0, 1, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_0: Low
  0, 0, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_1: Lower-mid
  0, 0, 0, 1, 1, 1, 1, 1, 1, 1,   // SP_2: Mid
  0, 0, 0, 0, 1, 1, 1, 1, 1, 1,   // SP_3: Upper-mid
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0    // SP_M4: Minute marker (no modulation)
};

//...................................................................
// Function Declarations: Time Code Pattern Generation

/**
 * Generate JJY (Fukushima/Fukuoka) time code pattern for the current second
 * Encodes: year, month, day, hour, minute, second, day-of-week, leap second, parity
 */
void mb_jjy(void);

/**
 * Generate WWVB (US) time code pattern for the current second
 * Encodes: year, month, day, hour, minute, second, day-of-year, leap second, parity
 */
void mb_wwvb(void);

/**
 * Generate DCF77 (Germany) time code pattern for the current second
 * Encodes: minute, hour, day, month, year, day-of-week, time-zone, parity, leap second
 */
void mb_dcf(void);

/**
 * Generate BSF (Taiwan) time code pattern for the current second
 * Uses quad (4-level) encoding for denser data
 */
void mb_bsf(void);

/**
 * Generate MSF (UK) time code pattern for the current second
 * Encodes: minute, hour, day, month, year, day-of-week, leap second, parity
 */
void mb_msf(void);

/**
 * Generate BPC (China) time code pattern for the current second
 * Uses quad (4-level) encoding with 5 amplitude levels
 */
void mb_bpc(void);

// Bit pattern lookup table: maps station index to its bit pattern array
int8_t *st_bits[] = {bits_jjy, bits_jjy, bits_jjy, bits_dcf, bits_bsf, bits_msf, bits_bpc};
int8_t *bits60;  // Pointer to current station's 60-bit pattern

// Amplitude modulation pattern lookup table: maps station index to its modulation pattern
int8_t *st_sp[]   = {sp_jjy, sp_jjy, sp_wwvb, sp_dcf, sp_bsf, sp_msf, sp_bpc};
int8_t *secpattern;  // Pointer to current station's per-second modulation pattern

// Function pointer lookup table: maps station index to its pattern generation function
void (*st_makebits[])(void) = {mb_jjy, mb_jjy, mb_wwvb, mb_dcf, mb_bsf, mb_msf, mb_bpc};
void (*makebitpattern)(void);  // Function pointer to current station's pattern generator

//...................................................................
// Hardware Timer and Interrupt Handling

// Hardware Timer 0 (used for radio signal generation)
hw_timer_t *tm0 = NULL;  // Timer handle (NULL = not initialized)

// Timer interrupt synchronization
volatile SemaphoreHandle_t timerSemaphore;  // Semaphore for timer interrupt synchronization
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;  // Spinlock for interrupt-safe operations
volatile uint32_t buzzup = 0;  // Incremented when buzzer cycle completed (counts half-cycles)

// Timer state flag
int istimerstarted = 0;  // 1 if timer is running, 0 if stopped

//...................................................................
// Interrupt Counter Variables
// These track sub-divisions of the 1 KHz interrupt frequency

int radioc = 0;  // Radio carrier phase counter: 0..(tm0cycle - 1)
int ampc = 0;    // Amplitude cycle counter: 0..(AMPDIV - 1), generates 10 Hz updates
int tssec = 0;   // Time base counter: 0..(SSECDIV - 1), generates 1 Hz clock ticks

//...................................................................
// Network Time and Synchronization

int ntpsync = 1;  // NTP synchronization flag: 1 = synchronized, 0 = not synchronized
time_t now;       // Current UNIX timestamp (seconds since epoch)
struct tm nowtm;  // Broken-down time structure (year, month, day, hour, minute, second, etc.)

//...................................................................
// Output Control Variables

int radioout = 0,  // Current radio output pin value (0 or 1)
    buzzout = 0;   // Current buzzer output pin value (0 or 1)

/**
 * Amplitude modulation value for current second
 * 1 = radio output active (transmitting)
 * 0 = radio output reduced (no transmission)
 * Applied to current amplitude subsecond within the second frame
 */
int ampmod;

/**
 * Buzzer control flag
 * 1 = sound on (generate buzzer output)
 * 0 = sound off (suppress buzzer output)
 */
int buzzsw = 1;

//...................................................................
// WiFi Access Point Mode Configuration

bool ap_mode = false;              // WiFi mode flag: true = AP mode, false = STA mode
unsigned long last_wifi_check = 0; // Timestamp of last WiFi connection check

// extern 
void IRAM_ATTR onTimer(void);
void setup(void);
void loop(void);
void starttimer(void);
void stoptimer(void);
void ampchange(void);
void setstation(int station);
void binarize(int v, int pos, int len);
void bcdize(int v, int pos, int len);
void rbinarize(int v, int pos, int len);
void rbcdize(int v, int pos, int len);
void quadize(int v, int pos, int len);
int parity(int pos, int len);
int qparity(int pos, int len);
void setlocaltime(void);
void getlocaltime(void);
void loadConfig(void);
void saveConfig(void);
void loadSchedules(void);
void saveSchedules(void);
void initSPIFFS(void);
void initWebServer(void);
void startAPMode(void);
void stopAPMode(void);
void checkWiFiConnection(void);
void applyCurrentSchedule(void);
void printbits60(void);
void ntpstart(void);
void ntpstop(void);

//...................................................................
// intr handler:
//   this routine is called once every 1/2f sec (where f is radio freq).
//   - reverse radio output pin if modulation flag "ampmod" == 1,
//   - count up "radioc", if exceeds "radiodev" then
//     turn on buzzer flag "buzzup"; the "buzzup" is set once in 1/1000sec
//     on every frequency of the station (so "radiodev" should be set
//     propery depending to the intr cycle "tm0cycle" of the station).
void IRAM_ATTR onTimer(void)
{
  portENTER_CRITICAL_ISR(&timerMux);  // 扩大临界区，保护radioout修改
  // 简化radio输出逻辑，避免中断中复杂判断
  radioout = (ampmod) ? !radioout : 0;  // 仅当ampmod=1时翻转，否则置0
  digitalWrite(PIN_RADIO, radioout);
  
  radioc++;
  if (radiodiv <= radioc) {
    radioc = 0;
    buzzup++;
    xSemaphoreGiveFromISR(timerSemaphore, NULL);
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

//...................................................................
void setup(void)
{
  uint8_t macBT[6];
  char bt_name[32];

  Serial.begin(115200);
  delay(100);
  Serial.print("started...\n");

  // Initialize SPIFFS for web files
  initSPIFFS();
  
  // Load configuration from SPIFFS
  loadConfig();
  loadSchedules();

  // Check if WiFi credentials are configured
  if (strlen(ssid) == 0) {
    Serial.println("No WiFi config found, starting AP mode for configuration...");
    startAPMode();
  } else {
    // Try to connect to WiFi
    if (ntpsync) {
      ntpstart();
    }
    // If WiFi connection established, start web server
    if (WiFi.status() == WL_CONNECTED) {
      IPAddress ip = WiFi.localIP();
      Serial.printf("WiFi connected! Web server at http://%s\n", ip.toString().c_str());
      initWebServer();
      Serial.println("Web server started");
    } else if (!ntpsync && strlen(ssid) > 0) {
      Serial.println("WiFi connection failed, starting AP mode...");
      startAPMode();
    }
  }

  pinMode(PIN_RADIO, OUTPUT);
  digitalWrite(PIN_RADIO, radioout);
  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_BUZZ, buzzout);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  
  applyCurrentSchedule();
  ampchange();
  Serial.print("radio started.\n");
}


void loop() {
  int buzzup2 = 0;    // copy of buzzup: to make critical section shorter
  static char buf[128];
  static int  bufp = 0;

  // Check WiFi connection periodically
  if (ap_mode == false && (millis() - last_wifi_check) > WIFI_CONNECT_CHECK_INTERVAL) {
    last_wifi_check = millis();
    checkWiFiConnection();
  }

  // Handle web server requests (in both AP mode and STA mode)
  if (ap_mode || WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }

  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE) { // get semaphre
    portENTER_CRITICAL(&timerMux);              // CRITICAL SECTION ---
    if (buzzup) {
      buzzup2++;
      buzzup--;
    }
    portEXIT_CRITICAL(&timerMux);               // --- CRITICAL SECTION
  }
  if (buzzup2) {
    buzzup2--;
    if (! buzzout && ampmod && buzzsw) {
      buzzout = 1;
      digitalWrite(PIN_BUZZ, HIGH);
    } else {
      buzzout = 0;
      digitalWrite(PIN_BUZZ, LOW);
    }
    ampc++;
    if (AMPDIV <= ampc) {
      ampc = 0;
      tssec++;
      if (SSECDIV <= tssec) { // 1 second action --v
        tssec = 0;
        int lastmin = nowtm.tm_min;
        getlocaltime();
        
        // Apply current schedule BEFORE making bit pattern
        // This ensures the correct station is selected before encoding
        applyCurrentSchedule();
            
        if (lastmin != nowtm.tm_min) {
          // Reset schedule index when minute changes - allow rotation to restart
          last_rotation_time = millis();
          makebitpattern();
          printbits60();
        } 
         else {
          // Every second, check if we need to apply a new schedule (for rotation)
          // Only regenerate pattern if station changes
          if (current_schedule_index != -1 && applicable_count > 1) {
            makebitpattern();  // Update pattern every second when rotating
          }
        }
        Serial.printf("%d-%d-%d, %02d:%02d:%02d (sched=%d/%d)\n",
          nowtm.tm_year + 1900, nowtm.tm_mon + 1, nowtm.tm_mday,
          nowtm.tm_hour, nowtm.tm_min, nowtm.tm_sec,
          current_schedule_index + 1, applicable_count);
      }
      ampchange();
    }
  }
  yield();// feed watchdog
}


//...................................................................
void starttimer(void)
{
  if (istimerstarted) {
    stoptimer();
    delayMicroseconds(100);  // 用微秒级延迟，避免任务阻塞
  }
  
  ampc = 0;
  radioc = 0;
  
  // 确保信号量先销毁再创建
  if (timerSemaphore != NULL) {
    vSemaphoreDelete(timerSemaphore);
    timerSemaphore = NULL;
  }
  timerSemaphore = xSemaphoreCreateBinary();
  
  portDISABLE_INTERRUPTS();  // 禁用全局中断
  
  tm0 = timerBegin(0, tm0cycle, true);  // 定时器0，预分频系数tm0cycle
  if (tm0 == NULL) {
    portENABLE_INTERRUPTS();
    Serial.println("ERROR: Failed to create timer");
    return;
  }
  timerAttachInterrupt(tm0, &onTimer, true);
  timerAlarmWrite(tm0, TM0RES, true);  // 周期：TM0RES=1，即1/(1.25MHz) = 0.8us
  timerAlarmEnable(tm0);
  
  portENABLE_INTERRUPTS();
  istimerstarted = 1;
  Serial.println("Timer started");
}

void stoptimer(void)
{
  if (istimerstarted) {
    portDISABLE_INTERRUPTS();
    if (tm0 != NULL) {
      timerAlarmDisable(tm0);
      timerDetachInterrupt(tm0);
      timerEnd(tm0);
      tm0 = NULL;
    }
    portENABLE_INTERRUPTS();
    
    if (timerSemaphore != NULL) {
      vSemaphoreDelete(timerSemaphore);
      timerSemaphore = NULL;
    }
    istimerstarted = 0;
    Serial.println("Timer stopped");
  }
}

// setup amplitude value ampmod depends on bit pattern & 0.1 second frame
void
ampchange(void)
{
  ampmod = secpattern[bits60[nowtm.tm_sec] * 10 + tssec];
  if (ampmod) {
    digitalWrite(PIN_LED, HIGH);
    Serial.print("~");
  } else {
    digitalWrite(PIN_LED, LOW);
    Serial.print(".");
  }
  return;
}


void
setstation(int station)
{
  stoptimer();
  delay(100);  // Extra time for cleanup after stoptimer
  
  Serial.printf("station #%d:\n", station);
  tm0cycle = 80000 / st_cycle2[station];
  radiodiv = st_cycle2[station];
  Serial.printf("  freq %fkHz, timer intr: 80M / (%d x %d), buzz/radio: /%d\n",
    (float)radiodiv / 2., tm0cycle, TM0RES, radiodiv);
  bits60 = st_bits[station];
  Serial.printf("  bits60 pattern: ");
  for (int i = 0; i < 60; i++) {
    Serial.printf("%d", (int)bits60[i]);
  }
  secpattern = st_sp[station];
  Serial.printf("\n  second pattern: ");
  for (int i = 0; i < 10; i++) {
    Serial.printf("%d", (int)secpattern[i]);
  }
  Serial.printf("...\n");
  makebitpattern = st_makebits[station];
  yield();
  makebitpattern();
  printbits60();
  yield();
  starttimer();
  return;
}

//...................................................................
// makeup bit pattern for current date, hour:min

void
mbc_wwvbjjy(void) //--- [0..33] are common in WWVB/JJY
{
  binarize(nowtm.tm_min / 10, 1, 3);
  binarize(nowtm.tm_min % 10, 5, 4);
  binarize(nowtm.tm_hour / 10, 12, 2);
  binarize(nowtm.tm_hour % 10, 15, 4);
  int y100 = nowtm.tm_yday / 100;
  int y1 = (nowtm.tm_yday - y100 * 100);
  int y10 = y1 / 10;
  y1 = y1 % 10;
  binarize(y100, 22, 2);
  binarize(y10, 25, 4);
  binarize(y1, 30, 4);
//  Serial.printf("min%d-%d hour%d-%d doy%d-%d-%d ", tmin / 10, tmin % 10, thour / 10, thour % 10,
//    y100, y10, y1);
  return;  
}

void
mb_jjy(void)   //---- JJY_E & JJY_W
{
  Serial.print("encode JJY format - ");
  mbc_wwvbjjy();
  bits60[36] = parity(12, 7);
  bits60[37] = parity(1, 8);
//  Serial.printf("pa2%d ", s % 2);
  binarize((nowtm.tm_year - 100) / 10, 41, 4);
  binarize(nowtm.tm_year % 10, 45, 4);
  binarize(nowtm.tm_wday, 50, 3);
//  Serial.printf("year%d-%d dow%d\n", tyear / 10, tyear % 10, tdow);
  yield();
  return;
}

void
mb_wwvb(void)
{
  Serial.print("encode WWVB format - ");
  mbc_wwvbjjy();
  binarize((nowtm.tm_year - 100) / 10, 45, 4);
  binarize(nowtm.tm_year % 10, 50, 4);
  yield();
  return;
}

void
mb_dcf(void)  //---- DCF77
{
  Serial.print("encode DCF77 format - ");
//  bits60[0] = SP_0; // (obsolate) this routine is used also by mb_hbg() which changes bits60[0]
  rbcdize(nowtm.tm_min, 21, 7);
  bits60[28] = parity(21, 7);
  rbcdize(nowtm.tm_hour, 29, 6);
  bits60[35] = parity(29, 6);
  rbcdize(nowtm.tm_mday, 36, 6);
  rbinarize(nowtm.tm_wday, 42, 3);
  rbcdize(nowtm.tm_mon + 1, 45, 5);
  rbcdize(nowtm.tm_year - 100, 50, 8);
  bits60[58] = parity(36, 22);
  yield();
  return;
}

/*
void
mb_hbg(void)   //---- HBG
{
    mb_dcf();
    if (tmin != 0) {
      bits60[0] = SP_M0;
    } else {
      if (thour % 12 != 0) {
        bits60[0] = SP_M00;
      } else {
        bits60[0] = SP_M000;
      }
    }
    return;
}
*/

void
mb_bsf(void)   //---- BSF
{
  Serial.print("encode BSF format - ");
  quadize(nowtm.tm_min, 41, 3);
  quadize(nowtm.tm_hour * 2, 44, 3);
  bits60[46] |= qparity(41, 6);
  quadize(nowtm.tm_mday * 2, 47, 3);
  bits60[49] |= nowtm.tm_wday / 4;
  quadize(nowtm.tm_wday % 4, 50, 1);
  quadize(nowtm.tm_mon + 1, 51, 2);
  quadize((nowtm.tm_year - 100) * 2, 53, 4);
  bits60[56] |= qparity(47, 10);
  yield();
  return;
}

void
mb_msf(void)
{
  Serial.print("encode MSF format - ");
  bcdize(nowtm.tm_year - 100, 17, 8);
  bcdize(nowtm.tm_mon + 1, 25, 5);
  bcdize(nowtm.tm_mday, 30, 6);
  binarize(nowtm.tm_wday, 36, 3);
  bcdize(nowtm.tm_hour, 39, 6);
  bcdize(nowtm.tm_min, 45, 7);
  bits60[54] = parity(17, 8) * 2 + 1;  // in MSF parity bits, values are {1, 3} (SP_1, SP_P1)
  bits60[55] = parity(25, 11) * 2 + 1; // for parity {0, 1}.
  bits60[56] = parity(36, 3) * 2 + 1;
  bits60[57] = parity(39, 13) * 2 + 1;
  bits60[58] = SP_1;  // change here to SP_P1 if summertime 
  yield();
  return;
}

void
mb_bpc(void)
{
  Serial.print("encode BPC format - ");
  quadize(nowtm.tm_hour % 12, 3, 2);
  quadize(nowtm.tm_min, 5, 3);
  quadize(nowtm.tm_wday, 8, 2);
  bits60[10] = (nowtm.tm_hour / 12) * 2 + qparity(3, 7);
  quadize(nowtm.tm_mday, 11, 3);
  quadize(nowtm.tm_mon + 1, 14, 2);
  quadize(nowtm.tm_year - 100, 16, 3);
  bits60[19] = qparity(11, 8);
  for (int i = 2; i < 20; i++) {
    bits60[20 + i] = bits60[i];
    bits60[40 + i] = bits60[i];
  }
  yield();
  return;
}


// write binary value into bit pattern (little endian)
void
binarize(int v, int pos, int len)
{
  for (pos = pos + len - 1; 0 < len; pos--, len--) {
    bits60[pos] = (uint8_t)(v & 1);
    v >>= 1;
  }
  return;
}

// continuous (over 4 bit) BCD to write
void
bcdize(int v, int pos, int len)
{
  int l;

  pos = pos + len - 1;
  while (0 < len) {
    if (4 <= len) {
      l = 4;
    } else {
      l = len;
    }
    binarize(v % 10, pos - l + 1, l);
    v = v / 10;
    pos = pos - l;
    len = len - l;
  }
  return;
}

// LSB->MSB (big endian) binarize
void
rbinarize(int v, int pos, int len)
{
  for ( ; 0 < len; pos++, len--) {
    bits60[pos] = (uint8_t)(v & 1);
    v >>= 1;
  }
  return;
}

// LSB->MSB BCDize
void
rbcdize(int v, int pos, int len)
{
  int l;

Serial.printf("\nrbcd %d[pos %d, len%d]=", v, pos, len);
  while (0 < len) {
    if (4 <= len) {
      l = 4;
    } else {
      l = len;
    }
    rbinarize(v % 10, pos, l);
    v = v / 10;
    pos = pos + l;
    len = len - l;
  }
  return;
}


// 4-ary encoding (little endian)
void
quadize(int v, int pos, int len)
{
  for (pos = pos + len - 1; 0 < len; pos--, len--) {
    bits60[pos] = (uint8_t)(v & 3);
    v >>= 2;
  }
  return;
}

// calculate even parity
int
parity(int pos, int len)
{
  int s = 0;
  
  for (pos; 0 < len; pos++, len--) {
    s += bits60[pos];
  }
  return (s % 2);
}

// binary parity for 4-ary data (for BSF/BPC): is it OK?
int
qparity(int pos, int len)
{
  int s = 0;
  
  for (pos; 0 < len; pos++, len--) {
    s += (bits60[pos] & 1) + ((bits60[pos] & 2) >> 1);
  }
  return (s % 2);
}


//// calculate doy (day of year)/dow (day of week) from YY/MM/DD
//void
//setdoydow(void)
//{
//  int j0 = julian(tyear, 1, 1);      // new year day of this year
//  int j1 = julian(tyear, tmon, tday);
//  tdoy = j1 - j0 + 1; // 1..365/366
//  tdow = j1 % 7;      // 0..6
//  return;
//}
//
//// return julian date (? relative date from a day)
//// sunday is multiple of 7
//int
//julian(int y, int m, int d)
//{
//  if (m <= 2) {
//    m = m + 12;
//    y--;
//  }
//  return y * 1461 / 4 + (m + 1) * 153 / 5 + d + 6;
////  1461 / 4 == 365.25, 153 / 5 == 30.6
//}
//
//// increment tday-tmon-tyear, tdoy, tdow
//void
//incday(void)
//{
//  int year1 = tyear;   // year of next month
//  int mon1 = tmon + 1; // next month
//  if (12 < mon1) {
//    mon1 = 1;
//    year1++;
//  }
//  int day1 = tday + 1; // date# of tomorrow
//  if (julian(year1, mon1, 1) - julian(tyear, tmon, 1) < day1) {
//    tday = 1;  // date# exceeds # of date in this month
//    tmon = mon1;
//    tyear = year1;
//  } else {
//    tday = day1;
//  }
//  setdoydow(); // tdoy, tdow is updated from tyear-tmonth-tday
//  return;
//}

void
printbits60(void)
{
  Serial.print("\n");
  for (int i = 0; i < 60; i++) {
    Serial.print(bits60[i]);
  }
  Serial.print("\n");
  return;
}

void
ntpstart(void)
{
  int i;

// WiFi, NTP setup
  Serial.print("Attempting to connect to Network named: ");
  Serial.println(ssid);                   // print the network name (SSID);
  WiFi.begin(ssid, passwd);
  for (i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
    Serial.print(".");
    delay(1000);
  }
  if (i == 10) {
    ntpsync = 0;
    return;
  }
  IPAddress ip = WiFi.localIP();
  Serial.printf("IP Address: ");
  Serial.println(ip);
  Serial.printf("configureing NTP...");
  configTime(timezone_offset, 0, "ntp.aliyun.com", "ntp.jst.mfeed.ad.jp"); // enable NTP
  for (int i = 0; i < 10 && ! getLocalTime(&nowtm); i++) {
    Serial.printf(".");
    delay(1000);
  }
  if (i == 10) {
    ntpsync = 0;
    return;
  }
  Serial.printf("done\n");
}


void
ntpstop(void)
{
  ntpsync = 0;
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}


void
setlocaltime(void)
{
  time_t nowtime = mktime(&nowtm) /*+ TZ */;
  struct timeval tv = {
    .tv_sec = nowtime
  };
  settimeofday(&tv, NULL);
  getlocaltime(); // to make wday/yday
}


void
getlocaltime(void)
{
  if (ntpsync) {
    getLocalTime(&nowtm);
  } else {
    time_t nowtime;
    time(&nowtime);
    struct tm *ntm;
    ntm = localtime(&nowtime);
    nowtm = *ntm;
  }
}

//...................................................................
// New functions for web UI and configuration

void initSPIFFS(void)
{
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  Serial.println("SPIFFS Mounted Successfully");
}

void loadConfig(void)
{
  File configFile = SPIFFS.open(CONFIG_FILE, "r");
  if (!configFile) {
    Serial.println("No config file found, using defaults");
    strcpy(ssid, "");
    strcpy(passwd, "");
    timezone_offset = DEFAULT_TZ;
    return;
  }
  
  // Read SSID and password
  configFile.readStringUntil('\n').toCharArray(ssid, sizeof(ssid));
  configFile.readStringUntil('\n').toCharArray(passwd, sizeof(passwd));
  
  // Try to read timezone offset
  String tz_str = configFile.readStringUntil('\n');
  if (tz_str.length() > 0) {
    timezone_offset = tz_str.toInt();
  } else {
    timezone_offset = DEFAULT_TZ;
  }
  
  // Remove newlines
  for (int i = 0; ssid[i]; i++) {
    if (ssid[i] == '\r' || ssid[i] == '\n') ssid[i] = '\0';
  }
  for (int i = 0; passwd[i]; i++) {
    if (passwd[i] == '\r' || passwd[i] == '\n') passwd[i] = '\0';
  }
  
  configFile.close();
  Serial.printf("Loaded config: SSID=%s, TZ=%ld\n", ssid, timezone_offset);
}

void saveConfig(void)
{
  File configFile = SPIFFS.open(CONFIG_FILE, "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return;
  }
  
  configFile.println(ssid);
  configFile.println(passwd);
  configFile.println(timezone_offset);
  configFile.close();
  Serial.println("Config saved");
}

void loadSchedules(void)
{
  schedule_count = 0;

  File schedFile = SPIFFS.open(STATION_CONFIG_FILE, "r");
  if (!schedFile) {
    Serial.println("No schedule file found, using default");
    schedules[0].station   = SN_JJY_E;
    schedules[0].start_min = 0;
    schedules[0].end_min   = 1439;
    schedule_count = 1;
    return;
  }

  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, schedFile);
  schedFile.close();

  if (err || !doc.is<JsonArray>()) {
    Serial.println("Schedule JSON invalid, using default");
    goto DEFAULT_SCHEDULE;
  }
{
  JsonArray arr = doc.as<JsonArray>();

  /* ---------- 遍历数组 ---------- */
  for (JsonObject obj : arr) {
    /* 普通 schedule 项 */
    if (!obj.containsKey("station") ||
        !obj.containsKey("start") ||
        !obj.containsKey("end")) {
      continue;
    }

    if (schedule_count >= MAX_SCHEDULES)
      break;

    schedules[schedule_count].station   = obj["station"].as<int>();
    schedules[schedule_count].start_min = obj["start"].as<int>();
    schedules[schedule_count].end_min   = obj["end"].as<int>();

    schedule_count++;
  }

  if (schedule_count == 0) {
    goto DEFAULT_SCHEDULE;
  }

  Serial.printf(
    "Loaded %d schedules\n",
    schedule_count
  );
  return;
}
DEFAULT_SCHEDULE:
  schedules[0].station   = SN_JJY_E;
  schedules[0].start_min = 0;
  schedules[0].end_min   = 1439;
  schedule_count = 1;
}

void saveSchedules(void)
{
  File schedFile = SPIFFS.open(STATION_CONFIG_FILE, "w");
  if (!schedFile) {
    Serial.println("Failed to open schedule file for writing");
    return;
  }
  
  // Write JSON format
  schedFile.print("[");
  for (int i = 0; i < schedule_count; i++) {
    if (i > 0) schedFile.print(",");
    schedFile.printf("{\"station\":%d,\"start\":%d,\"end\":%d}",
                     schedules[i].station,
                     schedules[i].start_min,
                     schedules[i].end_min);
  }
  schedFile.print("]");
  schedFile.close();
  Serial.println("Schedules saved");
  applyCurrentSchedule();
}

void applyCurrentSchedule(void)
{
  // Calculate current minute of day (0-1439)
  int current_min = nowtm.tm_hour * 60 + nowtm.tm_min;
  
  // Find all applicable schedules for current time
  applicable_count = 0;
  for (int i = 0; i < schedule_count; i++) {
    if (current_min >= schedules[i].start_min && 
        current_min < schedules[i].end_min) {
      applicable_schedules[applicable_count] = i;
      applicable_count++;
    }
  }
  
  int new_station = SN_JJY_E;  // Default
  
  if (applicable_count == 0) {
    // No applicable schedule, use default
    current_schedule_index = -1;
  } else if (applicable_count == 1) {
    // Single schedule, use it
    current_schedule_index = 0;
    new_station = schedules[applicable_schedules[0]].station;
  } else {
    // Multiple schedules - rotate through them
    unsigned long now = millis();
    if (current_schedule_index == -1) {
      current_schedule_index = 0;
      last_rotation_time = now;
      Serial.printf("Start rotation: %d schedules available\n", applicable_count);
    }
      // Serial.printf("now: %lu\n", now);
      // Serial.printf("last_rotation_time: %lu\n", last_rotation_time);
    // Check if it's time to rotate to next schedule
    if ((now - last_rotation_time) >= (ROTATION_INTERVAL_MINUTES * 60000UL)) {
      current_schedule_index++;
      Serial.print("current_schedule_index:");
      Serial.println(current_schedule_index);

      Serial.print("applicable_count:");
      Serial.println(applicable_count);

      if (current_schedule_index >= applicable_count) {
        current_schedule_index = 0;
      }
      last_rotation_time = now;
      Serial.printf("Rotating to schedule %d of %d (station=%d)\n", 
                    current_schedule_index + 1, applicable_count,
                    schedules[applicable_schedules[current_schedule_index]].station);
    }
    
    new_station = schedules[applicable_schedules[current_schedule_index]].station;
  }
  
  // Change station if needed
  if (new_station != last_station) {
    last_station = new_station;
    setstation(new_station);
  }
}

void checkWiFiConnection(void)
{
  if (ap_mode) {
    return;  // Already in AP mode
  }
  
  // Check if WiFi is still connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost, attempting to reconnect...");
    WiFi.reconnect();
  }
  
  // If connection takes too long, switch to AP mode
  if (WiFi.status() != WL_CONNECTED && 
      (millis() - wifi_connect_start) > WIFI_CONNECT_TIMEOUT) {
    Serial.println("WiFi connection timeout, starting AP mode...");
    startAPMode();
  }
}

void startAPMode(void)
{
  stoptimer();  // Stop radio while configuring
  ap_mode = true;
  WiFi.mode(WIFI_AP);
  
  // Create AP name from MAC address
  uint8_t macBT[6];
  esp_read_mac(macBT, ESP_MAC_BT);
  char ap_name[32];
  snprintf(ap_name, sizeof(ap_name), "%s_%02X%02X%02X",
           DEVICENAME_PREFIX, macBT[3], macBT[4], macBT[5]);
  
  WiFi.softAP(ap_name, "12345678");  // Open AP or with default password
  
  IPAddress IP = WiFi.softAPIP();
  Serial.printf("AP Mode started. SSID: %s, IP: %s\n", ap_name, IP.toString().c_str());
  
  initWebServer();
  Serial.println("Web server started");
}

void stopAPMode(void)
{
  ap_mode = false;
  WiFi.softAPdisconnect(true);
  // Keep web server running in STA mode
  WiFi.mode(WIFI_STA);
  starttimer();  // Resume radio
  Serial.println("AP mode stopped, web server continues on WiFi");
}

void initWebServer(void)
{
  // Serve index page
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", getIndexHTML());
  });
  
  // API endpoints
  server.on("/api/config", HTTP_GET, []() {
    String json = "{\"ssid\":\"" + String(ssid) + "\",\"timezone\":" + String(timezone_offset) + "}";
    server.send(200, "application/json", json);
  });
  
  server.on("/api/config", HTTP_POST, []() {
    if (server.hasArg("ssid") && server.hasArg("password")) {
      server.arg("ssid").toCharArray(ssid, sizeof(ssid));
      server.arg("password").toCharArray(passwd, sizeof(passwd));
      
      // Handle timezone if provided
      if (server.hasArg("timezone")) {
        timezone_offset = server.arg("timezone").toInt();
      }
      
      saveConfig();
      
      // Send response immediately
      server.send(200, "application/json", "{\"status\":\"ok\"}");
      
      // Connect to new WiFi
      Serial.printf("Connecting to WiFi: %s\n", ssid);
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, passwd);
      
      // Give WiFi time to connect (non-blocking in background)
      unsigned long connect_start = millis();
      while ((millis() - connect_start) < 10000) {
        delay(500);
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("WiFi connected!");
          IPAddress ip = WiFi.localIP();
          Serial.printf("Web server available at http://%s\n", ip.toString().c_str());
          if (ntpsync) {
            ntpstart();
          }
          // Keep web server running - don't stop AP mode
          ap_mode = false;
          Serial.println("Web interface now available on WiFi network");
          starttimer();  // Resume radio
          return;
        }
      }
      Serial.println("WiFi connection failed, AP mode still active");
    } else {
      server.send(400, "application/json", "{\"error\":\"missing parameters\"}");
    }
  });
  
  server.on("/api/stations", HTTP_GET, []() {
    String json = "[";
    for (int i = 0; i < NUM_STATIONS; i++) {
      if (i > 0) json += ",";
      json += "{\"id\":" + String(i) + ",\"name\":\"" + String(station_names[i]) + "\"}";
    }
    json += "]";
    server.send(200, "application/json", json);
  });
  
  server.on("/api/schedules", HTTP_GET, []() {
    String json = "[";
    for (int i = 0; i < schedule_count; i++) {
      if (i > 0) json += ",";
      json += "{\"station\":" + String(schedules[i].station) +
              ",\"start\":" + String(schedules[i].start_min) +
              ",\"end\":" + String(schedules[i].end_min) + "}";
    }
    json += "]";
    server.send(200, "application/json", json);
  });
  
  server.on("/api/schedule", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      // Parse JSON and add single schedule
      String json = server.arg("plain");
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, json);
      
      if (error) {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"invalid json\"}");
        return;
      }
      
      if (schedule_count < MAX_SCHEDULES) {
        schedules[schedule_count].station = doc["station"];
        schedules[schedule_count].start_min = doc["start"];
        schedules[schedule_count].end_min = doc["end"];
        schedule_count++;
        saveSchedules();
        server.send(200, "application/json", "{\"status\":\"ok\"}");
      } else {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"max schedules reached\"}");
      }
    } else {
      server.send(400, "application/json", "{\"error\":\"missing data\"}");
    }
  });
  
  server.on("/api/schedules", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      // Parse JSON and update schedules
      String json = server.arg("plain");
      StaticJsonDocument<2000> doc;
      DeserializationError error = deserializeJson(doc, json);
      
      if (error) {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"invalid json\"}");
        return;
      }
      
      schedule_count = 0;
      for (JsonObject item : doc.as<JsonArray>()) {
        if (schedule_count < MAX_SCHEDULES) {
          schedules[schedule_count].station = item["station"];
          schedules[schedule_count].start_min = item["start"];
          schedules[schedule_count].end_min = item["end"];
          schedule_count++;
        }
      }
      saveSchedules();
      server.send(200, "application/json", "{\"status\":\"ok\"}");
    } else {
      server.send(400, "application/json", "{\"error\":\"missing data\"}");
    }
  });
  
  server.on("/api/schedule", HTTP_DELETE, []() {
    if (server.hasArg("index")) {
      int idx = server.arg("index").toInt();
      if (idx >= 0 && idx < schedule_count) {
        // Remove schedule at index
        for (int i = idx; i < schedule_count - 1; i++) {
          schedules[i] = schedules[i + 1];
        }
        schedule_count--;
        saveSchedules();
        server.send(200, "application/json", "{\"status\":\"ok\"}");
      } else {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"invalid index\"}");
      }
    } else {
      server.send(400, "application/json", "{\"error\":\"missing index\"}");
    }
  });
  
  server.on("/api/status", HTTP_GET, []() {
    // Build response with rotation information
    String json = "{\"time\":\"" + String(nowtm.tm_hour) + ":" +
                  String(nowtm.tm_min) + ":" + String(nowtm.tm_sec) + "\"";
    
    // Add applicable schedules count for rotation display
    json += ",\"applicable_schedules\":[";
    for (int i = 0; i < applicable_count; i++) {
      if (i > 0) json += ",";
      json += String(applicable_schedules[i]);
    }
    json += "]";
    
    json += "}";
    server.send(200, "application/json", json);
  });
  
  server.begin();
}

String getIndexHTML(void)
{
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Radio Station</title>";
  html += "<style>";
  html += "body{font-family:'Segoe UI',Arial,sans-serif;background: linear-gradient(135deg, #89CFF0, #4682B4);";
  html += "min-height:100vh;margin:0;padding:10px}";
  html += ".container{background:white;border-radius:12px;box-shadow:0 20px 60px rgba(0,0,0,0.3);";
  html += "max-width:900px;margin:0 auto;overflow:hidden}";
  html += ".header{background:linear-gradient(135deg,#667eea,#764ba2);color:white;padding:30px;text-align:center;position:relative}";
  html += ".header h1{margin:0;font-size:28px}";
  html += ".lang-switch{position:absolute;top:10px;right:10px;display:flex;gap:5px}";
  html += ".lang-btn{padding:6px 12px;background:rgba(255,255,255,0.2);color:white;border:1px solid white;";
  html += "border-radius:4px;cursor:pointer;font-size:12px;transition:all 0.3s}";
  html += ".lang-btn.active{background:white;color:#667eea}";
  html += ".tabs{display:flex;background:#f5f5f5;border-bottom:1px solid #ddd}";
  html += ".tab-btn{flex:1;padding:15px;border:none;background:none;cursor:pointer;font-weight:600;color:#666;";
  html += "border-bottom:3px solid transparent;transition:all 0.3s}";
  html += ".tab-btn.active{color:#667eea;border-bottom-color:#667eea}";
  html += ".tab-content{display:none;padding:30px}";
  html += ".tab-content.active{display:block}";
  html += ".content{padding:30px}.section{margin-bottom:30px}";
  html += ".section h2{border-bottom:2px solid #667eea;padding-bottom:10px}";
  html += ".form-group{margin-bottom:15px}";
  html += "label{display:block;margin-bottom:8px;color:#555;font-weight:500;font-size:14px}";
  html += "input,select,textarea{width:100%;padding:12px;border:1px solid #ddd;border-radius:8px;";
  html += "font-size:14px;box-sizing:border-box;font-family:inherit}";
  html += "input:focus,select:focus,textarea:focus{outline:0;border-color:#667eea;";
  html += "box-shadow:0 0 0 3px rgba(102,126,234,0.1)}";
  html += ".button-group{display:flex;gap:10px;margin-top:20px}";
  html += "button{padding:12px;border:0;border-radius:8px;font-size:14px;font-weight:600;";
  html += "cursor:pointer;transition:all 0.3s}";
  html += ".btn-primary{background:linear-gradient(135deg,#667eea,#764ba2);color:white;flex:1}";
  html += ".btn-primary:hover{transform:translateY(-2px);box-shadow:0 10px 20px rgba(102,126,234,0.3)}";
  html += ".btn-secondary{background:#f0f0f0;color:#333;flex:1}";
  html += ".btn-danger{background:#dc3545;color:white;padding:8px 12px;flex:0}";
  html += ".time-display{background:#667eea;color:white;padding:20px;border-radius:8px;";
  html += "text-align:center;font-size:32px;font-weight:300;margin-bottom:10px;font-family:monospace}";
  html += ".rotation-info{background:#e7f3ff;color:#0066cc;padding:12px;border-radius:8px;";
  html += "margin-bottom:20px;font-size:13px;border-left:4px solid #0066cc}";
  html += ".status{padding:12px;border-radius:8px;margin-bottom:15px;font-size:14px}";
  html += ".status.success{background:#d4edda;color:#155724;border:1px solid #c3e6cb}";
  html += ".status.error{background:#f8d7da;color:#721c24;border:1px solid #f5c6cb}";
  html += ".schedule-item{background:#f9f9f9;padding:15px;border-radius:8px;margin-bottom:10px;";
  html += "display:flex;justify-content:space-between;align-items:center}";
  html += ".schedule-info{flex:1}";
  html += ".timezone-info{font-size:13px;color:#666;margin-top:5px}";
  html += ".time-inputs{display:flex;gap:10px;align-items:center}";
  html += ".time-inputs input{width:auto;flex:1}";
  html += ".modal{display:none;position:fixed;top:0;left:0;width:100%;height:100%;";
  html += "background:rgba(0,0,0,0.5);z-index:1000}";
  html += ".modal.active{display:flex;align-items:center;justify-content:center}";
  html += ".modal-content{background:white;border-radius:12px;padding:30px;max-width:500px;width:90%}";
  html += ".modal-header{font-size:20px;font-weight:600;margin-bottom:20px}";
  html += ".modal-body{margin-bottom:20px}";
  html += ".modal-footer{display:flex;gap:10px}";
  html += "</style></head><body>";
  html += "<div class='container'>";
  html += "<div class='header'>";
  html += "<div class='lang-switch'>";
  html += "<button class='lang-btn active' onclick='setLanguage(\"zh\")' id='langZh'>中文</button>";
  html += "<button class='lang-btn' onclick='setLanguage(\"en\")' id='langEn'>English</button>";
  html += "</div>";
  html += "<h1 data-i18n='title'>无线电时间服务</h1><p data-i18n='subtitle'>配置管理面板</p></div>";
  html += "<div class='tabs'>";
  html += "<button class='tab-btn active' onclick='switchTab(0)' data-i18n='tab_wifi'>WiFi设置</button>";
  html += "<button class='tab-btn' onclick='switchTab(1)' data-i18n='tab_schedule'>时间计划</button>";
  html += "<button class='tab-btn' onclick='switchTab(2)' data-i18n='tab_station'>电台信息</button>";
  html += "</div>";
  
  html += "<div class='tab-content active'>";
  html += "<div class='content'>";
  html += "<div id='statusMsg'></div>";
  html += "<div class='time-display' id='timeDisplay'>00:00:00</div>";
  html += "<div class='rotation-info' id='rotationInfo'></div>";
  html += "<div class='section'><h2 data-i18n='wifi_settings'>WiFi设置</h2>";
  html += "<div class='form-group'><label data-i18n='ssid_label'>网络名称(SSID)</label>";
  html += "<input type='text' id='ssid' placeholder='WiFi SSID'></div>";
  html += "<div class='form-group'><label data-i18n='password_label'>密码</label>";
  html += "<input type='password' id='password' placeholder='WiFi密码'></div>";
  html += "<div class='form-group'><label data-i18n='timezone_label'>时区(秒)</label>";
  html += "<select id='timezone'>";
  html += "<option value='32400'>UTC+9 (东京)</option>";
  html += "<option value='28800'>UTC+8 (北京)</option>";
  html += "<option value='19800'>UTC+5:30 (印度)</option>";
  html += "<option value='0'>UTC+0 (伦敦)</option>";
  html += "<option value='-18000'>UTC-5 (美国东部)</option>";
  html += "<option value='-28800'>UTC-8 (美国西部)</option>";
  html += "</select>";
  html += "<div class='timezone-info'><span data-i18n='current_tz'>当前时区偏移</span>: <span id='tzDisplay'>32400</span> s</div>";
  html += "</div>";
  html += "<div class='button-group'><button class='btn-primary' onclick='saveWiFiConfig()' data-i18n='save_wifi'>保存WiFi设置</button></div>";
  html += "</div></div></div>";
  
  html += "<div class='tab-content'>";
  html += "<div class='content'>";
  html += "<div class='section'><h2 data-i18n='schedule_manage'>时间计划</h2>";
  html += "<div id='schedulesList'></div>";
  html += "<div class='button-group'>";
  html += "<button class='btn-primary' onclick='showAddScheduleModal()' data-i18n='add_schedule'>添加计划</button>";
  html += "</div></div></div></div>";
  
  html += "<div class='tab-content'>";
  html += "<div class='content'>";
  html += "<div class='section'><h2 data-i18n='available_station'>可用电台</h2>";
  html += "<div id='stationsList'></div>";
  html += "</div></div></div>";
  
  html += "</div>";
  
  // Modal for adding schedule
  html += "<div id='scheduleModal' class='modal'>";
  html += "<div class='modal-content'>";
  html += "<div class='modal-header' data-i18n='add_plan'>添加时间计划</div>";
  html += "<div class='modal-body'>";
  html += "<div class='form-group'><label data-i18n='select_station'>选择电台</label>";
  html += "<select id='scheduleStation'></select></div>";
  html += "<div class='form-group'><label data-i18n='start_time'>开始时间</label>";
  html += "<input type='time' id='scheduleStartTime'></div>";
  html += "<div class='form-group'><label data-i18n='end_time'>结束时间</label>";
  html += "<input type='time' id='scheduleEndTime'></div>";
  html += "</div>";
  html += "<div class='modal-footer'>";
  html += "<button class='btn-primary' onclick='addScheduleFromModal()' data-i18n='add'>添加</button>";
  html += "<button class='btn-secondary' onclick='closeModal()' data-i18n='cancel'>取消</button>";
  html += "</div></div></div>";
  
  html += "<script>";
  html += "const i18n={zh:{title:'电波钟配置终端',subtitle:'配置管理面板',tab_wifi:'WiFi设置',";
  html += "tab_schedule:'时间计划',tab_station:'电台信息',wifi_settings:'WiFi设置',";
  html += "ssid_label:'网络名称(SSID)',password_label:'密码',timezone_label:'时区(秒)',";
  html += "current_tz:'当前时区偏移',save_wifi:'保存WiFi设置',schedule_manage:'时间计划',";
  html += "add_schedule:'添加计划',available_station:'可用电台',add_plan:'添加时间计划',";
  html += "select_station:'选择电台',start_time:'开始时间',end_time:'结束时间',";
  html += "add:'添加',cancel:'取消'},";
  html += "en:{title:'Radio Station',subtitle:'Configuration Portal',tab_wifi:'WiFi Settings',";
  html += "tab_schedule:'Schedules',tab_station:'Stations',wifi_settings:'WiFi Settings',";
  html += "ssid_label:'Network Name (SSID)',password_label:'Password',timezone_label:'Timezone (sec)',";
  html += "current_tz:'Current Timezone',save_wifi:'Save WiFi Settings',schedule_manage:'Schedule',";
  html += "add_schedule:'Add Schedule',available_station:'Available Stations',add_plan:'Add Schedule',";
  html += "select_station:'Select Station',start_time:'Start Time',end_time:'End Time',";
  html += "add:'Add',cancel:'Cancel'}};";
  html += "let lang='zh',stations=[];";
  html += "function setLanguage(l){lang=l;";
  html += "document.getElementById('langZh').classList.toggle('active',l==='zh');";
  html += "document.getElementById('langEn').classList.toggle('active',l==='en');";
  html += "document.querySelectorAll('[data-i18n]').forEach(el=>{";
  html += "el.textContent=i18n[l][el.getAttribute('data-i18n')]||el.textContent});";
  html += "loadData();}";
  html += "function switchTab(n){";
  html += "const btns=document.querySelectorAll('.tab-btn');const tabs=document.querySelectorAll('.tab-content');";
  html += "btns.forEach((b,i)=>b.classList.toggle('active',i===n));";
  html += "tabs.forEach((t,i)=>t.classList.toggle('active',i===n));}";
  html += "function updateTime(){fetch('/api/status').then(r=>r.json()).then(d=>{";
  html += "document.getElementById('timeDisplay').textContent=d.time;";
  html += "const activeStations=d.applicable_schedules?d.applicable_schedules.length:0;";
  html += "if(activeStations>1){";
  html += "document.getElementById('rotationInfo').textContent=(lang==='zh'?'轮流发波中: ':'Rotating: ')+activeStations+' '+";
  html += "(lang==='zh'?'个电台':'stations');}else{document.getElementById('rotationInfo').textContent='';}";
  html += "}).catch(e=>{})}";
  html += "async function loadData(){try{";
  html += "const cf=await fetch('/api/config');const cfg=await cf.json();";
  html += "document.getElementById('timezone').value=cfg.timezone;";
  html += "document.getElementById('tzDisplay').textContent=cfg.timezone;";
  html += "const sr=await fetch('/api/stations');stations=await sr.json();";
  html += "let html='';stations.forEach(st=>{html+='<div style=\"padding:12px;background:#f9f9f9;";
  html += "border-radius:8px;margin-bottom:8px;border-left:4px solid #667eea;\">'+st.name+'</div>'});";
  html += "document.getElementById('stationsList').innerHTML=html;";
  html += "let opts='';stations.forEach((s,i)=>{opts+='<option value=\"'+i+'\">'+s.name+'</option>'});";
  html += "document.getElementById('scheduleStation').innerHTML=opts;";
  html += "const scr=await fetch('/api/schedules');const sc=await scr.json();";
  html += "let shtml='';sc.forEach((x,i)=>{";
  html += "const sh=Math.floor(x.start/60);const sm=x.start%60;";
  html += "const eh=Math.floor(x.end/60);const em=x.end%60;";
  html += "const st=stations[x.station]?stations[x.station].name:'Unknown';";
  html += "shtml+='<div class=\"schedule-item\"><div class=\"schedule-info\">'";
  html += "+'<strong>'+st+'</strong><br/>'+String(sh).padStart(2,'0')+':'+String(sm).padStart(2,'0')";
  html += "+' - '+String(eh).padStart(2,'0')+':'+String(em).padStart(2,'0')+\"</div>\"";
  html += "+'<button class=\"btn-danger\" onclick=\"deleteSchedule(' + i + ')\">'"
        "+(lang === 'zh' ? '删除' : 'Delete')"
        "+'</button></div>'});";
  html += "document.getElementById('schedulesList').innerHTML=shtml||'<p>'+(lang==='zh'?'暂无计划':'No schedules')+'</p>';";
  html += "}catch(e){console.error(e)}}";
  html += "function showStatus(m,t){const d=document.getElementById('statusMsg');";
  html += "d.className='status '+t;d.textContent=m;d.style.display='block';";
  html += "if(t==='success')setTimeout(()=>{d.style.display='none'},3000)}";
  html += "function showAddScheduleModal(){document.getElementById('scheduleModal').classList.add('active')}";
  html += "function closeModal(){document.getElementById('scheduleModal').classList.remove('active')}";
  html += "function addScheduleFromModal(){";
  html += "const st=document.getElementById('scheduleStation').value;";
  html += "const start=document.getElementById('scheduleStartTime').value;";
  html += "const end=document.getElementById('scheduleEndTime').value;";
  html += "const msg=lang==='zh'?'请填写完整的时间':'Please fill in complete time';";
  html += "if(!start||!end){showStatus(msg,'error');return}";
  html += "const [sh,sm]=start.split(':').map(Number);const [eh,em]=end.split(':').map(Number);";
  html += "const startMin=sh*60+sm;const endMin=eh*60+em;";
  html += "fetch('/api/schedule',{method:'POST',headers:{'Content-Type':'application/json'},";
  html += "body:JSON.stringify({station:parseInt(st),start:startMin,end:endMin})})";
  html += ".then(r=>r.json()).then(d=>{";
  html += "if(d.status==='ok'){";
  html += "showStatus(lang==='zh'?'计划已添加':'Schedule added','success');";
  html += "closeModal();loadData()}else showStatus(lang==='zh'?'添加失败':'Failed','error')})";
  html += ".catch(e=>showStatus('Error: '+e,'error'))}";
  html += "function deleteSchedule(i){";
  html += "const msg=lang==='zh'?'确定删除此计划？':'Delete this schedule?';";
  html += "if(confirm(msg)){";
  html += "fetch('/api/schedule?index='+i,{method:'DELETE'})";
  html += ".then(r=>r.json()).then(d=>{";
  html += "if(d.status==='ok'){showStatus(lang==='zh'?'计划已删除':'Schedule deleted','success');loadData()}";
  html += "else showStatus(lang==='zh'?'删除失败':'Delete failed','error')}).catch(e=>showStatus('Error: '+e,'error'))}}";
  html += "function saveWiFiConfig(){const s=document.getElementById('ssid').value;";
  html += "const p=document.getElementById('password').value;const tz=document.getElementById('timezone').value;";
  html += "const msg=lang==='zh'?'请输入SSID和密码':'Please enter SSID and password';";
  html += "if(!s||!p){showStatus(msg,'error');return}";
  html += "const fd=new FormData();fd.append('ssid',s);fd.append('password',p);fd.append('timezone',tz);";
  html += "fetch('/api/config',{method:'POST',body:fd}).then(r=>r.json()).then(d=>{";
  html += "const saveMsg=lang==='zh'?'WiFi设置已保存，正在连接...':'WiFi saved, connecting...';";
  html += "showStatus(saveMsg,d.status==='ok'?'success':'error');";
  html += "if(d.status==='ok')setTimeout(()=>location.reload(),2000)})";
  html += ".catch(e=>showStatus('Error: '+e,'error'))}";
  html += "document.getElementById('timezone').addEventListener('change',e=>{";
  html += "document.getElementById('tzDisplay').textContent=e.target.value});";
  html += "setInterval(updateTime,1000);loadData();updateTime();";
  html += "document.getElementById('scheduleModal').addEventListener('click',e=>{";
  html += "if(e.target.id==='scheduleModal')closeModal()});";
  html += "</script></body></html>"; 
  return html;
}

//...................................................................
// End of file
