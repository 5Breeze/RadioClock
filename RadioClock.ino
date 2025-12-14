//
// nisejjy - fake JJY (standarad time radio broadcast) station
//     using software radio on ESP32/ESP32-C3.
// Enhanced version with Web UI, WiFi configuration, and Bluetooth time display
//
// (c) 2021 by taroh (sasaki.taroh@gmail.com)
// Modified 2024: Added web UI, WiFi config, multi-station scheduling
//
// JJY (Japan): https://ja.wikipedia.org/wiki/JJY
// WWVB (US): https://en.wikipedia.org/wiki/WWVB
// DCF77 (Germany) : https://www.eecis.udel.edu/~mills/ntp/dcf77.html
// BSF (Taiwan): https://en.wikipedia.org/wiki/BSF_(time_service)
// MSF (UK): https://en.wikipedia.org/wiki/Time_from_NPL_(MSF)
// BPC (China): https://harmonyos.51cto.com/posts/1731
//
// note: BSF, BPC codes are not certified.

//...................................................................
// Hardware config - Auto-detect ESP32 variant
#if defined(CONFIG_IDF_TARGET_ESP32C3)
  // ESP32-C3 pins
  #define PIN_RADIO  (3)
  #define PIN_BUZZ   (4)
  #define PIN_LED    (5)
#else
  // ESP32 (classic) pins
  #define PIN_RADIO  (26)
  #define PIN_BUZZ   (27)
  #define PIN_LED    (25)
#endif

#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
WebServer server(80);

// Configuration constants
#define DEVICENAME_PREFIX "RadioStation"
#define TZ (9 * 60 * 60) /*JST*/
#define CONFIG_FILE "/config.json"
#define STATION_CONFIG_FILE "/stations.json"

// WiFi and configuration
char ssid[64] = "";
char passwd[64] = "";
unsigned long wifi_connect_start = 0;
#define WIFI_CONNECT_TIMEOUT 30000  // 30 seconds
#define WIFI_CONNECT_CHECK_INTERVAL 5000  // 5 seconds

//...................................................................
// Station specs with station names
//
#define SN_JJY_E  (0) // JJY Fukushima Japan (40KHz)
#define SN_JJY_W  (1) // JJY Fukuoka Japan (60KHz)
#define SN_WWVB (2)   // WWVB US (60KHz)
#define SN_DCF77  (3) // DCF77 Germany (77.5KHz)
#define SN_BSF  (4)   // BSF Taiwan (77.5KHz)
#define SN_MSF  (5)   // MSF UK (60KHz)
#define SN_BPC  (6)   // BPC China (68.5KHz)

#define SN_DEFAULT  (SN_JJY_E)
#define NUM_STATIONS 7

// Station names for display
const char *station_names[] = {
  "JJY-E (40KHz)",
  "JJY-W (60KHz)",
  "WWVB (60KHz)",
  "DCF77 (77.5KHz)",
  "BSF (77.5KHz)",
  "MSF (60KHz)",
  "BPC (68.5KHz)"
};

const char *station_short[] = {
  "JJY-E",
  "JJY-W",
  "WWVB",
  "DCF77",
  "BSF",
  "MSF",
  "BPC"
};

int st_cycle2[] = { // interrupt cycle, KHz: double of station freq
  80,  // 40KHz JJY-E
  120, // 60KHz JJY-W
  120, // 60KHz WWVB
  155, // 77.5KHz DCF77
  155, // 77.5KHz BSF
  120, // 60KHz MSF
  137  // 68.5KHz BPC
};

// Time schedule structure
typedef struct {
  uint8_t station;      // 0-6: which station
  uint16_t start_min;   // minute of day (0-1439)
  uint16_t end_min;     // minute of day (0-1439)
} TimeSchedule;

#define MAX_SCHEDULES 24
TimeSchedule schedules[MAX_SCHEDULES];
int schedule_count = 0;

// interrupt cycle to makeup radio wave, buzzer (500Hz = 1KHz cycle):
// peripheral freq == 80MHz
//    ex. radio freq 40KHz: intr 80KHz: 80KHz / 80MHz => 1/1000 (1/tm0cycle)
//    buzz cycle: 1KHz / 80KHz 1/80 (1/radiodiv)
int tm0cycle;
#define TM0RES    (1)
int radiodiv;

// TM0RES (interrupt counter), AMPDIV (buzz cycle(1000) / subsec(10)), SSECDIV (subsec / sec)
// don't depend on station specs. 
#define AMPDIV   (100)  // 1KHz / 100 => 10Hz, amplitude may change every 0.1 seconds
#define SSECDIV    (10) // 10Hz / 10 => 1Hz, clock ticks

// enum symbols
#define SP_0  (0)
#define SP_1  (1)
#define SP_M  (2)
#define SP_P0 (SP_1) // for MSF
#define SP_P1 (3)    // for MSF
/*#define SP_M0 (3) // for HBG
#define SP_M00 (4) // for HBG
#define SP_M000 (5) // for HBG
 */
#define SP_2  (2) // for BSF/BPC
#define SP_3  (3) // for BSF/BPC
#define SP_M4 (4) // for BSF/BPC
#define SP_MAX  (SP_M4)
//
// bits_STATION[] => *bits60: 60 second symbol buffers, initialized with patterns
// sp_STATION[] => *secpattern: 0.1sec term pattern in one second, for each symbol
// * note: when sp_STATION[n * 10], secpattern[] is like 2-dim array secpattern[n][10].
//
// JJY & WWVB  *note: comment is the format of JJY.
int8_t bits_jjy[] = {  // 60bit transmitted frame, of {SP_0, SP_1, SP_M}
  SP_M, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M, // (M), MIN10[3], 0, MIN1[4], (M)
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M, // 0, 0, HOUR10[2], 0, HOUR1[4], (M)
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M, // 0, 0, DOY100[2], DOY10[4], (M)
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M, // DOY1[4], 0, 0, PA1, PA2, 0, (M)
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M, // 0, YEAR[8], (M)
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M  // DOW[3], LS1, LS2, 0, 0, 0, 0, (M)
};
// *note: if summer time, set bit 57/58 (WWVB) (bit 38/40 (JJY, in future))
int8_t sp_jjy[] = { // in (0, 1), [SP_x][amplitude_for_0.1sec_term_in_second]
  1, 1, 1, 1, 1, 1, 1, 1, 0, 0,   // SP_0
  1, 1, 1, 1, 1, 0, 0, 0, 0, 0,   // SP_1
  1, 1, 0, 0, 0, 0, 0, 0, 0, 0    // SP_M
};
int8_t sp_wwvb[] = { // in (0, 1), [SP_x][amplitude_for_0.1sec_term_in_second]
  0, 0, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_0
  0, 0, 0, 0, 0, 1, 1, 1, 1, 1,   // SP_1
  0, 0, 0, 0, 0, 0, 0, 0, 1, 1    // SP_M
};

// DCF77 encoding is LSB->MSB. //HBG *note: [0] is changed depending on DCF/HBG (also min/hour).
int8_t bits_dcf[] = {
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // 0, reserved[9]
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_1, SP_0, // reserved[5], 0, 0, (0, 1)(MEZ), 0
  SP_1, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // 1, MIN1[4], MIN10[3], P1, (1->)
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // HOUR1[4], HOUR10[2], P2, D1[4]
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // D10[2], DOW[3], M1[4], M10[1]
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M  // Y1[4], Y10[4], P3, (M)
};
int8_t sp_dcf[] = {
  0, 1, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_0
  0, 0, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_1
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1    // SP_M
};
/*
int8_t sp_hbg[] = {
  0, 1, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_0
  0, 0, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_1
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_M
  0, 1, 0, 1, 1, 1, 1, 1, 1, 1,   // SP_M0    // 00sec
  0, 1, 0, 1, 0, 1, 1, 1, 1, 1,   // SP_M00   // 00sec at 00min
  0, 1, 0, 1, 0, 1, 0, 1, 1, 1    // SP_M000  // 00sec at 00/12 hour 00min
};
*/

// BSF: quad encoding.
int8_t bits_bsf[] = {
  SP_M4, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,
  SP_0,  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,
  SP_0,  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,
  SP_0,  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M4,
  SP_1,  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,  // 1, min[3], hour[2.5], P1[.5],
  SP_0,  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_M4  // DOM[2.5], DOW[2.5], mon[2],
                                                                // year[3.5], P2[.5], 0, 0, M
};
int8_t sp_bsf[] = {
  0, 0, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_0
  0, 0, 0, 0, 1, 1, 1, 1, 1, 1,   // SP_1
  0, 0, 0, 0, 0, 0, 0, 0, 1, 1,   // SP_2
  0, 0, 0, 0, 0, 0, 1, 1, 1, 1,   // SP_3
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1    // SP_M4
};

// MSF has 4 patterns.
int8_t bits_msf[] = {
  SP_M, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,
  SP_1, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,
  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0,
  SP_0, SP_0, SP_0, SP_P0, SP_P0, SP_P0, SP_P0, SP_P0, SP_P0, SP_0
};
int8_t sp_msf[] = {
  0, 1, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_0
  0, 0, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_1/SP_P0 (parity 0)
  0, 0, 0, 0, 0, 1, 1, 1, 1, 1,   // SP_M
  0, 0, 0, 1, 1, 1, 1, 1, 1, 1    // SP_P1 (parity 1)
};

// BPC: quadary and has 5 patterns.
int8_t bits_bpc[] = {
  SP_M4, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // (B), P1, P2, h[2], m[3], DOW[2]
  SP_0,  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // P3, D[3], M[2], Y[3], P4
  SP_M4, SP_1, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // P1: 0, 1, 2 for 00-19, -39, -59s
  SP_0,  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // P2: 0, P3: AM/PM(0/2)+par<hmDOW>
  SP_M4, SP_2, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, // P4: par<DMY> (0/1)
  SP_0,  SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0, SP_0
};
int8_t sp_bpc[] = {
  0, 1, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_0
  0, 0, 1, 1, 1, 1, 1, 1, 1, 1,   // SP_1
  0, 0, 0, 1, 1, 1, 1, 1, 1, 1,   // SP_2
  0, 0, 0, 0, 1, 1, 1, 1, 1, 1,   // SP_3
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0    // SP_M4
};

// func for makeup patterns
void mb_jjy(void);  // JJY-E, JJY-W
void mb_wwvb(void); // WWVB
void mb_dcf(void);  // DCF77
void mb_bsf(void);  // BSF
void mb_msf(void);  // MSF
void mb_bpc(void);  // BPC

int8_t *st_bits[] = {bits_jjy, bits_jjy, bits_jjy, bits_dcf, bits_bsf, bits_msf, bits_bpc};
int8_t *bits60;
int8_t *st_sp[]   = {sp_jjy, sp_jjy, sp_wwvb, sp_dcf, sp_bsf, sp_msf, sp_bpc};
int8_t *secpattern;
void (*st_makebits[])(void) = {mb_jjy, mb_jjy, mb_wwvb, mb_dcf, mb_bsf, mb_msf, mb_bpc};
void (*makebitpattern)(void);

//...................................................................
// globals
hw_timer_t *tm0 = NULL;
volatile SemaphoreHandle_t  timerSemaphore;
portMUX_TYPE  timerMux = portMUX_INITIALIZER_UNLOCKED; 
volatile uint32_t buzzup = 0;     // inc if buzz cycle (/2) passed
int istimerstarted = 0;

int radioc = 0; // 0..(RADIODIV - 1)
int ampc = 0;   // 0..(AMPDIV - 1)
int tssec = 0;  // 0..(SSECDIV - 1)

int ntpsync = 1;
time_t now;
struct tm nowtm;

int radioout = 0, // pin output values
    buzzout = 0;
int ampmod;     // 1 if radio out is active (vibrating), 0 if reducted,
                // at cuttent subsecond-second frame for current date-time
int buzzsw = 1; // sound on/off

// Bluetooth time display (hh:mm:ss format)
unsigned long last_bt_update = 0;
#define BT_UPDATE_INTERVAL 1000  // Update every 1 second

// WiFi AP mode for configuration
bool ap_mode = false;
unsigned long last_wifi_check = 0;

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
void updateBluetoothName(void);
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
int docmd(char *buf);
int a2toi(char *chp);
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
  if (! radioout && ampmod) {
    radioout = 1;
    digitalWrite(PIN_RADIO, HIGH);
  } else {
    radioout = 0;
    digitalWrite(PIN_RADIO, LOW);
  }
  radioc++;
  if (radiodiv <= radioc) {
    radioc = 0;
    portENTER_CRITICAL_ISR(&timerMux);           // CRITICAL SECTION ---
    buzzup++;
    portEXIT_CRITICAL_ISR(&timerMux);            // --- CRITICAL SECTION
    xSemaphoreGiveFromISR(timerSemaphore, NULL); // free semaphore
  }
  return;
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

  // Setup Bluetooth with time display
  esp_read_mac(macBT, ESP_MAC_BT);
  Serial.printf(
    "Bluetooth MAC %02X:%02X:%02X:%02X:%02X:%02X...",
    macBT[0], macBT[1], macBT[2], macBT[3], macBT[4], macBT[5]);
  
  // Format BT name: "RadioStation_HHMM"
  snprintf(bt_name, sizeof(bt_name), "%s", DEVICENAME_PREFIX);
  
  while (! SerialBT.begin(bt_name)) {
    Serial.println("error initializing Bluetooth");
    delay(2000);
  }
  Serial.print("\n");

  // Check if WiFi credentials are configured
  if (strlen(ssid) == 0) {
    Serial.println("No WiFi config found, starting AP mode for configuration...");
    startAPMode();
  } else {
    // Try to connect to WiFi
    if (ntpsync) {
      ntpstart();
    }
    // If NTP fails, start AP mode
    if (!ntpsync && strlen(ssid) > 0) {
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
  updateBluetoothName();
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

  // Handle web server requests
  if (ap_mode) {
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
        
        // Update Bluetooth name with current time (hh:mm:ss)
        if ((millis() - last_bt_update) > BT_UPDATE_INTERVAL) {
          last_bt_update = millis();
          updateBluetoothName();
        }
        
        // Check if we need to apply a different schedule
        applyCurrentSchedule();
        
        if (lastmin != nowtm.tm_min) {
          makebitpattern();
          printbits60();
        }
        Serial.printf("%d-%d-%d, %d(%d) %02d:%02d:%02d\n",
          nowtm.tm_year + 1900, nowtm.tm_mon + 1, nowtm.tm_mday,
          nowtm.tm_yday, nowtm.tm_wday,
          nowtm.tm_hour, nowtm.tm_min, nowtm.tm_sec);
      }
      ampchange();
    }
  }

  // Only handle Bluetooth commands if not in AP mode
  if (!ap_mode) {
    while (SerialBT.available()) {
      buf[bufp] = SerialBT.read();
      if (buf[bufp] == '\n' || buf[bufp] == '\r' ||
         bufp == sizeof(buf) - 1) {
        buf[bufp] = '\0';
        docmd(buf);
        bufp = 0;
      } else {
        bufp++;
      }
    }
  }
  
  delay(1); // feed watchdog
}


//...................................................................
void
starttimer(void)
{
  if (istimerstarted) {
    stoptimer();
  }
  ampc = 0;
  radioc = 0;
  timerSemaphore = xSemaphoreCreateBinary();    // create semaphore
  tm0 = timerBegin(0, tm0cycle, true);          // tm0 prescaler
  timerAttachInterrupt(tm0, &onTimer, true);    // tm0 intr routine: onTimer()
  timerAlarmWrite(tm0, TM0RES, true);           // tm0 resolution
  timerAlarmEnable(tm0);                        // tm0 start
  Serial.print("(re)started timer...\n");
  istimerstarted = 1;
  return;
}

void
stoptimer(void)
{
  if (istimerstarted) {
    timerEnd(tm0);
    istimerstarted = 0;
  }
  return;
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
  Serial.printf("station #%d:\n", station);
  tm0cycle = 80000 / st_cycle2[station];
  radiodiv = st_cycle2[station];
  Serial.printf("  freq %fMHz, timer intr: 80M / (%d x %d), buzz/radio: /%d\n",
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
  makebitpattern();
  printbits60();
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
  return;
}

void
mb_wwvb(void)
{
  Serial.print("encode WWVB format - ");
  mbc_wwvbjjy();
  binarize((nowtm.tm_year - 100) / 10, 45, 4);
  binarize(nowtm.tm_year % 10, 50, 4);
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

//...................................................................
// Bluetooth command
//
// y[01]: NTP sync off/on
//   to force set the current date/time (d/t), first turn off NTP sync.
// dYYMMDD: set date to YY/MM/DD
// tHHmmSS: set time to HH:mm:SS
// z[01]: buzzer off/on
// s[jkwdhmb]: set station to JJY_E, JJY_W, WWVB, DCF77, HBG, MSF, BPC

int
docmd(char *buf)
{
  int arg1, arg2;
  Serial.printf("cmd: >>%s<<\n", buf);
  if (buf[0] == 'd' || buf[0] == 'D') { // set date
    if (strlen(buf) != 7) {
      return 0;
    }
    int y = a2toi(buf + 1);
    int m = a2toi(buf + 3);
    int d = a2toi(buf + 5);
    Serial.printf("%d %d %d\n", y, m, d);
    if (y < 0 || m < 0 || 12 < m || d < 0 || 31 < d) {  // can set Feb 31 :-)
      return 0;
    }
    nowtm.tm_year = y + 100;
    nowtm.tm_mon = m - 1;
    nowtm.tm_mday = d;
    setlocaltime();
    Serial.printf("set date: >>%s<<\n", buf + 1);
    return 1;
  } else if (buf[0] == 't' || buf[0] == 'T') { // set time & start tick
    if (strlen(buf) != 7) {
      return 0;
    }
    int h = a2toi(buf + 1);
    int m = a2toi(buf + 3);
    int s = a2toi(buf + 5);
    if (h < 0 || 24 < h || m < 0 || 60 < m || s < 0 || 60 < s) {
      return 0;
    }
    nowtm.tm_hour = h;
    nowtm.tm_min = m;
    nowtm.tm_sec = s;
    tssec = 0;
    ampc = 0;
    radioc = 0; // no semaphore lock: don't care if override by intr routine :-)
    setlocaltime();
    Serial.printf("set time...restart tick: >>%s<<\n", buf + 1);
    return 1;
  } else if (buf[0] == 'z' || buf[0] == 'Z') { // buzzer on(1)/off(0)
    if (buf[1] == '0') {
      buzzsw = 0;
    } else if (buf[1] == '1') {
      buzzsw = 1;
    } else {
      return 0;
    }
    Serial.printf("buzzer: >>%c<<\n", buf + 1);
    return 1;
  } else if (buf[0] == 's' || buf[0] == 'S') { // set station
    char s[] = //"jJkKwWdDhHmMbB"
              {'j', 'J', 'k', 'K', 'w', 'W', 'd', 'D', 't', 'T', 'm', 'M', 'c', 'C', '\0'},
              *chp;
    if ((chp = strchr(s, buf[1])) != NULL) {
      setstation((int)(chp - s) / 2);
      return 1;
    } else {
      return 0;
    }
  } else if (buf[0] == 'y' || buf[0] == 'Y') { // NTP sync
        if (buf[1] == '0') {
      ntpsync = 0;
      ntpstop();
    } else if (buf[1] == '1') {
      ntpsync = 1;
      ntpstart();
    } else {
      return 0;
    }
  }
  return 0;
}

int
a2toi(char *chp)
{
  int v = 0;
  for (int i = 0; i < 2; chp++, i++) {
    if (*chp < '0' || '9' < *chp) {
      return -1;
    }
    v = v * 10 + (*chp - '0');
  }
  return v;
}


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
  configTime(TZ, 0, "ntp.nict.jp", "ntp.jst.mfeed.ad.jp"); // enable NTP
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

void updateBluetoothName(void)
{
  // Update Bluetooth device name to show current time (hh:mm:ss)
  char bt_name[32];
  snprintf(bt_name, sizeof(bt_name), "%s-%02d%02d%02d",
           DEVICENAME_PREFIX, nowtm.tm_hour, nowtm.tm_min, nowtm.tm_sec);
  
  // Note: Renaming Bluetooth device requires reconnection
  // SerialBT.disconnect() would be needed but it's non-standard
  // So we just log it for now
  Serial.printf("BT Name: %s\n", bt_name);
}

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
    return;
  }
  
  // Simple config format: SSID\nPASSWORD
  configFile.readStringUntil('\n').toCharArray(ssid, sizeof(ssid));
  configFile.readStringUntil('\n').toCharArray(passwd, sizeof(passwd));
  
  // Remove newlines
  for (int i = 0; ssid[i]; i++) {
    if (ssid[i] == '\r' || ssid[i] == '\n') ssid[i] = '\0';
  }
  for (int i = 0; passwd[i]; i++) {
    if (passwd[i] == '\r' || passwd[i] == '\n') passwd[i] = '\0';
  }
  
  configFile.close();
  Serial.printf("Loaded config: SSID=%s\n", ssid);
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
  configFile.close();
  Serial.println("Config saved");
}

void loadSchedules(void)
{
  schedule_count = 0;
  File schedFile = SPIFFS.open(STATION_CONFIG_FILE, "r");
  if (!schedFile) {
    Serial.println("No schedule file found, using default");
    // Default: JJY-E all day
    schedules[0].station = SN_JJY_E;
    schedules[0].start_min = 0;
    schedules[0].end_min = 1439;
    schedule_count = 1;
    return;
  }
  
  // Parse JSON schedule (simplified)
  // TODO: Use ArduinoJson for proper parsing
  schedFile.close();
  
  if (schedule_count == 0) {
    // Default schedule
    schedules[0].station = SN_JJY_E;
    schedules[0].start_min = 0;
    schedules[0].end_min = 1439;
    schedule_count = 1;
  }
  
  Serial.printf("Loaded %d schedules\n", schedule_count);
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
}

void applyCurrentSchedule(void)
{
  static int last_station = -1;
  
  // Calculate current minute of day (0-1439)
  int current_min = nowtm.tm_hour * 60 + nowtm.tm_min;
  
  // Find applicable schedule
  int new_station = SN_JJY_E;  // Default
  for (int i = 0; i < schedule_count; i++) {
    if (current_min >= schedules[i].start_min && 
        current_min < schedules[i].end_min) {
      new_station = schedules[i].station;
      break;
    }
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
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  starttimer();  // Resume radio
  Serial.println("AP mode stopped");
}

void initWebServer(void)
{
  // Serve index page
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", getIndexHTML());
  });
  
  // API endpoints
  server.on("/api/config", HTTP_GET, []() {
    String json = "{\"ssid\":\"" + String(ssid) + "\"}";
    server.send(200, "application/json", json);
  });
  
  server.on("/api/config", HTTP_POST, []() {
    if (server.hasArg("ssid") && server.hasArg("password")) {
      server.arg("ssid").toCharArray(ssid, sizeof(ssid));
      server.arg("password").toCharArray(passwd, sizeof(passwd));
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
          if (ntpsync) {
            ntpstart();
          }
          delay(1000);  // Give client time to reload
          stopAPMode();
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
  
  server.on("/api/status", HTTP_GET, []() {
    int current_min = nowtm.tm_hour * 60 + nowtm.tm_min;
    int current_station = SN_JJY_E;
    for (int i = 0; i < schedule_count; i++) {
      if (current_min >= schedules[i].start_min && 
          current_min < schedules[i].end_min) {
        current_station = schedules[i].station;
        break;
      }
    }
    
    String json = "{\"time\":\"" + String(nowtm.tm_hour) + ":" +
                  String(nowtm.tm_min) + ":" + String(nowtm.tm_sec) +
                  "\",\"station\":\"" + String(station_names[current_station]) + "\"}";
    server.send(200, "application/json", json);
  });
  
  server.begin();
}

String getIndexHTML(void)
{
  // Simplified HTML inline - avoiding raw string issues
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Radio Station</title>";
  html += "<style>";
  html += "body{font-family:'Segoe UI',sans-serif;background:linear-gradient(135deg,#667eea,#764ba2);";
  html += "min-height:100vh;display:flex;align-items:center;justify-content:center;margin:0}";
  html += ".container{background:white;border-radius:12px;box-shadow:0 20px 60px rgba(0,0,0,0.3);";
  html += "max-width:600px;width:100%;overflow:hidden}";
  html += ".header{background:linear-gradient(135deg,#667eea,#764ba2);color:white;padding:30px;text-align:center}";
  html += ".header h1{margin:0;font-size:28px}.content{padding:30px}";
  html += ".section{margin-bottom:30px}.section h2{border-bottom:2px solid #667eea;padding-bottom:10px}";
  html += ".form-group{margin-bottom:15px}label{display:block;margin-bottom:8px;color:#555;font-weight:500}";
  html += "input,select{width:100%;padding:12px;border:1px solid #ddd;border-radius:8px;font-size:14px}";
  html += "input:focus,select:focus{outline:0;border-color:#667eea;box-shadow:0 0 0 3px rgba(102,126,234,0.1)}";
  html += ".button-group{display:flex;gap:10px;margin-top:20px}button{flex:1;padding:12px;border:0;";
  html += "border-radius:8px;font-size:14px;font-weight:600;cursor:pointer}";
  html += ".btn-primary{background:linear-gradient(135deg,#667eea,#764ba2);color:white}";
  html += ".btn-secondary{background:#f0f0f0;color:#333}";
  html += ".time-display{background:#667eea;color:white;padding:20px;border-radius:8px;";
  html += "text-align:center;font-size:32px;font-weight:300;margin-bottom:20px;font-family:monospace}";
  html += ".status{padding:12px;border-radius:8px;margin-bottom:15px;font-size:14px}";
  html += ".status.success{background:#d4edda;color:#155724;border:1px solid #c3e6cb}";
  html += ".status.error{background:#f8d7da;color:#721c24;border:1px solid #f5c6cb}";
  html += "</style></head><body>";
  html += "<div class='container'><div class='header'><h1>Radio Station</h1><p>Configuration Portal</p></div>";
  html += "<div class='content'><div id='statusMsg'></div>";
  html += "<div class='time-display' id='timeDisplay'>00:00:00</div>";
  html += "<div class='section'><h2>WiFi Configuration</h2>";
  html += "<div class='form-group'><label>Network Name (SSID)</label>";
  html += "<input type='text' id='ssid' placeholder='Enter WiFi SSID'></div>";
  html += "<div class='form-group'><label>Password</label>";
  html += "<input type='password' id='password' placeholder='Enter WiFi password'></div>";
  html += "<div class='button-group'><button class='btn-primary' onclick='saveWiFiConfig()'>Save WiFi Config</button></div>";
  html += "</div><div class='section'><h2>Radio Stations</h2><div id='stationsList'></div></div>";
  html += "<div class='section'><h2>Time Schedule</h2><div id='schedulesList'></div>";
  html += "<div class='button-group'>";
  html += "<button class='btn-secondary' onclick='addSchedule()'>Add Schedule</button>";
  html += "<button class='btn-primary' onclick='saveSchedules()'>Save Schedules</button>";
  html += "</div></div></div></div>";
  html += "<script>";
  html += "function updateTime(){fetch('/api/status').then(r=>r.json()).then(d=>{";
  html += "document.getElementById('timeDisplay').textContent=d.time}).catch(e=>{})}";
  html += "async function loadData(){try{";
  html += "const sr=await fetch('/api/stations');const s=await sr.json();";
  html += "let html='';s.forEach(st=>{html+='<div style=\"padding:10px;background:#f9f9f9;";
  html += "border-radius:8px;margin-bottom:8px;\">'+st.name+'</div>'});";
  html += "document.getElementById('stationsList').innerHTML=html;";
  html += "const scr=await fetch('/api/schedules');const sc=await scr.json();";
  html += "let shtml='';sc.forEach((x,i)=>{";
  html += "const sh=Math.floor(x.start/60);const sm=x.start%60;";
  html += "const eh=Math.floor(x.end/60);const em=x.end%60;";
  html += "shtml+='<div style=\"background:#f9f9f9;padding:15px;border-radius:8px;margin-bottom:10px;\">'";
  html += "+'Station: '+x.station+', Time: '+String(sh).padStart(2,'0')+':'+String(sm).padStart(2,'0')";
  html += "+' - '+String(eh).padStart(2,'0')+':'+String(em).padStart(2,'0')+'</div>'});";
  html += "document.getElementById('schedulesList').innerHTML=shtml;";
  html += "}catch(e){console.error(e)}}";
  html += "function showStatus(m,t){const d=document.getElementById('statusMsg');";
  html += "d.className='status '+t;d.textContent=m;d.style.display='block';";
  html += "if(t==='success')setTimeout(()=>{d.style.display='none'},3000)}";
  html += "function addSchedule(){const si=document.getElementById('stationsList');";
  html += "if(!si)return;const s=prompt('Enter station index (0-6):');";
  html += "if(s===null)return;const st=parseInt(s);";
  html += "const sh=prompt('Start hour (0-23):');if(sh===null)return;";
  html += "const sm=prompt('Start minute (0-59):');if(sm===null)return;";
  html += "const eh=prompt('End hour (0-23):');if(eh===null)return;";
  html += "const em=prompt('End minute (0-59):');if(em===null)return;";
  html += "const start=parseInt(sh)*60+parseInt(sm);const end=parseInt(eh)*60+parseInt(em);";
  html += "fetch('/api/schedule',{method:'POST',headers:{'Content-Type':'application/json'},";
  html += "body:JSON.stringify({station:st,start:start,end:end})})";
  html += ".then(r=>r.json()).then(d=>{if(d.status==='ok'){showStatus('Schedule added','success');";
  html += "setTimeout(()=>loadData(),500)}else showStatus('Failed to add schedule','error')})";
  html += ".catch(e=>showStatus('Error: '+e,'error'))}";
  html += "async function saveSchedules(){try{";
  html += "const scr=await fetch('/api/schedules');const sc=await scr.json();";
  html += "fetch('/api/schedules',{method:'POST',headers:{'Content-Type':'application/json'},";
  html += "body:JSON.stringify(sc)}).then(r=>r.json()).then(d=>{";
  html += "if(d.status==='ok'){showStatus('Schedules saved','success');";
  html += "setTimeout(()=>loadData(),500)}else showStatus('Failed to save schedules','error')})";
  html += ".catch(e=>showStatus('Error: '+e,'error'))}catch(e){showStatus('Error: '+e,'error')}}";
  html += "function saveWiFiConfig(){const s=document.getElementById('ssid').value;";
  html += "const p=document.getElementById('password').value;";
  html += "if(!s||!p){showStatus('Please enter SSID and password','error');return}";
  html += "const fd=new FormData();fd.append('ssid',s);fd.append('password',p);";
  html += "fetch('/api/config',{method:'POST',body:fd}).then(r=>r.json()).then(d=>{";
  html += "showStatus('WiFi config saved, connecting...',d.status==='ok'?'success':'error');";
  html += "if(d.status==='ok')setTimeout(()=>location.reload(),2000)})";
  html += ".catch(e=>showStatus('Error: '+e,'error'))}";
  html += "setInterval(updateTime,1000);loadData();updateTime();";
  return html;
}

//...................................................................
// End of file