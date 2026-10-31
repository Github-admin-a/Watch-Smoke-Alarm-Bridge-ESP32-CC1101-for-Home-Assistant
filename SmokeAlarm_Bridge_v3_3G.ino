/*
  SmokeAlarm_Bridge_v3_3.ino  v3.3
  ESP32 + CC1101 bridge for 433.25 MHz RF-interconnected smoke alarms.

  - Receives the alarm system's 2-FSK packets (100 kbaud, sync 2F33, 8-byte frame)
  - Classifies frames as ALARM / SILENCE / unknown by system ID + command code. The wall panel's
    test frame is identical to what the detectors send and can't be told apart when it arrives,
    so every received alarm/test frame is an ALARM (any alarm could go off at any time).
  - Logs ANY frame that repeats (other IDs / unknown commands too) so new devices show up
  - Device names: map frame hex (or a prefix) to a name, e.g. which detector triggered
  - Logs every event with time + RSSI to the web page (kept in LittleFS, survives reboot)
  - Publishes to MQTT with Home Assistant discovery (smoke binary sensor, last event, RSSI)
  - HA "Event log" sensor (last 20 events as attributes, for a dashboard table) and an
    HA event entity "Smoke alarm events" (history + automation trigger)
  - Web pages: status + log, radio/code setup, Wi-Fi/MQTT setup
  - Main page shows one row per button press; UNKNOWN frames, bursts and radio stats are
    on the Diagnostics page
  - Burst log: every separate transmission (one per press, ~7 s) with duration, copies, RSSI and
    bit-corrected copies (Diagnostics page)
  - Scanner page: sweeps a frequency range and shows where signals appear (to find the
    smoke detectors' own channel) plus a raw capture that decodes any FSK signal's bit rate,
    preamble and frame. Alarm monitoring pauses while either runs.
  - Receives both the wall controller (~433.25 MHz) and the smoke detectors (~433.35 MHz):
    centred on 433.30 MHz with a 325 kHz window and automatic frequency correction
  - Records each event's frequency offset - every transmitter's crystal is slightly different,
    so the offset shows which device sent it. The Transmitters page learns devices with a
    Learn button and lets you rename / delete them.
  - Optional TEST / SILENCE transmit buttons (off by default, enable on the setup page)
  - Home Assistant "Clear event log" button (MQTT button entity) clears the log, same as the web page

  Libraries (Arduino Library Manager):
    WiFiManager (tzapu), PubSubClient (Nick O'Leary), ArduinoJson (v7, Benoit Blanchon)
  Boards (pins are picked automatically from the board selected in the IDE):

  ESP32-C3 Super Mini  (Tools > Board: "ESP32C3 Dev Module", USB CDC On Boot: Enabled)
    CC1101 VCC  -> 3V3      CC1101 GND  -> GND
    CC1101 SCK  -> GPIO4    CC1101 MISO -> GPIO5
    CC1101 MOSI -> GPIO6    CC1101 CSN  -> GPIO7
    CC1101 GDO0 -> GPIO3 (needed only for the Scanner page's raw capture)

  ESP32 Dev Module (ESP32-WROOM-32)
    CC1101 SCK  -> GPIO14   CC1101 MISO -> GPIO25
    CC1101 MOSI -> GPIO26   CC1101 CSN  -> GPIO13
    CC1101 GDO0 -> GPIO32 (needed only for the Scanner page's raw capture)

  First boot: join Wi-Fi AP "SmokeAlarmBridge-Setup" to enter your Wi-Fi.
  Then browse to http://smokealarm.local/ (or the IP shown on serial).
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <SPI.h>
#include <time.h>

#define FW_VERSION "3.3"

// ---------------- pins ----------------
#if defined(CONFIG_IDF_TARGET_ESP32C3)      // ESP32-C3 Super Mini
  #define PIN_SCK   4
  #define PIN_MISO  5
  #define PIN_MOSI  6
  #define PIN_CSN   7
  #define PIN_GDO0  3
  #define BOARD_NAME "ESP32-C3 Super Mini"
#else                                        // classic ESP32 (WROOM-32 dev board)
  #define PIN_SCK   14
  #define PIN_MISO  25
  #define PIN_MOSI  26
  #define PIN_CSN   13
  #define PIN_GDO0  32
  #define BOARD_NAME "ESP32"
#endif

// ---------------- configuration ----------------
struct Config {
  String devName   = "Smoke Alarm Bridge";
  String devId     = "smoke_alarm_bridge";      // HA identity (independent of MAC)
  String hostname  = "smokealarm";
  String mqttHost  = "";
  uint16_t mqttPort = 1883;
  String mqttUser  = "";
  String mqttPass  = "";
  String baseTopic = "smokealarm";
  String tz        = "UTC0";                     // POSIX TZ - set yours on the Wi-Fi / MQTT page
  float  freqMHz   = 433.30;                     // between controller (433.25) and detectors (~433.35)
  uint16_t rxBwKHz = 325;                        // receive bandwidth
  String syncHex   = "2F33";
  String sysIdHex  = "0000000000";               // first bytes of the 8-byte frame - replace with your system's ID (see Diagnostics page)
  String alarmHex  = "";                         // real smoke alarm code - blank until captured
  String silenceHex= "0E6580";
  String testHex   = "0AA9C8";                   // frame sent by the wall panel TEST button and by the detectors (received as ALARM)
  uint8_t minRepeats = 3;                        // identical frames needed to accept
  uint8_t bitTolerance = 2;                      // frame bits allowed to differ from a known frame
  uint8_t unknownRepeats = 20;                   // copies needed before an UNKNOWN frame is logged
  uint16_t alarmAutoOff = 120;                   // s after last alarm frame -> OFF
  uint16_t pressGapMs = 500;                     // silence in transmission longer than this = a new press
  uint16_t minPressMs = 7500;                    // ...but never sooner than this after the previous press started
  bool   txEnabled = false;
  float  txSeconds = 3.0;
  String devicesText = "";                       // lines of FRAMEHEX=Name, e.g. "<system ID>0E6580=Wall controller"
  String txNamesText = "";                       // lines of OFFSET=Name, e.g. "-1=Entry smoke alarm"
  uint8_t offsetTol = 8;                         // kHz either side of a transmitter's offset
} cfg;

const char* CFG_FILE = "/config.json";
bool configMigrated = false;
const char* LOG_FILE = "/log.json";
const int FRAME_LEN = 8;
// The last bit of the frame toggles within one burst (0E6580 / 0E6581), so ignore it
const uint8_t LAST_BYTE_MASK = 0xFE;

// ---------------- globals ----------------
WebServer server(80);
WiFiClient netClient;
PubSubClient mqtt(netClient);
WiFiManager wm;

uint8_t sysId[FRAME_LEN], codeAlarm[FRAME_LEN], codeSilence[FRAME_LEN], codeTest[FRAME_LEN];
int sysIdLen = 5, codeLen = 3;
bool testCodeSet = false, alarmCodeSet = false;

#define MAX_DEVICES 16
struct DeviceName { uint8_t prefix[FRAME_LEN]; int len; String name; } devices[MAX_DEVICES];
int deviceCount = 0;

struct RxFrame { uint8_t d[FRAME_LEN]; int16_t rssi; int16_t foff; uint32_t ms; };   // foff = freq offset, kHz
QueueHandle_t rxQueue;

volatile bool radioOk = false;
volatile bool radioReconfig = false;
volatile int  txRequest = 0;          // 0 none, 1 test, 2 silence
volatile bool txBusy = false;
volatile uint32_t radioFrames = 0, radioOverflows = 0, radioResyncs = 0;

uint32_t framesOwn = 0, framesOther = 0, bitFixes = 0;
String lastOtherFrame = "-", lastUnknownCode = "-";

enum EvType { EV_ALARM = 0, EV_SILENCE, EV_TEST, EV_UNKNOWN, EV_COUNT };
const char* EV_NAMES[]  = {"alarm", "silence", "test", "unknown"};            // machine names (MQTT event_type)
const char* EV_LABELS[] = {"Alarm", "Silence", "Alarm test", "Unknown"};      // shown on the web page / in HA

// one tracker per distinct frame, so frames from different devices are counted separately
#define MAX_TRACK 32
struct Tracker {
  uint8_t frame[FRAME_LEN]; bool used; uint16_t cnt; uint32_t first, last; bool reported; int peak;
  int32_t foffSum; uint16_t foffN; int32_t bFoffSum; int type;
  // current transmission burst (frames less than cfg.pressGapMs apart)
  bool bOpen; int bType; uint32_t bStart, bLast; uint16_t bCnt, bFix; int32_t bRssiSum; int bMin, bMax;
} trk[MAX_TRACK];


#define BURST_MAX 40
struct Burst { time_t t; uint32_t up; uint32_t startMs; char type[10]; uint16_t durMs, cnt, fixes; int avg, mn, mx, foff; };
#define NO_OFFSET 9999
int eventOffset = NO_OFFSET;                     // frequency offset of the event being reported
Burst bursts[BURST_MAX];

#define UNK_MAX 20
struct UnknownFrame { time_t t; uint32_t up; int rssi; uint16_t cnt; char frame[17]; };
UnknownFrame unknowns[UNK_MAX];
int unknownCount = 0;
int burstCount = 0;
bool lastClassifyFixed = false;
uint32_t lastAlarmFrameMs = 0;

bool alarmOn = false;
String lastEvent = "none", lastEventTime = "-", lastDevice = "-";
int lastRssi = 0;

// ---------------- log ----------------
#define LOG_MAX 50
struct LogEntry { time_t t; uint32_t up; char type[12]; char source[6]; int rssi; char frame[17]; char device[24]; int16_t foff; };
LogEntry logBuf[LOG_MAX];
int logCount = 0;

// =====================================================================
//  helpers
// =====================================================================
const char* labelFor(const char* type) {
  for (int i = 0; i < 4; i++) if (strcmp(type, EV_NAMES[i]) == 0) return EV_LABELS[i];
  return type;
}

int hexToBytes(const String& hex, uint8_t* out, int maxLen) {
  String h = hex; h.trim(); h.replace(" ", "");
  if (h.length() % 2) return -1;
  int n = h.length() / 2;
  if (n > maxLen) return -1;
  for (int i = 0; i < n; i++) {
    char buf[3] = { h[2 * i], h[2 * i + 1], 0 };
    char* end;
    long v = strtol(buf, &end, 16);
    if (*end) return -1;
    out[i] = (uint8_t)v;
  }
  return n;
}

String bytesToHex(const uint8_t* b, int n) {
  String s; char t[3];
  for (int i = 0; i < n; i++) { sprintf(t, "%02X", b[i]); s += t; }
  return s;
}

bool timeValid() { return time(nullptr) > 1700000000; }

String fmtTime(time_t t, uint32_t up) {
  if (t > 1700000000) {
    struct tm tmv; localtime_r(&t, &tmv);
    char b[32]; strftime(b, sizeof(b), "%Y-%m-%d %H:%M:%S", &tmv);
    return b;
  }
  return "uptime " + String(up) + "s";
}

String isoTime(time_t t) {
  struct tm tmv; localtime_r(&t, &tmv);
  char b[32]; strftime(b, sizeof(b), "%Y-%m-%dT%H:%M:%S%z", &tmv);
  return b;
}

String htmlEsc(const String& s) {
  String o; for (unsigned i = 0; i < s.length(); i++) { char c = s[i];
    if (c == '<') o += "&lt;"; else if (c == '>') o += "&gt;";
    else if (c == '&') o += "&amp;"; else if (c == '"') o += "&quot;"; else o += c;
  } return o;
}

// device names: lines of HEX=Name, HEX is the whole 8-byte frame or a leading part of it
void parseDevices() {
  deviceCount = 0;
  String txt = cfg.devicesText; txt.replace("\r", "");
  int start = 0;
  while (start < (int)txt.length() && deviceCount < MAX_DEVICES) {
    int nl = txt.indexOf('\n', start); if (nl < 0) nl = txt.length();
    String line = txt.substring(start, nl); start = nl + 1;
    int eq = line.indexOf('=');
    if (eq < 1) continue;
    String hex = line.substring(0, eq); hex.trim();
    String name = line.substring(eq + 1); name.trim();
    DeviceName& d = devices[deviceCount];
    d.len = hexToBytes(hex, d.prefix, FRAME_LEN);
    if (d.len < 1 || name.length() == 0) continue;
    if (d.len == FRAME_LEN) d.prefix[FRAME_LEN - 1] &= LAST_BYTE_MASK;
    d.name = name;
    deviceCount++;
  }
}

#define MAX_TX_NAMES 16
struct TxName { int offset; String name; int rssi; uint32_t seen; time_t lastT; uint32_t lastUp; } txNames[MAX_TX_NAMES];
int txNameCount = 0;

// Learn mode: the next complete transmission is added as a new transmitter
#define LEARN_TIMEOUT_MS 120000UL
bool learnArmed = false;
uint32_t learnStartMs = 0;
String learnMsg = "";

int matchTx(int offset) {
  if (offset == NO_OFFSET) return -1;
  int best = -1, bestD = 1000;
  for (int i = 0; i < txNameCount; i++) {
    int d = abs(offset - txNames[i].offset);
    if (d <= cfg.offsetTol && d < bestD) { bestD = d; best = i; }
  }
  return best;
}

void parseTxNames() {
  txNameCount = 0;
  String txt = cfg.txNamesText; txt.replace("\r", "");
  int start = 0;
  while (start < (int)txt.length() && txNameCount < MAX_TX_NAMES) {
    int nl = txt.indexOf('\n', start); if (nl < 0) nl = txt.length();
    String line = txt.substring(start, nl); start = nl + 1;
    int eq = line.indexOf('=');
    if (eq < 1) continue;
    String off = line.substring(0, eq); off.trim(); off.replace("kHz", ""); off.replace("+", ""); off.trim();
    String name = line.substring(eq + 1); name.trim();
    if (name.length() == 0 || (off.toInt() == 0 && off != "0" && off != "-0")) continue;
    txNames[txNameCount].offset = off.toInt();
    txNames[txNameCount].name = name;
    txNames[txNameCount].rssi = 0; txNames[txNameCount].seen = 0;
    txNames[txNameCount].lastT = 0; txNames[txNameCount].lastUp = 0;
    txNameCount++;
  }
}

String deviceForFrame(const uint8_t* frame);
// name by frequency offset (nearest transmitter within tolerance), else by frame
String deviceFor(const uint8_t* frame, int offset);
String deviceFor(const uint8_t* frame) { return deviceFor(frame, NO_OFFSET); }
String deviceFor(const uint8_t* frame, int offset) {
  int m = matchTx(offset);
  if (m >= 0) return txNames[m].name;
  return deviceForFrame(frame);
}

String deviceForFrame(const uint8_t* frame) {
  int best = -1, bestLen = 0;
  for (int i = 0; i < deviceCount; i++)
    if (devices[i].len > bestLen && memcmp(frame, devices[i].prefix, devices[i].len) == 0) { best = i; bestLen = devices[i].len; }
  return best >= 0 ? devices[best].name : String("");
}

// =====================================================================
//  config
// =====================================================================
bool applyCodes(String& err) {
  uint8_t tmp[FRAME_LEN];
  uint8_t s[2];
  if (hexToBytes(cfg.syncHex, s, 2) != 2) { err = "Sync word must be 4 hex digits"; return false; }
  int n = hexToBytes(cfg.sysIdHex, tmp, FRAME_LEN - 1);
  if (n < 1) { err = "System ID must be 1-7 bytes of hex"; return false; }
  int cl = FRAME_LEN - n;
  uint8_t a[FRAME_LEN], si[FRAME_LEN], te[FRAME_LEN];
  bool aset = cfg.alarmHex.length() > 0;
  if (aset && hexToBytes(cfg.alarmHex, a, FRAME_LEN) != cl) { err = "Smoke alarm code must be " + String(cl) + " bytes (" + String(cl * 2) + " hex digits) or blank"; return false; }
  if (hexToBytes(cfg.silenceHex, si, FRAME_LEN) != cl) { err = "Silence code must be " + String(cl) + " bytes (" + String(cl * 2) + " hex digits)"; return false; }
  bool tset = cfg.testHex.length() > 0;
  if (tset && hexToBytes(cfg.testHex, te, FRAME_LEN) != cl) { err = "Alarm code must be " + String(cl) + " bytes or blank"; return false; }
  memcpy(sysId, tmp, n); sysIdLen = n; codeLen = cl;
  if (aset) memcpy(codeAlarm, a, cl);
  memcpy(codeSilence, si, cl);
  if (tset) memcpy(codeTest, te, cl);
  codeAlarm[cl - 1] &= LAST_BYTE_MASK; codeSilence[cl - 1] &= LAST_BYTE_MASK; codeTest[cl - 1] &= LAST_BYTE_MASK;
  testCodeSet = tset; alarmCodeSet = aset;
  return true;
}

void loadConfig() {
  if (!LittleFS.exists(CFG_FILE)) return;
  File f = LittleFS.open(CFG_FILE, "r");
  JsonDocument d;
  if (deserializeJson(d, f)) { f.close(); return; }
  f.close();
  cfg.devName    = d["devName"]    | cfg.devName;
  cfg.devId      = d["devId"]      | cfg.devId;
  cfg.hostname   = d["hostname"]   | cfg.hostname;
  cfg.mqttHost   = d["mqttHost"]   | cfg.mqttHost;
  cfg.mqttPort   = d["mqttPort"]   | cfg.mqttPort;
  cfg.mqttUser   = d["mqttUser"]   | cfg.mqttUser;
  cfg.mqttPass   = d["mqttPass"]   | cfg.mqttPass;
  cfg.baseTopic  = d["baseTopic"]  | cfg.baseTopic;
  cfg.tz         = d["tz"]         | cfg.tz;
  cfg.freqMHz    = d["freqMHz"]    | cfg.freqMHz;
  cfg.rxBwKHz    = d["rxBwKHz"]    | cfg.rxBwKHz;
  // v2.7: detectors transmit ~100 kHz above the controller - move the old default up to 433.30
  if (fabs(cfg.freqMHz - 433.25) < 0.001) { cfg.freqMHz = 433.30; configMigrated = true; }
  cfg.syncHex    = d["syncHex"]    | cfg.syncHex;
  cfg.sysIdHex   = d["sysIdHex"]   | cfg.sysIdHex;
  cfg.alarmHex   = d["alarmHex"]   | cfg.alarmHex;
  cfg.silenceHex = d["silenceHex"] | cfg.silenceHex;
  cfg.testHex    = d["testHex"]    | cfg.testHex;
  cfg.minRepeats = d["minRepeats"] | cfg.minRepeats;
  cfg.bitTolerance = d["bitTolerance"] | cfg.bitTolerance;
  cfg.unknownRepeats = d["unknownRepeats"] | cfg.unknownRepeats;
  cfg.pressGapMs = d["pressGapMs"] | cfg.pressGapMs;
  cfg.minPressMs = d["minPressMs"] | cfg.minPressMs;
  cfg.alarmAutoOff = d["alarmAutoOff"] | cfg.alarmAutoOff;
  cfg.txEnabled  = d["txEnabled"]  | cfg.txEnabled;
  cfg.txSeconds  = d["txSeconds"]  | cfg.txSeconds;
  cfg.devicesText = d["devicesText"] | cfg.devicesText;
  cfg.txNamesText = d["txNamesText"] | cfg.txNamesText;
  txNameCount = 0;
  if (d["transmitters"].is<JsonArray>()) {
    for (JsonObject o : d["transmitters"].as<JsonArray>()) {
      if (txNameCount >= MAX_TX_NAMES) break;
      TxName& x = txNames[txNameCount++];
      x.name = o["n"] | "Transmitter"; x.offset = o["o"] | 0; x.rssi = o["r"] | 0;
      x.seen = 0; x.lastT = 0; x.lastUp = 0;
    }
  } else if (cfg.txNamesText.length()) {        // v2.9 text list -> table
    parseTxNames();
    cfg.txNamesText = "";
    configMigrated = true;
  }
  cfg.offsetTol = d["offsetTol"] | cfg.offsetTol;
  // v2.3: 0AA9C8 turned out to be the wall controller's TEST button, not a smoke alarm
  if (cfg.alarmHex == "0AA9C8" && cfg.testHex.length() == 0) {
    cfg.testHex = "0AA9C8"; cfg.alarmHex = "";
    configMigrated = true;
  }
}

void saveConfig() {
  JsonDocument d;
  d["devName"] = cfg.devName;   d["devId"] = cfg.devId;     d["hostname"] = cfg.hostname;
  d["mqttHost"] = cfg.mqttHost; d["mqttPort"] = cfg.mqttPort;
  d["mqttUser"] = cfg.mqttUser; d["mqttPass"] = cfg.mqttPass; d["baseTopic"] = cfg.baseTopic;
  d["tz"] = cfg.tz;             d["freqMHz"] = cfg.freqMHz;  d["rxBwKHz"] = cfg.rxBwKHz;
  d["syncHex"] = cfg.syncHex;   d["sysIdHex"] = cfg.sysIdHex;
  d["alarmHex"] = cfg.alarmHex; d["silenceHex"] = cfg.silenceHex; d["testHex"] = cfg.testHex;
  d["minRepeats"] = cfg.minRepeats; d["alarmAutoOff"] = cfg.alarmAutoOff;
  d["bitTolerance"] = cfg.bitTolerance; d["unknownRepeats"] = cfg.unknownRepeats;
  d["pressGapMs"] = cfg.pressGapMs; d["minPressMs"] = cfg.minPressMs;
  d["txEnabled"] = cfg.txEnabled;   d["txSeconds"] = cfg.txSeconds;
  d["devicesText"] = cfg.devicesText;
  d["offsetTol"] = cfg.offsetTol;
  JsonArray ta = d["transmitters"].to<JsonArray>();
  for (int i = 0; i < txNameCount; i++) {
    JsonObject o = ta.add<JsonObject>();
    o["n"] = txNames[i].name; o["o"] = txNames[i].offset; o["r"] = txNames[i].rssi;
  }
  File f = LittleFS.open(CFG_FILE, "w");
  serializeJson(d, f);
  f.close();
}

// =====================================================================
//  log
// =====================================================================
void saveLog() {
  JsonDocument d;
  JsonArray a = d.to<JsonArray>();
  for (int i = 0; i < logCount; i++) {
    JsonObject o = a.add<JsonObject>();
    o["t"] = (long)logBuf[i].t; o["up"] = logBuf[i].up; o["type"] = logBuf[i].type;
    o["src"] = logBuf[i].source; o["rssi"] = logBuf[i].rssi; o["frame"] = logBuf[i].frame;
    o["dev"] = logBuf[i].device; o["foff"] = logBuf[i].foff;
  }
  File f = LittleFS.open(LOG_FILE, "w");
  serializeJson(d, f);
  f.close();
}

void loadLog() {
  if (!LittleFS.exists(LOG_FILE)) return;
  File f = LittleFS.open(LOG_FILE, "r");
  JsonDocument d;
  if (deserializeJson(d, f)) { f.close(); return; }
  f.close();
  logCount = 0;
  for (JsonObject o : d.as<JsonArray>()) {
    if (logCount >= LOG_MAX) break;
    LogEntry& e = logBuf[logCount++];
    e.t = (time_t)(long)(o["t"] | 0L); e.up = o["up"] | 0;
    strlcpy(e.type, o["type"] | "?", sizeof(e.type));
    strlcpy(e.source, o["src"] | "rf", sizeof(e.source));
    e.rssi = o["rssi"] | 0;
    strlcpy(e.frame, o["frame"] | "", sizeof(e.frame));
    strlcpy(e.device, o["dev"] | "", sizeof(e.device));
    e.foff = o["foff"] | NO_OFFSET;
  }
}

void addLog(const char* type, const char* source, int rssi, const String& frame, const String& device) {
  // newest first
  if (logCount < LOG_MAX) logCount++;
  for (int i = logCount - 1; i > 0; i--) logBuf[i] = logBuf[i - 1];
  LogEntry& e = logBuf[0];
  e.t = timeValid() ? time(nullptr) : 0;
  e.up = millis() / 1000;
  strlcpy(e.type, type, sizeof(e.type));
  strlcpy(e.source, source, sizeof(e.source));
  e.rssi = rssi;
  strlcpy(e.frame, frame.c_str(), sizeof(e.frame));
  strlcpy(e.device, device.c_str(), sizeof(e.device));
  e.foff = eventOffset;
  saveLog();
}

// =====================================================================
//  CC1101 driver (direct SPI)  -- only touched from the radio task
// =====================================================================
const float FXOSC = 26e6;
SPISettings ccSPI(4000000, MSBFIRST, SPI_MODE0);

void ccSelect() {
  digitalWrite(PIN_CSN, LOW);
  uint32_t t = micros();
  while (digitalRead(PIN_MISO) && micros() - t < 5000) {}
}
void ccDeselect() { digitalWrite(PIN_CSN, HIGH); }
void ccWrite(uint8_t a, uint8_t v) {
  SPI.beginTransaction(ccSPI); ccSelect(); SPI.transfer(a); SPI.transfer(v); ccDeselect(); SPI.endTransaction();
}
void ccWriteBurst(uint8_t a, const uint8_t* b, int n) {
  SPI.beginTransaction(ccSPI); ccSelect(); SPI.transfer(a | 0x40);
  for (int i = 0; i < n; i++) SPI.transfer(b[i]);
  ccDeselect(); SPI.endTransaction();
}
void ccReadBurst(uint8_t a, uint8_t* b, int n) {
  SPI.beginTransaction(ccSPI); ccSelect(); SPI.transfer(a | 0xC0);
  for (int i = 0; i < n; i++) b[i] = SPI.transfer(0);
  ccDeselect(); SPI.endTransaction();
}
uint8_t ccStatus(uint8_t a) {
  SPI.beginTransaction(ccSPI); ccSelect(); SPI.transfer(a | 0xC0); uint8_t v = SPI.transfer(0);
  ccDeselect(); SPI.endTransaction(); return v;
}
void ccStrobe(uint8_t c) {
  SPI.beginTransaction(ccSPI); ccSelect(); SPI.transfer(c); ccDeselect(); SPI.endTransaction();
}
uint8_t ccRxBytes() {           // errata: read until two reads agree
  uint8_t a, b;
  do { a = ccStatus(0x3B); b = ccStatus(0x3B); } while (a != b);
  return a;
}
uint8_t ccMarc() { return ccStatus(0x35) & 0x1F; }

void ccReset() {
  digitalWrite(PIN_CSN, LOW);  delayMicroseconds(5);
  digitalWrite(PIN_CSN, HIGH); delayMicroseconds(40);
  ccStrobe(0x30); delay(10);
}

void ccConfigure() {
  uint8_t sync[2];
  hexToBytes(cfg.syncHex, sync, 2);
  ccStrobe(0x36);                      // SIDLE
  ccWrite(0x00, 0x2E);                 // IOCFG2 hi-Z
  ccWrite(0x02, 0x06);                 // IOCFG0 sync/end of packet
  ccWrite(0x03, 0x47);                 // FIFOTHR
  ccWrite(0x04, sync[0]);              // SYNC1
  ccWrite(0x05, sync[1]);              // SYNC0
  ccWrite(0x06, FRAME_LEN);            // PKTLEN
  ccWrite(0x07, 0x04);                 // PKTCTRL1: append RSSI/LQI
  ccWrite(0x08, 0x00);                 // PKTCTRL0: fixed length, no CRC, no whitening
  ccWrite(0x09, 0x00); ccWrite(0x0A, 0x00);
  ccWrite(0x0B, 0x08);                 // FSCTRL1 IF ~203 kHz
  ccWrite(0x0C, 0x00);
  uint32_t F = (uint32_t)lround(cfg.freqMHz * 1e6 * 65536.0 / FXOSC);
  ccWrite(0x0D, (F >> 16) & 0xFF); ccWrite(0x0E, (F >> 8) & 0xFF); ccWrite(0x0F, F & 0xFF);
  int bwE = 1, bwM = 1; float bwBest = 1e12;     // RX bandwidth = fxosc / (8 * (4+M) * 2^E)
  for (int e = 0; e < 4; e++) for (int m = 0; m < 4; m++) {
    float bw = FXOSC / (8.0 * (4 + m) * (1 << e));
    if (fabs(bw - cfg.rxBwKHz * 1000.0) < bwBest) { bwBest = fabs(bw - cfg.rxBwKHz * 1000.0); bwE = e; bwM = m; }
  }
  bool wideBw = FXOSC / (8.0 * (4 + bwM) * (1 << bwE)) > 325000;
  ccWrite(0x10, (bwE << 6) | (bwM << 4) | 0x0B);  // MDMCFG4: RX bandwidth, DRATE_E 11
  ccWrite(0x11, 0xF8);                 // MDMCFG3: ~99.97 kbaud
  ccWrite(0x12, 0x02);                 // MDMCFG2: 2-FSK, 16/16 sync bits
  ccWrite(0x13, 0x22);                 // MDMCFG1: 4 preamble bytes (TX)
  ccWrite(0x14, 0xF8);                 // MDMCFG0
  ccWrite(0x15, 0x47);                 // DEVIATN 47.6 kHz
  ccWrite(0x16, 0x07);                 // MCSM2
  ccWrite(0x17, 0x3F);                 // MCSM1: stay in RX after RX, RX after TX
  ccWrite(0x18, 0x18);                 // MCSM0: autocal IDLE->RX/TX
  ccWrite(0x19, 0x1E);                 // FOCCFG: frequency offset compensation up to +-BW/4
  ccWrite(0x1A, 0x1C);                 // BSCFG
  ccWrite(0x1B, 0xC7);                 // AGCCTRL2
  ccWrite(0x1C, 0x00);                 // AGCCTRL1
  ccWrite(0x1D, 0xB2);                 // AGCCTRL0
  ccWrite(0x21, 0xB6);                 // FREND1
  ccWrite(0x22, 0x10);                 // FREND0 (PATABLE[0])
  ccWrite(0x23, 0xEA); ccWrite(0x24, 0x2A); ccWrite(0x25, 0x00); ccWrite(0x26, 0x1F);
  ccWrite(0x2C, wideBw ? 0x88 : 0x81); ccWrite(0x2D, wideBw ? 0x31 : 0x35); ccWrite(0x2E, 0x09);
  ccWrite(0x3E, 0xC0);                 // PATABLE: ~+10 dBm at 433 MHz
  ccStrobe(0x33); delay(2);            // SCAL
  ccStrobe(0x3A);                      // SFRX
  ccStrobe(0x34);                      // SRX
}

int rssiFromRaw(uint8_t r) { int v = r; if (v >= 128) v -= 256; return v / 2 - 74; }

void ccTransmit(const uint8_t* frame, float seconds) {
  ccStrobe(0x36);                      // SIDLE
  ccWrite(0x17, 0x31);                 // MCSM1: TXOFF -> FSTXON (no recal between packets)
  ccStrobe(0x3B);                      // SFTX
  uint32_t end = millis() + (uint32_t)(seconds * 1000);
  uint32_t sent = 0;
  while ((int32_t)(millis() - end) < 0) {
    ccWriteBurst(0x3F, frame, FRAME_LEN);
    ccStrobe(0x35);                    // STX
    uint32_t t = micros();
    delayMicroseconds(200);
    uint8_t m;
    while ((m = ccMarc()) != 0x12 && micros() - t < 5000) {   // wait until back in FSTXON
      if (m == 0x16) { ccStrobe(0x3B); break; }               // TX underflow
    }
    sent++;
  }
  ccStrobe(0x36);
  ccWrite(0x17, 0x3F);
  ccStrobe(0x3B); ccStrobe(0x3A);
  ccStrobe(0x34);
  Serial.printf("TX done: %u frames\n", sent);
}

// radio task: owns the SPI bus
// ---------------- frequency scanner (runs in the radio task) ----------------
#define SCAN_MAX_CH 241
#define SCAN_LEARN_SWEEPS 20
#define SCAN_HITS 40
#define SCAN_AUTO_STOP_MS (10UL * 60 * 1000)     // resume alarm monitoring after 10 min
volatile bool scanActive = false, scanStopReq = false, scanStartReq = false;
float scanStart = 432.0, scanEnd = 435.0, scanStep = 0.025;
int scanN = 0, scanCh = 0;
float chFloor[SCAN_MAX_CH];
int8_t chPeak[SCAN_MAX_CH], chNow[SCAN_MAX_CH];
uint16_t chHits[SCAN_MAX_CH];
uint32_t chLastHit[SCAN_MAX_CH];
volatile uint32_t scanSweeps = 0, scanBeganMs = 0;
struct ScanHit { time_t t; uint32_t up; float f; int8_t rssi, over; };
ScanHit scanHits[SCAN_HITS];
volatile int scanHitCount = 0;

void ccSetFreq(float mhz) {
  uint32_t F = (uint32_t)lround(mhz * 1e6 * 65536.0 / FXOSC);
  ccStrobe(0x36);                                 // SIDLE
  ccStrobe(0x3A);                                 // SFRX: packets received while scanning would fill the FIFO
  ccWrite(0x0D, (F >> 16) & 0xFF); ccWrite(0x0E, (F >> 8) & 0xFF); ccWrite(0x0F, F & 0xFF);
  ccStrobe(0x34);                                 // SRX, autocalibrates (MCSM0)
}

void scanBegin() {
  scanN = (int)lround((scanEnd - scanStart) / scanStep) + 1;
  if (scanN > SCAN_MAX_CH) { scanN = SCAN_MAX_CH; scanStep = (scanEnd - scanStart) / (scanN - 1); }
  for (int i = 0; i < SCAN_MAX_CH; i++) { chFloor[i] = -100; chPeak[i] = -128; chNow[i] = -128; chHits[i] = 0; chLastHit[i] = 0; }
  scanCh = 0; scanSweeps = 0; scanHitCount = 0; scanBeganMs = millis();
  scanActive = true;
  Serial.printf("Scan %.3f-%.3f MHz, %d channels\n", scanStart, scanEnd, scanN);
}

void scanStepOnce() {
  float f = scanStart + scanCh * scanStep;
  ccSetFreq(f);
  delayMicroseconds(1200);                       // calibration + RSSI settle
  int r = (rssiFromRaw(ccStatus(0x34)) + rssiFromRaw(ccStatus(0x34))) / 2;
  chNow[scanCh] = r;
  bool learning = scanSweeps < SCAN_LEARN_SWEEPS;
  if (learning || r < chFloor[scanCh] + 6) chFloor[scanCh] = chFloor[scanCh] * 0.9f + r * 0.1f;
  if (!learning) {
    if (r > chPeak[scanCh]) chPeak[scanCh] = r;
    int over = r - (int)chFloor[scanCh];
    if (over >= 12) {
      chHits[scanCh]++;
      uint32_t now = millis();
      if (now - chLastHit[scanCh] > 500) {
        chLastHit[scanCh] = now;
        int n = scanHitCount < SCAN_HITS ? scanHitCount + 1 : SCAN_HITS;
        for (int i = n - 1; i > 0; i--) scanHits[i] = scanHits[i - 1];
        scanHits[0] = { timeValid() ? time(nullptr) : 0, now / 1000, f, (int8_t)r, (int8_t)over };
        scanHitCount = n;
      }
    }
  }
  if (++scanCh >= scanN) { scanCh = 0; scanSweeps++; }
}

// ---------------- raw capture (CC1101 async serial mode, bits timed on GDO0) ----------------
#define RAW_MAX 6000
#define RAW_RESULTS 3
#define RAW_AUTO_STOP_MS (10UL * 60 * 1000)
volatile bool rawActive = false, rawStartReq = false, rawStopReq = false, rawRec = false;
float rawFreq = 433.30, rawDev = 47.6, rawBw = 270;
int rawTrig = 12;
uint32_t rawBuf[RAW_MAX];
volatile uint16_t rawN = 0;
volatile uint32_t rawLastEdge = 0, rawBeganMs = 0, rawCaptures = 0;
volatile float rawFloor = -100;
static uint32_t wbuf[RAW_MAX];
static uint8_t  lbuf[RAW_MAX];
String rawResults[RAW_RESULTS];
String rawResultTime[RAW_RESULTS];
int rawResultCount = 0;
SemaphoreHandle_t rawMutex;

void IRAM_ATTR rawEdge() {
  uint32_t now = micros();
  uint32_t w = now - rawLastEdge;
  rawLastEdge = now;
  if (!rawRec || rawN >= RAW_MAX) return;
  uint32_t lvl = digitalRead(PIN_GDO0) ? 0 : 1;      // level of the pulse that just ended
  rawBuf[rawN++] = (lvl << 31) | (w & 0x7FFFFFFF);
}

void ap(String& o, const char* fmt, ...) {
  char b[200];
  va_list a; va_start(a, fmt); vsnprintf(b, sizeof(b), fmt, a); va_end(a);
  o += b;
}

void ccConfigureRaw() {
  ccStrobe(0x36);
  ccWrite(0x00, 0x2E);  // IOCFG2 hi-Z
  ccWrite(0x02, 0x0D);  // IOCFG0: serial data output (raw demodulated bits)
  ccWrite(0x03, 0x47);
  ccWrite(0x07, 0x00);
  ccWrite(0x08, 0x32);  // PKTCTRL0: async serial mode
  ccWrite(0x0B, 0x06);  // FSCTRL1
  ccWrite(0x0C, 0x00);
  uint32_t F = (uint32_t)lround(rawFreq * 1e6 * 65536.0 / FXOSC);
  ccWrite(0x0D, (F >> 16) & 0xFF); ccWrite(0x0E, (F >> 8) & 0xFF); ccWrite(0x0F, F & 0xFF);
  int bwE = 3, bwM = 3; float best = 1e12;
  for (int e = 0; e < 4; e++) for (int m = 0; m < 4; m++) {
    float bw = FXOSC / (8.0 * (4 + m) * (1 << e));
    if (fabs(bw - rawBw * 1000) < best) { best = fabs(bw - rawBw * 1000); bwE = e; bwM = m; }
  }
  ccWrite(0x10, (bwE << 6) | (bwM << 4) | 0x0A);
  ccWrite(0x11, 0xF8);
  ccWrite(0x12, 0x00);  // 2-FSK, no sync
  ccWrite(0x13, 0x22); ccWrite(0x14, 0xF8);
  int dE = 4, dM = 7; best = 1e12;
  for (int e = 0; e < 8; e++) for (int m = 0; m < 8; m++) {
    float d = FXOSC / 131072.0 * (8 + m) * (1 << e);
    if (fabs(d - rawDev * 1000) < best) { best = fabs(d - rawDev * 1000); dE = e; dM = m; }
  }
  ccWrite(0x15, (dE << 4) | dM);
  ccWrite(0x17, 0x30);  // MCSM1
  ccWrite(0x18, 0x18);
  ccWrite(0x19, 0x16); ccWrite(0x1A, 0x6C);
  ccWrite(0x1B, 0x43); ccWrite(0x1C, 0x40); ccWrite(0x1D, 0x91);
  ccWrite(0x21, 0x56); ccWrite(0x22, 0x10);
  ccWrite(0x23, 0xE9); ccWrite(0x24, 0x2A); ccWrite(0x25, 0x00); ccWrite(0x26, 0x1F);
  ccWrite(0x2C, 0x81); ccWrite(0x2D, 0x35); ccWrite(0x2E, 0x09);
  ccStrobe(0x33); delay(2);
  ccStrobe(0x34); delay(2);
}

void rawAnalyse(String& out, uint16_t n, uint32_t durMs, int peakRssi, float floorRssi) {
  for (uint16_t i = 0; i < n; i++) {
    uint32_t p = rawBuf[i];
    wbuf[i] = p & 0x7FFFFFFF;
    lbuf[i] = p >> 31;
  }

  out += "\n";
  out += "==================================================\n";
  ap(out, "Burst: %u edges, %lu ms, peak RSSI %d dBm (floor %.0f)\n",
                n, (unsigned long)durMs, peakRssi, floorRssi);

  // histogram, 5 us bins up to 2 ms
  const int NB = 400;
  static uint16_t hist[NB];  // NB bins
  memset(hist, 0, sizeof(hist));
  uint16_t valid = 0;
  for (uint16_t i = 1; i < n; i++) {          // skip first (time since last edge before burst)
    uint32_t w = wbuf[i];
    if (w >= 8 && w < NB * 5) { hist[w / 5]++; valid++; }
  }
  if (valid < 30) { out += "Too few valid pulses - probably noise.\n"; return; }

  // shortest well-populated cluster = 1 bit time
  int need = max(5, (int)(valid * 0.03));
  int cbin = -1;
  for (int b = 1; b < NB - 1; b++) {
    if (hist[b - 1] + hist[b] + hist[b + 1] >= need) { cbin = b; break; }
  }
  if (cbin < 0) { out += "No clear pulse cluster.\n"; return; }
  float c = cbin * 5 + 2.5;
  double s = 0; int cnt = 0;
  for (uint16_t i = 1; i < n; i++)
    if (wbuf[i] > c * 0.6 && wbuf[i] < c * 1.6) { s += wbuf[i]; cnt++; }
  float T = cnt ? s / cnt : c;
  // refine with pulses up to 8 bits long
  double sw = 0, sk = 0;
  for (uint16_t i = 1; i < n; i++) {
    int k = lround(wbuf[i] / T);
    if (k >= 1 && k <= 8) { sw += wbuf[i]; sk += k; }
  }
  if (sk > 0) T = sw / sk;

  ap(out, "Estimated bit time: %.1f us  ->  ~%.0f baud\n", T, 1e6 / T);

  // multiples histogram
  uint16_t mult[11] = {0};
  for (uint16_t i = 1; i < n; i++) {
    int k = lround(wbuf[i] / T);
    k = constrain(k, 0, 10);
    mult[k]++;
  }
  out += "Pulse lengths (x bit time): ";
  ap(out, "<0.5:%u ", mult[0]);
  for (int k = 1; k < 10; k++) ap(out, "%d:%u ", k, mult[k]);
  ap(out, ">=10:%u\n", mult[10]);

  // first raw pulses for sanity
  out += "First 40 pulses (us, H/L): ";
  for (uint16_t i = 1; i < n && i <= 40; i++) ap(out, "%lu%c ", (unsigned long)wbuf[i], lbuf[i] ? 'H' : 'L');
  out += "\n";

  // decode to bits
  String bits; bits.reserve(8000);
  for (uint16_t i = 1; i < n; i++) {
    int k = lround(wbuf[i] / T);
    if (k < 1) continue;                       // glitch
    if (k > 24) { bits += ' '; continue; }     // gap between packets
    char ch = lbuf[i] ? '1' : '0';
    for (int j = 0; j < k && bits.length() < 7900; j++) bits += ch;
  }

  out += "Bits:\n";
  for (unsigned int i = 0; i < bits.length(); i += 100) out += bits.substring(i, i + 100); out += "\n";

  // longest alternating run = preamble
  int bestLen = 0, bestEnd = -1, run = 1;
  for (unsigned int i = 1; i < bits.length(); i++) {
    char a = bits[i], b = bits[i - 1];
    if (a != ' ' && b != ' ' && a != b) {
      run++;
      if (run > bestLen) { bestLen = run; bestEnd = i; }
    } else run = 1;
  }
  if (bestLen < 16) { out += "No clear preamble found (try adjusting freq/dev).\n"; return; }

  ap(out, "Preamble: %d alternating bits ending at bit %d\n", bestLen, bestEnd);
  String after = bits.substring(bestEnd + 1, bestEnd + 1 + 160);
  out += "Bits after preamble: "; out += after; out += "\n";

  // hex of what follows, both inversions
  for (int inv = 0; inv < 2; inv++) {
    out += inv ? "Hex (inverted):      " : "Hex after preamble:  ";
    for (unsigned int i = 0; i + 8 <= after.length(); i += 8) {
      uint8_t v = 0; bool ok = true;
      for (int j = 0; j < 8; j++) {
        char ch = after[i + j];
        if (ch == ' ') { ok = false; break; }
        v = (v << 1) | ((ch == '1') ^ inv);
      }
      if (!ok) break;
      ap(out, "%02X ", v);
    }
    out += "\n";
  }

  // how often does the 24 bits after the preamble repeat? (= packet repeats in burst)
  if (after.length() >= 24) {
    String pat = after.substring(0, 24);
    int reps = 0, pos = 0;
    while ((pos = bits.indexOf(pat, pos)) >= 0) { reps++; pos += 24; }
    ap(out, "Pattern %s seen %d times in this burst\n", pat.c_str(), reps);
  }
  out += "==================================================\n";
}

void rawAddResult(const String& r) {
  String tm = fmtTime(timeValid() ? time(nullptr) : 0, millis() / 1000);
  xSemaphoreTake(rawMutex, portMAX_DELAY);
  for (int i = RAW_RESULTS - 1; i > 0; i--) { rawResults[i] = rawResults[i - 1]; rawResultTime[i] = rawResultTime[i - 1]; }
  rawResults[0] = r; rawResultTime[0] = tm;
  if (rawResultCount < RAW_RESULTS) rawResultCount++;
  xSemaphoreGive(rawMutex);
}

// called from the radio task every ~2 ms while raw capture is active
void rawPoll() {
  static int st = 0;                     // 0 idle, 1 capturing, 2 waiting for end
  static uint32_t burstStart = 0, lastAbove = 0;
  static int peak = -128;
  static bool fresh = true;
  if (rawStartReq) { st = 0; fresh = true; }
  uint32_t ms = millis();
  int r = rssiFromRaw(ccStatus(0x34));
  if (fresh) { rawFloor = r; fresh = false; }
  bool above = r > rawFloor + rawTrig;
  if (st == 0) {
    if (above && ms - rawBeganMs > 1000) {
      rawN = 0; rawLastEdge = micros(); rawRec = true;
      burstStart = ms; lastAbove = ms; peak = r; st = 1;
    } else rawFloor = rawFloor * 0.99f + r * 0.01f;
  } else if (st == 1) {
    if (r > peak) peak = r;
    if (r > rawFloor + rawTrig - 3) lastAbove = ms;
    if (rawN >= RAW_MAX || ms - lastAbove > 60) {
      rawRec = false;
      String out; out.reserve(9000);
      ap(out, "Freq %.3f MHz, dev %.1f kHz, RXBW %.0f kHz\n", rawFreq, rawDev, rawBw);
      rawAnalyse(out, rawN, ms - burstStart, peak, rawFloor);
      rawAddResult(out);
      rawCaptures++;
      st = (ms - lastAbove > 60) ? 0 : 2;
    }
  } else {
    if (above) lastAbove = ms;
    if (ms - lastAbove > 200) st = 0;
  }
}

void radioTask(void*) {
  for (;;) {
    if (radioReconfig) { radioReconfig = false; ccConfigure(); }

    if (rawStartReq) {
      scanActive = false;
      ccConfigureRaw();
      rawBeganMs = millis(); rawRec = false;
      attachInterrupt(digitalPinToInterrupt(PIN_GDO0), rawEdge, CHANGE);
      rawActive = true;
      rawPoll();                                 // resets its state while rawStartReq is set
      rawStartReq = false;
      Serial.printf("Raw capture at %.3f MHz\n", rawFreq);
    }
    if (rawActive) {
      if (rawStopReq || millis() - rawBeganMs > RAW_AUTO_STOP_MS) {
        detachInterrupt(digitalPinToInterrupt(PIN_GDO0));
        rawActive = false; rawStopReq = false; rawRec = false;
        ccConfigure();
        Serial.println("Raw capture stopped, alarm monitoring resumed");
      } else {
        rawPoll();
        vTaskDelay(2);
        continue;
      }
    }

    if (scanStartReq) { scanStartReq = false; scanBegin(); }
    if (scanActive) {
      if (scanStopReq || millis() - scanBeganMs > SCAN_AUTO_STOP_MS) {
        scanActive = false; scanStopReq = false;
        ccConfigure();                           // back to alarm monitoring
        Serial.println("Scan stopped, alarm monitoring resumed");
      } else {
        scanStepOnce();
        vTaskDelay(1);
        continue;
      }
    }

    if (txRequest) {
      int which = txRequest;
      uint8_t frame[FRAME_LEN];
      memcpy(frame, sysId, sysIdLen);
      memcpy(frame + sysIdLen, which == 1 ? codeTest : codeSilence, codeLen);
      txBusy = true;
      ccTransmit(frame, cfg.txSeconds);
      txBusy = false;
      txRequest = 0;
    }

    static uint8_t prevCount = 0;
    uint8_t rb = ccRxBytes();
    if (rb & 0x80 || ccMarc() == 0x11) {          // overflow
      ccStrobe(0x36); ccStrobe(0x3A); ccStrobe(0x34);
      radioOverflows++;
      prevCount = 0;
    } else {
      const uint8_t PKT = FRAME_LEN + 2;           // frame + RSSI + LQI
      uint8_t n = rb & 0x7F;
      // CC1101 errata: never read the LAST byte in the RX FIFO while bytes are still
      // arriving, or the FIFO pointer slips. Read a packet only if more bytes follow it,
      // or if the count hasn't changed since the last poll (no packet in progress).
      while (n > PKT || (n == PKT && n == prevCount)) {
        uint8_t buf[PKT];
        ccReadBurst(0x3F, buf, PKT);
        // alignment check: own system ID shifted inside the frame = FIFO slipped -> flush
        bool shifted = false;
        if (memcmp(buf, sysId, sysIdLen) != 0)
          for (int off = 1; off <= 3 && !shifted; off++)
            if (off + sysIdLen <= FRAME_LEN && memcmp(buf + off, sysId, sysIdLen) == 0) shifted = true;
        if (shifted) {
          ccStrobe(0x36); ccStrobe(0x3A); ccStrobe(0x34);
          radioResyncs++;
          n = 0;
          break;
        }
        RxFrame f;
        memcpy(f.d, buf, FRAME_LEN);
        f.rssi = rssiFromRaw(buf[FRAME_LEN]);
        f.foff = (int16_t)lround((int8_t)ccStatus(0x32) * (FXOSC / 16384.0) / 1000.0);   // FREQEST
        f.ms = millis();
        xQueueSend(rxQueue, &f, 0);
        radioFrames++;
        n = ccRxBytes() & 0x7F;
      }
      prevCount = n;
      if (ccMarc() == 0x01) ccStrobe(0x34);      // fell to IDLE -> back to RX
    }
    vTaskDelay(1);
  }
}

// =====================================================================
//  MQTT
// =====================================================================
String t(const char* sub) { return cfg.baseTopic + "/" + sub; }

void publishDiscovery() {
  String dev = "\"dev\":{\"ids\":[\"" + cfg.devId + "\"],\"name\":\"" + cfg.devName +
               "\",\"cu\":\"http://" + WiFi.localIP().toString() + "/\",\"mf\":\"DIY\",\"mdl\":\"" BOARD_NAME " + CC1101 Smoke Alarm Bridge\",\"sw\":\"" FW_VERSION "\"}";
  String avail = "\"avty_t\":\"" + t("status") + "\"";
  String p;

  p = "{\"name\":\"Smoke alarm\",\"uniq_id\":\"" + cfg.devId + "_smoke\",\"dev_cla\":\"smoke\",\"stat_t\":\"" +
      t("state") + "\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\"," + avail + "," + dev + "}";
  mqtt.publish(("homeassistant/binary_sensor/" + cfg.devId + "/smoke/config").c_str(), p.c_str(), true);

  p = "{\"name\":\"Last event\",\"uniq_id\":\"" + cfg.devId + "_last_event\",\"stat_t\":\"" +
      t("last_event") + "\",\"ic\":\"mdi:smoke-detector-variant\"," + avail + "," + dev + "}";
  mqtt.publish(("homeassistant/sensor/" + cfg.devId + "/last_event/config").c_str(), p.c_str(), true);

  p = "{\"name\":\"Last event time\",\"uniq_id\":\"" + cfg.devId + "_last_event_time\",\"stat_t\":\"" +
      t("last_event_time") + "\",\"dev_cla\":\"timestamp\"," + avail + "," + dev + "}";
  mqtt.publish(("homeassistant/sensor/" + cfg.devId + "/last_event_time/config").c_str(), p.c_str(), true);

  p = "{\"name\":\"Triggered by\",\"uniq_id\":\"" + cfg.devId + "_last_device\",\"stat_t\":\"" +
      t("last_device") + "\",\"ic\":\"mdi:map-marker-alert\"," + avail + "," + dev + "}";
  mqtt.publish(("homeassistant/sensor/" + cfg.devId + "/last_device/config").c_str(), p.c_str(), true);

  p = "{\"name\":\"Smoke alarm events\",\"uniq_id\":\"" + cfg.devId + "_events\",\"stat_t\":\"" +
      t("rf_event") + "\",\"evt_typ\":[\"alarm\",\"silence\",\"test\",\"unknown\"],\"ic\":\"mdi:smoke-detector-alert\"," +
      avail + "," + dev + "}";
  mqtt.publish(("homeassistant/event/" + cfg.devId + "/events/config").c_str(), p.c_str(), true);

  p = "{\"name\":\"Event log\",\"uniq_id\":\"" + cfg.devId + "_log\",\"stat_t\":\"" + t("log") +
      "\",\"val_tpl\":\"{{ value_json.count }}\",\"json_attr_t\":\"" + t("log") +
      "\",\"ic\":\"mdi:format-list-bulleted\"," + avail + "," + dev + "}";
  mqtt.publish(("homeassistant/sensor/" + cfg.devId + "/log/config").c_str(), p.c_str(), true);

  p = "{\"name\":\"Clear event log\",\"uniq_id\":\"" + cfg.devId + "_clear_log\",\"cmd_t\":\"" + t("clear_log") +
      "\",\"pl_prs\":\"PRESS\",\"ic\":\"mdi:delete-sweep\",\"ent_cat\":\"config\",\"avty_t\":\"" + t("status") + "\"," + dev + "}";
  mqtt.publish(("homeassistant/button/" + cfg.devId + "/clear_log/config").c_str(), p.c_str(), true);

  p = "{\"name\":\"IP address\",\"uniq_id\":\"" + cfg.devId + "_ip\",\"stat_t\":\"" +
      t("ip") + "\",\"ic\":\"mdi:ip-network\",\"ent_cat\":\"diagnostic\"," + avail + "," + dev + "}";
  mqtt.publish(("homeassistant/sensor/" + cfg.devId + "/ip/config").c_str(), p.c_str(), true);

  p = "{\"name\":\"Alarm RF signal\",\"uniq_id\":\"" + cfg.devId + "_rssi\",\"stat_t\":\"" +
      t("rssi") + "\",\"dev_cla\":\"signal_strength\",\"unit_of_meas\":\"dBm\",\"ent_cat\":\"diagnostic\"," +
      avail + "," + dev + "}";
  mqtt.publish(("homeassistant/sensor/" + cfg.devId + "/rssi/config").c_str(), p.c_str(), true);
}

#define MQTT_LOG_ENTRIES 20
void publishLog() {
  if (!mqtt.connected()) return;
  JsonDocument d;
  d["count"] = logCount;
  d["last"] = logCount ? String(logBuf[0].type) : String("none");
  JsonArray a = d["entries"].to<JsonArray>();
  for (int i = 0; i < logCount && i < MQTT_LOG_ENTRIES; i++) {
    JsonObject o = a.add<JsonObject>();
    o["time"] = fmtTime(logBuf[i].t, logBuf[i].up);
    o["event"] = logBuf[i].type;
    o["label"] = labelFor(logBuf[i].type);
    o["device"] = logBuf[i].device[0] ? logBuf[i].device : "?";
    o["source"] = logBuf[i].source;
    o["rssi"] = logBuf[i].rssi;
    o["frame"] = logBuf[i].frame;
    if (logBuf[i].foff != NO_OFFSET) o["offset_khz"] = logBuf[i].foff;
  }
  String s; serializeJson(d, s);
  mqtt.publish(t("log").c_str(), s.c_str(), true);
}

String publishedIp = "";
void publishIp() {
  String ip = WiFi.localIP().toString();
  mqtt.publish(t("ip").c_str(), ip.c_str(), true);
  if (publishedIp.length() && ip != publishedIp) publishDiscovery();   // update the HA "Visit" link
  publishedIp = ip;
}

// Home Assistant "Clear event log" button. The clear itself is done in mqttLoop(), not here,
// because PubSubClient shares one buffer for incoming and outgoing messages.
volatile bool clearLogRequested = false;
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  if (String(topic) == t("clear_log")) clearLogRequested = true;
}

void mqttLoop() {
  if (cfg.mqttHost.length() == 0) return;
  static uint32_t lastTry = 0;
  if (!mqtt.connected()) {
    if (millis() - lastTry < 5000) return;
    lastTry = millis();
    mqtt.setServer(cfg.mqttHost.c_str(), cfg.mqttPort);
    String cid = cfg.devId + "_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    bool ok = cfg.mqttUser.length()
      ? mqtt.connect(cid.c_str(), cfg.mqttUser.c_str(), cfg.mqttPass.c_str(), t("status").c_str(), 0, true, "offline")
      : mqtt.connect(cid.c_str(), t("status").c_str(), 0, true, "offline");
    if (ok) {
      Serial.println("MQTT connected");
      mqtt.publish(t("status").c_str(), "online", true);
      publishDiscovery();
      mqtt.subscribe(t("clear_log").c_str());
      mqtt.publish(t("state").c_str(), alarmOn ? "ON" : "OFF", true);
      mqtt.publish(t("last_event").c_str(), lastEvent.c_str(), true);
      publishIp();
      publishLog();
    }
    return;
  }
  mqtt.loop();
  if (clearLogRequested) {
    clearLogRequested = false;
    logCount = 0; saveLog(); publishLog();
    Serial.println("Event log cleared from Home Assistant");
  }
  static uint32_t lastIpCheck = 0;
  if (millis() - lastIpCheck > 30000) {
    lastIpCheck = millis();
    if (WiFi.localIP().toString() != publishedIp) publishIp();
  }
}

void mqttEvent(const char* type, const char* source, int rssi, const String& frame, const String& device) {
  if (!mqtt.connected()) return;
  JsonDocument d;
  d["event"] = type; d["label"] = labelFor(type); d["source"] = source; d["rssi"] = rssi; d["frame"] = frame;
  d["device"] = device.length() ? device : String("unknown");
  if (eventOffset != NO_OFFSET) d["freq_offset_khz"] = eventOffset;
  d["time"] = timeValid() ? isoTime(time(nullptr)) : String("unknown");
  String s; serializeJson(d, s);
  mqtt.publish(t("event").c_str(), s.c_str(), false);
  // HA event entity
  JsonDocument e;
  e["event_type"] = type; e["label"] = labelFor(type); e["device"] = d["device"]; e["source"] = source; e["rssi"] = rssi; e["frame"] = frame;
  String es; serializeJson(e, es);
  mqtt.publish(t("rf_event").c_str(), es.c_str(), false);
  mqtt.publish(t("last_event").c_str(), labelFor(type), true);
  mqtt.publish(t("last_device").c_str(), device.length() ? device.c_str() : frame.c_str(), true);
  if (timeValid()) mqtt.publish(t("last_event_time").c_str(), isoTime(time(nullptr)).c_str(), true);
  if (rssi) mqtt.publish(t("rssi").c_str(), String(rssi).c_str(), true);
}

void setAlarmState(bool on) {
  if (alarmOn == on) return;
  alarmOn = on;
  if (mqtt.connected()) mqtt.publish(t("state").c_str(), on ? "ON" : "OFF", true);
  Serial.printf("Alarm state -> %s\n", on ? "ON" : "OFF");
}

// =====================================================================
//  event processing
// =====================================================================
void reportEvent(int type, const char* source, int rssi, const String& frame, const String& device) {
  const char* name = EV_NAMES[type];
  if (type == EV_UNKNOWN) {                       // diagnostics only, not a button press
    if (unknownCount < UNK_MAX) unknownCount++;
    for (int i = unknownCount - 1; i > 0; i--) unknowns[i] = unknowns[i - 1];
    UnknownFrame& u = unknowns[0];
    u.t = timeValid() ? time(nullptr) : 0; u.up = millis() / 1000; u.rssi = rssi; u.cnt = 0;
    strlcpy(u.frame, frame.c_str(), sizeof(u.frame));
    Serial.printf("UNKNOWN frame %s rssi %d\n", frame.c_str(), rssi);
    if (mqtt.connected()) {
      JsonDocument e;
      e["event_type"] = "unknown"; e["frame"] = frame; e["rssi"] = rssi; e["source"] = source;
      String es; serializeJson(e, es);
      mqtt.publish(t("rf_event").c_str(), es.c_str(), false);
    }
    return;
  }
  lastEvent = EV_LABELS[type];
  lastDevice = device.length() ? device : frame;
  lastEventTime = fmtTime(timeValid() ? time(nullptr) : 0, millis() / 1000);
  if (rssi) lastRssi = rssi;
  Serial.printf("EVENT %s (%s) dev '%s' rssi %d frame %s\n", name, source, device.c_str(), rssi, frame.c_str());
  addLog(name, source, rssi, frame, device);
  mqttEvent(name, source, rssi, frame, device);
  publishLog();
  if (type == EV_ALARM && strcmp(source, "rf") == 0) setAlarmState(true);
  if (type == EV_SILENCE) setAlarmState(false);
}

int bitDiff(const uint8_t* a, const uint8_t* b, int n) {
  int d = 0;
  for (int i = 0; i < n; i++) d += __builtin_popcount(a[i] ^ b[i]);
  return d;
}

// Classify a frame by comparing the WHOLE frame (system ID + command) with each known
// frame. Up to cfg.bitTolerance flipped bits are accepted (weak signals misread bits, and
// with ~2000 copies per press every 1-bit variant shows up several times). Alarm and
// silence frames differ by 7 bits, so a tolerance of 2 can never confuse them.
// A near-match is corrected in place to the exact frame, so it counts towards the same press.
int classify(uint8_t* f) {
  lastClassifyFixed = false;
  const uint8_t* codes[3] = { alarmCodeSet ? codeAlarm : nullptr, codeSilence, testCodeSet ? codeTest : nullptr };
  // v3.1: the test frame is what the detectors send too, so a received one is always an ALARM
  // (EV_TEST is now only used for the test button on this bridge's own status page)
  const int types[3] = { EV_ALARM, EV_SILENCE, EV_ALARM };
  int idD = bitDiff(f, sysId, sysIdLen);
  int best = -1, bestD = 99, ties = 0;
  for (int i = 0; i < 3; i++) {
    if (!codes[i]) continue;
    int d = idD + bitDiff(f + sysIdLen, codes[i], codeLen);
    if (d < bestD) { bestD = d; best = i; ties = 0; }
    else if (d == bestD && types[i] != types[best]) ties++;
  }
  if (best < 0 || bestD > cfg.bitTolerance || ties) return EV_UNKNOWN;
  lastClassifyFixed = bestD > 0;
  if (bestD > 0) {
    memcpy(f, sysId, sysIdLen);
    memcpy(f + sysIdLen, codes[best], codeLen);
    bitFixes++;
  }
  return types[best];
}

Tracker* trackerFor(const uint8_t* f, uint32_t now) {
  int freeIdx = -1, oldest = 0, oldestJunk = -1;
  for (int i = 0; i < MAX_TRACK; i++) {
    if (trk[i].used && memcmp(trk[i].frame, f, FRAME_LEN) == 0) return &trk[i];
    if (!trk[i].used && freeIdx < 0) freeIdx = i;
    if (trk[i].last < trk[oldest].last) oldest = i;
    // corrupted one-off frames are the first to be replaced, never an active real frame
    if (trk[i].used && !trk[i].reported && trk[i].cnt < cfg.minRepeats &&
        (oldestJunk < 0 || trk[i].last < trk[oldestJunk].last)) oldestJunk = i;
  }
  int i = freeIdx >= 0 ? freeIdx : (oldestJunk >= 0 ? oldestJunk : oldest);
  memcpy(trk[i].frame, f, FRAME_LEN);
  trk[i].used = true; trk[i].cnt = 0; trk[i].reported = false; trk[i].last = 0; trk[i].bOpen = false; trk[i].type = EV_UNKNOWN;
  return &trk[i];
}

void closeBurst(Tracker* k) {
  k->bOpen = false;
  uint16_t need = k->bType == EV_UNKNOWN ? cfg.unknownRepeats : cfg.minRepeats;
  if (k->bCnt < need) return;                       // stray corrupted copies, not a transmission
  if (burstCount < BURST_MAX) burstCount++;
  for (int i = burstCount - 1; i > 0; i--) bursts[i] = bursts[i - 1];
  Burst& b = bursts[0];
  uint32_t agoMs = millis() - k->bStart;
  b.t = timeValid() ? time(nullptr) - agoMs / 1000 : 0;
  b.up = k->bStart / 1000;
  b.startMs = k->bStart;
  strlcpy(b.type, EV_NAMES[k->bType], sizeof(b.type));
  { uint32_t dur = k->bLast - k->bStart; b.durMs = dur > 65535 ? 65535 : (uint16_t)dur; }
  b.cnt = k->bCnt; b.fixes = k->bFix;
  b.avg = k->bRssiSum / k->bCnt; b.mn = k->bMin; b.mx = k->bMax;
  b.foff = (int)lround((float)k->bFoffSum / k->bCnt);
  Serial.printf("BURST %s %u ms, %u copies (%u corrected), RSSI avg %d (%d..%d), offset %+d kHz\n",
                b.type, b.durMs, b.cnt, b.fixes, b.avg, b.mn, b.mx, b.foff);

  // The log entry was written after the first few copies; now that the whole transmission
  // has been heard, replace its frequency offset with the full-transmission average.
  if (k->bType != EV_UNKNOWN && b.cnt >= 50) {
    // remember when each known transmitter was last heard
    int m = matchTx(b.foff);
    if (m >= 0) {
      txNames[m].seen++; txNames[m].lastUp = millis() / 1000;
      txNames[m].lastT = timeValid() ? time(nullptr) : 0;
      txNames[m].rssi = txNames[m].rssi ? (txNames[m].rssi * 3 + b.avg) / 4 : b.avg;
    }
    // learn mode: add this transmission as a new transmitter
    if (learnArmed) {
      learnArmed = false;
      if (m >= 0) {
        learnMsg = "That was <b>" + htmlEsc(txNames[m].name) + "</b> (offset " + String(b.foff) + " kHz), which is already in the table.";
      } else if (txNameCount >= MAX_TX_NAMES) {
        learnMsg = "Table full (" + String(MAX_TX_NAMES) + " transmitters) - delete one first.";
      } else {
        TxName& x = txNames[txNameCount++];
        x.name = "Transmitter " + String(txNameCount); x.offset = b.foff; x.rssi = b.avg;
        x.seen = 1; x.lastUp = millis() / 1000; x.lastT = timeValid() ? time(nullptr) : 0;
        saveConfig();
        learnMsg = "Learned a new transmitter at offset <b>" + String(b.foff) + " kHz</b> (" + String(b.avg) +
                   " dBm, " + String(EV_LABELS[k->bType]) + "). Give it a name below and press Save.";
      }
    }
    String hex = bytesToHex(k->frame, FRAME_LEN);
    for (int i = 0; i < logCount && i < 5; i++) {
      LogEntry& e = logBuf[i];
      if (strcmp(e.source, "rf") == 0 && hex == e.frame &&
          e.up + 1 >= k->bStart / 1000 && e.up <= k->bLast / 1000 + 1) {
        e.foff = b.foff;
        String dev = deviceFor(k->frame, b.foff);
        strlcpy(e.device, dev.c_str(), sizeof(e.device));
        if (i == 0) {
          lastDevice = dev.length() ? dev : hex;
          if (mqtt.connected()) mqtt.publish(t("last_device").c_str(), lastDevice.c_str(), true);
        }
        saveLog();
        publishLog();
        break;
      }
    }
  }
}

#define OFFSET_SAMPLES 100

void reportTracker(Tracker* k) {
  k->reported = true;
  String hex = bytesToHex(k->frame, FRAME_LEN);
  if (k->type == EV_UNKNOWN) lastUnknownCode = hex;
  eventOffset = k->foffN ? (int)lround((float)k->foffSum / k->foffN) : NO_OFFSET;
  reportEvent(k->type, "rf", k->peak, hex, deviceFor(k->frame, eventOffset));
  eventOffset = NO_OFFSET;
}

void processFrames() {
  RxFrame f;
  while (xQueueReceive(rxQueue, &f, 0) == pdTRUE) {
    f.d[FRAME_LEN - 1] &= LAST_BYTE_MASK;      // toggle bit, not part of the command
    int type = classify(f.d);                  // may correct bit errors in f.d
    bool own = memcmp(f.d, sysId, sysIdLen) == 0;
    if (own) framesOwn++; else { framesOther++; lastOtherFrame = bytesToHex(f.d, FRAME_LEN); }
    if (type == EV_ALARM) lastAlarmFrameMs = f.ms;

    bool fixed = lastClassifyFixed;
    Tracker* k = trackerFor(f.d, f.ms);

    // burst tracking: a gap of more than cfg.pressGapMs starts a new transmission
    if (k->bOpen && f.ms - k->bLast > cfg.pressGapMs) closeBurst(k);
    if (!k->bOpen) {
      k->bOpen = true; k->bType = type; k->bStart = f.ms;
      k->bCnt = 0; k->bFix = 0; k->bRssiSum = 0; k->bFoffSum = 0; k->bMin = 127; k->bMax = -128;
    }
    k->bCnt++; k->bLast = f.ms; k->bRssiSum += f.rssi; k->bFoffSum += f.foff;
    if (f.rssi < k->bMin) k->bMin = f.rssi;
    if (f.rssi > k->bMax) k->bMax = f.rssi;
    if (fixed) k->bFix++;

    // each press is one continuous ~7 s transmission (6144 copies ~1.15 ms apart), and the
    // controller ignores presses while it transmits. A new press needs a gap in reception
    // AND at least minPressMs since the previous press started, so signal dropouts in the
    // middle of a weak transmission can't split one press into two.
    if (f.ms - k->last > cfg.pressGapMs && (f.ms - k->first > cfg.minPressMs || f.ms - k->last > cfg.minPressMs)) { k->cnt = 0; k->reported = false; k->first = f.ms; k->peak = -128; k->foffSum = 0; k->foffN = 0; }
    k->cnt++; k->last = f.ms;
    if (f.rssi > k->peak) k->peak = f.rssi;
    k->foffSum += f.foff; k->foffN++;
    // known codes need a few copies; unknown frames need many, because corrupted copies of a
    // known frame can repeat a handful of times while a real new code repeats ~2000 times
    k->type = type;
    // known codes: wait for OFFSET_SAMPLES copies (~0.12 s) so the frequency offset - and so the
    // transmitter name - is accurate. Weak transmissions with fewer copies are reported when they end.
    uint16_t need = type == EV_UNKNOWN ? cfg.unknownRepeats : (cfg.minRepeats > OFFSET_SAMPLES ? cfg.minRepeats : OFFSET_SAMPLES);
    if (!k->reported && k->cnt >= need) reportTracker(k);
  }
  // close bursts that have gone quiet; report short known transmissions that never reached OFFSET_SAMPLES
  uint32_t now = millis();
  for (int i = 0; i < MAX_TRACK; i++) {
    Tracker* k = &trk[i];
    if (!k->used || now - k->last <= cfg.pressGapMs) continue;
    if (!k->reported && k->type != EV_UNKNOWN && k->cnt >= cfg.minRepeats) reportTracker(k);
    if (k->bOpen) closeBurst(k);
  }
  // alarm auto-off
  if (alarmOn && cfg.alarmAutoOff && millis() - lastAlarmFrameMs > cfg.alarmAutoOff * 1000UL) setAlarmState(false);
}

// =====================================================================
//  web pages
// =====================================================================
const char CSS[] PROGMEM = R"(
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
body{font-family:system-ui,Arial;background:#111;color:#eee;margin:0;padding:16px;max-width:900px;margin:auto}
h1{font-size:1.4em;margin:.2em 0}h2{font-size:1.1em;margin-top:1.4em;color:#9cf}
a{color:#9cf}nav a{margin-right:14px}
.card{background:#1c1c1c;border-radius:10px;padding:14px;margin:12px 0}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px}
.k{color:#999;font-size:.85em}.v{font-size:1.15em;font-weight:600}
.on{color:#f55}.off{color:#5d5}
table{width:100%;border-collapse:collapse;font-size:.9em}td,th{padding:6px;border-bottom:1px solid #333;text-align:left}
tr.alarm td{color:#f77}tr.silence td{color:#7d7}tr.test td{color:#fc6}tr.unknown td{color:#aaa}
label{display:block;margin-top:10px;color:#bbb;font-size:.9em}
input[type=text],input[type=number],input[type=password]{width:100%;box-sizing:border-box;padding:8px;background:#222;color:#eee;border:1px solid #444;border-radius:6px}
button,.btn{background:#2a6;color:#fff;border:0;border-radius:6px;padding:10px 16px;font-size:1em;cursor:pointer;margin:6px 6px 0 0}
.warn{background:#c33}.amber{background:#c80}.grey{background:#555}
.note{color:#999;font-size:.85em}.err{color:#f66}
code{background:#222;padding:2px 5px;border-radius:4px}
</style>)";

String pageHead(const String& title) {
  String s = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>" + htmlEsc(title) + "</title>";
  s += FPSTR(CSS);
  s += "</head><body><h1>&#128293; " + htmlEsc(cfg.devName) + "</h1>";
  s += "<nav><a href='/'>Status</a><a href='/setup'>Alarm setup</a><a href='/network'>Wi-Fi / MQTT</a><a href='/diag'>Diagnostics</a><a href='/tx'>Transmitters</a><a href='/scan'>Scanner</a></nav>";
  return s;
}

void handleRoot() {
  String s = pageHead(cfg.devName);
  s += R"HTML(<div class="card"><div class="grid">
<div><div class="k">Smoke alarm</div><div class="v" id="st">-</div></div>
<div><div class="k">Last event</div><div class="v" id="le">-</div></div>
<div><div class="k">Last event time</div><div class="v" id="lt">-</div></div>
<div><div class="k">Triggered by</div><div class="v" id="ld">-</div></div>
<div><div class="k">Signal</div><div class="v" id="rs">-</div></div>
<div><div class="k">Radio</div><div class="v" id="ra">-</div></div>
<div><div class="k">MQTT</div><div class="v" id="mq">-</div></div>
</div></div>)HTML";
  if (cfg.txEnabled) {
    s += "<div class='card'><b>Transmit</b> <span class='note'>(sends to your alarms for " + String(cfg.txSeconds, 1) + " s)</span><br>";
    s += testCodeSet ? "<button class='amber' onclick=\"tx('test')\">Send ALARM TEST</button>"
                     : "<button class='grey' disabled>ALARM TEST (no code set)</button>";
    s += "<button onclick=\"tx('silence')\">Send SILENCE</button> <span id='txs' class='note'></span></div>";
  }
  s += R"HTML(<div class="card"><b>Event log</b> <button class="grey" style="float:right;padding:4px 10px" onclick="clr()">Clear</button>
<table><thead><tr><th>Time</th><th>Event</th><th>Device</th><th>Source</th><th>RSSI</th><th>Offset</th><th>Frame</th></tr></thead><tbody id="log"></tbody></table></div>
<p class="note">Firmware )HTML" FW_VERSION " &middot; " BOARD_NAME R"HTML( &middot; <span id="ip"></span> &middot; up <span id="up"></span></p>
<script>
const LBL={alarm:'ALARM',silence:'SILENCE',test:'ALARM TEST',unknown:'UNKNOWN'};
async function upd(){try{let r=await fetch('/api/status');let j=await r.json();
st.innerHTML=j.alarm?'<span class=on>ALARM</span>':'<span class=off>Clear</span>';
le.textContent=j.lastEvent;lt.textContent=j.lastEventTime;ld.textContent=j.lastDevice;rs.textContent=j.rssi?j.rssi+' dBm':'-';
ra.innerHTML=j.radio?(j.scanning?'<span class=on>Scanning - alarms not monitored</span>':(j.txBusy?'Transmitting...':'Listening')):'NOT FOUND';
mq.innerHTML=j.mqtt?'<span class=off>Connected</span>':(j.mqttHost?'<span class=on>Disconnected</span>':'Not set');
ip.textContent=j.ip;up.textContent=j.uptime;
let h='';for(const e of j.log){h+=`<tr class="${e.type}"><td>${e.time}</td><td>${LBL[e.type]||e.type}</td><td>${e.dev||'?'}</td><td>${e.src}</td><td>${e.rssi||''}</td><td>${(e.foff===undefined||e.foff==9999)?'':(e.foff>0?'+':'')+e.foff+' kHz'}</td><td><code>${e.frame}</code></td></tr>`}
log.innerHTML=h||'<tr><td colspan=7 class=note>No events yet</td></tr>';}catch(e){}}
async function tx(c){if(!confirm('Transmit '+c.toUpperCase()+' to your smoke alarms?'))return;
let r=await fetch('/api/tx',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'cmd='+c});
document.getElementById('txs').textContent=await r.text();setTimeout(upd,500)}
async function clr(){if(!confirm('Clear the event log?'))return;await fetch('/api/clearlog',{method:'POST'});upd()}
upd();setInterval(upd,2000);
</script></body></html>)HTML";
  server.send(200, "text/html", s);
}

void handleStatus() {
  JsonDocument d;
  d["alarm"] = alarmOn; d["lastEvent"] = lastEvent; d["lastEventTime"] = lastEventTime; d["lastDevice"] = lastDevice;
  d["rssi"] = lastRssi; d["radio"] = (bool)radioOk; d["txBusy"] = (bool)txBusy; d["scanning"] = (bool)(scanActive || rawActive);
  d["mqtt"] = mqtt.connected(); d["mqttHost"] = cfg.mqttHost.length() > 0;
  d["own"] = framesOwn; d["other"] = framesOther; d["unknown"] = lastUnknownCode;
  d["lastOther"] = lastOtherFrame; d["overflows"] = (uint32_t)radioOverflows;
  d["ip"] = WiFi.localIP().toString();
  uint32_t up = millis() / 1000;
  char b[24]; sprintf(b, "%ud %02uh %02um", up / 86400, (up / 3600) % 24, (up / 60) % 60);
  d["uptime"] = b;
  JsonArray a = d["log"].to<JsonArray>();
  for (int i = 0; i < logCount; i++) {
    JsonObject o = a.add<JsonObject>();
    o["time"] = fmtTime(logBuf[i].t, logBuf[i].up);
    o["type"] = logBuf[i].type; o["src"] = logBuf[i].source;
    o["rssi"] = logBuf[i].rssi; o["frame"] = logBuf[i].frame; o["dev"] = logBuf[i].device; o["foff"] = logBuf[i].foff;
  }
  String s; serializeJson(d, s);
  server.send(200, "application/json", s);
}

void handleTx() {
  if (!cfg.txEnabled) { server.send(403, "text/plain", "Transmit is disabled on the setup page"); return; }
  String c = server.arg("cmd");
  int which = c == "test" ? 1 : c == "silence" ? 2 : 0;
  if (!which) { server.send(400, "text/plain", "Unknown command"); return; }
  if (which == 1 && !testCodeSet) { server.send(400, "text/plain", "No test code set"); return; }
  if (txBusy || txRequest) { server.send(409, "text/plain", "Already transmitting"); return; }
  uint8_t frame[FRAME_LEN];
  memcpy(frame, sysId, sysIdLen);
  memcpy(frame + sysIdLen, which == 1 ? codeTest : codeSilence, codeLen);
  txRequest = which;
  reportEvent(which == 1 ? EV_TEST : EV_SILENCE, "web", 0, bytesToHex(frame, FRAME_LEN), "This bridge");
  server.send(200, "text/plain", "Sent " + c);
}

void handleDiagApi() {
  JsonDocument d;
  d["own"] = framesOwn; d["other"] = framesOther; d["fixed"] = bitFixes;
  d["radioFrames"] = (uint32_t)radioFrames; d["overflows"] = (uint32_t)radioOverflows;
  d["resyncs"] = (uint32_t)radioResyncs; d["lastOther"] = lastOtherFrame;
  d["heap"] = ESP.getFreeHeap();
  JsonArray ba = d["bursts"].to<JsonArray>();
  for (int i = 0; i < burstCount; i++) {
    JsonObject o = ba.add<JsonObject>();
    o["time"] = fmtTime(bursts[i].t, bursts[i].up);
    o["type"] = bursts[i].type; o["dur"] = bursts[i].durMs; o["cnt"] = bursts[i].cnt;
    o["fix"] = bursts[i].fixes; o["foff"] = bursts[i].foff; o["avg"] = bursts[i].avg; o["mn"] = bursts[i].mn; o["mx"] = bursts[i].mx;
    // seconds after the previous (older) burst started
    if (i + 1 < burstCount) o["after"] = (bursts[i].startMs - bursts[i + 1].startMs) / 1000.0;
  }
  JsonArray ua = d["unknowns"].to<JsonArray>();
  for (int i = 0; i < unknownCount; i++) {
    JsonObject o = ua.add<JsonObject>();
    o["time"] = fmtTime(unknowns[i].t, unknowns[i].up); o["rssi"] = unknowns[i].rssi; o["frame"] = unknowns[i].frame;
  }
  String s; serializeJson(d, s);
  server.send(200, "application/json", s);
}

void handleDiag() {
  String s = pageHead("Diagnostics");
  s += R"HTML(<div class="card"><div class="grid">
<div><div class="k">Frames own / other</div><div class="v" id="fo">-</div></div>
<div><div class="k">Bit-corrected frames</div><div class="v" id="fx">-</div></div>
<div><div class="k">FIFO overflows / resyncs</div><div class="v" id="ov">-</div></div>
<div><div class="k">Free heap</div><div class="v" id="hp">-</div></div>
</div><div class="note" style="margin-top:8px">Last frame from another system / noise: <code id="lo">-</code></div></div>
<div class="card"><b>Transmission bursts</b> <span class="note">each separate transmission - one per button press (a full one is 7.07 s / 6144 copies)</span>
<button class="grey" style="float:right;padding:4px 10px" onclick="clrb()">Clear</button>
<table><thead><tr><th>Time</th><th>Code</th><th>After prev</th><th>Duration</th><th>Copies</th><th>Corrected</th><th>Offset</th><th>RSSI avg (min..max)</th></tr></thead><tbody id="bl"></tbody></table></div>
<div class="card"><b>Unknown frames</b> <span class="note">repeated frames that don't match a known code (possible new device or command)</span>
<button class="grey" style="float:right;padding:4px 10px" onclick="clru()">Clear</button>
<table><thead><tr><th>Time</th><th>RSSI</th><th>Frame</th></tr></thead><tbody id="ul"></tbody></table></div>
<script>
async function upd(){try{let j=await (await fetch('/api/diag')).json();
fo.textContent=j.own+' / '+j.other;fx.textContent=j.fixed;ov.textContent=j.overflows+' / '+j.resyncs;
hp.textContent=Math.round(j.heap/1024)+' KB';lo.textContent=j.lastOther;
let b='';for(const e of j.bursts){b+=`<tr class="${e.type}"><td>${e.time}</td><td>${({alarm:'ALARM',silence:'SILENCE',test:'ALARM TEST',unknown:'UNKNOWN'})[e.type]||e.type}</td><td>${e.after!==undefined?'+'+e.after.toFixed(1)+' s':''}</td><td>${(e.dur/1000).toFixed(2)} s</td><td>${e.cnt}</td><td>${e.fix}</td><td>${e.foff>0?'+':''}${e.foff} kHz</td><td>${e.avg} (${e.mn}..${e.mx})</td></tr>`}
bl.innerHTML=b||'<tr><td colspan=8 class=note>No bursts yet</td></tr>';
let u='';for(const e of j.unknowns){u+=`<tr class="unknown"><td>${e.time}</td><td>${e.rssi}</td><td><code>${e.frame}</code></td></tr>`}
ul.innerHTML=u||'<tr><td colspan=3 class=note>None</td></tr>';}catch(e){}}
async function clrb(){await fetch('/api/clearbursts',{method:'POST'});upd()}
async function clru(){await fetch('/api/clearunknown',{method:'POST'});upd()}
upd();setInterval(upd,2000);
</script></body></html>)HTML";
  server.send(200, "text/html", s);
}

void handleClearUnknown() {
  unknownCount = 0;
  server.send(200, "text/plain", "ok");
}

void handleClearBursts() {
  burstCount = 0;
  server.send(200, "text/plain", "ok");
}

void handleClearLog() {
  logCount = 0; saveLog(); publishLog();
  server.send(200, "text/plain", "ok");
}

String field(const char* label, const char* name, const String& val, const char* type = "text", const char* note = "") {
  String s = "<label>" + String(label) + "<input type='" + type + "' name='" + name + "' value='" + htmlEsc(val) + "'";
  if (strcmp(type, "number") == 0) s += " step='any'";
  s += "></label>";
  if (note[0]) s += "<div class='note'>" + String(note) + "</div>";
  return s;
}

void handleSetup() {
  String msg;
  if (server.method() == HTTP_POST) {
    Config old = cfg;
    cfg.freqMHz    = server.arg("freq").toFloat();
    cfg.rxBwKHz    = constrain(server.arg("rxbw").toInt(), 58, 812);
    cfg.syncHex    = server.arg("sync");    cfg.syncHex.toUpperCase();
    cfg.sysIdHex   = server.arg("sysid");   cfg.sysIdHex.toUpperCase();
    cfg.alarmHex   = server.arg("alarm");   cfg.alarmHex.toUpperCase();
    cfg.silenceHex = server.arg("silence"); cfg.silenceHex.toUpperCase();
    cfg.testHex    = server.arg("test");    cfg.testHex.toUpperCase();
    cfg.minRepeats = constrain(server.arg("rep").toInt(), 1, 50);
    cfg.bitTolerance = constrain(server.arg("bittol").toInt(), 0, 3);
    cfg.unknownRepeats = constrain(server.arg("unkrep").toInt(), 3, 500);
    cfg.alarmAutoOff = constrain(server.arg("autooff").toInt(), 0, 3600);
    cfg.pressGapMs = constrain(server.arg("gap").toInt(), 100, 5000);
    cfg.minPressMs = constrain(server.arg("minpress").toInt(), 0, 30000);
    cfg.txEnabled  = server.hasArg("txen");
    cfg.txSeconds  = constrain(server.arg("txsec").toFloat(), 0.5f, 10.0f);
    cfg.devicesText = server.arg("devices");
    String err;
    if (cfg.freqMHz < 387 || cfg.freqMHz > 464) err = "Frequency must be 387-464 MHz";
    if (err.length() == 0 && !applyCodes(err)) {}
    if (err.length()) { cfg = old; String e2; applyCodes(e2); msg = "<p class='err'>" + err + "</p>"; }
    else { parseDevices(); saveConfig(); radioReconfig = true; msg = "<p class='off'>Saved and applied.</p>"; }
  }
  String s = pageHead("Alarm setup");
  s += msg + "<form method='POST'><div class='card'><b>Radio</b>";
  s += field("Frequency (MHz)", "freq", String(cfg.freqMHz, 3), "number",
             "433.30 sits between the wall controller (~433.25) and the smoke detectors (~433.35)");
  s += field("Receive bandwidth (kHz)", "rxbw", String(cfg.rxBwKHz), "number",
             "325 covers both. Steps: 58, 68, 81, 102, 116, 135, 162, 203, 232, 270, 325, 406, 464, 541, 650, 812");
  s += field("Sync word (hex, 2 bytes)", "sync", cfg.syncHex, "text", "Found by the sniffer: 2F33");
  s += "</div><div class='card'><b>Codes</b><div class='note'>Frame = 8 bytes after the sync word = System ID + command code.</div>";
  s += field("System ID (hex)", "sysid", cfg.sysIdHex, "text", "Your alarm system's ID, first 5 bytes of the frame, from the Diagnostics page (5 bytes &rarr; codes are 3 bytes)");
  s += field("Smoke alarm code (hex, blank until captured)", "alarm", cfg.alarmHex, "text",
             "Optional second alarm code, e.g. a real smoke alarm if it ever differs. Treated as an ALARM");
  s += field("Silence code (hex)", "silence", cfg.silenceHex);
  s += field("Alarm code from wall panel test / detectors (hex)", "test", cfg.testHex, "text",
             ("Unknown frames are listed on the Diagnostics page. Last unknown frame: <code>" + lastUnknownCode + "</code>").c_str());
  s += "</div><div class='card'><b>Transmitter names</b><div class='note'>Name each smoke alarm / controller on the "
       "<a href='/tx'>Transmitters</a> page (Learn button). Those names take priority over the frame names below.</div>";
  s += "</div><div class='card'><b>Device names</b> <span class='note'>(by frame - used when no transmitter name matches)</span><div class='note'>One per line: <code>HEX=Name</code>. HEX is the whole frame "
       "(16 hex digits, copy it from the log) or the leading part of it. The longest match wins.</div>";
  s += "<textarea name='devices' rows='7' style='width:100%;box-sizing:border-box;background:#222;color:#eee;border:1px solid #444;border-radius:6px;padding:8px;font-family:monospace'>" +
       htmlEsc(cfg.devicesText) + "</textarea>";
  s += "</div><div class='card'><b>Detection</b>";
  s += field("Identical frames needed", "rep", String(cfg.minRepeats), "number", "Each press sends ~2000 frames; 3 filters out corrupted ones");
  s += field("Bit error tolerance (0-3)", "bittol", String(cfg.bitTolerance), "number",
             ("Accept a frame this many bits off a known frame and correct it. 2 is safe (alarm and silence differ by 7 bits). Frames corrected so far: " + String(bitFixes)).c_str());
  s += field("Copies needed to log an UNKNOWN frame", "unkrep", String(cfg.unknownRepeats), "number",
             "A real new code sends ~2000 copies; corrupted copies of a known frame rarely reach 20");
  s += field("Alarm auto-clear (seconds, 0 = only on silence)", "autooff", String(cfg.alarmAutoOff), "number");
  s += field("Gap between presses (ms)", "gap", String(cfg.pressGapMs), "number",
             "Each press is one ~7 s transmission. A pause in reception longer than this starts a new press. 500 is a good value");
  s += field("Minimum time between presses (ms)", "minpress", String(cfg.minPressMs), "number",
             "The controller transmits for 7.07 s and ignores presses meanwhile, so a new press can't start sooner. 7500 stops weak-signal dropouts splitting one press");
  s += "</div><div class='card'><b>Transmit (testing)</b>";
  s += "<label><input type='checkbox' name='txen'" + String(cfg.txEnabled ? " checked" : "") +
       "> Show TEST / SILENCE buttons on the status page</label>";
  s += field("Transmit duration (seconds)", "txsec", String(cfg.txSeconds, 1), "number", "The real controller sends for about 2.5 s");
  s += "</div><button type='submit'>Save</button></form>";
  s += "</body></html>";
  server.send(200, "text/html", s);
}

void handleNetwork() {
  String msg;
  if (server.method() == HTTP_POST) {
    cfg.devName   = server.arg("name");
    cfg.devId     = server.arg("devid");
    cfg.hostname  = server.arg("host");
    cfg.mqttHost  = server.arg("mhost");
    cfg.mqttPort  = server.arg("mport").toInt() ? server.arg("mport").toInt() : 1883;
    cfg.mqttUser  = server.arg("muser");
    if (server.arg("mpass") != "********") cfg.mqttPass = server.arg("mpass");
    cfg.baseTopic = server.arg("base");
    cfg.tz        = server.arg("tz");
    saveConfig();
    server.send(200, "text/html", pageHead("Saved") + "<p>Saved. Restarting...</p><script>setTimeout(()=>location='/',8000)</script></body></html>");
    delay(500); ESP.restart();
    return;
  }
  String s = pageHead("Wi-Fi / MQTT");
  s += "<form method='POST'><div class='card'><b>Device</b>";
  s += field("Device name", "name", cfg.devName);
  s += field("Device ID (Home Assistant identity)", "devid", cfg.devId, "text", "Keep this the same when swapping boards so HA keeps the entities");
  s += field("Hostname (mDNS)", "host", cfg.hostname);
  s += field("Timezone (POSIX)", "tz", cfg.tz, "text", "e.g. UTC: UTC0 &middot; Sydney: AEST-10AEDT,M10.1.0,M4.1.0/3 &middot; London: GMT0BST,M3.5.0/1,M10.5.0");
  s += "</div><div class='card'><b>MQTT</b>";
  s += field("Broker host / IP", "mhost", cfg.mqttHost);
  s += field("Port", "mport", String(cfg.mqttPort), "number");
  s += field("Username", "muser", cfg.mqttUser);
  s += field("Password", "mpass", cfg.mqttPass.length() ? "********" : "", "password");
  s += field("Base topic", "base", cfg.baseTopic, "text",
             ("Publishes " + cfg.baseTopic + "/state, /event, /last_event, /last_event_time, /last_device, /rssi, /ip, /log, /rf_event, /status").c_str());
  s += "</div><button type='submit'>Save &amp; restart</button></form>";
  s += "<div class='card'><b>Wi-Fi</b><p>Connected to <b>" + htmlEsc(WiFi.SSID()) + "</b> (" + String(WiFi.RSSI()) + " dBm)</p>";
  s += "<form method='POST' action='/wifireset' onsubmit=\"return confirm('Forget Wi-Fi and start the setup access point?')\">"
       "<button class='warn'>Change Wi-Fi</button></form>"
       "<div class='note'>Restarts into the <b>SmokeAlarmBridge-Setup</b> access point.</div></div></body></html>";
  server.send(200, "text/html", s);
}

void handleWifiReset() {
  server.send(200, "text/html", pageHead("Wi-Fi") + "<p>Wi-Fi cleared. Join <b>SmokeAlarmBridge-Setup</b> to configure.</p></body></html>");
  delay(500);
  wm.resetSettings();
  ESP.restart();
}

// =====================================================================
//  scanner page
// =====================================================================
void handleScanApi() {
  JsonDocument d;
  d["active"] = (bool)scanActive; d["start"] = scanStart; d["step"] = scanStep; d["n"] = scanN;
  d["sweeps"] = (uint32_t)scanSweeps; d["learning"] = scanSweeps < SCAN_LEARN_SWEEPS;
  d["left"] = scanActive ? (long)((SCAN_AUTO_STOP_MS - (millis() - scanBeganMs)) / 1000) : 0;
  JsonArray fl = d["floor"].to<JsonArray>(), pk = d["peak"].to<JsonArray>(), nw = d["now"].to<JsonArray>(), ht = d["hits"].to<JsonArray>();
  for (int i = 0; i < scanN; i++) { fl.add((int)chFloor[i]); pk.add((int)chPeak[i]); nw.add((int)chNow[i]); ht.add(chHits[i]); }
  JsonArray hl = d["log"].to<JsonArray>();
  for (int i = 0; i < scanHitCount; i++) {
    JsonObject o = hl.add<JsonObject>();
    o["time"] = fmtTime(scanHits[i].t, scanHits[i].up); o["f"] = scanHits[i].f;
    o["rssi"] = scanHits[i].rssi; o["over"] = scanHits[i].over;
  }
  String s; serializeJson(d, s);
  server.send(200, "application/json", s);
}

void handleScanCtl() {
  String a = server.arg("action");
  if (a == "start") {
    float st = server.arg("start").toFloat(), en = server.arg("end").toFloat();
    if (st < 387 || en > 464 || en <= st) { server.send(400, "text/plain", "Range must be within 387-464 MHz"); return; }
    if (txBusy || txRequest) { server.send(409, "text/plain", "Transmitting, try again"); return; }
    scanStart = st; scanEnd = en; scanStep = 0.025;
    if (rawActive) rawStopReq = true;
    scanStartReq = true;
    server.send(200, "text/plain", "started");
  } else {
    scanStopReq = true;
    server.send(200, "text/plain", "stopped");
  }
}

void handleRawApi() {
  JsonDocument d;
  d["active"] = (bool)rawActive; d["floor"] = (int)rawFloor; d["trig"] = rawTrig;
  d["captures"] = (uint32_t)rawCaptures; d["freq"] = rawFreq; d["recording"] = (bool)rawRec;
  d["left"] = rawActive ? (long)((RAW_AUTO_STOP_MS - (millis() - rawBeganMs)) / 1000) : 0;
  JsonArray a = d["results"].to<JsonArray>();
  xSemaphoreTake(rawMutex, portMAX_DELAY);
  for (int i = 0; i < rawResultCount; i++) { JsonObject o = a.add<JsonObject>(); o["time"] = rawResultTime[i]; o["text"] = rawResults[i]; }
  xSemaphoreGive(rawMutex);
  String s; serializeJson(d, s);
  server.send(200, "application/json", s);
}

void handleRawCtl() {
  String a = server.arg("action");
  if (a == "start") {
    float f = server.arg("freq").toFloat();
    if (f < 387 || f > 464) { server.send(400, "text/plain", "Frequency must be 387-464 MHz"); return; }
    if (txBusy || txRequest) { server.send(409, "text/plain", "Transmitting, try again"); return; }
    rawFreq = f;
    rawDev = constrain(server.arg("dev").toFloat(), 1.6f, 380.0f);
    rawBw = constrain(server.arg("bw").toFloat(), 58.0f, 812.0f);
    rawTrig = constrain(server.arg("trig").toInt(), 3, 40);
    if (scanActive) scanStopReq = true;
    rawStartReq = true;
    server.send(200, "text/plain", "raw capture started");
  } else if (a == "clear") {
    xSemaphoreTake(rawMutex, portMAX_DELAY); rawResultCount = 0; xSemaphoreGive(rawMutex);
    server.send(200, "text/plain", "cleared");
  } else {
    rawStopReq = true;
    server.send(200, "text/plain", "stopped");
  }
}

// =====================================================================
//  transmitters page
// =====================================================================
void handleTxApi() {
  if (learnArmed && millis() - learnStartMs > LEARN_TIMEOUT_MS) { learnArmed = false; learnMsg = "Learn timed out - nothing heard."; }
  JsonDocument d;
  d["learning"] = learnArmed; d["msg"] = learnMsg; d["tol"] = cfg.offsetTol;
  d["left"] = learnArmed ? (long)((LEARN_TIMEOUT_MS - (millis() - learnStartMs)) / 1000) : 0;
  JsonArray a = d["tx"].to<JsonArray>();
  for (int i = 0; i < txNameCount; i++) {
    JsonObject o = a.add<JsonObject>();
    o["name"] = txNames[i].name; o["offset"] = txNames[i].offset; o["rssi"] = txNames[i].rssi;
    o["seen"] = txNames[i].seen;
    o["last"] = txNames[i].seen ? fmtTime(txNames[i].lastT, txNames[i].lastUp) : String("-");
  }
  String s; serializeJson(d, s);
  server.send(200, "application/json", s);
}

void handleTxCtl() {
  String a = server.arg("action");
  int i = server.arg("i").toInt();
  if (a == "learn") {
    learnArmed = true; learnStartMs = millis();
    learnMsg = "";
  } else if (a == "cancel") {
    learnArmed = false; learnMsg = "Learn cancelled.";
  } else if (a == "save" && i >= 0 && i < txNameCount) {
    String n = server.arg("name"); n.trim();
    if (n.length() == 0) n = "Transmitter " + String(i + 1);
    if (n.length() > 23) n = n.substring(0, 23);
    txNames[i].name = n;
    txNames[i].offset = constrain(server.arg("offset").toInt(), -400, 400);
    saveConfig();
    learnMsg = "Saved <b>" + htmlEsc(n) + "</b>.";
  } else if (a == "add") {
    if (txNameCount < MAX_TX_NAMES) {
      TxName& x = txNames[txNameCount++];
      x.name = "Transmitter " + String(txNameCount); x.offset = constrain(server.arg("offset").toInt(), -400, 400);
      x.rssi = 0; x.seen = 0; x.lastT = 0; x.lastUp = 0;
      saveConfig();
      learnMsg = "Added - give it a name and press Save.";
    }
  } else if (a == "delete" && i >= 0 && i < txNameCount) {
    learnMsg = "Deleted <b>" + htmlEsc(txNames[i].name) + "</b>.";
    for (int j = i; j < txNameCount - 1; j++) txNames[j] = txNames[j + 1];
    txNameCount--;
    saveConfig();
  } else if (a == "tol") {
    cfg.offsetTol = constrain(server.arg("tol").toInt(), 1, 50);
    saveConfig();
    learnMsg = "Tolerance set to &plusmn;" + String(cfg.offsetTol) + " kHz.";
  }
  server.send(200, "text/plain", "ok");
}

void handleTxPage() {
  String s = pageHead("Transmitters");
  s += R"HTML(<div class="card"><b>Learn a transmitter</b>
<p class="note">Each smoke alarm and the wall controller send the same codes, but every transmitter's crystal is slightly
off-frequency. The bridge measures that <b>offset</b> and uses it to tell them apart.<br>
Press <b>Learn</b>, then press the TEST (or silence) button on <b>one</b> device and let it finish. It's added to the table below,
ready to be named.</p>
<button class="amber" id="lb" onclick="ctl('learn')">Learn</button> <button class="grey" onclick="ctl('cancel')">Cancel</button>
<p id="lm" class="note"></p></div>
<div class="card"><b>Transmitters</b>
<table><thead><tr><th>Name</th><th>Offset (kHz)</th><th>Signal</th><th>Heard</th><th>Last heard</th><th></th></tr></thead><tbody id="tb"></tbody></table>
<p class="note" style="margin-top:10px">Match tolerance &plusmn;<input type="number" id="tol" style="width:60px;padding:4px"> kHz
<button class="grey" style="padding:4px 10px" onclick="ctl('tol','&tol='+tol.value)">Set</button>
&middot; keep it under half the gap between your two closest transmitters.
&middot; <button class="grey" style="padding:4px 10px" onclick="add()">Add manually</button></p></div>
<script>
let editing=false,shown='';
function esc(t){return String(t).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/\x22/g,'&quot;')}
async function ctl(a,extra){await fetch('/api/txctl',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'action='+a+(extra||'')});editing=false;shown='';upd()}
function save(i){ctl('save','&i='+i+'&name='+encodeURIComponent(document.getElementById('n'+i).value)+'&offset='+document.getElementById('o'+i).value)}
function del(i){if(confirm('Delete '+document.getElementById('n'+i).value+'?'))ctl('delete','&i='+i)}
function add(){let o=prompt('Offset in kHz (from the log)');if(o!==null&&o!=='')ctl('add','&offset='+encodeURIComponent(o))}
async function upd(){try{let j=await (await fetch('/api/tx')).json();
lb.textContent=j.learning?'Listening...':'Learn';
lm.innerHTML=j.learning?'Listening for '+j.left+' s - press the test button on the device now.':j.msg;
if(document.activeElement!==tol)tol.value=j.tol;
let key=JSON.stringify(j.tx.map(t=>[t.name,t.offset]));
if(!editing&&key!==shown){shown=key;let h='';
j.tx.forEach((t,i)=>{h+=`<tr><td><input type="text" id="n${i}" value="${esc(t.name)}" maxlength="23" onfocus="editing=true" style="min-width:150px"></td>
<td><input type="number" id="o${i}" value="${t.offset}" onfocus="editing=true" style="width:80px"></td>
<td id="r${i}"></td><td id="s${i}"></td><td id="l${i}"></td>
<td><button style="padding:5px 10px" onclick="save(${i})">Save</button><button class="warn" style="padding:5px 10px" onclick="del(${i})">Delete</button></td></tr>`});
tb.innerHTML=h||'<tr><td colspan=6 class=note>None yet - press Learn</td></tr>';}
j.tx.forEach((t,i)=>{let r=document.getElementById('r'+i);if(r){r.textContent=t.rssi?t.rssi+' dBm':'';
document.getElementById('s'+i).textContent=t.seen;document.getElementById('l'+i).textContent=t.last;}});
}catch(e){}}
upd();setInterval(upd,1500);
</script></body></html>)HTML";
  server.send(200, "text/html", s);
}

void handleScanPage() {
  String s = pageHead("Scanner");
  s += R"HTML(<div class="card"><b>Frequency scanner</b>
<p class="note">Sweeps the range in 25 kHz steps and shows where signals appear. <b>Alarm monitoring is paused while scanning</b>
(it resumes when you press Stop, or automatically after 10 minutes).<br>
Start the scan, wait for "Ready", then press the smoke detector's TEST button.
Known signals: 433.25 = wall controller, ~433.85-433.96 every 16 s = weather station.</p>
<div style="display:flex;gap:10px;flex-wrap:wrap;align-items:end">
<label style="margin:0">From (MHz)<input type="number" id="fa" value="432.0" step="0.1" style="width:110px"></label>
<label style="margin:0">To (MHz)<input type="number" id="fb" value="435.0" step="0.1" style="width:110px"></label>
<button onclick="ctl('start')">Start</button><button class="warn" onclick="ctl('stop')">Stop</button>
</div><p id="ss" class="note"></p>
<canvas id="cv" style="width:100%;height:220px;background:#151515;border-radius:6px"></canvas>
<p class="note">Bars: strongest signal seen above each channel's noise floor (red = 12 dB+). Tap the chart to read a frequency. <span id="tip"></span></p></div>
<div class="card"><b>Raw capture</b>
<p class="note">Decodes any FSK signal at one frequency: bit rate, preamble, sync word and frame. Needs CC1101 GDO0 wired
(GPIO3 on the C3 Super Mini). <b>Alarm monitoring is paused</b> while it runs (auto-stops after 10 minutes).
Start it, wait 2 s, then press the TEST button.</p>
<div style="display:flex;gap:10px;flex-wrap:wrap;align-items:end">
<label style="margin:0">Freq (MHz)<input type="number" id="rf" value="433.30" step="0.005" style="width:110px"></label>
<label style="margin:0">Bandwidth (kHz)<input type="number" id="rb" value="270" style="width:90px"></label>
<label style="margin:0">Deviation (kHz)<input type="number" id="rd" value="47.6" step="0.1" style="width:90px"></label>
<label style="margin:0">Trigger (dB)<input type="number" id="rt" value="12" style="width:70px"></label>
<button onclick="rctl('start')">Start</button><button class="warn" onclick="rctl('stop')">Stop</button>
<button class="grey" onclick="rctl('clear')">Clear</button>
</div><p id="rs" class="note"></p><div id="rr"></div></div>
<div class="card"><b>Hits</b> <span class="note">newest first</span>
<table><thead><tr><th>Time</th><th>Frequency</th><th>RSSI</th><th>Above floor</th></tr></thead><tbody id="hl"></tbody></table></div>
<script>
let J=null;
async function ctl(a){let b='action='+a+'&start='+fa.value+'&end='+fb.value;
let r=await fetch('/api/scanctl',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b});
ss.textContent=await r.text();setTimeout(upd,500)}
function draw(){const c=cv,x=c.getContext('2d');c.width=c.clientWidth*devicePixelRatio;c.height=c.clientHeight*devicePixelRatio;
const W=c.width,H=c.height,pad=22*devicePixelRatio;x.clearRect(0,0,W,H);if(!J||!J.n)return;
const n=J.n,bw=(W-2*pad)/n,maxdB=50;x.font=(10*devicePixelRatio)+'px sans-serif';x.fillStyle='#777';
for(let d=10;d<=maxdB;d+=10){let y=H-pad-(d/maxdB)*(H-2*pad);x.fillRect(pad,y,W-2*pad,1);x.fillText(d+'dB',2,y+3)}
for(let i=0;i<n;i++){let o=J.peak[i]>-128?J.peak[i]-J.floor[i]:0;if(o<0)o=0;if(o>maxdB)o=maxdB;
let h=(o/maxdB)*(H-2*pad);x.fillStyle=o>=12?'#f55':'#3a7';x.fillRect(pad+i*bw,H-pad-h,Math.max(bw-1,1),h)}
x.fillStyle='#aaa';for(let k=0;k<=4;k++){let i=Math.round(k*(n-1)/4),f=(J.start+i*J.step).toFixed(2);x.fillText(f,pad+i*bw-14*devicePixelRatio,H-6*devicePixelRatio)}}
cv.onclick=e=>{if(!J||!J.n)return;const r=cv.getBoundingClientRect(),pad=22,i=Math.floor((e.clientX-r.left-pad)/((r.width-2*pad)/J.n));
if(i<0||i>=J.n)return;tip.textContent=(J.start+i*J.step).toFixed(3)+' MHz: peak '+J.peak[i]+' dBm, floor '+J.floor[i]+' dBm, hits '+J.hits[i]};
async function upd(){try{J=await (await fetch('/api/scan')).json();
ss.textContent=J.active?(J.learning?'Learning noise floor... (sweep '+J.sweeps+')':'Ready - press the TEST button. Sweeps: '+J.sweeps+', auto-stop in '+Math.floor(J.left/60)+'m '+(J.left%60)+'s'):'Not scanning (alarm monitoring active)';
draw();let h='';for(const e of J.log){h+=`<tr><td>${e.time}</td><td><b>${e.f.toFixed(3)} MHz</b></td><td>${e.rssi} dBm</td><td>+${e.over} dB</td></tr>`}
hl.innerHTML=h||'<tr><td colspan=4 class=note>No hits yet</td></tr>';}catch(e){}}
let lastRaw='';
async function rctl(a){let b='action='+a+'&freq='+rf.value+'&bw='+rb.value+'&dev='+rd.value+'&trig='+rt.value;
let r=await fetch('/api/rawctl',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b});
rs.textContent=await r.text();lastRaw='';setTimeout(rupd,500)}
async function rupd(){try{let j=await (await fetch('/api/raw')).json();
rs.textContent=j.active?(j.recording?'Capturing...':'Listening at '+j.freq.toFixed(3)+' MHz - noise floor '+j.floor+' dBm, triggers at '+(j.floor+j.trig)+' dBm. Captures: '+j.captures+'. Auto-stop in '+Math.floor(j.left/60)+'m'):'Not running (alarm monitoring active)';
let key=j.results.map(r=>r.time).join('|');if(key!==lastRaw){lastRaw=key;let h='';
j.results.forEach((r,i)=>{h+=`<div style="margin-top:10px"><b>${r.time}</b> <button class="grey" style="padding:3px 10px" onclick="cp(${i})">Copy</button>
<pre id="rp${i}" style="background:#151515;padding:8px;border-radius:6px;overflow:auto;max-height:340px;font-size:11px;white-space:pre">${r.text.replace(/</g,'&lt;')}</pre></div>`});
rr.innerHTML=h||'<p class=note>No captures yet</p>';}}catch(e){}}
function cp(i){const t=document.getElementById('rp'+i).textContent;
if(navigator.clipboard&&window.isSecureContext)navigator.clipboard.writeText(t);else{const a=document.createElement('textarea');a.value=t;document.body.appendChild(a);a.select();document.execCommand('copy');a.remove()}
rs.textContent='Copied to clipboard'}
upd();setInterval(upd,1000);window.onresize=draw;rupd();setInterval(rupd,1500);
</script></body></html>)HTML";
  server.send(200, "text/html", s);
}

// =====================================================================
//  setup / loop
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nSmoke Alarm Bridge v" FW_VERSION " on " BOARD_NAME);
  Serial.printf("CC1101 pins: SCK=%d MISO=%d MOSI=%d CSN=%d\n", PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);

  if (!LittleFS.begin(true)) Serial.println("LittleFS mount failed");
  loadConfig();
  String err;
  if (!applyCodes(err)) {
    Serial.println("Config codes invalid (" + err + "), using defaults");
    Config def; cfg.syncHex = def.syncHex; cfg.sysIdHex = def.sysIdHex;
    cfg.alarmHex = def.alarmHex; cfg.silenceHex = def.silenceHex; cfg.testHex = def.testHex;
    applyCodes(err);
  }
  if (configMigrated) { saveConfig(); Serial.println("Config: updated settings saved by an older version"); }
  parseDevices();
  loadLog();

  // radio
  pinMode(PIN_CSN, OUTPUT); digitalWrite(PIN_CSN, HIGH);
  pinMode(PIN_MISO, INPUT_PULLUP);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  ccReset();
  uint8_t ver = ccStatus(0x31);
  radioOk = (ver != 0x00 && ver != 0xFF);
  Serial.printf("CC1101 VERSION=0x%02X %s\n", ver, radioOk ? "OK" : "NOT FOUND");
  if (radioOk) ccConfigure();
  rxQueue = xQueueCreate(64, sizeof(RxFrame));
  rawMutex = xSemaphoreCreateMutex();
  pinMode(PIN_GDO0, INPUT);
  // the ESP32-C3 has a single core, so let FreeRTOS place the task (no core pinning)
  if (radioOk) xTaskCreate(radioTask, "radio", 4096, nullptr, 2, nullptr);

  // Wi-Fi
  WiFi.mode(WIFI_STA);
#if defined(CONFIG_IDF_TARGET_ESP32C3)
  // Many C3 Super Mini boards won't hold a Wi-Fi connection at full power (antenna/regulator
  // design). 8.5 dBm fixes it on most boards with little range lost.
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
#endif
  WiFi.setHostname(cfg.hostname.c_str());
  wm.setConfigPortalTimeout(300);
  if (!wm.autoConnect("SmokeAlarmBridge-Setup")) { Serial.println("Wi-Fi setup timed out, restarting"); ESP.restart(); }
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  configTzTime(cfg.tz.c_str(), "pool.ntp.org", "time.google.com");
  if (MDNS.begin(cfg.hostname.c_str())) MDNS.addService("http", "tcp", 80);

  mqtt.setBufferSize(4096);   // room for the 20-entry log
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(30);

  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/tx", HTTP_POST, handleTx);
  server.on("/api/clearlog", HTTP_POST, handleClearLog);
  server.on("/api/clearbursts", HTTP_POST, handleClearBursts);
  server.on("/api/clearunknown", HTTP_POST, handleClearUnknown);
  server.on("/api/diag", handleDiagApi);
  server.on("/diag", handleDiag);
  server.on("/scan", handleScanPage);
  server.on("/tx", handleTxPage);
  server.on("/api/tx", handleTxApi);
  server.on("/api/txctl", HTTP_POST, handleTxCtl);
  server.on("/api/scan", handleScanApi);
  server.on("/api/scanctl", HTTP_POST, handleScanCtl);
  server.on("/api/raw", handleRawApi);
  server.on("/api/rawctl", HTTP_POST, handleRawCtl);
  server.on("/setup", handleSetup);
  server.on("/network", handleNetwork);
  server.on("/wifireset", HTTP_POST, handleWifiReset);
  server.begin();
  Serial.printf("Web UI: http://%s.local/  or  http://%s/\n", cfg.hostname.c_str(), WiFi.localIP().toString().c_str());
}

void loop() {
  server.handleClient();
  mqttLoop();
  processFrames();

  static uint32_t lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 30000) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
  }
  delay(1);
}
