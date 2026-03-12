/**
 * ESP32-C6 BLE Message Server + AHT20+BMP280 Data Logger  v4.0
 * ─────────────────────────────────────────────────────────────
 * Changelog v4.0 (vs v3.0)
 *   ~ ADC replaced with AHT20+BMP280 I2C sensor
 *   ~ I2C pins: SDA=GPIO20, SCL=GPIO18
 *   ~ Sample interval: 500 ms → 1000 ms
 *   ~ Buffer size: 200 → 20 samples (circular, oldest overwritten)
 *   + Each sample stores: temp_aht, humidity, pressure, temp_bmp
 *   ~ LED EMPTY   (0 samples)   : slow RED flash  (1 s on / 1 s off)
 *   ~ LED FILLING (1-19 samples): fast BLUE flash (150 ms cycle)
 *   ~ LED FULL    (20 samples)  : fast GREEN flash (300 ms cycle)
 *   + OTA: solid WHITE during update
 * ─────────────────────────────────────────────────────────────
 * Features
 *   BLE     – "HELLO WORLD" message + command characteristic
 *             CMD "CLRBUF" clears the sensor buffer
 *             Latest sensor reading notified every 1000 ms (ab000005)
 *             Sample count readable (ab000006)
 *   Sensor  – AHT20 (temp + humidity) + BMP280 (pressure + temp)
 *             sampled every 1000 ms into a 20-sample circular buffer
 *             Buffer persisted to NVS every 5 samples
 *   LED     – WS2812 RGB on GPIO8:
 *               EMPTY   (0 samples) → slow RED flash  (1 s / 1 s)
 *               FILLING (1–19)      → fast BLUE flash (150 ms cycle)
 *               FULL    (20 samples)→ fast GREEN flash (300 ms cycle)
 *   OTA     – firmware update over WiFi
 *   Web UI  – configure SSID / password / MQTT on port 80
 *             GET /data → JSON array of sensor samples (oldest first)
 *   MQTT    – publishes events and sensor readings
 *
 * Required libraries (install via Library Manager):
 *   NimBLE-Arduino     >= 2.x
 *   PubSubClient       >= 2.8
 *   Adafruit NeoPixel  >= 1.10
 *   Adafruit AHTX0     >= 2.0
 *   Adafruit BMP280    >= 2.6
 *   Adafruit Unified Sensor >= 1.1
 *
 * Board FQBN: esp32:esp32:esp32c6:CDCOnBoot=cdc
 */

#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// ── I2C pins ──────────────────────────────────────────────────────────────────
#define I2C_SDA   20
#define I2C_SCL   18

Adafruit_AHTX0  aht;
Adafruit_BMP280 bmp;
bool ahtOk = false;
bool bmpOk = false;

// ── RGB LED (WS2812 NeoPixel on GPIO8) ───────────────────────────────────────
#define NEOPIXEL_PIN   8
#define NEOPIXEL_COUNT 1
#define LED_BRIGHTNESS 40   // 0–255, keep low to avoid USB power issues

Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

enum LedMode { LED_EMPTY, LED_FILLING, LED_FULL };
LedMode       ledMode    = LED_EMPTY;
bool          ledState   = false;
unsigned long ledLastMs  = 0;

// Blink half-periods per mode (ms)
#define LED_EMPTY_HALF   1000   // slow red:   1 s on / 1 s off
#define LED_FILLING_HALF  150   // fast blue: 150 ms on / 150 ms off
#define LED_FULL_HALF     300   // fast green:300 ms on / 300 ms off

// ── Sensor Data Logger ────────────────────────────────────────────────────────
#define SAMPLE_INTERVAL_MS 1000
#define BUF_SIZE            20

struct SensorSample {
    float temp_aht;   // °C  from AHT20
    float humidity;   // %RH from AHT20
    float pressure;   // hPa from BMP280
    float temp_bmp;   // °C  from BMP280
};

SensorSample  sampleBuf[BUF_SIZE];
uint16_t      bufHead       = 0;   // next write slot
uint16_t      bufCount      = 0;   // samples stored (0–20)
unsigned long sampleLastMs  = 0;

#define NVS_FLUSH_EVERY 5
uint16_t sinceFlush = 0;

// ── BLE UUIDs ─────────────────────────────────────────────────────────────────
#define BLE_SVC_UUID    "ab000001-0000-1000-8000-00805f9b34fb"
#define BLE_MSG_UUID    "ab000002-0000-1000-8000-00805f9b34fb"  // READ + NOTIFY
#define BLE_CMD_UUID    "ab000003-0000-1000-8000-00805f9b34fb"  // WRITE
#define BLE_STAT_UUID   "ab000004-0000-1000-8000-00805f9b34fb"  // READ + NOTIFY
#define BLE_SENSOR_UUID "ab000005-0000-1000-8000-00805f9b34fb"  // READ + NOTIFY (latest reading)
#define BLE_COUNT_UUID  "ab000006-0000-1000-8000-00805f9b34fb"  // READ (sample count)

// ── NVS keys ──────────────────────────────────────────────────────────────────
#define PREF_NS     "cfg"
#define KEY_SSID    "ssid"
#define KEY_PASS    "pass"
#define KEY_MQTT_H  "mhost"
#define KEY_MQTT_P  "mport"
#define KEY_MSG_OK  "msgok"
#define KEY_SBUF    "sbuf"
#define KEY_SHEAD   "shead"
#define KEY_SCNT    "scnt"

// ── Constants ─────────────────────────────────────────────────────────────────
#define DEFAULT_MSG       "HELLO WORLD"
#define AP_SSID           "ESP32C6-Setup"
#define AP_PASS           "12345678"
#define OTA_HOST          "esp32c6-ble"
#define MQTT_TOPIC        "esp32c6/ble/event"
#define MQTT_SENSOR_TOPIC "esp32c6/ble/sensor"

// ── Globals ───────────────────────────────────────────────────────────────────
Preferences  prefs;

String   cfgSSID, cfgPass, cfgMQTTHost;
uint16_t cfgMQTTPort = 1883;

bool messagePresent = true;
bool bleConnected   = false;
bool wifiConnected  = false;
bool apMode         = false;

NimBLEServer         *pBleServer  = nullptr;
NimBLECharacteristic *pMsgChar    = nullptr;
NimBLECharacteristic *pStatChar   = nullptr;
NimBLECharacteristic *pSensorChar = nullptr;
NimBLECharacteristic *pCountChar  = nullptr;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
WebServer    webServer(80);

// ── Buffer helpers ────────────────────────────────────────────────────────────

// Return the i-th oldest sample (i=0 → oldest, i=bufCount-1 → newest)
SensorSample getOrdered(uint16_t i) {
    if (bufCount < BUF_SIZE) return sampleBuf[i];
    return sampleBuf[(bufHead + i) % BUF_SIZE];
}

void flushNVS() {
    prefs.begin(PREF_NS, false);
    prefs.putBytes(KEY_SBUF,   sampleBuf, sizeof(sampleBuf));
    prefs.putUShort(KEY_SHEAD, bufHead);
    prefs.putUShort(KEY_SCNT,  bufCount);
    prefs.end();
    sinceFlush = 0;
    Serial.println("[SENSOR] Buffer flushed to NVS");
}

void clearBuffer() {
    memset(sampleBuf, 0, sizeof(sampleBuf));
    bufHead    = 0;
    bufCount   = 0;
    sinceFlush = 0;
    flushNVS();
    updateLedMode();
    pCountChar->setValue("0");
    if (bleConnected) pCountChar->notify();
    Serial.println("[SENSOR] Buffer cleared");
}

void updateLedMode() {
    if      (bufCount == 0)          ledMode = LED_EMPTY;
    else if (bufCount >= BUF_SIZE)   ledMode = LED_FULL;
    else                             ledMode = LED_FILLING;
}

// ── Sensor sampling ───────────────────────────────────────────────────────────
void takeSample() {
    SensorSample s = {0.0f, 0.0f, 0.0f, 0.0f};

    if (ahtOk) {
        sensors_event_t hum, temp;
        aht.getEvent(&hum, &temp);
        s.temp_aht = temp.temperature;
        s.humidity = hum.relative_humidity;
    }
    if (bmpOk) {
        s.temp_bmp = bmp.readTemperature();
        s.pressure = bmp.readPressure() / 100.0f;  // Pa → hPa
    }

    // Write into ring buffer (overwrites oldest when full)
    sampleBuf[bufHead] = s;
    bufHead = (bufHead + 1) % BUF_SIZE;
    if (bufCount < BUF_SIZE) bufCount++;

    updateLedMode();

    sinceFlush++;
    if (sinceFlush >= NVS_FLUSH_EVERY) flushNVS();

    // BLE: notify latest reading
    char sensorStr[80];
    snprintf(sensorStr, sizeof(sensorStr), "T:%.1f H:%.1f P:%.1f Tb:%.1f",
             s.temp_aht, s.humidity, s.pressure, s.temp_bmp);
    pSensorChar->setValue(sensorStr);
    if (bleConnected) pSensorChar->notify();

    // BLE: update count
    char cntStr[8];
    snprintf(cntStr, sizeof(cntStr), "%u", bufCount);
    pCountChar->setValue(cntStr);

    // MQTT
    char mqttMsg[128];
    snprintf(mqttMsg, sizeof(mqttMsg),
        "{\"temp_aht\":%.1f,\"humidity\":%.1f,\"pressure\":%.1f,\"temp_bmp\":%.1f,\"n\":%u}",
        s.temp_aht, s.humidity, s.pressure, s.temp_bmp, bufCount);
    if (mqtt.connected()) mqtt.publish(MQTT_SENSOR_TOPIC, mqttMsg);

    Serial.printf("[SENSOR] #%u  AHT:%.1f°C %.1f%%RH  BMP:%.1f°C %.1fhPa  LED=%s\n",
        bufCount, s.temp_aht, s.humidity, s.temp_bmp, s.pressure,
        ledMode == LED_EMPTY ? "RED" : ledMode == LED_FILLING ? "BLUE" : "GREEN");
}

// ── LED (NeoPixel RGB) ────────────────────────────────────────────────────────
void updateLed() {
    unsigned long halfCycle;
    uint32_t colour;

    switch (ledMode) {
        case LED_EMPTY:
            halfCycle = LED_EMPTY_HALF;
            colour    = strip.Color(LED_BRIGHTNESS, 0, 0);   // RED
            break;
        case LED_FILLING:
            halfCycle = LED_FILLING_HALF;
            colour    = strip.Color(0, 0, LED_BRIGHTNESS);   // BLUE
            break;
        default: // LED_FULL
            halfCycle = LED_FULL_HALF;
            colour    = strip.Color(0, LED_BRIGHTNESS, 0);   // GREEN
            break;
    }

    if (millis() - ledLastMs >= halfCycle) {
        ledLastMs = millis();
        ledState  = !ledState;
        strip.setPixelColor(0, ledState ? colour : 0);
        strip.show();
    }
}

// ── Message state ─────────────────────────────────────────────────────────────
void applyMessageState(bool present) {
    messagePresent = present;
    prefs.begin(PREF_NS, false);
    prefs.putBool(KEY_MSG_OK, present);
    prefs.end();
    pMsgChar->setValue(present ? DEFAULT_MSG : "");
    pStatChar->setValue(present ? "READY" : "EMPTY");
    if (bleConnected) { pMsgChar->notify(); pStatChar->notify(); }
    Serial.printf("[MSG] State → %s\n", present ? "READY" : "EMPTY");
}

void mqttPublish(const char *event) {
    if (mqtt.connected()) {
        mqtt.publish(MQTT_TOPIC, event);
        Serial.printf("[MQTT] Published: %s\n", event);
    }
}

// ── BLE: server callbacks ─────────────────────────────────────────────────────
class BleServerCb : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, NimBLEConnInfo &info) override {
        bleConnected = true;
        Serial.printf("[BLE] Connected: %s\n", info.getAddress().toString().c_str());
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo &info, int reason) override {
        bleConnected = false;
        if (sinceFlush > 0) flushNVS();
        Serial.printf("[BLE] Disconnected (reason %d) — re-advertising\n", reason);
        NimBLEDevice::startAdvertising();
    }
};

// ── BLE: message read callback ────────────────────────────────────────────────
class MsgCb : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic*, NimBLEConnInfo &info) override {
        Serial.printf("[BLE] Message READ by %s\n", info.getAddress().toString().c_str());
        if (messagePresent) mqttPublish("MESSAGE_DOWNLOADED");
    }
};

// ── BLE: command characteristic ───────────────────────────────────────────────
// Commands (UTF-8 string):
//   CLEAR   – erase BLE message
//   RESET   – restore "HELLO WORLD"
//   READ    – force notify current message
//   CLRBUF  – clear sensor circular buffer
class CmdCb : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &info) override {
        std::string val = pChar->getValue();
        Serial.printf("[BLE] CMD: '%s' from %s\n",
                      val.c_str(), info.getAddress().toString().c_str());

        if      (val == "CLEAR")  { applyMessageState(false); mqttPublish("MESSAGE_CLEARED"); }
        else if (val == "RESET")  { applyMessageState(true);  mqttPublish("MESSAGE_RESET"); }
        else if (val == "READ")   { pMsgChar->notify(); if (messagePresent) mqttPublish("MESSAGE_DOWNLOADED"); }
        else if (val == "CLRBUF") { clearBuffer(); mqttPublish("BUFFER_CLEARED"); }
        else Serial.println("[BLE] Unknown command (CLEAR/RESET/READ/CLRBUF)");
    }
};

// ── BLE init ──────────────────────────────────────────────────────────────────
void setupBLE() {
    NimBLEDevice::init("ESP32-C6 BLE");
    NimBLEDevice::setPower(9);

    pBleServer = NimBLEDevice::createServer();
    pBleServer->setCallbacks(new BleServerCb());

    NimBLEService *svc = pBleServer->createService(BLE_SVC_UUID);

    // Message characteristic
    pMsgChar = svc->createCharacteristic(
        BLE_MSG_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    pMsgChar->setCallbacks(new MsgCb());
    pMsgChar->setValue(messagePresent ? DEFAULT_MSG : "");

    // Command characteristic
    NimBLECharacteristic *pCmdChar = svc->createCharacteristic(
        BLE_CMD_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    pCmdChar->setCallbacks(new CmdCb());

    // Status characteristic
    pStatChar = svc->createCharacteristic(
        BLE_STAT_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    pStatChar->setValue(messagePresent ? "READY" : "EMPTY");

    // Latest sensor reading (READ + NOTIFY, updated every 1 s)
    pSensorChar = svc->createCharacteristic(
        BLE_SENSOR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    pSensorChar->setValue("T:0.0 H:0.0 P:0.0 Tb:0.0");

    // Sample count (READ only)
    pCountChar = svc->createCharacteristic(
        BLE_COUNT_UUID, NIMBLE_PROPERTY::READ);
    char cnt[8];
    snprintf(cnt, sizeof(cnt), "%u", bufCount);
    pCountChar->setValue(cnt);

    svc->start();

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SVC_UUID);
    adv->setName("ESP32-C6 BLE");
    adv->enableScanResponse(true);
    NimBLEDevice::startAdvertising();

    Serial.println("[BLE] Advertising started");
}

// ── Web UI ────────────────────────────────────────────────────────────────────
static const char HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-C6 Config</title>
<style>
  *{box-sizing:border-box}
  body{font-family:Arial,sans-serif;background:#f0f2f5;margin:0;padding:24px}
  h2{color:#1a1a2e;margin-bottom:20px}
  .card{background:#fff;border-radius:10px;padding:24px;max-width:480px;
        margin:auto;box-shadow:0 2px 12px rgba(0,0,0,.1)}
  .row{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:16px}
  .badge{padding:3px 10px;border-radius:20px;font-size:12px;font-weight:700}
  .on{background:#d4edda;color:#155724}.off{background:#f8d7da;color:#721c24}
  label{display:block;margin-top:14px;font-size:13px;color:#555;font-weight:600}
  input{width:100%;padding:9px 11px;margin-top:5px;border:1px solid #ccc;
        border-radius:6px;font-size:14px;transition:border .2s}
  input:focus{outline:none;border-color:#0078d4}
  .btn{display:block;width:100%;margin-top:22px;padding:11px;font-size:15px;
       font-weight:600;color:#fff;background:#0078d4;border:none;
       border-radius:6px;cursor:pointer;transition:background .2s}
  .btn:hover{background:#005fa3}
  .hint{font-size:11px;color:#888;margin-top:4px}
  .sensorinfo{margin-top:16px;padding:10px;background:#e8f4fd;border-radius:8px;
              font-size:13px;color:#0277bd}
</style>
</head>
<body>
<div class="card">
  <h2>ESP32-C6 Configuration</h2>
  <div class="row">
    <span>WiFi:</span><span class="badge %WC%">%WS%</span>
    <span>MQTT:</span><span class="badge %MC%">%MS%</span>
    <span>Message:</span><span class="badge %GC%">%GS%</span>
  </div>
  <div class="sensorinfo">
    AHT20: <b>%AHTST%</b> &nbsp;|&nbsp; BMP280: <b>%BMPST%</b>
    &nbsp;(SDA=GPIO20, SCL=GPIO18)<br>
    Samples stored: <b>%AN%/20</b>
    &nbsp;|&nbsp; Latest &mdash; Temp: <b>%TV%&deg;C</b>
    &nbsp; Hum: <b>%HV%%</b> &nbsp; Pres: <b>%PV% hPa</b>
    &nbsp;&nbsp;<a href="/data">Download JSON</a>
  </div>
  <form method="POST" action="/save">
    <label>WiFi SSID</label>
    <input name="ssid" value="%SSID%" placeholder="Network name" autocomplete="off">
    <label>WiFi Password</label>
    <input name="pass" type="password" placeholder="Leave blank to keep current">
    <label>MQTT Server</label>
    <input name="mhost" value="%MH%" placeholder="e.g. 192.168.1.10 or broker.hivemq.com">
    <p class="hint">Event topic: <code>esp32c6/ble/event</code> &nbsp; Sensor topic: <code>esp32c6/ble/sensor</code></p>
    <label>MQTT Port</label>
    <input name="mport" value="%MP%" placeholder="1883" type="number" min="1" max="65535">
    <button class="btn" type="submit">Save &amp; Reboot</button>
  </form>
</div>
</body></html>
)HTML";

String buildPage() {
    String h = FPSTR(HTML);
    h.replace("%SSID%",  cfgSSID);
    h.replace("%MH%",    cfgMQTTHost);
    h.replace("%MP%",    String(cfgMQTTPort));
    h.replace("%WS%",    wifiConnected    ? "Connected"    : "Disconnected");
    h.replace("%WC%",    wifiConnected    ? "on"           : "off");
    h.replace("%MS%",    mqtt.connected() ? "Connected"    : "Disconnected");
    h.replace("%MC%",    mqtt.connected() ? "on"           : "off");
    h.replace("%GS%",    messagePresent   ? "READY"        : "EMPTY");
    h.replace("%GC%",    messagePresent   ? "on"           : "off");
    h.replace("%AHTST%", ahtOk            ? "OK"           : "ERROR");
    h.replace("%BMPST%", bmpOk            ? "OK"           : "ERROR");
    h.replace("%AN%",    String(bufCount));
    if (bufCount > 0) {
        SensorSample latest = getOrdered(bufCount - 1);
        h.replace("%TV%", String(latest.temp_aht, 1));
        h.replace("%HV%", String(latest.humidity,  1));
        h.replace("%PV%", String(latest.pressure,  1));
    } else {
        h.replace("%TV%", "—");
        h.replace("%HV%", "—");
        h.replace("%PV%", "—");
    }
    return h;
}

// GET /data → JSON array ordered oldest → newest
void handleDataEndpoint() {
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.send(200, "application/json", "");

    String chunk = "{\"count\":";
    chunk += bufCount;
    chunk += ",\"interval_ms\":";
    chunk += SAMPLE_INTERVAL_MS;
    chunk += ",\"samples\":[";
    webServer.sendContent(chunk);

    for (uint16_t i = 0; i < bufCount; i++) {
        SensorSample s = getOrdered(i);
        char entry[128];
        snprintf(entry, sizeof(entry),
            "{\"temp_aht\":%.1f,\"humidity\":%.1f,\"pressure\":%.1f,\"temp_bmp\":%.1f}",
            s.temp_aht, s.humidity, s.pressure, s.temp_bmp);
        webServer.sendContent(String(entry));
        if (i < bufCount - 1) webServer.sendContent(",");
    }
    webServer.sendContent("]}");
    webServer.sendContent("");
}

void setupWebServer() {
    webServer.on("/", HTTP_GET, []() {
        webServer.send(200, "text/html", buildPage());
    });

    webServer.on("/data", HTTP_GET, handleDataEndpoint);

    webServer.on("/save", HTTP_POST, []() {
        String ss = webServer.arg("ssid");
        String pp = webServer.arg("pass");
        String mh = webServer.arg("mhost");
        String mp = webServer.arg("mport");

        prefs.begin(PREF_NS, false);
        if (ss.length()) prefs.putString(KEY_SSID,   ss);
        if (pp.length()) prefs.putString(KEY_PASS,   pp);
        if (mh.length()) prefs.putString(KEY_MQTT_H, mh);
        if (mp.length()) prefs.putUShort(KEY_MQTT_P, (uint16_t)mp.toInt());
        prefs.end();

        webServer.send(200, "text/html",
            "<html><body style='font-family:Arial;text-align:center;padding:48px'>"
            "<h2>Saved! Rebooting&hellip;</h2></body></html>");
        delay(1500);
        ESP.restart();
    });

    webServer.begin();
    Serial.println("[WEB] Server ready on port 80");
}

// ── WiFi ──────────────────────────────────────────────────────────────────────
void setupWiFi() {
    if (cfgSSID.isEmpty()) {
        Serial.println("[WiFi] No SSID — starting AP");
        goto ap;
    }

    Serial.printf("[WiFi] Connecting to '%s'", cfgSSID.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfgSSID.c_str(), cfgPass.c_str());

    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
        delay(500); Serial.print('.');
    }

    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
        return;
    }
    Serial.println("\n[WiFi] Failed — starting AP");

ap:
    apMode = true;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[WiFi] AP '%s'  IP: %s\n", AP_SSID,
                  WiFi.softAPIP().toString().c_str());
}

// ── OTA ───────────────────────────────────────────────────────────────────────
void setupOTA() {
    ArduinoOTA.setHostname(OTA_HOST);
    ArduinoOTA.onStart([]() {
        Serial.println("[OTA] Starting…");
        NimBLEDevice::stopAdvertising();
        if (sinceFlush > 0) flushNVS();
        // Solid white during OTA
        strip.setPixelColor(0, strip.Color(LED_BRIGHTNESS, LED_BRIGHTNESS, LED_BRIGHTNESS));
        strip.show();
    });
    ArduinoOTA.onEnd([]()  { Serial.println("\n[OTA] Done"); });
    ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
        Serial.printf("[OTA] %u%%\r", (p * 100) / t);
    });
    ArduinoOTA.onError([](ota_error_t e) {
        Serial.printf("[OTA] Error[%u]\n", e);
    });
    ArduinoOTA.begin();
    Serial.printf("[OTA] Hostname: %s.local\n", OTA_HOST);
}

// ── MQTT ──────────────────────────────────────────────────────────────────────
void connectMQTT() {
    if (cfgMQTTHost.isEmpty() || !wifiConnected) return;
    mqtt.setServer(cfgMQTTHost.c_str(), cfgMQTTPort);
    Serial.printf("[MQTT] Connecting to %s:%u… ", cfgMQTTHost.c_str(), cfgMQTTPort);
    if (mqtt.connect("ESP32C6-BLE")) {
        Serial.println("OK");
        mqttPublish("DEVICE_ONLINE");
    } else {
        Serial.printf("failed (state=%d)\n", mqtt.state());
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(3000);
    Serial.println("\n===== ESP32-C6 BLE + AHT20+BMP280 Data Logger =====");

    // NeoPixel LED init
    strip.begin();
    strip.setBrightness(LED_BRIGHTNESS);
    strip.clear();
    strip.show();

    // I2C init
    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.printf("[I2C] SDA=GPIO%d  SCL=GPIO%d\n", I2C_SDA, I2C_SCL);

    // AHT20
    ahtOk = aht.begin();
    Serial.printf("[AHT20] %s\n", ahtOk ? "OK" : "NOT FOUND — check wiring");

    // BMP280 — try address 0x76 first, then 0x77
    bmpOk = bmp.begin(0x76);
    if (!bmpOk) bmpOk = bmp.begin(0x77);
    if (bmpOk) {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X2,
                        Adafruit_BMP280::SAMPLING_X16,
                        Adafruit_BMP280::FILTER_X16,
                        Adafruit_BMP280::STANDBY_MS_500);
    }
    Serial.printf("[BMP280] %s\n", bmpOk ? "OK" : "NOT FOUND — check wiring");

    // Load config + sensor buffer from NVS
    prefs.begin(PREF_NS, true);
    cfgSSID        = prefs.getString(KEY_SSID,   "");
    cfgPass        = prefs.getString(KEY_PASS,   "");
    cfgMQTTHost    = prefs.getString(KEY_MQTT_H, "");
    cfgMQTTPort    = prefs.getUShort(KEY_MQTT_P, 1883);
    messagePresent = prefs.getBool  (KEY_MSG_OK,  true);
    bufHead        = prefs.getUShort(KEY_SHEAD, 0);
    bufCount       = prefs.getUShort(KEY_SCNT,  0);
    prefs.getBytes(KEY_SBUF, sampleBuf, sizeof(sampleBuf));
    prefs.end();

    Serial.printf("[CFG] SSID='%s'  MQTT=%s:%u  Message=%s\n",
        cfgSSID.c_str(), cfgMQTTHost.c_str(), cfgMQTTPort,
        messagePresent ? "READY" : "EMPTY");
    Serial.printf("[BUF] Restored %u/%u samples from NVS (head=%u)\n",
        bufCount, BUF_SIZE, bufHead);
    updateLedMode();

    // Network stack
    setupWiFi();
    setupWebServer();
    if (wifiConnected) {
        setupOTA();
        connectMQTT();
    }

    // BLE (after bufCount restored so count char is initialised correctly)
    setupBLE();

    sampleLastMs = millis();

    Serial.println("[SYS] All systems ready\n");
    Serial.println("  BLE  → connect with nRF Connect / LightBlue");
    Serial.printf ("  Web  → http://%s/\n",
        apMode ? WiFi.softAPIP().toString().c_str()
               : WiFi.localIP().toString().c_str());
    Serial.printf ("  Data → http://%s/data\n",
        apMode ? WiFi.softAPIP().toString().c_str()
               : WiFi.localIP().toString().c_str());
    if (wifiConnected)
        Serial.printf("  OTA  → arduino-cli upload --port %s.local\n", OTA_HOST);
    Serial.printf("[SENSOR] Sampling every %d ms into %d-slot buffer\n",
        SAMPLE_INTERVAL_MS, BUF_SIZE);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    // Sensor sampling (non-blocking, every 1000 ms)
    if (millis() - sampleLastMs >= SAMPLE_INTERVAL_MS) {
        sampleLastMs = millis();
        takeSample();
    }

    if (wifiConnected) {
        ArduinoOTA.handle();

        if (!cfgMQTTHost.isEmpty() && !mqtt.connected()) {
            static unsigned long lastRetry = 0;
            if (millis() - lastRetry > 10000) {
                lastRetry = millis();
                connectMQTT();
            }
        }
        mqtt.loop();
    }

    webServer.handleClient();
    updateLed();
}
