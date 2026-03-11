/**
 * ESP32-C6 BLE Message Server + ADC Data Logger  v2.0
 * ──────────────────────────────────────────────────
 * Changelog v2.0 (vs v1.0)
 *   + ADC circular buffer: 200 samples on GPIO2, every 2 s
 *   + Oldest sample overwritten automatically when buffer is full
 *   + NVS persistence (flush every 10 samples, on disconnect & OTA)
 *   + BLE characteristic ab000005: latest ADC value (READ + NOTIFY)
 *   + BLE characteristic ab000006: sample count (READ)
 *   + BLE command CLRADC: clears the ADC buffer
 *   + GET /data web endpoint: JSON array oldest → newest
 *   + MQTT topic esp32c6/ble/adc: {adc, n} per sample
 * ──────────────────────────────────────────────────
 * Features
 *   BLE     – Android reads "HELLO WORLD" from onboard memory, clears it,
 *             or resets it via a command characteristic.
 *             New: ADC latest value notified every 2 s (ab000005).
 *             New: ADC sample count readable (ab000006).
 *             New: CMD "CLRADC" clears the ADC buffer.
 *   ADC     – GPIO2 sampled every 2 s, stored in a 200-sample circular
 *             buffer (oldest entry overwritten when full).
 *             Buffer persisted to NVS every 10 samples.
 *   LED     – quick flash (150 ms) = message READY
 *             long  flash (1 s)   = message EMPTY
 *   OTA     – firmware update over WiFi (arduino-cli / Arduino IDE)
 *   Web UI  – configure SSID / password / MQTT server on port 80
 *             GET /data  → JSON array of ADC samples (oldest first)
 *   MQTT    – publishes events: DEVICE_ONLINE, MESSAGE_DOWNLOADED,
 *             MESSAGE_CLEARED, MESSAGE_RESET, ADC_SAMPLE
 *
 * Required libraries (install via Library Manager):
 *   NimBLE-Arduino   ≥ 2.x
 *   PubSubClient     ≥ 2.8
 *
 * Board FQBN: esp32:esp32:esp32c6:CDCOnBoot=cdc
 */

#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <PubSubClient.h>

// ── LED pin ───────────────────────────────────────────────────────────────────
#ifndef LED_BUILTIN
#define LED_BUILTIN 8
#endif
#define LED_PIN LED_BUILTIN

// ── ADC / Data Logger ─────────────────────────────────────────────────────────
#define ADC_PIN          2       // GPIO2 = ADC1_CH2 on ESP32-C6
#define ADC_INTERVAL_MS  2000   // sample period  (2 seconds)
#define ADC_BUF_SIZE     200    // circular buffer depth

// Ring buffer – indices stored as uint16_t to fit NVS cleanly
uint16_t adcBuf[ADC_BUF_SIZE];  // raw ADC values (0–4095)
uint16_t adcHead  = 0;          // next write slot (oldest after wrap)
uint16_t adcCount = 0;          // samples stored  (0 → 200)
unsigned long adcLastMs = 0;    // millis() of last sample

// NVS flush every N samples to limit flash wear (2 s × 10 = every 20 s)
#define ADC_NVS_EVERY  10
uint16_t adcSinceFlush = 0;

// ── BLE UUIDs ─────────────────────────────────────────────────────────────────
#define BLE_SVC_UUID    "ab000001-0000-1000-8000-00805f9b34fb"
#define BLE_MSG_UUID    "ab000002-0000-1000-8000-00805f9b34fb"  // READ + NOTIFY
#define BLE_CMD_UUID    "ab000003-0000-1000-8000-00805f9b34fb"  // WRITE
#define BLE_STAT_UUID   "ab000004-0000-1000-8000-00805f9b34fb"  // READ + NOTIFY
#define BLE_ADC_UUID    "ab000005-0000-1000-8000-00805f9b34fb"  // READ + NOTIFY  (latest ADC)
#define BLE_ADCN_UUID   "ab000006-0000-1000-8000-00805f9b34fb"  // READ           (sample count)

// ── NVS keys ──────────────────────────────────────────────────────────────────
#define PREF_NS       "cfg"
#define KEY_SSID      "ssid"
#define KEY_PASS      "pass"
#define KEY_MQTT_H    "mhost"
#define KEY_MQTT_P    "mport"
#define KEY_MSG_OK    "msgok"
#define KEY_ADC_BUF   "adcbuf"    // 400-byte blob
#define KEY_ADC_HEAD  "adchead"
#define KEY_ADC_CNT   "adccnt"

// ── Constants ─────────────────────────────────────────────────────────────────
#define DEFAULT_MSG   "HELLO WORLD"
#define AP_SSID       "ESP32C6-Setup"
#define AP_PASS       "12345678"
#define OTA_HOST      "esp32c6-ble"
#define MQTT_TOPIC    "esp32c6/ble/event"
#define MQTT_ADC_TOPIC "esp32c6/ble/adc"

#define LED_QUICK_MS  150
#define LED_LONG_MS   1000

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
NimBLECharacteristic *pAdcChar    = nullptr;  // latest ADC value
NimBLECharacteristic *pAdcNChar   = nullptr;  // sample count

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
WebServer    webServer(80);

unsigned long ledLastToggle = 0;
bool          ledState      = false;

// ── ADC helpers ───────────────────────────────────────────────────────────────

// Return the i-th oldest sample (i=0 → oldest, i=adcCount-1 → newest)
uint16_t adcGetOrdered(uint16_t i) {
    if (adcCount < ADC_BUF_SIZE) {
        // Buffer not yet full: slot 0 is the very first sample
        return adcBuf[i];
    }
    // Buffer full: oldest is at adcHead
    return adcBuf[(adcHead + i) % ADC_BUF_SIZE];
}

void adcFlushNVS() {
    prefs.begin(PREF_NS, false);
    prefs.putBytes(KEY_ADC_BUF,  adcBuf,  sizeof(adcBuf));
    prefs.putUShort(KEY_ADC_HEAD, adcHead);
    prefs.putUShort(KEY_ADC_CNT,  adcCount);
    prefs.end();
    adcSinceFlush = 0;
    Serial.println("[ADC] Buffer flushed to NVS");
}

void adcClearBuffer() {
    memset(adcBuf, 0, sizeof(adcBuf));
    adcHead  = 0;
    adcCount = 0;
    adcSinceFlush = 0;
    adcFlushNVS();

    // Update BLE count characteristic
    pAdcNChar->setValue("0");
    if (bleConnected) pAdcNChar->notify();

    Serial.println("[ADC] Buffer cleared");
}

// Called every ADC_INTERVAL_MS from loop()
void adcSample() {
    uint16_t val = (uint16_t)analogRead(ADC_PIN);

    // Write into ring buffer (overwrites oldest when full)
    adcBuf[adcHead] = val;
    adcHead = (adcHead + 1) % ADC_BUF_SIZE;
    if (adcCount < ADC_BUF_SIZE) adcCount++;

    adcSinceFlush++;
    if (adcSinceFlush >= ADC_NVS_EVERY) {
        adcFlushNVS();
    }

    // BLE: notify latest value
    char s[8];
    snprintf(s, sizeof(s), "%u", val);
    pAdcChar->setValue(s);
    if (bleConnected) pAdcChar->notify();

    // BLE: update count
    char cnt[8];
    snprintf(cnt, sizeof(cnt), "%u", adcCount);
    pAdcNChar->setValue(cnt);

    // MQTT
    char msg[48];
    snprintf(msg, sizeof(msg), "{\"adc\":%u,\"n\":%u}", val, adcCount);
    if (mqtt.connected()) mqtt.publish(MQTT_ADC_TOPIC, msg);

    Serial.printf("[ADC] #%u  GPIO%d → %u\n", adcCount, ADC_PIN, val);
}

// ── LED ───────────────────────────────────────────────────────────────────────
void updateLed() {
    unsigned long interval = messagePresent ? LED_QUICK_MS : LED_LONG_MS;
    if (millis() - ledLastToggle >= interval) {
        ledLastToggle = millis();
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
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

    if (bleConnected) {
        pMsgChar->notify();
        pStatChar->notify();
    }

    Serial.printf("[MSG] State → %s\n", present ? "READY" : "EMPTY");
}

// ── MQTT publish (event topic) ────────────────────────────────────────────────
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
        Serial.printf("[BLE] Android connected: %s\n",
                      info.getAddress().toString().c_str());
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo &info, int reason) override {
        bleConnected = false;
        // Flush ADC buffer on disconnect to ensure NVS is up to date
        if (adcSinceFlush > 0) adcFlushNVS();
        Serial.printf("[BLE] Disconnected (reason %d) — re-advertising\n", reason);
        NimBLEDevice::startAdvertising();
    }
};

// ── BLE: message characteristic ───────────────────────────────────────────────
class MsgCb : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic*, NimBLEConnInfo &info) override {
        Serial.printf("[BLE] Message READ by %s\n",
                      info.getAddress().toString().c_str());
        if (messagePresent) mqttPublish("MESSAGE_DOWNLOADED");
    }
};

// ── BLE: command characteristic ───────────────────────────────────────────────
// Commands (UTF-8 string):
//   CLEAR   – erase BLE message
//   RESET   – restore "HELLO WORLD"
//   READ    – force notify current message
//   CLRADC  – clear ADC circular buffer
class CmdCb : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &info) override {
        std::string val = pChar->getValue();
        Serial.printf("[BLE] CMD: '%s' from %s\n",
                      val.c_str(), info.getAddress().toString().c_str());

        if (val == "CLEAR") {
            applyMessageState(false);
            mqttPublish("MESSAGE_CLEARED");
        } else if (val == "RESET") {
            applyMessageState(true);
            mqttPublish("MESSAGE_RESET");
        } else if (val == "READ") {
            pMsgChar->notify();
            if (messagePresent) mqttPublish("MESSAGE_DOWNLOADED");
        } else if (val == "CLRADC") {
            adcClearBuffer();
            mqttPublish("ADC_CLEARED");
        } else {
            Serial.println("[BLE] Unknown command (CLEAR/RESET/READ/CLRADC)");
        }
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

    // ADC latest value (READ + NOTIFY, updated every 2 s)
    pAdcChar = svc->createCharacteristic(
        BLE_ADC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    pAdcChar->setValue("0");

    // ADC sample count (READ only)
    pAdcNChar = svc->createCharacteristic(
        BLE_ADCN_UUID, NIMBLE_PROPERTY::READ);
    char cnt[8];
    snprintf(cnt, sizeof(cnt), "%u", adcCount);
    pAdcNChar->setValue(cnt);

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
  .adcinfo{margin-top:16px;padding:10px;background:#e8f4fd;border-radius:8px;
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
  <div class="adcinfo">
    ADC GPIO%AP% &nbsp;|&nbsp; Samples stored: <b>%AN%/200</b>
    &nbsp;|&nbsp; Latest: <b>%AV%</b>
    &nbsp;&nbsp;<a href="/data">Download JSON</a>
  </div>
  <form method="POST" action="/save">
    <label>WiFi SSID</label>
    <input name="ssid" value="%SSID%" placeholder="Network name" autocomplete="off">
    <label>WiFi Password</label>
    <input name="pass" type="password" placeholder="Leave blank to keep current">
    <label>MQTT Server</label>
    <input name="mhost" value="%MH%" placeholder="e.g. 192.168.1.10 or broker.hivemq.com">
    <p class="hint">Event topic: <code>esp32c6/ble/event</code> &nbsp; ADC topic: <code>esp32c6/ble/adc</code></p>
    <label>MQTT Port</label>
    <input name="mport" value="%MP%" placeholder="1883" type="number" min="1" max="65535">
    <button class="btn" type="submit">Save &amp; Reboot</button>
  </form>
</div>
</body></html>
)HTML";

String buildPage() {
    String h = FPSTR(HTML);
    h.replace("%SSID%", cfgSSID);
    h.replace("%MH%",   cfgMQTTHost);
    h.replace("%MP%",   String(cfgMQTTPort));
    h.replace("%WS%",   wifiConnected ? "Connected"    : "Disconnected");
    h.replace("%WC%",   wifiConnected ? "on"           : "off");
    h.replace("%MS%",   mqtt.connected() ? "Connected" : "Disconnected");
    h.replace("%MC%",   mqtt.connected() ? "on"        : "off");
    h.replace("%GS%",   messagePresent ? "READY"       : "EMPTY");
    h.replace("%GC%",   messagePresent ? "on"          : "off");
    h.replace("%AP%",   String(ADC_PIN));
    h.replace("%AN%",   String(adcCount));
    h.replace("%AV%",   adcCount ? String(adcGetOrdered(adcCount - 1)) : "—");
    return h;
}

// /data  →  JSON array ordered oldest → newest
void handleDataEndpoint() {
    // Build JSON in chunks to avoid huge String allocation
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.send(200, "application/json", "");

    String chunk = "{\"count\":";
    chunk += adcCount;
    chunk += ",\"pin\":";
    chunk += ADC_PIN;
    chunk += ",\"values\":[";
    webServer.sendContent(chunk);

    for (uint16_t i = 0; i < adcCount; i++) {
        String v = String(adcGetOrdered(i));
        if (i < adcCount - 1) v += ',';
        webServer.sendContent(v);
    }
    webServer.sendContent("]}");
    webServer.sendContent("");  // end chunked response
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
        if (ss.length())  prefs.putString(KEY_SSID,   ss);
        if (pp.length())  prefs.putString(KEY_PASS,   pp);
        if (mh.length())  prefs.putString(KEY_MQTT_H, mh);
        if (mp.length())  prefs.putUShort(KEY_MQTT_P, (uint16_t)mp.toInt());
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
        if (adcSinceFlush > 0) adcFlushNVS();
        ledLastToggle = 0; ledState = true;
        digitalWrite(LED_PIN, HIGH);
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
    Serial.println("\n===== ESP32-C6 BLE Message Server + ADC Logger =====");

    // LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // ADC pin
    pinMode(ADC_PIN, INPUT);

    // Load config + ADC buffer from NVS
    prefs.begin(PREF_NS, true);
    cfgSSID        = prefs.getString(KEY_SSID,   "");
    cfgPass        = prefs.getString(KEY_PASS,   "");
    cfgMQTTHost    = prefs.getString(KEY_MQTT_H, "");
    cfgMQTTPort    = prefs.getUShort(KEY_MQTT_P, 1883);
    messagePresent = prefs.getBool  (KEY_MSG_OK,  true);
    adcHead        = prefs.getUShort(KEY_ADC_HEAD, 0);
    adcCount       = prefs.getUShort(KEY_ADC_CNT,  0);
    prefs.getBytes(KEY_ADC_BUF, adcBuf, sizeof(adcBuf));
    prefs.end();

    Serial.printf("[CFG] SSID='%s'  MQTT=%s:%u  Message=%s\n",
        cfgSSID.c_str(), cfgMQTTHost.c_str(), cfgMQTTPort,
        messagePresent ? "READY" : "EMPTY");
    Serial.printf("[ADC] Restored %u samples from NVS (head=%u)\n",
        adcCount, adcHead);

    // Network stack
    setupWiFi();
    setupWebServer();
    if (wifiConnected) {
        setupOTA();
        connectMQTT();
    }

    // BLE (must happen after adcCount is restored so count char is correct)
    setupBLE();

    // Stagger first sample so we don't hit NVS immediately at boot
    adcLastMs = millis();

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
    Serial.printf("[ADC] Sampling GPIO%d every %d ms, buffer %d slots\n",
        ADC_PIN, ADC_INTERVAL_MS, ADC_BUF_SIZE);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    // ADC sampling (non-blocking, every 2 s)
    if (millis() - adcLastMs >= ADC_INTERVAL_MS) {
        adcLastMs = millis();
        adcSample();
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
