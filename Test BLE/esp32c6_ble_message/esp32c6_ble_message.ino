/**
 * ESP32-C6 BLE Message Server
 * ───────────────────────────
 * Features
 *   BLE     – Android reads "HELLO WORLD" from onboard memory, clears it,
 *             or resets it via a command characteristic
 *   LED     – quick flash (150 ms) = message READY
 *             long  flash (1 s)   = message EMPTY
 *   OTA     – firmware update over WiFi (arduino-cli / Arduino IDE)
 *   Web UI  – configure SSID, password, MQTT server on port 80
 *   MQTT    – publishes events: DEVICE_ONLINE, MESSAGE_DOWNLOADED,
 *             MESSAGE_CLEARED, MESSAGE_RESET
 *
 * Required libraries (install via Library Manager):
 *   NimBLE-Arduino   ≥ 2.x
 *   PubSubClient     ≥ 2.8
 *
 * Board FQBN: esp32:esp32:esp32c6:CDCOnBoot=cdc
 *
 * Android BLE app: any "BLE Scanner" app (e.g. nRF Connect) works out of the box.
 * Use "LightBlue" or "Serial Bluetooth Terminal" for a friendlier UI.
 */

#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <PubSubClient.h>

// ── LED pin ───────────────────────────────────────────────────────────────────
// ESP32-C6 DevKitC-1 has a plain LED on GPIO8 (active HIGH).
// If your board uses an RGB/WS2812 LED on GPIO8, replace digitalWrite calls
// with a NeoPixel call and add the Adafruit NeoPixel library.
#ifndef LED_BUILTIN
#define LED_BUILTIN 8
#endif
#define LED_PIN LED_BUILTIN

// ── BLE UUIDs ─────────────────────────────────────────────────────────────────
#define BLE_SVC_UUID    "ab000001-0000-1000-8000-00805f9b34fb"
#define BLE_MSG_UUID    "ab000002-0000-1000-8000-00805f9b34fb"  // READ + NOTIFY
#define BLE_CMD_UUID    "ab000003-0000-1000-8000-00805f9b34fb"  // WRITE
#define BLE_STAT_UUID   "ab000004-0000-1000-8000-00805f9b34fb"  // READ + NOTIFY

// ── NVS keys ──────────────────────────────────────────────────────────────────
#define PREF_NS      "cfg"
#define KEY_SSID     "ssid"
#define KEY_PASS     "pass"
#define KEY_MQTT_H   "mhost"
#define KEY_MQTT_P   "mport"
#define KEY_MSG_OK   "msgok"

// ── Constants ─────────────────────────────────────────────────────────────────
#define DEFAULT_MSG   "HELLO WORLD"
#define AP_SSID       "ESP32C6-Setup"
#define AP_PASS       "12345678"
#define OTA_HOST      "esp32c6-ble"
#define MQTT_TOPIC    "esp32c6/ble/event"

#define LED_QUICK_MS  150   // period half-cycle when READY  (300 ms full cycle)
#define LED_LONG_MS   1000  // period half-cycle when EMPTY (2 s full cycle)

// ── Globals ───────────────────────────────────────────────────────────────────
Preferences  prefs;

// Config (loaded from NVS)
String  cfgSSID, cfgPass, cfgMQTTHost;
uint16_t cfgMQTTPort = 1883;

// State
bool messagePresent = true;
bool bleConnected   = false;
bool wifiConnected  = false;
bool apMode         = false;

// BLE
NimBLEServer         *pBleServer  = nullptr;
NimBLECharacteristic *pMsgChar    = nullptr;
NimBLECharacteristic *pStatChar   = nullptr;

// Network
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
WebServer    webServer(80);

// LED (non-blocking)
unsigned long ledLastToggle = 0;
bool          ledState      = false;

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

    // Persist
    prefs.begin(PREF_NS, false);
    prefs.putBool(KEY_MSG_OK, present);
    prefs.end();

    // Push to BLE characteristics
    pMsgChar->setValue(present ? DEFAULT_MSG : "");
    pStatChar->setValue(present ? "READY" : "EMPTY");

    if (bleConnected) {
        pMsgChar->notify();
        pStatChar->notify();
    }

    Serial.printf("[MSG] State → %s\n", present ? "READY" : "EMPTY");
}

// ── MQTT publish ──────────────────────────────────────────────────────────────
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
        Serial.printf("[BLE] Disconnected (reason %d) — re-advertising\n", reason);
        NimBLEDevice::startAdvertising();
    }
};

// ── BLE: message characteristic – track reads ─────────────────────────────────
class MsgCb : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic*, NimBLEConnInfo &info) override {
        Serial.printf("[BLE] Message READ by %s\n",
                      info.getAddress().toString().c_str());
        if (messagePresent) mqttPublish("MESSAGE_DOWNLOADED");
    }
};

// ── BLE: command characteristic ───────────────────────────────────────────────
// Commands accepted (write as plain UTF-8 string):
//   CLEAR  – erase message (LED → long flash)
//   RESET  – restore "HELLO WORLD" (LED → quick flash)
//   READ   – force notify current message value
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
        } else {
            Serial.println("[BLE] Unknown command (use CLEAR / RESET / READ)");
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
        BLE_MSG_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    pMsgChar->setCallbacks(new MsgCb());
    pMsgChar->setValue(messagePresent ? DEFAULT_MSG : "");

    // Command characteristic
    NimBLECharacteristic *pCmdChar = svc->createCharacteristic(
        BLE_CMD_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pCmdChar->setCallbacks(new CmdCb());

    // Status characteristic
    pStatChar = svc->createCharacteristic(
        BLE_STAT_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    pStatChar->setValue(messagePresent ? "READY" : "EMPTY");

    svc->start();

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SVC_UUID);
    adv->setName("ESP32-C6 BLE");
    adv->enableScanResponse(true);
    NimBLEDevice::startAdvertising();

    Serial.println("[BLE] Advertising started");
}

// ── Web UI ────────────────────────────────────────────────────────────────────
// Served at http://<device-ip>/   or   http://192.168.4.1/ (AP mode)
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
  <form method="POST" action="/save">
    <label>WiFi SSID</label>
    <input name="ssid" value="%SSID%" placeholder="Network name" autocomplete="off">
    <label>WiFi Password</label>
    <input name="pass" type="password" placeholder="Leave blank to keep current">
    <label>MQTT Server</label>
    <input name="mhost" value="%MH%" placeholder="e.g. 192.168.1.10 or broker.hivemq.com">
    <p class="hint">Topic: <code>esp32c6/ble/event</code></p>
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
    return h;
}

void setupWebServer() {
    webServer.on("/", HTTP_GET, []() {
        webServer.send(200, "text/html", buildPage());
    });

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
        // Solid LED during OTA
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
    delay(3000);  // wait for USB-CDC to enumerate after reset
    Serial.println("\n===== ESP32-C6 BLE Message Server =====");

    // LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Load config from NVS
    prefs.begin(PREF_NS, true);
    cfgSSID        = prefs.getString(KEY_SSID,   "");
    cfgPass        = prefs.getString(KEY_PASS,   "");
    cfgMQTTHost    = prefs.getString(KEY_MQTT_H, "");
    cfgMQTTPort    = prefs.getUShort(KEY_MQTT_P, 1883);
    messagePresent = prefs.getBool  (KEY_MSG_OK,  true);
    prefs.end();

    Serial.printf("[CFG] SSID='%s'  MQTT=%s:%u  Message=%s\n",
        cfgSSID.c_str(), cfgMQTTHost.c_str(), cfgMQTTPort,
        messagePresent ? "READY" : "EMPTY");

    // Network stack
    setupWiFi();
    setupWebServer();          // works in both STA and AP mode
    if (wifiConnected) {
        setupOTA();
        connectMQTT();
    }

    // BLE
    setupBLE();

    Serial.println("[SYS] All systems ready\n");
    Serial.println("  BLE  → connect with nRF Connect / LightBlue");
    Serial.printf ("  Web  → http://%s/\n",
        apMode ? WiFi.softAPIP().toString().c_str()
               : WiFi.localIP().toString().c_str());
    if (wifiConnected)
        Serial.printf("  OTA  → arduino-cli upload --port %s.local\n", OTA_HOST);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    if (wifiConnected) {
        ArduinoOTA.handle();

        // MQTT reconnect every 10 s if disconnected
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
