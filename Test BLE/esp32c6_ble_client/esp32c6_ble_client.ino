/**
 * ESP32-C6 BLE Client (Central) — NimBLE stack
 *
 * - Scans for the server by service UUID
 * - Subscribes to the TX characteristic (receives server notifications)
 * - Writes dummy data to the server's RX characteristic every 3 s
 *
 * Flash this sketch onto ESP32-C6 #2.
 *
 * Required library: NimBLE-Arduino
 *   Arduino IDE  → Library Manager → search "NimBLE-Arduino" → install
 *   Board package: esp32 by Espressif ≥ 3.x (supports C6)
 */

#include <NimBLEDevice.h>

// ── UUIDs — must match the server ────────────────────────────────────────────
#define SERVICE_UUID   "12345678-1234-1234-1234-1234567890ab"
#define CHAR_TX_UUID   "12345678-1234-1234-1234-1234567890cd"  // server → client
#define CHAR_RX_UUID   "12345678-1234-1234-1234-1234567890ef"  // client → server

// ── Globals ───────────────────────────────────────────────────────────────────
NimBLEClient               *pClient   = nullptr;
NimBLERemoteCharacteristic *pTxRemote = nullptr;
NimBLERemoteCharacteristic *pRxRemote = nullptr;
NimBLEAdvertisedDevice     *pTarget   = nullptr;

bool doConnect  = false;
bool connected  = false;
uint32_t counter = 0;

// ── Notification callback — called when server sends data ─────────────────────
void notifyCallback(NimBLERemoteCharacteristic *pChar,
                    uint8_t *pData, size_t length, bool isNotify) {
    Serial.print("[CLIENT] Received from server: ");
    Serial.write(pData, length);
    Serial.println();
}

// ── Client callbacks ──────────────────────────────────────────────────────────
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient *pcl) override {
        Serial.println("[CLIENT] Connected to server");
    }

    void onDisconnect(NimBLEClient *pcl, int reason) override {
        connected = false;
        Serial.printf("[CLIENT] Disconnected (reason: %d) — will re-scan\n", reason);
    }
};

// ── Scan callbacks — fires on each found advertisement ────────────────────────
class ScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *device) override {
        if (device->isAdvertisingService(NimBLEUUID(SERVICE_UUID))) {
            Serial.print("[CLIENT] Found server: ");
            Serial.println(device->toString().c_str());

            NimBLEDevice::getScan()->stop();
            // Keep a copy so we can connect after the scan finishes
            pTarget   = new NimBLEAdvertisedDevice(*device);
            doConnect = true;
        }
    }

    // Called when a scan period ends (if continuous scan is not set)
    void onScanEnd(const NimBLEScanResults &results, int reason) override {
        if (!connected && !doConnect) {
            Serial.println("[CLIENT] Scan ended, server not found — retrying…");
            NimBLEDevice::getScan()->start(10, false);
        }
    }
};

// ── Connect to server and configure characteristics ───────────────────────────
bool connectToServer() {
    Serial.print("[CLIENT] Connecting to ");
    Serial.println(pTarget->getAddress().toString().c_str());

    if (!pClient) {
        pClient = NimBLEDevice::createClient();
        pClient->setClientCallbacks(new ClientCallbacks());
    }

    pClient->setConnectionParams(12, 12, 0, 200);   // interval, latency, timeout
    pClient->setConnectTimeout(10);                  // seconds

    if (!pClient->connect(pTarget)) {
        Serial.println("[CLIENT] Connection failed");
        return false;
    }

    // ── Locate service ────────────────────────────────────────────────────────
    NimBLERemoteService *pService = pClient->getService(SERVICE_UUID);
    if (!pService) {
        Serial.println("[CLIENT] Service not found — disconnecting");
        pClient->disconnect();
        return false;
    }

    // ── TX characteristic (subscribe for notifications) ───────────────────────
    pTxRemote = pService->getCharacteristic(CHAR_TX_UUID);
    if (!pTxRemote) {
        Serial.println("[CLIENT] TX characteristic not found");
        pClient->disconnect();
        return false;
    }
    if (pTxRemote->canNotify()) {
        if (!pTxRemote->subscribe(true, notifyCallback)) {
            Serial.println("[CLIENT] Failed to subscribe to TX notifications");
            pClient->disconnect();
            return false;
        }
        Serial.println("[CLIENT] Subscribed to server notifications");
    }

    // ── RX characteristic (write to send data to server) ─────────────────────
    pRxRemote = pService->getCharacteristic(CHAR_RX_UUID);
    if (!pRxRemote) {
        Serial.println("[CLIENT] RX characteristic not found");
        pClient->disconnect();
        return false;
    }

    connected = true;
    Serial.println("[CLIENT] Setup complete — exchanging data");
    return true;
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(3000);   // allow USB-CDC (HWCDC) to re-enumerate after reset
    Serial.println("\n[CLIENT] ESP32-C6 BLE Client starting…");

    NimBLEDevice::init("C6-BLE-Client");
    NimBLEDevice::setPower(9);

    NimBLEScan *pScan = NimBLEDevice::getScan();
    pScan->setScanCallbacks(new ScanCallbacks());
    pScan->setActiveScan(true);      // active scan to get full advertisement data
    pScan->setInterval(100);
    pScan->setWindow(99);

    Serial.println("[CLIENT] Scanning for server (10 s)…");
    pScan->start(10, false);         // 10-second scan, non-blocking
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    // Attempt connection after scan found the target
    if (doConnect) {
        doConnect = false;
        connectToServer();
    }

    // Write dummy data to the server every 3 seconds
    if (connected && pRxRemote && pRxRemote->canWrite()) {
        char payload[64];
        snprintf(payload, sizeof(payload),
                 "CLIENT_MSG | uptime=%lums | cnt=%u",
                 (unsigned long)millis(), counter++);

        if (pRxRemote->writeValue(payload, strlen(payload), false)) {
            Serial.print("[CLIENT] Sent to server: ");
            Serial.println(payload);
        } else {
            Serial.println("[CLIENT] Write failed — connection may have dropped");
            connected = false;
        }
        delay(3000);
    }

    // Reconnect if disconnected
    if (!connected && !doConnect) {
        delay(2000);
        if (pTarget) {
            // Already know the address — reconnect directly without re-scanning
            Serial.println("[CLIENT] Reconnecting…");
            doConnect = true;
        } else {
            Serial.println("[CLIENT] Re-scanning…");
            NimBLEDevice::getScan()->start(10, false);
        }
    }
}
