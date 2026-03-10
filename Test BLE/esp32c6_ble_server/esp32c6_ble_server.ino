/**
 * ESP32-C6 BLE Server (Peripheral) — NimBLE stack
 *
 * - Advertises a custom GATT service
 * - TX characteristic: server notifies client with dummy data every 2 s
 * - RX characteristic: client writes dummy data here; server prints it
 *
 * Flash this sketch onto ESP32-C6 #1.
 *
 * Required library: NimBLE-Arduino
 *   Arduino IDE  → Library Manager → search "NimBLE-Arduino" → install
 *   Board package: esp32 by Espressif ≥ 3.x (supports C6)
 */

#include <NimBLEDevice.h>

// ── UUIDs ─────────────────────────────────────────────────────────────────────
#define SERVICE_UUID   "12345678-1234-1234-1234-1234567890ab"
#define CHAR_TX_UUID   "12345678-1234-1234-1234-1234567890cd"  // server → client
#define CHAR_RX_UUID   "12345678-1234-1234-1234-1234567890ef"  // client → server

// ── Globals ───────────────────────────────────────────────────────────────────
NimBLEServer          *pServer  = nullptr;
NimBLECharacteristic  *pTxChar  = nullptr;
bool                   connected = false;
uint32_t               counter   = 0;

// ── Server callbacks ──────────────────────────────────────────────────────────
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *pSrv, NimBLEConnInfo &info) override {
        connected = true;
        Serial.print("[SERVER] Client connected: ");
        Serial.println(info.getAddress().toString().c_str());
        // Stop advertising while a client is connected (optional)
        NimBLEDevice::stopAdvertising();
    }

    void onDisconnect(NimBLEServer *pSrv, NimBLEConnInfo &info, int reason) override {
        connected = false;
        Serial.printf("[SERVER] Client disconnected (reason: %d) — re-advertising\n", reason);
        NimBLEDevice::startAdvertising();
    }
};

// ── RX characteristic callbacks — fired when client writes ────────────────────
class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &info) override {
        std::string value = pChar->getValue();
        Serial.print("[SERVER] Received from client: ");
        Serial.println(value.c_str());
    }
};

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(3000);   // allow USB-CDC (HWCDC) to re-enumerate after reset
    Serial.println("\n[SERVER] ESP32-C6 BLE Server starting…");

    NimBLEDevice::init("C6-BLE-Server");
    NimBLEDevice::setPower(9);          // max TX power (dBm); adjust as needed

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService *pService = pServer->createService(SERVICE_UUID);

    // ── TX characteristic (server → client, notify) ──────────────────────────
    pTxChar = pService->createCharacteristic(
        CHAR_TX_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    // ── RX characteristic (client → server, write) ────────────────────────────
    NimBLECharacteristic *pRxChar = pService->createCharacteristic(
        CHAR_RX_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pRxChar->setCallbacks(new RxCallbacks());

    pService->start();

    // ── Advertising ───────────────────────────────────────────────────────────
    NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->setName("C6-BLE-Server");
    pAdv->enableScanResponse(true);
    NimBLEDevice::startAdvertising();

    Serial.println("[SERVER] Advertising — waiting for client…");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    if (connected) {
        char payload[64];
        snprintf(payload, sizeof(payload),
                 "SERVER_MSG | uptime=%lums | cnt=%u",
                 (unsigned long)millis(), counter++);

        pTxChar->setValue(payload);
        pTxChar->notify();

        Serial.print("[SERVER] Notified client: ");
        Serial.println(payload);

        delay(2000);
    } else {
        // Print a heartbeat every 5 s so the monitor shows activity
        static unsigned long lastBeat = 0;
        if (millis() - lastBeat >= 5000) {
            lastBeat = millis();
            Serial.println("[SERVER] Waiting for BLE client…");
        }
        delay(200);
    }
}
