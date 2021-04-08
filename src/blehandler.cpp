#include "blehandler.h"
#include "leds.h"

static CS_CONFIG_t *ble_config;
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
String bleReadBuffer = "";

// TODO: handle advertising restart on BLE disconnect
// TODO: maybe prevent BLE usage when BT is enabled?

class BleServerEvents: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        led_set(LED_BLUE, deviceConnected);
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        led_set(LED_BLUE, deviceConnected);
    }
};

class BleCharacteristicEvents: public BLECharacteristicCallbacks {
    public:
        String buffer;
    
    BleCharacteristicEvents(String &readBuffer) {
        buffer = readBuffer;
    }

    void onWrite(BLECharacteristic* pCharacteristic) {
        if (!ble_config->mode_ble) return;
        
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            for (int i = 0; i < rxValue.length(); i++) {
                char ch = rxValue[i];
                if (ch == '\n' || ch == '\r') {
                    if (buffer != "") {
                        if (ble_config->command_handler) ble_config->command_handler();
                        buffer = "";
                    }
                } else {
                    buffer += ch;
                }
            }
        }

        return;
    }
};

void ble_init() {
    ble_config = getConfig();

    if (ble_config->mode_ble) {
        if (ble_config->mode_debug) Serial.println("BLE '" + String (ble_config->name_ble) + "' started.");

        // Create the BLE device
        BLEDevice::init(ble_config->name_ble);

        // Create the BLE server
        pServer = BLEDevice::createServer();
        pServer->setCallbacks(new BleServerEvents());

        // Create the BLE service
        BLEService *pService = pServer->createService(SERVICE_UUID);

        // Create the TX BLE characteristic
        // FIXME: notify characteristic is size limited (20B), this could cause issues
        pTxCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID_TX,
            BLECharacteristic::PROPERTY_NOTIFY
        );
        pTxCharacteristic->addDescriptor(new BLE2902());

        // Create the RX BLE characteristic
        BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID_RX,
            BLECharacteristic::PROPERTY_WRITE
        );
        pRxCharacteristic->setCallbacks(new BleCharacteristicEvents(bleReadBuffer));

        // Start the service
        pService->start();

        // Start advertising
        pServer->getAdvertising()->addServiceUUID(pService->getUUID());
        pServer->getAdvertising()->start();
    }
}

void readIncomingBle(String &readBuffer) {
    // Empty as handled by event callback    
}

void writeOutgoingBle(String o) {
    if (deviceConnected) {
        pTxCharacteristic->setValue(o.c_str());
        pTxCharacteristic->notify();
        delay(10); // Prevent BLE stack congestion
    }
}