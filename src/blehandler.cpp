#include "blehandler.h"
#include "leds.h"

static CS_CONFIG_t *ble_config;
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool bufferAvailable = false;
String bleReadBuffer = "";

// TODO: maybe prevent BLE usage when BT is enabled?

class BleServerEvents : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
        led_set(LED_BLUE, deviceConnected);
    }

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
        led_set(LED_BLUE, deviceConnected);
    }
};

class BleCharacteristicEvents : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        if (!ble_config->mode_ble)
            return;

        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0)
        {
            for (int i = 0; i < rxValue.length(); i++)
            {
                bleReadBuffer += rxValue[i];
                if (i == rxValue.length() - 1)
                {
                    if (bleReadBuffer != "")
                    {
                        if (ble_config->mode_debug & DEBUG_COMMAND)
                            Serial.println("BLE read:" + bleReadBuffer);
                        bufferAvailable = true;
                    }
                }
            }
        }

        return;
    }
};

void ble_init()
{
    ble_config = getConfig();

    if (ble_config->mode_ble)
    {
        if (ble_config->mode_debug)
            Serial.println("BLE '" + String(ble_config->name_ble) + "' started.");

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
            BLECharacteristic::PROPERTY_NOTIFY);
        pTxCharacteristic->addDescriptor(new BLE2902());

        // Create the RX BLE characteristic
        BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID_RX,
            BLECharacteristic::PROPERTY_WRITE);
        pRxCharacteristic->setCallbacks(new BleCharacteristicEvents());

        // Start the service
        pService->start();

        // Start advertising
        pServer->getAdvertising()->addServiceUUID(pService->getUUID());
        pServer->getAdvertising()->start();
    }
}

void readIncomingBle(String &readBuffer)
{
    if (bufferAvailable) {
        readBuffer = bleReadBuffer;

        if (ble_config->command_handler)
            ble_config->command_handler();
        readBuffer = "";
        bleReadBuffer = "";
        bufferAvailable = false;
    }
}

void writeOutgoingBle(String o)
{
    if (deviceConnected)
    {
        pTxCharacteristic->setValue(o.c_str());
        pTxCharacteristic->notify();
        delay(10); // Prevent BLE stack congestion
    }
}

void checkStateBle()
{
    // Disconnecting
    if (!deviceConnected && oldDeviceConnected)
    {
        delay(500);                  // Give time to the bluetooth stack to recover
        pServer->startAdvertising(); // Restart advertising
        if (ble_config->mode_debug)
            Serial.println("BLE start advertising");
        oldDeviceConnected = deviceConnected;
    }

    // Connecting
    if (deviceConnected && !oldDeviceConnected)
    {
        // Things to do when BLE device is connected should go here
        oldDeviceConnected = deviceConnected;
    }
}