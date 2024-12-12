#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <NimBLEDevice.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEVICE_NAME "TUNER HRT-1"

static NimBLEServer* pServer;


class BLEHandler {
public:
    BLEHandler(const std::string& deviceName, const std::string& serviceUUID, const std::string& characteristicUUID);
    void init();
    void startAdvertising();
    void notifyPitch(float pitch);
    bool isConnected();

private:
    class ServerCallbacks : public NimBLEServerCallbacks {
      void onConnect(NimBLEServer* pServer) override;
      void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override;
      void onDisconnect(NimBLEServer* pServer) override;
      void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) override;
      uint32_t onPassKeyRequest() override;
      bool onConfirmPIN(uint32_t pass_key) override;
      void onAuthenticationComplete(ble_gap_conn_desc* desc) override;
    };

    class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
        void onWrite(NimBLECharacteristic* pCharacteristic) override;
        void onRead(NimBLECharacteristic* pCharacteristic) override;
        void onNotify(NimBLECharacteristic* pCharacteristic) override;
        void onStatus(NimBLECharacteristic* pCharacteristic, Status status, int code) override;
        void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) override;
    };

    NimBLEService* pService;
    NimBLECharacteristic* pCharacteristic;
    std::string serviceUUID;
    std::string characteristicUUID;
};


// #ifdef __cplusplus
// }
// #endif

#endif // BLE_HANDLER_H
