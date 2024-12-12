#include "BLEHandler.h"
#include <Arduino.h>

BLEHandler::BLEHandler(const std::string& deviceName, const std::string& serviceUUID, const std::string& characteristicUUID)
    : serviceUUID(serviceUUID), characteristicUUID(characteristicUUID) {
    NimBLEDevice::init(deviceName);
    NimBLEDevice::setPower(ESP_PWR_LVL_N12); // Set transmit power
    NimBLEDevice::setMTU(128);              // Set MTU size
}

void BLEHandler::init() {
  NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_SC);  /*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ 
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  pService = pServer->createService(serviceUUID);
  pCharacteristic = pService->createCharacteristic(
      characteristicUUID,
      NIMBLE_PROPERTY::NOTIFY
  );
  pCharacteristic->setCallbacks(new CharacteristicCallbacks());
  pService->start();
}

void BLEHandler::startAdvertising() {
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());

  pAdvertising->setMaxInterval(12);
  pAdvertising->setMinInterval(12);
  // pAdvertising->setMaxPreferred(12);
  // pAdvertising->setMinPreferred(12);
  // pAdvertising->addTxPower();

  std::string macAddress = NimBLEDevice::getAddress().toString();
  pAdvertising->setManufacturerData(macAddress);

  pAdvertising->setScanResponse(true);
  pAdvertising->start();
  Serial.println("Advertising Started");
}

void BLEHandler::notifyPitch(float pitch) {
  uint8_t pitchBytes[sizeof(float)];
  memcpy(pitchBytes, &pitch, sizeof(float));
  pCharacteristic->setValue(pitchBytes, sizeof(float));
  pCharacteristic->notify();
}

bool BLEHandler::isConnected(){
  if(pServer->getConnectedCount()){
    return true;
  }
  return false;
}

void BLEHandler::ServerCallbacks::onConnect(NimBLEServer* pServer) {
    Serial.println("Client connected");
    NimBLEDevice::startAdvertising();
}

void BLEHandler::ServerCallbacks::onDisconnect(NimBLEServer* pServer) {
    Serial.println("Client disconnected");
    NimBLEDevice::startAdvertising();
}

void BLEHandler::ServerCallbacks::onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
  Serial.print("Client address: ");
  Serial.println(NimBLEAddress(desc->peer_ota_addr).toString().c_str());
  pServer->updateConnParams(desc->conn_handle, 6, 6, 0, 10);
};

void BLEHandler::ServerCallbacks::onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
  pServer->updateConnParams(desc->conn_handle, 6, 6, 0, 10);
  Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);
};

uint32_t BLEHandler::ServerCallbacks::onPassKeyRequest() {
  Serial.println("Server Passkey Request");
  return 123456;
};

bool BLEHandler::ServerCallbacks::onConfirmPIN(uint32_t pass_key) {
  Serial.print("The passkey YES/NO number: ");
  Serial.println(pass_key);
  return true;
};

void BLEHandler::ServerCallbacks::onAuthenticationComplete(ble_gap_conn_desc* desc) {
  /** Check that encryption was successful, if not we disconnect the client */
  if (!desc->sec_state.encrypted) {
    NimBLEDevice::getServer()->disconnect(desc->conn_handle);
    Serial.println("Encrypt connection failed - disconnecting client");
    return;
  }
  Serial.println("Starting BLE work!");
};


void BLEHandler::CharacteristicCallbacks::onWrite(NimBLECharacteristic* pCharacteristic) {
  Serial.print(pCharacteristic->getUUID().toString().c_str());
  Serial.print(": onWrite(), value: ");
  Serial.println(pCharacteristic->getValue().c_str());
}

void BLEHandler::CharacteristicCallbacks::onRead(NimBLECharacteristic* pCharacteristic) {
  Serial.print(pCharacteristic->getUUID().toString().c_str());
  Serial.print(": onRead(), value: ");
  Serial.println(pCharacteristic->getValue().c_str());
}

void BLEHandler::CharacteristicCallbacks::onNotify(NimBLECharacteristic* pCharacteristic){
  // Serial.println("Sending notification to clients");
};

void BLEHandler::CharacteristicCallbacks::onStatus(NimBLECharacteristic* pCharacteristic, Status status, int code) {
  String str = ("Notification/Indication status code: ");
  str += status;
  str += ", return code: ";
  str += code;
  str += ", ";
  str += NimBLEUtils::returnCodeToString(code);
  // Serial.println(str);
};

void BLEHandler::CharacteristicCallbacks::onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
  String str = "Client ID: ";
  str += desc->conn_handle;
  str += " Address: ";
  str += std::string(NimBLEAddress(desc->peer_ota_addr)).c_str();
  if (subValue == 0) {
    str += " Unsubscribed to ";
  } else if (subValue == 1) {
    str += " Subscribed to notfications for ";
  } else if (subValue == 2) {
    str += " Subscribed to indications for ";
  } else if (subValue == 3) {
    str += " Subscribed to notifications and indications for ";
  }
  str += std::string(pCharacteristic->getUUID()).c_str();

  // Serial.println(str);
};
