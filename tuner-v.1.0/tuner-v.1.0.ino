#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include <TaskScheduler.h>

/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updated by chegewara

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 4fafc201-1fb5-459e-8fcc-c5c9c331914b
   And has a characteristic of: beb5483e-36e1-4688-b7f5-ea07361b26a8

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   A connect hander associated with the server starts a background task that performs notification
   every couple of seconds.
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "yinacf.h"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

Yin yin;
size_t buffer_size = 1024 * 2;
int16_t *buffer;
bool isBufferFull = false;
int nextBufferItem = 0;
int threshold = 0;
float accuracyTab[3];

// Flag which will be set in ISR when conversion is done
volatile bool adc_coversion_done = false;

// Result structure for ADC Continuous reading
adc_continuous_data_t *result = NULL;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEVICE_NAME         "TUNER IOT"

#define CONVERSIONS_PER_PIN 20
#ifdef CONFIG_IDF_TARGET_ESP32
uint8_t adc_pins[] = {32};  //some of ADC1 pins for ESP32
#else
uint8_t adc_pins[] = {4};  //ADC1 common pins for ESP32S2/S3 + ESP32C3/C6 + ESP32H2
#endif
uint8_t adc_pins_count = sizeof(adc_pins) / sizeof(uint8_t);

bool timeLog = true;
static unsigned long startTime = 0;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void ARDUINO_ISR_ATTR adcComplete() {
  adc_coversion_done = true;
}

void readData(){
   if (adc_coversion_done == true) {
    // Set ISR flag back to false
    adc_coversion_done = false;
   
    // Read data from ADC
    if (analogContinuousRead(&result, 0)) {
       if(!isBufferFull){
          // Serial.printf("%d\n", micOut);
          // Serial.printf("\n   Avg raw value = %d", result[0].avg_read_raw);
          buffer[nextBufferItem] = result[0].avg_read_mvolts;  // Store sample in buffer
          nextBufferItem++;
          //delayMicroseconds(22.67/10);
          if(nextBufferItem >= buffer_size){
            isBufferFull = true;
          }
        }
    } else {
      Serial.println("Error occurred during reading data. Set Core Debug Level to error or lower for more information.");
    }
  }
}

void setup() {
  Serial.begin(115200);

  // analogReadResolution(12);
  analogContinuousSetWidth(12);
  analogContinuousSetAtten(ADC_11db);
  analogContinuous(adc_pins, adc_pins_count, CONVERSIONS_PER_PIN, (YIN_SAMPLING_RATE * CONVERSIONS_PER_PIN + 200000), &adcComplete);

  buffer = (int16_t*)malloc(buffer_size * sizeof(int16_t)); 
  if (buffer == NULL) {
    Serial.print("Buffer don't work");
    return;
  }    
  Yin_init(&yin, buffer_size, 0.15);

  // Create the BLE Device
  BLEDevice::init(DEVICE_NAME);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
  // analogContinuousStart();
}

void loop() {
   if (deviceConnected) {
      readData();
      if(isBufferFull){
        analogContinuousStop();
        nextBufferItem = 0;
        isBufferFull = 0;
        float pitch = Yin_getPitch(&yin, buffer);
        Serial.printf("Pitch = %f\n", pitch);
        if(pitch == -1){
          pitch = 0;
        }
        // Zalezy co w momencie zmiany dzwieku czy nie bvedzie delaya
        switch(threshold){
          case 0:
            if(pitch == 0){
              pCharacteristic->setValue(pitch);
              pCharacteristic->notify();
              break;
            }
            accuracyTab[threshold] = pitch;
            threshold++;
            break;
          case 1:
            if(pitch != 0){
              accuracyTab[threshold] = pitch;
              threshold++;
            }
            else{
              threshold = 0;
              pCharacteristic->setValue(pitch);
              pCharacteristic->notify();
            }
            break;
          case 2:
            if(pitch != 0){
              accuracyTab[threshold] = pitch;
              pitch = (accuracyTab[0] + accuracyTab[1] + accuracyTab[2]) / 3;
              pCharacteristic->setValue(pitch);
              pCharacteristic->notify();
              threshold = 0;
            }
            break;
        }

        // pCharacteristic->setValue(pitch);
        // pCharacteristic->notify();
    
        delay(5);
        analogContinuousStart();
      }
    }
   
    if (!deviceConnected && oldDeviceConnected) {
        analogContinuousStop();
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }

    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
        analogContinuousStart();
    }
}