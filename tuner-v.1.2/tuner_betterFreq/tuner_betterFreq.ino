#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "yinacf.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Define constants for ADC and BLE
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEVICE_NAME         "TUNER IOT"
#define PIN                 32              // Pin where the mic is connected
#define BUFFER_SIZE         1024            // Buffer size for ADC samples

// Initialize YIN pitch detection
Yin yin;
int16_t *buffer;
bool isBufferFull = false;
int nextBufferItem = 0;
float pitchValue = 0.0;

// BLE server and characteristic
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool isSend = false;

#define PITCH_BUFFER_SIZE 10  // Define the size of the pitch buffer

// Create a buffer to store the last 5 pitch values
float pitchBuffer[PITCH_BUFFER_SIZE] = {0};  // Initialize with zeros
int bufferIndex = 0;

// Function to calculate the average pitch (excluding zeros)
float calculateAveragePitch() {
    float sum = 0.0;
    int count = 0;

    // Iterate through the buffer and sum non-zero values
    for (int i = 0; i < PITCH_BUFFER_SIZE; i++) {
        if (pitchBuffer[i] > 0) {  // Exclude zeros
            sum += pitchBuffer[i];
            count++;
        }
    }

    // If there are no valid values, return 0. Otherwise, return the average.
    return (count > 0) ? (sum / count) : 0;
}

// BLE server callbacks to manage connection state
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        // Stop advertising when connected to save CPU resources
        BLEDevice::getAdvertising()->stop();
        Serial.println("Client connected.");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        // Restart advertising to allow new connections
        BLEDevice::getAdvertising()->start();
        Serial.println("Client disconnected. Restarting advertising.");
    }
};

// Define a mutex handle
SemaphoreHandle_t xSerialMutex;

// Task 1: Handles ADC reading and pitch calculation (runs on Core 1)
void audioProcessingTask(void *param) {
    while (true) {
       if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE) {  
        // Fill buffer with ADC data
        if (!isBufferFull && !isSend) {
            // Serial.println("Audio");
              while (nextBufferItem < BUFFER_SIZE) {
                      // Serial.println(nextBufferItem);  // Protected by mutex
                   

                  buffer[nextBufferItem++] = (int16_t)analogRead(PIN);
                  delayMicroseconds(4);  // Adjust for correct sampling rate
              }
              isBufferFull = true;
          }
      
        

        // Process pitch once buffer is full
        if (isBufferFull && !isSend) {
            nextBufferItem = 0;
            isBufferFull = false;
            isSend = true;

            // Calculate pitch
            pitchValue = Yin_getPitch(&yin, buffer) / 2.0;  // Adjust pitch as needed

            // if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE) {  // Take mutex for serial print
                // Serial.printf("Pitch = %.1f Hz\n", pitchValue);  // Protected by mutex
                // xSemaphoreGive(xSerialMutex);  // Release mutex
            // }

            // Optional: Add a small delay to prevent overloading the CPU
            // delayMicroseconds(5);
         
        }
       xSemaphoreGive(xSerialMutex);  
    }
  }
}

// Task 2: Handles Bluetooth communication (runs on Core 0)
void bluetoothTask(void *param) {
    while (true) {
        if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE) {
            // Serial.println("Bluetooth");

            if (deviceConnected && isSend) {
                pitchBuffer[bufferIndex] = pitchValue;
                bufferIndex++;

                float avgPitch = calculateAveragePitch();

                if (bufferIndex >= PITCH_BUFFER_SIZE) {
                    Serial.printf("\navg = %.1f Hz\n", avgPitch);
                    pCharacteristic->setValue(avgPitch);
                    pCharacteristic->notify();
                    bufferIndex = 0;
                }

                isSend = false;
            }

            xSemaphoreGive(xSerialMutex);  // Always release the semaphore
        }

        delay(2);
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize mutex for serial access
    xSerialMutex = xSemaphoreCreateMutex();

    // Allocate memory for the buffer
    buffer = (int16_t*)malloc(BUFFER_SIZE * sizeof(int16_t));
    if (buffer == NULL) {
        Serial.println("Buffer allocation failed");
        while (1);  // Halt execution if buffer allocation fails
    }

    // Initialize YIN pitch detection algorithm
    Yin_init(&yin, BUFFER_SIZE, 0.10);

    // Initialize BLE
    BLEDevice::init(DEVICE_NAME);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );

    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();

    // Configure advertising but do not start it yet
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);

    // Start advertising to allow client connections
    BLEDevice::getAdvertising()->start();
    Serial.println("Advertising started. Waiting for a client connection to notify...");

    // Create the audio processing task on Core 1
    xTaskCreatePinnedToCore(
        audioProcessingTask,   // Task function
        "Audio Processing",    // Name of the task
        10000,                 // Stack size in words
        NULL,                  // Task input parameters
        1,                     // Priority of the task
        NULL,                  // Task handle
        1                      // Core where the task should run (Core 1)
    );

    // Create the Bluetooth communication task on Core 0
    xTaskCreatePinnedToCore(
        bluetoothTask,         // Task function
        "Bluetooth Task",      // Name of the task
        10000,                 // Stack size in words
        NULL,                  // Task input parameters
        2,                     // Priority of the task
        NULL,                  // Task handle
        1                      // Core where the task should run (Core 0)
    );
}

void loop() {
  // Serial.println("loop");
    // Handle device connection/disconnection and advertisement
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);  // Delay for Bluetooth stack to reset
        pServer->startAdvertising();  // Restart advertising
        oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
    delay(2);
}