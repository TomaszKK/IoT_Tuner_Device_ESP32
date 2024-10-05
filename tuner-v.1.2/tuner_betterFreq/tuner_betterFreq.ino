#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "yinacf.h"

// Define constants and variables for ADC and BLE
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEVICE_NAME         "TUNER IOT"
#define PIN 32              // Pin where the mic is connected
#define BUFFER_SIZE 1024    // Buffer size for ADC samples
#define PITCH_BUFFER_SIZE 5

Yin yin;
int16_t *buffer;
bool isBufferFull = false;
int nextBufferItem = 0;

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

float pitchBuffer[PITCH_BUFFER_SIZE] = {0};  // Initialize with zeros
int bufferIndex = 0;
float alpha = 0.2;  // Smoothing factor, adjust between 0.0 (very smooth) and 1.0 (no smoothing)
float filteredValue = 0;

float exponentialMovingAverageFilter(int newValue) {
    filteredValue = alpha * newValue + (1 - alpha) * filteredValue;
    return filteredValue;
}


// Setup BLE server callbacks
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        // BLEDevice::stopAdvertising();
        // (pServer->getAdvertising())->stop();
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        // BLEDevice::stopAdvertising();
        // (pServer->getAdvertising())->stop();
    }
};

float calculateAveragePitch() {
    float sum = 0.0;
    int count = 0;

    // Iterate through the buffer and find values that occur multiple times within ±1 Hz
    for (int i = 0; i < PITCH_BUFFER_SIZE; i++) {
        if (pitchBuffer[i] > 0) {  // Exclude zeros
            int occurrenceCount = 0;
            float currentPitch = pitchBuffer[i];

            // Check how many times the current pitch occurs within ±1 Hz in the buffer
            for (int j = 0; j < PITCH_BUFFER_SIZE; j++) {
                if (j != i && fabs(pitchBuffer[j] - currentPitch) <= 10.0) {
                    occurrenceCount++;
                }
            }

            // If the pitch occurs more than once, add it to the sum
            if (occurrenceCount > 0) {  // Meaning it has at least one "close match"
                sum += currentPitch;
                count++;
            }
        }
    }

    // Return the average of the filtered values, or 0 if no valid values were found
    return (count > 0) ? (sum / count) : 0;
}

// Function to read ADC data into the buffer
void readData() {
  // unsigned long startTime = millis();
    // Read data from ADC until buffer is full
    while (nextBufferItem < BUFFER_SIZE) {
        buffer[nextBufferItem++] = (int16_t)exponentialMovingAverageFilter(analogRead(PIN));
        // Serial.printf("Buffer id: %d,   Analog: %d\n", nextBufferItem - 1, buffer[nextBufferItem - 1]);
        // This delay allows for adjusting the sampling rate without blocking
        delayMicroseconds(4);
    }

    // Indicate buffer is full
    if (nextBufferItem >= BUFFER_SIZE) {
        isBufferFull = true;
    }

    // unsigned long endTime = millis();
    // Serial.printf("Time to read data 1111: %lu ms\n", endTime - startTime);
}

void setup() {
    Serial.begin(115200);

    // Allocate memory for the buffer
    buffer = (int16_t*)malloc(BUFFER_SIZE * sizeof(int16_t));
    if (buffer == NULL) {
        Serial.print("Buffer allocation failed");
        return;
    }

    // Initialize YIN pitch detection algorithm
    Yin_init(&yin, BUFFER_SIZE, 0.10);

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
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE 
    );

    // Add a BLE Descriptor
    pCharacteristic->addDescriptor(new BLE2902());

    // Start the service
    pService->start();

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);  // Set value to 0x00 to not advertise this parameter
    pAdvertising->setMinInterval(500);  // Set lower frequency (in 0.625 ms units)
    pAdvertising->setMaxInterval(600);
    // pAdvertising->setAdvertisementType(ADV_TYPE_DIRECT_IND_HIGH);
    pAdvertising->start();
    // pService->stop();
    // Serial.println("Waiting for a client connection to notify...");
}

unsigned long startTime = 0;
unsigned long endTime = 0;

void loop() {
    // Check if device is connected

    // if (deviceConnected) {
        
        // Disable BLE functionality while reading data
        // Stop advertising
        // endTime = millis();
        // Serial.printf("Time to read data: %lu ms\n", endTime - startTime);
        // startTime = millis();
        // Read ADC data and wait for buffer to fill
        readData();

        // Process pitch if buffer is full
        if (isBufferFull) {
            // Reset buffer index and flag
            nextBufferItem = 0;
            isBufferFull = false;

            // Call the pitch detection algorithm with the filled buffer
            float pitch = Yin_getPitch(&yin, buffer);
            pitch = pitch / 2;  // Adjust pitch as needed
            Serial.printf("Pitch = %.1f Hz\n", pitch);  // Print pitch to Serial Monitor
            
            if(pitch == -0.5){
              pitch = 0;
            }

            pitchBuffer[bufferIndex] = pitch;
            bufferIndex++;

        
            if (bufferIndex >= PITCH_BUFFER_SIZE) {
              float avgPitch = calculateAveragePitch();
              // Serial.printf("avg = %.1f Hz\n", avgPitch);
              pCharacteristic->setValue(avgPitch);
              pCharacteristic->notify();
              delay(10);
              bufferIndex = 0;
            }
            // BLEDevice::startAdvertising();
            // pCharacteristic->setValue(pitch);
            // // pCharacteristic->notify();
            // pCharacteristic->notify();
    
            // Serial.printf("Time to read data: %lu ms\n", endTime - startTime);
            // pServer->startAdvertising();  // Restart advertising
        }
    // }

    if (!deviceConnected && oldDeviceConnected) {
        // delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }

    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }

    delay(2);
}
