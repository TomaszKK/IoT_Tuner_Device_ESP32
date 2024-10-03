#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "arduinoFFT.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEVICE_NAME         "TUNER IOT"

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

#define CONVERSIONS_PER_PIN 30
#ifdef CONFIG_IDF_TARGET_ESP32
uint8_t adc_pins[] = {32};  // some of ADC1 pins for ESP32
#else
uint8_t adc_pins[] = {4};   // ADC1 common pins for ESP32S2/S3 + ESP32C3/C6 + ESP32H2
#endif
uint8_t adc_pins_count = sizeof(adc_pins) / sizeof(uint8_t);

const uint16_t samples = 128; // This value MUST ALWAYS be a power of 2
const double samplingFrequency = 2048*2; // Sampling frequency
unsigned long samplingPeriod;
unsigned long microSeconds;

double vReal[samples];
double vImag[samples];

int16_t *buffer; // Buffer to store samples
volatile bool adc_conversion_done = false;
bool isBufferFull = false;
int nextBufferItem = 0;
const int buffer_size = samples; // Set buffer size equal to FFT samples

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
double pitch = 0.0;

adc_continuous_data_t *result = NULL;

/* Create FFT object */
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
        deviceConnected = true;
        BLEDevice::startAdvertising();
    }

    void onDisconnect(BLEServer *pServer) {
        deviceConnected = false;
    }
};

void ARDUINO_ISR_ATTR adcComplete() {
    adc_conversion_done = true;
}

void readData() {
  for(int i = 0; i < samples; i++){
    microSeconds = micros();
    // Serial.println(analogRead(32));
    vReal[i] = analogRead(32);
    vImag[i] = 0;
    if(i == samples - 1){
      isBufferFull = true;
    }
    while(micros() < microSeconds + samplingPeriod){}
  }
  
    // if (adc_conversion_done) {
    //     adc_conversion_done = false;
    //     // Read data from ADC
    //     if (analogContinuousRead(&result, 0)) {
    //         buffer[nextBufferItem] = result[0].avg_read_mvolts; // Store sample in buffer
    //         nextBufferItem++;
    //         delayMicroseconds(22.67);
    //         if (nextBufferItem >= buffer_size) {
    //             isBufferFull = true;
    //             nextBufferItem = 0; // Reset buffer index after it gets full
    //         }
    //     } else {
    //         Serial.println("Error occurred during reading data.");
    //     }
    // }
}

void calculateFFT() {
    // Copy the buffer data to the FFT input arrays
    // for (int i = 0; i < samples; i++) {
    //     vReal[i] = (double) buffer[i]; // Real values from buffer
    //     vImag[i] = 0.0;       // Imaginary part set to 0
    // }

    // Perform FFT
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); // Apply windowing
    FFT.compute(FFTDirection::Forward); // Compute FFT
    FFT.complexToMagnitude(); // Compute magnitudes

    // Find the major peak (dominant frequency)
    pitch = FFT.majorPeak(); // Calculate the peak frequency (pitch)
    Serial.printf("Pitch = %f Hz\n", pitch);
}

void setup() {
    Serial.begin(115200);

    // analogContinuousSetWidth(12);
    // analogContinuousSetAtten(ADC_11db);
    // analogContinuous(adc_pins, adc_pins_count, CONVERSIONS_PER_PIN, (samplingFrequency * CONVERSIONS_PER_PIN), &adcComplete);
    // buffer = (int16_t *)malloc(buffer_size * sizeof(int16_t));
    if (buffer == NULL) {
        Serial.print("Buffer allocation failed");
        return;
    }

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
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY |
        BLECharacteristic::PROPERTY_INDICATE
    );

    // Create a BLE Descriptor
    pCharacteristic->addDescriptor(new BLE2902());

    // Start the service
    pService->start();
    analogReadResolution(12);
    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
    // analogContinuousStart();
    Serial.println("Waiting for a client connection to notify...");

    samplingPeriod = round(1000000*(1.0/samplingFrequency));
}

void loop() {
    // if (deviceConnected) {
        readData();
        if (isBufferFull) {
            // analogContinuousStop(); // Stop continuous ADC reading
            isBufferFull = false;   // Reset buffer flag
            calculateFFT();         // Calculate FFT and find the pitch

            // pCharacteristic->setValue(pitch); // Send the pitch value over BLE
            // pCharacteristic->notify();

            delay(5);
            // analogContinuousStart(); // Restart ADC continuous reading
        }
    // }

    if (!deviceConnected && oldDeviceConnected) {
        analogContinuousStop();
        delay(500); // Give the Bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // Restart advertising
        Serial.println("Start advertising");
        oldDeviceConnected = deviceConnected;
    }

    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
        analogContinuousStart();
    }
}
