#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "yinacf.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_private/adc_share_hw_ctrl.h"
#include "esp_adc_cal.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// Define constants and variables for ADC and BLE
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEVICE_NAME         "TUNER IOT"
#define PIN 32              // Pin where the mic is connected
#define BUFFER_SIZE 1024   // Buffer size for ADC samples
#define PITCH_BUFFER_SIZE 5

#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12

static adc1_channel_t channel = ADC1_CHANNEL_4;    
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_12;
static const adc_unit_t unit = ADC_UNIT_1;

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

adc_oneshot_unit_handle_t adc1_handle;
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

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

static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}

// Function to read ADC data into the buffer
void readData() {
  // unsigned long startTime = millis();
    // Read data from ADC until buffer is full
    while (nextBufferItem < BUFFER_SIZE) {
        buffer[nextBufferItem++] = (int16_t)adc1_get_raw((adc1_channel_t)channel);
        // int adcAnalog;
        // adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &adcAnalog);
        // buffer[nextBufferItem++] = adcAnalog;
        // Serial.printf("Buffer id: %d,   Analog: %d\n", nextBufferItem - 1, buffer[nextBufferItem - 1]);
        // This delay allows for adjusting the sampling rate without blocking
        delayMicroseconds(1);
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

    check_efuse();
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);
    // adc_power_acquire();

    // adc_oneshot_unit_init_cfg_t init_config1 = {
    //     .unit_id = ADC_UNIT_1,
    // };
    // ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // //-------------ADC1 Config---------------//
    // adc_oneshot_chan_cfg_t config = {
    //   .atten = EXAMPLE_ADC_ATTEN,
    //   .bitwidth = ADC_BITWIDTH_12,
    // };
    // ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));

    // adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    // bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_4, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);

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
    pAdvertising->setMinInterval(2000 * 2);  // Set lower frequency (in 0.625 ms units)
    pAdvertising->setMaxInterval(2200 * 2);
    pAdvertising->start();
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


static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}