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

void readData() {
    while (nextBufferItem < BUFFER_SIZE) {
        buffer[nextBufferItem++] = (int16_t)adc1_get_raw((adc1_channel_t)channel);
        delayMicroseconds(1);
    }

    if (nextBufferItem >= BUFFER_SIZE) {
        isBufferFull = true;
    }    
}

void setup() {
    Serial.begin(115200);

    check_efuse();
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);

    buffer = (int16_t*)malloc(BUFFER_SIZE * sizeof(int16_t));
    if (buffer == NULL) {
        Serial.print("Buffer allocation failed");
        return;
    }

    // Initialize YIN pitch detection algorithm
    Yin_init(&yin, BUFFER_SIZE, 0.10);


}



void loop() {

        readData();

        if (isBufferFull) {
       
            nextBufferItem = 0;
            isBufferFull = false;

            float pitch = Yin_getPitch(&yin, buffer);
            pitch = pitch / 2; 
            Serial.printf("Pitch = %.1f Hz\n", pitch); 
            
            if(pitch == -0.5){
              pitch = 0;
            }

           
    
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