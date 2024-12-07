#include "yin.h"
#include "BLEHandler.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_private/adc_share_hw_ctrl.h"
#include "esp_adc_cal.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
// #include <NimBLEDevice.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEVICE_NAME "TUNER HRT-1"
#define PIN 32
#define BUFFER_SIZE 2048

#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_12

static adc1_channel_t channel = ADC1_CHANNEL_4;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_12;
static const adc_unit_t unit = ADC_UNIT_1;
adc_oneshot_unit_handle_t adc1_handle;

BLEHandler* bleHandler;
Yin yin;

int16_t* buffer;
bool isBufferFull = false;
int nextBufferItem = 0;
float alpha = 0.15;  
float filteredValue = 0;
int checkPitch = 0;


static void check_efuse(void) {
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

float exponentialMovingAverageFilter(int newValue) {
  filteredValue = alpha * newValue + (1 - alpha) * filteredValue;
  return filteredValue;
}


void readData() {
  while (nextBufferItem < BUFFER_SIZE) {
    // buffer[nextBufferItem++] = exponentialMovingAverageFilter((int16_t)adc1_get_raw((adc1_channel_t)channel));
    buffer[nextBufferItem++] = (int16_t)adc1_get_raw((adc1_channel_t)channel);
    // Serial.println(buffer[nextBufferItem-1]);

    delayMicroseconds(2);
  }

  if (nextBufferItem >= BUFFER_SIZE) {
    isBufferFull = true;
  }
}


void setup() {
  Serial.begin(115200);
  Serial.println("Starting NimBLE Server");

  check_efuse();
  adc1_config_width(width);
  adc1_config_channel_atten(channel, atten);

  buffer = (int16_t*)malloc(BUFFER_SIZE * sizeof(int16_t));
  if (buffer == NULL) {
    Serial.print("Buffer allocation failed");
    return;
  }

  // Initialize YIN pitch detection algorithm
  Yin_init(&yin, BUFFER_SIZE, YIN_DEFAULT_THRESHOLD);

  bleHandler = new BLEHandler(DEVICE_NAME, SERVICE_UUID, CHARACTERISTIC_UUID);
  bleHandler->init();
  bleHandler->startAdvertising();
}


void loop() {
  if (bleHandler->isConnected()) {
    readData();
    if (isBufferFull) {
      nextBufferItem = 0;
      isBufferFull = false;

      float pitch = Yin_getPitch(&yin, buffer);

      if (pitch < 0) {
        pitch = 0;
        checkPitch++;
      } else {
        checkPitch = 0;
      }

      if (checkPitch < 3) {
        Serial.printf("Pitch = %.1f Hz\n", pitch);
        bleHandler->notifyPitch(pitch);
        delay(10);
      }
    }
  }
  delay(2);
}
