#include "yin.h"
#include "BLEHandler.h"
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"

#include <NimBLEDevice.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEVICE_NAME "TUNER HRT-1"
#define PIN 32
#define BUFFER_SIZE 2048 * 2

#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_12
#define EXAMPLE_ADC_UNIT ADC_UNIT_1
#define EXAMPLE_READ_LEN 2048 * 8
#define SAMPLE_FREQUENCY_HZ  107800

BLEHandler* bleHandler;
Yin yin;

int16_t* buffer;
bool isBufferFull = false;
int nextBufferItem = 0;
float alpha = 0.15;
float filteredValue = 0;
int checkPitch = 0;

static TaskHandle_t s_task_handle;
static const char* TAG = "ADC_CONTINUOUS";
adc_continuous_handle_t handle = NULL;


float exponentialMovingAverageFilter(int newValue) {
  filteredValue = alpha * newValue + (1 - alpha) * filteredValue;
  return filteredValue;
}

static bool IRAM_ATTR adc_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t* edata, void* user_data) {
  BaseType_t mustYield = pdFALSE;
  vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
  return (mustYield == pdTRUE);
}


void continuous_adc_init() {
  adc_continuous_handle_cfg_t adc_config = {
    .max_store_buf_size = EXAMPLE_READ_LEN,
    .conv_frame_size = EXAMPLE_READ_LEN,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

  adc_continuous_config_t dig_cfg = {
    .sample_freq_hz = SAMPLE_FREQUENCY_HZ,
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
  };

  adc_digi_pattern_config_t adc_pattern[1] = {
    { .atten = EXAMPLE_ADC_ATTEN,
      .channel = ADC_CHANNEL_4,
      .unit = EXAMPLE_ADC_UNIT,
      .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH }
  };

  dig_cfg.pattern_num = 1;
  dig_cfg.adc_pattern = adc_pattern;

  ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

  adc_continuous_evt_cbs_t cbs = {
    .on_conv_done = adc_conv_done_cb,
  };
  ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
  ESP_ERROR_CHECK(adc_continuous_start(handle));
}

void readData() {
  esp_err_t ret;
  uint32_t ret_num = 0;
  uint8_t* result = NULL;
  result = (uint8_t*)malloc(EXAMPLE_READ_LEN * sizeof(uint8_t));

  if (result == NULL) {
    Serial.println("Memory allocation for result buffer failed");
    return;  // Exit if memory allocation fails
  }


  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  while (nextBufferItem < BUFFER_SIZE) {
    ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);
    // Serial.println(ret_num);
    // Serial.println("......................");
    if (ret == ESP_OK) {
      for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES * 2) {
        adc_digi_output_data_t* p = (adc_digi_output_data_t*)&result[i];
        uint32_t data = exponentialMovingAverageFilter(p->type1.data);
        buffer[nextBufferItem++] = (int16_t)data;
        // Serial.println(data);
        
        if (nextBufferItem >= BUFFER_SIZE) {
          isBufferFull = true;
          // Serial.println("Break for");
          break;
        }
      }
    } else if (ret == ESP_ERR_TIMEOUT) {
      // Serial.println("Break read");
      break;
    }
    // Serial.println("While end");
  }
  
  free(result);

}


void setup() {
  Serial.begin(115200);
  Serial.println("Starting NimBLE Server");

  buffer = (int16_t*)malloc(BUFFER_SIZE * sizeof(int16_t));
  if (buffer == NULL) {
    Serial.print("Buffer allocation failed");
    return;
  }

  // Initialize YIN pitch detection algorithm
  Yin_init(&yin, BUFFER_SIZE, YIN_DEFAULT_THRESHOLD);

  continuous_adc_init();
  s_task_handle = xTaskGetCurrentTaskHandle();

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
      // Serial.printf("Pitch = %.1f Hz\n", pitch);
      if (pitch < 0) {
        pitch = 0;
        checkPitch++;
      } else {
        checkPitch = 0;
      }

      if (checkPitch < 3) {
        Serial.printf("Pitch = %.2f Hz\n", pitch);
        bleHandler->notifyPitch(pitch);
        delay(20);
      }
    }
  }
  delay(2);
}
