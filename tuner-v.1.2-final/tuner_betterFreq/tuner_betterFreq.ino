#include "yinacf.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_private/adc_share_hw_ctrl.h"
#include "esp_adc_cal.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <NimBLEDevice.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEVICE_NAME         "TUNER HRT-1"
#define PIN 32             
#define BUFFER_SIZE 1024 * 2
#define PITCH_BUFFER_SIZE 1

#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12

static adc1_channel_t channel = ADC1_CHANNEL_4;    
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_12;
static const adc_unit_t unit = ADC_UNIT_1;
adc_oneshot_unit_handle_t adc1_handle;

Yin yin;
int16_t *buffer;
bool isBufferFull = false;
int nextBufferItem = 0;
float pitchBuffer[PITCH_BUFFER_SIZE] = {0};  // Initialize with zeros
int bufferIndex = 0;
float alpha = 0.15;  // Smoothing factor, adjust between 0.0 (very smooth) and 1.0 (no smoothing)
float filteredValue = 0;

static NimBLEServer* pServer;
static BLEDescriptorCallbacks dscCallbacks;
static BLECharacteristicCallbacks chrCallbacks;

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

float exponentialMovingAverageFilter(int newValue) {
    filteredValue = alpha * newValue + (1 - alpha) * filteredValue;
    return filteredValue;
}

float calculateAveragePitch() {
    float sum = 0.0;
    int count = 0;

   for (int i = 0; i < PITCH_BUFFER_SIZE; i++) {
        if (pitchBuffer[i] > 0) {  // Exclude zeros
            int occurrenceCount = 0;
            float currentPitch = pitchBuffer[i];

            // Check how many times the current pitch occurs within ±1 Hz in the buffer
            for (int j = 0; j < PITCH_BUFFER_SIZE; j++) {
                if (j != i && fabs(pitchBuffer[j] - currentPitch) <= 3.0) {
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

void readData() {
    while (nextBufferItem < BUFFER_SIZE) {
        buffer[nextBufferItem++] = exponentialMovingAverageFilter((int16_t)adc1_get_raw((adc1_channel_t)channel));
        // buffer[nextBufferItem++] = (int16_t)adc1_get_raw((adc1_channel_t)channel);
        // Serial.println(buffer[nextBufferItem-1]);
        delayMicroseconds(2);
    }

    if (nextBufferItem >= BUFFER_SIZE) {
        isBufferFull = true;
    }    
}

class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        Serial.println("Client connected");
        // Serial.println("Multi-connect support: start advertising");
        NimBLEDevice::startAdvertising();
    };

    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        Serial.print("Client address: ");
        Serial.println(NimBLEAddress(desc->peer_ota_addr).toString().c_str());
        /** We can use the connection handle here to ask for different connection parameters.
         *  Args: connection handle, min connection interval, max connection interval
         *  latency, supervision timeout.
         *  Units; Min/Max Intervals: 1.25 millisecond increments.
         *  Latency: number of intervals allowed to skip.
         *  Timeout: 10 millisecond increments, try for 5x interval time for best results.
         */
        Serial.print("Interval: ");
        Serial.println(desc->conn_itvl * 1.25);
        Serial.print("Latency: ");
         Serial.println(desc->conn_latency);
    
        pServer->updateConnParams(desc->conn_handle, 6, 6, 0, 10);
    };

    void onDisconnect(NimBLEServer* pServer) {
        Serial.println("Client disconnected - start advertising");
        NimBLEDevice::startAdvertising();
    };

    void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
        pServer->updateConnParams(desc->conn_handle, 6, 6, 0, 10);
        Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);
    };

    uint32_t onPassKeyRequest(){
        Serial.println("Server Passkey Request");
        /** This should return a random 6 digit number for security
         *  or make your own static passkey as done here.
         */
        return 123456;
    };

    bool onConfirmPIN(uint32_t pass_key){
        Serial.print("The passkey YES/NO number: ");Serial.println(pass_key);
        /** Return false if passkeys don't match. */
        return true;
    };

    void onAuthenticationComplete(ble_gap_conn_desc* desc){
        /** Check that encryption was successful, if not we disconnect the client */
        if(!desc->sec_state.encrypted) {
            NimBLEDevice::getServer()->disconnect(desc->conn_handle);
            Serial.println("Encrypt connection failed - disconnecting client");
            return;
        }
        Serial.println("Starting BLE work!");
    };

};

class CharacteristicCallbacks: public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pCharacteristic){
        Serial.print(pCharacteristic->getUUID().toString().c_str());
        Serial.print(": onRead(), value: ");
        Serial.println(pCharacteristic->getValue().c_str());
    };

    void onWrite(NimBLECharacteristic* pCharacteristic) {
        Serial.print(pCharacteristic->getUUID().toString().c_str());
        Serial.print(": onWrite(), value: ");
        Serial.println(pCharacteristic->getValue().c_str());
    };

    void onNotify(NimBLECharacteristic* pCharacteristic) {
        // Serial.println("Sending notification to clients");
    };

    void onStatus(NimBLECharacteristic* pCharacteristic, Status status, int code) {
        String str = ("Notification/Indication status code: ");
        str += status;
        str += ", return code: ";
        str += code;
        str += ", ";
        str += NimBLEUtils::returnCodeToString(code);
        // Serial.println(str);
    };

    void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
        String str = "Client ID: ";
        str += desc->conn_handle;
        str += " Address: ";
        str += std::string(NimBLEAddress(desc->peer_ota_addr)).c_str();
        if(subValue == 0) {
            str += " Unsubscribed to ";
        }else if(subValue == 1) {
            str += " Subscribed to notfications for ";
        } else if(subValue == 2) {
            str += " Subscribed to indications for ";
        } else if(subValue == 3) {
            str += " Subscribed to notifications and indications for ";
        }
        str += std::string(pCharacteristic->getUUID()).c_str();

        // Serial.println(str);
    };
};

class DescriptorCallbacks : public NimBLEDescriptorCallbacks {
    void onWrite(NimBLEDescriptor* pDescriptor) {
        std::string dscVal = pDescriptor->getValue();
        Serial.print("Descriptor witten value:");
        Serial.println(dscVal.c_str());
    };

    void onRead(NimBLEDescriptor* pDescriptor) {
        Serial.print(pDescriptor->getUUID().toString().c_str());
        Serial.println(" Descriptor read");
    };
};


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
    
    /** sets device name */
    NimBLEDevice::init(DEVICE_NAME);

    /** Optional: set the transmit power, default is 3db */
    NimBLEDevice::setPower(ESP_PWR_LVL_N12); /** +9db */
    NimBLEDevice::setMTU(128);


    /** Set the IO capabilities of the device, each option will trigger a different pairing method.
     *  BLE_HS_IO_DISPLAY_ONLY    - Passkey pairing
     *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
     *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
     */
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); // use passkey
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison

    /** 2 different ways to set security - both calls achieve the same result.
     *  no bonding, no man in the middle protection, secure connections.
     *
     *  These are the default values, only shown here for demonstration.
     */
    //NimBLEDevice::setSecurityAuth(false, false, true);
    NimBLEDevice::setSecurityAuth(/*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pBaadService = pServer->createService(SERVICE_UUID);
    NimBLECharacteristic* pFoodCharacteristic = pBaadService->createCharacteristic(
                                               CHARACTERISTIC_UUID,
                                               NIMBLE_PROPERTY::NOTIFY
                                              );

    pFoodCharacteristic->setValue("Fries");
    pFoodCharacteristic->setCallbacks(&chrCallbacks);
    pBaadService->start();

    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(pBaadService->getUUID());
    pAdvertising->setMaxInterval(100);
    pAdvertising->setMinInterval(100);
    // pAdvertising->setMaxPreferred(12);
    // pAdvertising->setMinPreferred(12);
    // pAdvertising->addTxPower();
    
    std::string macAddress = NimBLEDevice::getAddress().toString();
    pAdvertising->setManufacturerData(macAddress);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    Serial.println("Advertising Started");
}


void loop() {
  if(pServer->getConnectedCount()) {
    readData(); 
    if (isBufferFull) {
      nextBufferItem = 0;
      isBufferFull = false;

      float pitch = Yin_getPitch(&yin, buffer);
      pitch = pitch / 2; 
      // Serial.printf("Pitch = %.1f Hz\n", pitch); 
          
      if(pitch == -0.5){
        pitch = 0;
      }

      // pitchBuffer[bufferIndex] = pitch;
      // bufferIndex++;

      // if (bufferIndex >= PITCH_BUFFER_SIZE) {
      //   float avgPitch = calculateAveragePitch();
        
      //   bufferIndex = 0;

        // char pitchStr[10]; 
        // snprintf(pitchStr, sizeof(pitchStr), "%.1f", pitch);  
        uint8_t pitchBytes[sizeof(float)];
        memcpy(pitchBytes, &pitch, sizeof(float));

        NimBLEService* pSvc = pServer->getServiceByUUID(SERVICE_UUID);
        if(pSvc) {
          NimBLECharacteristic* pChr = pSvc->getCharacteristic(CHARACTERISTIC_UUID);
            if(pChr) {
              Serial.printf("Pitch = %.1f Hz\n", pitch); 
              pChr->setValue(pitchBytes, sizeof(float));
              pChr->notify();
              delay(10);
            }
        }
      // }
    }
  }
  delay(1);
}

