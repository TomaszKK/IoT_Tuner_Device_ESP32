#include <TaskScheduler.h>
#include "yinacf.h"

Yin yin;
size_t buffer_size = 1024;
int16_t *buffer;
bool isBufferFull = false;
int nextBufferItem = 0;

#define PIN 32  // Pin where the mic is connected
#define CONVERSIONS_PER_PIN 10
#define ADC_MAX 4095  // Max ADC value for 12-bit resolution
#define VREF 3.3  // ESP32 system voltage (3.3V)

// Variables for sampling
// const int sampleTime = ;  // Duration to sample in ms
unsigned long startTime = 0;

void readData(){
  if (nextBufferItem >= buffer_size) {
    isBufferFull = true;
    return;
  }

  // Sample for a defined time window
  while (nextBufferItem < buffer_size) {
    buffer[nextBufferItem++] = (int16_t)analogRead(PIN);

    delayMicroseconds(4);
  }
  //  Serial.println("here2");
}

void setup() {
  Serial.begin(115200);

  // Allocate memory for the buffer
  buffer = (int16_t*)malloc(buffer_size * sizeof(int16_t)); 
  if (buffer == NULL) {
    Serial.print("Buffer allocation failed");
    return;
  }    

  // Initialize YIN pitch detection algorithm
  Yin_init(&yin, buffer_size, 0.10);

  Serial.println("Waiting for buffer to fill with audio data...");
}

void loop() {
  
  readData();

  if(isBufferFull) {
    nextBufferItem = 0;  // Reset buffer index
    isBufferFull = false;

    // Call the pitch detection algorithm with the filled buffer
    float pitch = Yin_getPitch(&yin, buffer);
    pitch = pitch / 2;
    Serial.printf("Pitch = %.1f Hz\n", pitch);

    delay(5);  // Small delay between pitch calculations
  }
}
