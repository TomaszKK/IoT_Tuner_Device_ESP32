# IoT Tuner Device

## Overview
The IoT Tuner is a standalone embedded device that captures environmental sound, calculates its frequency, and transmits it via Bluetooth to a mobile application. The device is powered by the ESP32-DEVKITC-32D V4 IoT development board and integrates an analog MEMS microphone for sound detection. This project leverages Bluetooth Low Energy (BLE) for low-power communication, ensuring energy-efficient operation suitable for portable use.

## Key Features
- **Real-Time Frequency Detection**: Captures audio signals and calculates pitch using the YIN algorithm.
- **BLE Communication**: Sends frequency data to a connected mobile application via BLE.

## Hardware Components
1. **ESP32-DEVKITC-32D V4**:
   - Features:
     - Xtensa LX6 dual-core processor clocked at up to 240 MHz.
     - Integrated BLE and Wi-Fi module.
     - Analog-to-Digital Converter (ADC) pins.

2. **Analog MEMS Microphone (SPH8878LR5H-1)**:
   - Features:
     - Frequency range: 7 Hz to 36 kHz.
     - Sensitivity: -44 dBV/Pa.
     - Signal-to-noise ratio: 66 dBV/Pa.

## Software
### Programming Frameworks
- **ESP-IDF**: Core libraries written in C/C++ for ESP32 development.
- **Arduino Framework for ESP**: Provides seamless library integration for ADC input and BLE.

### Key Libraries
- **NimBLEDevice**: For BLE communication.
- **YIN**: For pitch detection using the YIN algorithm.
