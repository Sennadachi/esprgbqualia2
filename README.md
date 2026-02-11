# ESP RGB Qualia 2

An ESP-IDF firmware project for the Adafruit Qualia S3 RGB666 board and the NV3052C 4" 720x720 TFT Display, featuring custom display initialization and control via I2C expander.

## Overview

This project implements low-level display control for the Adafruit Qualia S3 RGB666 development board. It utilizes the ESP32-S3's RGB parallel display interface and manages display initialization through a PCA9554 I2C GPIO expander.

## Hardware

- **Board**: Adafruit Qualia S3 RGB666
- **MCU**: ESP32-S3
- **I/O Expander**: PCA9554 (I2C address 0x3F)
- **Display Interface**: RGB666 parallel interface

### Pin Configuration

#### RGB666 Parallel Interface
- **Clock/Control**: PCLK (1), DE (2), HSYNC (41), VSYNC (42)
- **Red Channel**: R1-R5 (pins 11, 10, 9, 46, 3)
- **Green Channel**: G2-G5 (pins 21, 14, 13, 12)
- **Blue Channel**: B1-B5 (pins 40, 39, 38, -, 45)

#### I2C Bus
- **SDA**: GPIO 8
- **SCL**: GPIO 18

#### PCA9554 Expander Pins
- **TFT_SCK**: Pin 0
- **TFT_CS**: Pin 1
- **TFT_RESET**: Pin 2
- **TFT_IRQ**: Pin 3 (input)
- **TFT_BACKLIGHT**: Pin 4
- **TFT_MOSI**: Pin 7

#### SPI (ESP Direct)
- **SCK**: GPIO 5
- **MISO**: GPIO 6
- **MOSI**: GPIO 7
- **CS**: GPIO 15

## Features

- PCA9554 I2C GPIO expander initialization and control
- Bit-banged SPI communication through I2C expander
- Complete display initialization sequence
- FreeRTOS task management
- Low-level display register configuration

## Prerequisites

- [PlatformIO](https://platformio.org/) installed
- ESP-IDF framework (configured via PlatformIO)
- USB cable for programming and debugging

## Building and Flashing

### Using PlatformIO

1. Clone the repository:
   ```bash
   git clone https://github.com/Sennadachi/esprgbqualia2the_electric_boogaloo.git
   cd esprgbqualia2the_electric_boogaloo
   ```

2. Build the project:
   ```bash
   pio run
   ```

3. Upload to the board:
   ```bash
   pio run --target upload
   ```

4. Monitor serial output:
   ```bash
   pio device monitor
   ```

### Using ESP-IDF directly

If you prefer to use ESP-IDF directly:

1. Set up ESP-IDF environment:
   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```

2. Build and flash:
   ```bash
   idf.py build
   idf.py -p (PORT) flash monitor
   ```

## Project Structure

```
├── src/
│   ├── main.c              # Main application code
│   └── CMakeLists.txt      # CMake configuration for src
├── include/                # Header files
├── lib/                    # Project libraries
├── test/                   # Unit tests
├── CMakeLists.txt          # Root CMake configuration
├── platformio.ini          # PlatformIO configuration
├── sdkconfig.adafruit_qualia_s3_rgb666  # ESP-IDF SDK configuration
└── command_words.txt       # Display register initialization commands
```

## Code Overview

### Main Components

- **I2C Master Bus**: Configured for communication with PCA9554 expander
- **PCA9554 Functions**:
  - `pcaInit()`: Initialize GPIO expander
  - `pcaWriteRegister()`: Write to expander registers
  - `pcaSet()` / `pcaClear()`: Set/clear individual GPIO pins
  
- **Display Communication**:
  - `tftWriteBit()`: Bit-bang SPI bit transmission
  - `tftWriteByte()`: Send byte via bit-banged SPI
  - `tftWriteCommand()`: Send command and data to display controller
  
- **Display Initialization**:
  - `displayInit()`: Complete display initialization sequence with timing control


## Repository

[https://github.com/Sennadachi/esprgbqualia2](https://github.com/Sennadachi/esprgbqualia2)

## Notes

