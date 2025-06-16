# STM32_EXAMPLES

# ADXL345 Accelerometer with STM32 HAL (I2C Interface)

This project demonstrates how to interface the **ADXL345** 3-axis digital accelerometer with an **STM32 microcontroller** using the **HAL (Hardware Abstraction Layer)** I2C library. It includes I2C device scanning, sensor initialization, and continuous XYZ-axis data reading, with output through UART.

## 📦 Features

- I2C scanner to detect connected I2C devices
- Initializes ADXL345 with:
  - Measurement mode
  - ±4g range with full resolution
- Reads acceleration data from X, Y, Z axes
- Sends acceleration data over UART
- LED blinking on failure conditions

## 🧰 Hardware Required

- STM32 development board (e.g., STM32F103C8T6 "Blue Pill")
- ADXL345 sensor module (I2C interface)
- USB to UART module for serial output (e.g., FTDI)
- Jumper wires and breadboard

## 🔌 Pin Connections (Example)

| ADXL345 | STM32     |
|---------|-----------|
| VCC     | 5V      |
| GND     | GND       |
| SDA     | PB7 (I2C1 SDA) |
| SCL     | PB6 (I2C1 SCL) |
| UART1	  | PA9  (TX) |
| UART1	  | PA10 (RX) |	


> UART TX (STM32 PA9) connected to USB-to-Serial module for serial monitoring.

## ⚙️ Project Configuration

- **I2C1**: 100 kHz
- **UART1**: 9600 baud, 8-N-1
- **PC13**: Used as error/status LED

## 🚀 How It Works

1. The code starts by scanning for all I2C devices.
2. If ADXL345 (0x53) is found, it initializes the device.
3. Continuously reads 6 bytes from the sensor (X0, X1, Y0, Y1, Z0, Z1).
4. Data is converted to 16-bit signed integers and sent over UART.
5. LED on PC13 toggles if the sensor is not found or initialization fails.


## 🧪 Example Output (UART)

Scan Start ...
Found: 0x53
Adxl found ...
X: 34 Y: -12 Z: 1023
X: 30 Y: -11 Z: 1021


## 🛠️ Development

- STM32CubeMX for peripheral initialization
- STM32CubeIDE for coding and flashing
- HAL Drivers for I2C and UART





