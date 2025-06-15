# STM32_EXAMPLES

# STM32 I2C Scanner

This project demonstrates an **I2C bus scanner** using an STM32 microcontroller. It scans all 7-bit I2C addresses (0x00 to 0x7F) and prints the addresses of detected I2C devices over UART.

## 📦 Features

- I2C bus scan on startup
- UART output showing found I2C device addresses
- Compatible with STM32 HAL Library
- Easy to adapt for different STM32 boards

## 🧰 Requirements

- STM32 development board (e.g., STM32F103C8T6, STM32F401RE, etc.)
- ST-Link programmer or compatible debugger
- STM32CubeIDE (or STM32CubeMX + IDE of choice)
- One or more I2C devices connected to the correct pins (with pull-up resistors)

## ⚙️ Pin Configuration

Ensure the following connections:

| Peripheral | Pins Used   | Description      |
|------------|-------------|------------------|
| I2C1       | PB6 (SCL)   | I2C Clock Line    |
| I2C1       | PB7 (SDA)   | I2C Data Line     |
| UART1      | PA9 (TX)    | UART Transmit     |
| UART1      | PA10 (RX)   | UART Receive (not mandatory) |

_Note: Modify pin assignments as per your board in `MX_GPIO_Init()` if different._

## 🧪 How It Works

1. Initializes I2C and UART peripherals.
2. Scans all I2C addresses (0x00 to 0x7F).
3. Uses `HAL_I2C_IsDeviceReady()` to check each address.
4. Outputs results via UART (e.g., to a terminal or serial monitor like Tera Term or PuTTY).

Sample output:
Scan Start ...
Found: 0x29
Found: 0x3C
Scan Stop ...


## 🛠 Code Structure

- `main.c` - Main application file with I2C scanner logic.
- `MX_I2C1_Init()` - Configures I2C peripheral.
- `MX_USART1_UART_Init()` - Configures UART for output.
- `i2cScanner()` - Scans and prints I2C device addresses.

## 🖥 Usage

1. Flash the firmware using STM32CubeIDE or other tools.
2. Connect UART TX to USB-UART converter (e.g., FTDI).
3. Open a terminal at 9600 baud to see output.
4. Connect I2C devices and reset the board to rescan.

## 🔧 Customization

- Change baud rate or UART instance in `MX_USART1_UART_Init()`
- Change I2C bus or address range in `i2cScanner()`


