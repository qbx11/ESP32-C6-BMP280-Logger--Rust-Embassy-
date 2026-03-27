# ESP32-C6 BMP280 Logger [Rust/Embassy]

A lightweight, `#![no_std]` data logger for the ESP32-C6. It reads temperature and pressure from a BMP280 sensor over I2C and streams the formatted data over UART. Built on top of `esp-hal` and the Embassy async runtime.

## Hardware
Tested on a standard ESP32-C6 (RISC-V) dev board with a BMP280 breakout module.


## Wiring
| ESP32-C6 Pin | BMP280 / Interface |
|--------------|--------------------|
| GPIO19       | I2C SDA            |
| GPIO20       | I2C SCL            |
| GPIO16       | UART TX            |
| GPIO17       | UART RX            | 


## Output Format

Data is pushed via UART0 at 115200 baud. It uses a simple, pseudo-CSV format for easy parsing on the host side (e.g., in Python or a serial plotter):

```text
T:23.50,P:101325,CNT:42
```
