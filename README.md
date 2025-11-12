# 🧠 MicroBlaze Firmware — Sensor & DMA Integration

This repository contains the **Vitis (C-based) firmware** developed for the MicroBlaze system implemented on the Artix-7 FPGA (e.g., **Nexys-4 DDR**).  
The firmware demonstrates a complete **hardware–software co-design**, handling sensor communication (SPI/I²C), ultrasonic distance measurement, DDR2 memory access, and AXI DMA data streaming.

---

## 🔧 System Overview

The firmware runs on a **MicroBlaze soft processor** connected to:
- **DDR2 memory (MIG)** for data storage  
- **AXI DMA** for high-speed streaming  
- **AXI GPIO** for LED control and ultrasonic sensors  
- **AXI SPI** and **AXI I²C** for sensor interfaces  
- **AXI UARTLite** for serial terminal communication  

The main firmware (`main.c`) initializes all peripherals, verifies DDR2 calibration, tests memory, and continuously acquires and displays sensor data.

---

## 📡 Features

| Feature | Description |
|----------|--------------|
| 🧠 **MicroBlaze Integration** | Runs fully on custom MicroBlaze system from Vivado design |
| 💾 **DDR2 Memory Test** | Verifies memory access with `Xil_TestMem32()` |
| ⚙️ **AXI DMA** | Streams data from FPGA logic to DDR2 (S2MM) and reports completion |
| 💡 **GPIO Control** | Toggles onboard LEDs and triggers ultrasonic sensor pulses |
| 🌡️ **ADT7420 I²C Temperature Sensor** | Reads 13-bit or 16-bit mode data, converts to °C |
| 🧭 **ADXL362 SPI Accelerometer** | Reads X/Y/Z acceleration values over SPI |
| 🌀 **MPU6500 I²C IMU** | Reads accelerometer and gyroscope data; converts gyro readings to °/s |
| 📶 **HC-SR04 Ultrasonic Sensor** | Measures distance using GPIO trigger/echo with timer capture |
| 🕒 **Hardware Timer (XTmrCtr)** | Provides high-resolution time measurement at 100 MHz |
| 🧩 **Interrupt Controller (XIntc)** | Manages GPIO and DMA interrupts |
| 💬 **UART Output** | Streams formatted sensor data to the host PC console |

---

## 🧱 Source Structure

All firmware sources are organized under `src/` for Vitis integration:

| File | Description |
|------|--------------|
| **main.c** | Core firmware logic: initializes peripherals, runs sensor loops, handles DMA |
| **adxl362.c / adxl362.h** | SPI driver for ADXL362 accelerometer |
| **mpu6500.c / mpu6500.h** | SPI & I²C driver for MPU6500 gyroscope/accelerometer |
| *(BSP files)* | Generated automatically by Vitis platform (drivers, system parameters, etc.) |

---

## 🧩 Peripheral Map

| Peripheral | Function | Interface |
|-------------|-----------|-----------|
| `AXI_GPIO_0` | LED output | GPIO |
| `AXI_GPIO_1` | HC-SR04 trigger | GPIO |
| `AXI_GPIO_2` | HC-SR04 echo (interrupt) | GPIO |
| `AXI_GPIO_3` | MIG DDR2 calibration check | GPIO |
| `AXI_IIC_0` | ADT7420 temperature sensor | I²C |
| `AXI_IIC_1` | MPU6500 IMU | I²C |
| `AXI_QUAD_SPI_0` | ADXL362 accelerometer | SPI |
| `AXI_UARTLITE_0` | USB serial output | UART |
| `AXI_TIMER_0` | Time measurement | Timer |
| `AXI_DMA_0` | DDR2 streaming | AXI DMA |

---

## ⚙️ Build & Run

1. **Open Vitis** → Create a new *Application Project* using your exported hardware (`.xsa`) from Vivado.  
2. Add the source files under `src/` to the application.  
3. Build the project and program the FPGA from Vitis (use `boot.bin` or JTAG).  
4. Open a serial terminal (115200 baud, 8-N-1).  
5. You’ll see real-time sensor readouts and DMA transfer logs.

---

## 📊 Console Output Example

| Log Output | Description |
|-------------|-------------|
| **CODE STARTED** | Firmware initialized successfully |
| **MIG calibration done!** | DDR2 memory interface calibrated and ready |
| **DDR memory test PASSED!** | Memory verified via `Xil_TestMem32()` |
| **ADXL362 dev_id: 173** | SPI accelerometer detected successfully |
| **ADT7420 dev_id: 192** | I²C temperature sensor initialized |
| **MPU6050 dev_id: 104** | Secondary IMU device identified |
| **MPU6500 I²C init success** | Primary IMU initialized over I²C |
| **X: 12, Y: -8, Z: 1024** | ADXL362 accelerometer readings |
| **MPU6500 I²C ACC: X=123 Y=-22 Z=1050** | MPU6500 accelerometer data |
| **MPU6500 I²C GYRO: X=0.18 Y=-0.06 Z=0.02** | MPU6500 gyroscope data (°/s) |
| **Temperature: 24.75 °C** | ADT7420 temperature output |
| **Distance: 37.420 cm** | Ultrasonic distance measurement (HC-SR04) |
| **DMA transfer complete** | AXI DMA transfer to DDR2 finished successfully |

---


## 🧾 License

**MIT License** © 2025 — *Cenk Keskin*  
Developed for educational and embedded-system integration purposes.

---

> This firmware demonstrates how to combine multiple sensor interfaces, memory operations, and hardware DMA into one unified MicroBlaze-based system, forming a compact but feature-rich embedded control platform.
