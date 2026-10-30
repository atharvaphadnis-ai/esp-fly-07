# esp-fly-07: ESP32 DIY Quadcopter with NRF24L01 Controller

An open-source, cost-effective DIY drone project powered by the **ESP32 microcontroller**, utilizing a custom, low-cost transmitter and receiver system built around the **NRF24L01 PA+LNA** wireless modules instead of expensive commercial radio configurations. 

This repository provides everything you need to build your own drone from scratch, including source codes, sensor calibration tools, custom PCB Gerber files, and pinout wiring configurations.

Special thanks to Pratik Phadte and the *Simple connections* YouTube channel for their invaluable guides and contributions to this architecture.

---

## 🚀 Key Features

* **ESP32 Flight Controller:** High-performance processing core handling real-time sensor loops and flight stability.
* **Low-Cost DIY RC Link:** Uses high-power **NRF24L01 PA+LNA** transceiver modules for extended-range control without expensive traditional RC transmitters/receivers.
* **Custom Hardware Design:** Ready-to-order **Gerber files** included for building your own specialized PCBs.
* **On-board Diagnostics:** Dedicated calibration routines for sensor alignment and initial hardware setup.

---

## 📁 Repository Structure

```text
esp-fly-07/
├── calibration/      # Scripts and routines to calibrate the IMU/sensors
├── gerber_files/     # Production-ready files for manufacturing the custom PCB
├── src/              # Main firmware source code for the drone and RC controller
└── README.md         # Project overview and documentation
```

---

## 🛠️ Hardware Requirements

To replicate this project, you will need the following core components:

* **Microcontroller:** ESP32 development board (NodeMCU or similar footprint)
* **Wireless Modules:** 2x NRF24L01 PA+LNA wireless modules (one for the drone, one for the transmitter)
* **Sensors:** Compatible Inertial Measurement Unit (IMU) like an MPU6050 (for orientation and balancing)
* **Motors & ESCs:** 4x Brushless or Coreless DC motors paired with appropriate Electronic Speed Controllers (ESCs)
* **Power:** LiPo battery (matching motor size) + 5V/3.3V voltage regulators for the microcontrollers
* **Transmitter Inputs:** Analog joysticks (potentiometers) for Pitch, Roll, Yaw, and Throttle control

---

## 💻 Software Setup

### Prerequisites
1. Download and install the [Arduino IDE](https://arduino.cc) or use [PlatformIO](https://platformio.org) in VS Code.
2. Install the **ESP32 Board Support Package** in your development environment.
3. Install the required libraries via the Arduino Library Manager:
   * `RF24` by TMRh20 (for NRF24L01 communication)
   * `MPU6050` or custom I2C sensor libraries (depending on your choice of IMU)

### Installation & Flashing
1. **Clone the Repository:**
   ```bash
   git clone https://github.com
   cd esp-fly-07
   ```
2. **Sensor Calibration:**
   * Open the firmware inside the `calibration/` folder.
   * Connect your assembled quadcopter to your PC, place it on a perfectly flat surface, and flash the code.
   * Open the Serial Monitor at the designated baud rate to get your IMU offset values. Record these offsets inside your main flight code.
3. **Flash the Firmware:**
   * Open the main project code inside the `src/` directory.
   * Upload the transmitter code to your remote controller hardware.
   * Upload the flight controller code to your drone's ESP32.

---

## 🔧 Hardware Assembly & PCB

1. **PCB Fabrication:** Take the ZIP file inside the `gerber_files/` folder and upload it directly to any PCB manufacturing service (e.g., JLCPCB, PCBWay) to get your custom circuit boards manufactured.
2. **Wiring Connections:** Detailed wiring pins are explicitly declared in the headers of the main source files located in the `src/` directory. Double-check your SPI pins for the NRF24L01 and I2C pins for your orientation sensor before powering on the device.

---

## 🤝 Contributing

Contributions to improve flight stability, add extra features, or optimize telemetry data are always welcome! 

1. Fork the Project.
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`).
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`).
4. Push to the Branch (`git push origin feature/AmazingFeature`).
5. Open a Pull Request.

---

## 📝 License

Distributed under the MIT License. See `LICENSE` for more information (if applicable).
