<p align="center">
  <img src="./PAYLOADX.png" width="220" alt="PayloadX Logo">
</p>

<h2 align="center"><strong>PayloadX – System Architecture</strong></h2>

<p align="center">
  <em>Modular firmware structure for open-source CubeSat development, based on EdgeFlyte v1.</em>
</p>

---

## 📦 Main Modules

| Module               | Description |
|----------------------|-------------|
| `imu.cpp` / `imu.h`  | Reads data from IMU (accelerometer, gyroscope, magnetometer) |
| `gps.cpp` / `gps.h`  | Reads GPS coordinates, altitude, speed, satellites |
| `radio.cpp` / `radio.h` | Manages NRF24 radio communication |
| `bms.cpp` / `bms.h`  | Parses messages from the Battery Management System |
| `main.cpp`           | Reads sensor data and sends telemetry in loop |
| `CubeSat.ino`        | Entry point for Arduino IDE including all modules |

---

## 🔁 System Flow Overview

The following image illustrates the data flow from sensors to telemetry output:

![PayloadX – System Architecture](./architettura.png)

---

## 📄 License

Fork of the [EdgeFlyte CubeSat v1](https://github.com/edgeflyte/CubeSatV1) project.  
Licensed under the GNU GPL v3 license.  
Maintained by **PezzaliStack** as part of the **PayloadX** open-source CubeSat initiative.
