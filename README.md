# Autonomous & Semi-Autonomous Competition Robot (ESP32)

An open-source embedded robotics firmware written in C++ for the ESP32 microcontroller. The system features closed-loop PID trajectory following, real-time ultrasonic obstacle evasion using a Finite State Machine (FSM), and dual-core FreeRTOS task scheduling.

---

## Key Technical Features
* **Closed-Loop PID Control:** Line tracking via a 5-channel infrared sensor array with tuned proportional, integral, and derivative loops.
* **Non-Blocking Architecture:** Built on FreeRTOS with multi-threading:
  * **Core 1:** High-frequency PID control loop and motor PWM regulation.
  * **Core 0:** Ultrasonic sensor polling (HC-SR04) and serial telemetry output.
* **FSM-Driven Evasion:** Automated switching between `LINE_TRACKING`, `OBSTACLE_AVOIDANCE`, and `MANUAL_CONTROL`.
* **Hardware PWM Driving:** Direct ESP32 LEDC peripheral configuration for smooth dual-channel DC motor speed control.

---

## Hardware Architecture
| Component | Part / Spec | ESP32 GPIO Pin |
| :--- | :--- | :--- |
| **Microcontroller** | ESP32 NodeMCU-32S | - |
| **Motor Driver** | L298N / DRV8833 Dual H-Bridge | PWM: `18`, `23` \| DIR: `19`, `22` |
| **IR Sensor Bar** | 5-Channel TCRT5000 Array | `34`, `35`, `32`, `33`, `25` |
| **Ultrasonic** | HC-SR04 | TRIG: `5` \| ECHO: `17` |

---

## Project Structure
```text
├── src/
│   └── main.cpp         # Main application firmware and FreeRTOS tasks
├── platformio.ini       # PlatformIO build configuration
├── LICENSE              # MIT License
└── README.md            # Technical documentation
