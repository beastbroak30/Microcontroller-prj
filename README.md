# 🤖 Microcontroller Project Repository by Beastbroak30
<!-- Replace with your image link or upload your own banner -->

*A comprehensive collection of advanced microcontroller-based robotics and IoT projects built with passion, precision, and competitive-grade performance.*

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Arduino](https://img.shields.io/badge/Arduino-00979D?style=flat&logo=Arduino&logoColor=white)](https://www.arduino.cc/)
[![C++](https://img.shields.io/badge/C++-00599C?style=flat&logo=c%2B%2B&logoColor=white)](https://isocpp.org/)

## 📚 Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Boards Used](#boards-used)
- [Programming Language](#programming-language)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Project Structure](#project-structure)
- [Projects Overview](#projects-overview)
  - [Line Follower Projects](#1-line-follower-projects)
  - [Maze Solver Projects](#2-maze-solver-projects)
  - [PID Line Follower Projects](#3-pid-line-follower-projects)
  - [RC RF Projects](#4-rc-rf-projects)
  - [Arduino Projects](#5-arduino-projects)
  - [Firmware](#6-firmware)
- [Getting Started](#getting-started)
- [Usage Examples](#usage-examples)
- [Troubleshooting](#troubleshooting)
- [Contribution Guidelines](#contribution-guidelines)
- [Support](#support)
- [License](#license)

---

## 🎯 Overview

This repository is a curated collection of **microcontroller-based projects** focused on robotics, automation, and IoT applications. The projects range from basic line-following robots to advanced competition-grade robots with PID control, maze-solving capabilities, RF-controlled vehicles, and sensor integration systems.

Whether you're a beginner learning embedded systems or an advanced developer building competition robots, this repository provides production-ready code, comprehensive documentation, and practical implementations for various microcontroller platforms.

## ✨ Features

- 🏆 **Competition-Ready Code**: Optimized algorithms for robotics competitions (IIT and international standards)
- 🎛️ **PID Control Implementation**: Advanced tuning for precise motor control and line following
- 🧭 **Maze Solving Algorithms**: Left-hand rule, right-hand rule, and shortest path optimization
- 📡 **Wireless Control**: RF-based remote control systems for RC vehicles
- 🌡️ **IoT Integration**: Temperature/humidity monitoring with LCD displays (ESP8266 + DHT11)
- 🤖 **Obstacle Avoidance**: Ultrasonic sensor-based autonomous navigation
- 📷 **Camera Integration**: ESP32-CAM projects for image capture
- 🎮 **Interactive Projects**: Chrome Dino game implementation on microcontrollers

## 🛠️ Boards Used

This repository consists of projects based on the following microcontroller boards:

| Board | Use Cases | Projects |
|-------|-----------|----------|
| **Arduino Nano** | Line followers, PID control, maze solvers | Most competition robots |
| **ESP8266 (NodeMCU/Wemos D1 Mini)** | WiFi-enabled projects, IoT sensors | LCD displays, DHT11 sensors, 6WD cars |
| **ESP32** | Camera projects, advanced wireless | ESP32-CAM image capture |
| **Raspberry Pi Pico** | Fast processing, multiple PWM channels | Future projects |
| **Raspberry Pi Zero 2W** | Linux-based robotics, computer vision | Advanced applications |

## 🖥️ Programming Language

All projects in this repository are primarily developed in **C++** using the Arduino IDE framework. The code is compatible with:
- Arduino IDE (v1.8.x or v2.x)
- PlatformIO
- Arduino CLI

**Note**: Separate repositories will be maintained for projects using **MicroPython** and **Raspberry Pi** (Python).

## 🔧 Hardware Requirements

### Common Components Used Across Projects:

#### Motors & Drivers
- L298N Motor Driver Module
- N20 Geared DC Motors
- Servo Motors (SG90/MG90S)
- TB6612FNG Motor Driver (alternative)

#### Sensors
- **Line Detection**: 
  - TCRT5000 IR sensors
  - QTR-8A/QTR-8RC sensor arrays (8-channel)
  - RLS-08 sensor arrays
- **Distance Measurement**: HC-SR04 Ultrasonic sensor
- **Environmental**: DHT11/DHT22 Temperature & Humidity sensors
- **Vision**: ESP32-CAM module

#### Displays & Indicators
- I2C LCD Display (16x2)
- Status LEDs

#### Communication Modules
- 433MHz RF Transmitter/Receiver modules
- WiFi (built-in on ESP8266/ESP32)

#### Power Supply
- 7.4V Li-Po batteries (2S)
- 18650 batteries with holders
- LM7805 voltage regulators
- Buck converters

#### Miscellaneous
- Push buttons for calibration
- Jumper wires and breadboards
- Perfboards for permanent circuits

## 💻 Software Requirements

### Development Environment
```
- Arduino IDE (1.8.13 or later) or Arduino IDE 2.x
- USB drivers for your microcontroller board
- Git for version control
```

### Required Libraries
Install these libraries through Arduino IDE Library Manager or manually:

```cpp
// Core Libraries (Most Projects)
- Arduino.h (built-in)

// Motor Control
- No external library needed (using digitalWrite/analogWrite)

// Sensor Libraries
- QTRSensors.h          // For QTR sensor arrays (Pololu)
- NewPing.h             // For ultrasonic sensors
- DHT.h                 // For DHT11/DHT22 sensors
- Adafruit_Sensor.h     // Adafruit unified sensor library

// Display Libraries
- LiquidCrystal_I2C.h   // For I2C LCD displays
- Wire.h                // I2C communication (built-in)

// ESP32-CAM Specific
- esp_camera.h          // ESP32 camera library
```

### Installation Commands
```bash
# Via Arduino Library Manager, search and install:
# - QTRSensors by Pololu
# - NewPing by Tim Eckel
# - DHT sensor library by Adafruit
# - Adafruit Unified Sensor
# - LiquidCrystal I2C by Frank de Brabander
```

## 📁 Project Structure

The repository is organized into the following main directories:

```
Microcontroller-prj/
├── Line_follower/           # Basic line following robots
├── Maze Solver/             # Maze solving algorithms and implementations
├── PID_Linefollower/        # Advanced PID-controlled line followers
├── RC_RF/                   # RF remote-controlled vehicles
├── arduino-prj/             # Miscellaneous Arduino projects
│   ├── ESP8266_LCD_DHT11/   # IoT sensor projects
│   ├── avoid_it/            # Obstacle avoidance robots
│   ├── cam_test/            # ESP32-CAM projects
│   ├── dino_FPS/            # Game implementations
│   └── *.ino                # Standalone Arduino sketches
├── firmware/                # MicroPython firmware files
└── README.md               # This file
```

Each project directory typically contains:
- **Source Code**: `.ino` Arduino sketch files
- **Documentation**: Markdown files with setup guides (where available)
- **Resources**: PDF guides, schematics, and layouts (for advanced projects)

## 🚀 Projects Overview

### 1. Line Follower Projects
**Directory**: `Line_follower/`

Basic to intermediate line following robot implementations using IR sensors.

| File | Description | Complexity |
|------|-------------|------------|
| `Line_v1.ino` | Basic 8-sensor line follower with simple logic | ⭐ Beginner |
| `line.ino` | Simplified line follower implementation | ⭐ Beginner |
| `linev2.ino` | Improved version with better turning logic | ⭐⭐ Intermediate |
| `line_follower_pid.ino` | Basic PID implementation for line following | ⭐⭐⭐ Advanced |
| `Line[GPT].ino` | GPT-optimized line follower | ⭐⭐ Intermediate |
| `Line[GPT(nostop)].ino` | Continuous operation variant | ⭐⭐ Intermediate |

**Key Features**:
- 8-channel IR sensor array support
- L298N motor driver control
- Speed and direction control
- Basic turn detection

**Hardware Required**: Arduino Nano, L298N, TCRT5000 sensors (x8), DC motors (x2)

---

### 2. Maze Solver Projects
**Directory**: `Maze Solver/`

Advanced maze navigation robots using wall-following algorithms and shortest path optimization.

| File | Description | Algorithm |
|------|-------------|-----------|
| `MAZE_WIN.ino` | Competition-grade maze solver with node tracking | Left/Right-hand + Path optimization |
| `MAZE_IITv1.ino` | IIT competition version 1 | Wall following with memory |
| `MAZE_IITv1.5.ino` | Improved IIT version with bug fixes | Enhanced path recording |
| `Maze_PID_LSRB.ino` | PID-based maze solver (Left-Straight-Right-Back logic) | Weighted decision-making |

**Key Features**:
- QTR-8A sensor array integration
- Node detection and mapping
- Path optimization algorithms
- U-turn and tight corner handling
- Run button for dry-run and actual-run modes
- Consecutive turn limiting

**Hardware Required**: Arduino Nano, TB6612/L298N, QTR-8A sensor array, push button

**Algorithm Highlights**:
- Dry run: Maps the maze and finds all possible paths
- Actual run: Executes the shortest path at high speed
- Node detection with threshold-based white line recognition

---

### 3. PID Line Follower Projects
**Directory**: `PID_Linefollower/`

Competition-ready, high-performance PID-controlled line followers with advanced tuning.

| File | Description | Competition Level |
|------|-------------|-------------------|
| `PID_Linefollower.ino` | **Main competition code** - International standards | 🏆 Advanced |
| `PID_Linefollowerv2.ino` | Version 2 with improved calibration | 🏆 Advanced |
| `PID_Linefollower[TEST].ino` | Testing and debugging version | 🔧 Testing |
| `Maze_runner.ino` | PID + Maze solving hybrid | 🏆 Expert |

**Key Features**:
- **Tunable PID parameters**: Kp, Ki, Kd adjustments
- **Sensor calibration**: Button-triggered min/max calibration
- **Position calculation**: Weighted line position (0-7000 scale)
- **Speed optimization**: Dynamic base speed + PID correction
- **Sharp turn handling**: Enhanced algorithms for 90° and 180° turns
- **Competition optimized**: Fast response times, minimal oscillation

**Hardware Required**: 
- Arduino Nano
- RLS-08 or QTR-8A sensor array
- L298N motor driver
- N20 geared motors
- Calibration button
- Status LED

**Documentation**:
- `Competition-Level_PID_Line_Follower_Robot.pdf` - Complete project guide
- `Perfboard_Layout_and_Setup_Guide_for_PID_Line_Follower_Robot.pdf` - Circuit assembly guide
- `Documentation.md` - Quick reference (currently empty, contributions welcome!)

**PID Tuning Guide**:
1. Start with Kp = 2.0, Ki = 0, Kd = 0
2. Increase Kp until stable oscillation
3. Add Kd (typically 0.5 * Kp) to reduce oscillation
4. Add Ki (small value) if needed for steady-state error

---

### 4. RC RF Projects
**Directory**: `RC_RF/`

RF remote-controlled vehicles using 433MHz transmitter/receiver modules.

| File | Description | Type |
|------|-------------|------|
| `CAR_RF.ino` | RF-controlled car (receiver code) | Receiver |
| `RFCARv2.ino` | Improved version with better control | Receiver |
| `RFCARv3.ino` | Latest stable version | Receiver |
| `RF_RCv3.1.ino` | Enhanced remote control features | Receiver |
| `RF_REM.ino` | Remote control (transmitter code) | Transmitter |
| `RF_TEST.ino` | **Testing code for remote** | Transmitter |
| `Test_NonRF.ino` | Testing without RF (manual control) | Testing |

**Key Features**:
- 433MHz wireless communication
- 4-direction control (Forward, Backward, Left, Right)
- Speed control via PWM
- Multiple car versions with different configurations

**Hardware Required**:
- **Transmitter**: Arduino Nano, 433MHz TX module, buttons/joystick
- **Receiver**: Arduino Nano, 433MHz RX module, L298N, DC motors

**Documentation**: See `Documentation.md` for car and remote setup

---

### 5. Arduino Projects
**Directory**: `arduino-prj/`

Diverse collection of microcontroller projects spanning IoT, robotics, and interactive applications.

#### 5.1 IoT Sensor Project
**Subdirectory**: `ESP8266_LCD_DHT11/`
- **File**: `ESP8266_LCD_DHT11.ino`
- **Description**: Temperature and humidity monitoring system with LCD display
- **Hardware**: ESP8266 (Wemos D1 Mini), DHT11 sensor, 16x2 I2C LCD
- **Features**: Real-time temp/humidity display, error handling, WiFi-ready platform

#### 5.2 Obstacle Avoidance Robot
**Subdirectory**: `avoid_it/`
- **File**: `avoid_it.ino`
- **Description**: Autonomous robot that avoids obstacles using ultrasonic sensors
- **Hardware**: Arduino Nano, HC-SR04, servo motor, L298N, DC motors
- **Features**: Servo-mounted ultrasonic sensor, distance-based navigation, automatic path planning

#### 5.3 ESP32-CAM Project
**Subdirectory**: `cam_test/`
- **Files**: `cam_test.ino`, `camera_pins.h`
- **Description**: ESP32-CAM image capture and streaming
- **Hardware**: ESP32-CAM module
- **Features**: Image capture, WiFi streaming (implementation-dependent)

#### 5.4 Interactive Game Project
**Subdirectory**: `dino_FPS/`
- **Files**: `dino_FPS.ino`, `sprite.c`
- **Description**: Chrome Dino game implementation on microcontroller
- **Hardware**: Arduino + Display module
- **Features**: Sprite-based graphics, game logic, FPS optimization

#### 5.5 Standalone Sketches

| File | Description | Hardware |
|------|-------------|----------|
| `wemoscar2.ino` | 4WD WiFi-controlled car | ESP8266, L298N, 4 motors |
| `wemoscar_6WD.ino` | 6-wheel drive WiFi car | ESP8266, L298N, 6 motors |
| `wall_finalv1.ino` | Wall-following robot | Arduino, ultrasonic, motors |
| `walle_slavev1.ino` | Slave robot for multi-robot system | Arduino, motors |
| `line-8.ino` | 8-sensor line follower | Arduino, TCRT5000 x8 |
| `line8_v2.5.ino.ino` | Enhanced 8-sensor version | Arduino, sensors |
| `line8_v3.ino` | Latest 8-sensor implementation | Arduino, sensors |
| `motor_line.ino` | Motor control + line following | Arduino, L298N |
| `irmotor.ino` | IR-based motor control | Arduino, IR sensors |
| `ir_test.ino` | IR sensor testing sketch | Arduino, IR sensors |
| `graphpot.ino` | Potentiometer graph plotter | Arduino, potentiometer |

---

### 6. Firmware
**Directory**: `firmware/`

Contains MicroPython firmware for ESP8266/ESP32 boards.

- **File**: `ESP8266_GENERIC-FLASH_1M-20240602-v1.23.0.bin`
- **Description**: MicroPython firmware v1.23.0 for ESP8266 (1M flash variant)
- **Use Case**: Flash this to ESP8266 for running Python scripts instead of C++ code

**How to Flash**:
```bash
esptool.py --port /dev/ttyUSB0 erase_flash
esptool.py --port /dev/ttyUSB0 --baud 460800 write_flash --flash_size=detect 0 ESP8266_GENERIC-FLASH_1M-20240602-v1.23.0.bin
```

---

## 🚦 Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/beastbroak30/Microcontroller-prj.git
cd Microcontroller-prj
```

### 2. Install Arduino IDE
Download and install from [Arduino Official Website](https://www.arduino.cc/en/software)

### 3. Install Required Libraries
Open Arduino IDE → Tools → Manage Libraries, then search and install:
- QTRSensors
- NewPing
- DHT sensor library
- Adafruit Unified Sensor
- LiquidCrystal I2C

### 4. Select Your Board
- Tools → Board → Select your microcontroller (e.g., Arduino Nano, ESP8266)
- Tools → Port → Select the correct COM port

### 5. Open a Project
Navigate to a project folder and open the `.ino` file in Arduino IDE.

### 6. Configure Pin Definitions
Review and adjust pin definitions at the top of each sketch to match your wiring:
```cpp
#define ENA_PIN 9   // Adjust according to your connections
#define IN1_PIN 8
// ... etc
```

### 7. Upload
Click the Upload button (→) in Arduino IDE.

### 8. Test and Calibrate
For line followers and maze solvers:
1. Press the calibration button (if available)
2. Move the robot over the line/track during calibration
3. Wait for calibration to complete
4. Place the robot on the track and press start

---

## 📖 Usage Examples

### Example 1: Running a PID Line Follower

```cpp
// 1. Open PID_Linefollower.ino
// 2. Adjust PID constants if needed
float Kp = 2.0;  // Start with these values
float Ki = 0.0;
float Kd = 1.0;

// 3. Upload to Arduino Nano
// 4. Calibration process:
//    - Press calibration button
//    - Move robot left and right over the line
//    - Wait for LED to stop blinking
// 5. Place robot on line and it will start automatically
```

### Example 2: Setting Up RF Remote Control Car

**Transmitter (Remote)**:
```cpp
// Upload RF_TEST.ino to transmitter Arduino
// Connect:
// - 433MHz TX module to pin 12
// - Control buttons to pins 2-5
```

**Receiver (Car)**:
```cpp
// Upload RFCARv3.ino to receiver Arduino
// Connect:
// - 433MHz RX module to pin 2
// - L298N motor driver as per pin definitions
// - Motors to L298N outputs
```

### Example 3: IoT Temperature Monitor

```cpp
// Upload ESP8266_LCD_DHT11.ino to Wemos D1 Mini
// Connections:
// - DHT11 data pin to D4 (GPIO2)
// - I2C LCD: SDA to D2, SCL to D1
// - Power DHT11 and LCD with 5V/3.3V as appropriate
// Open Serial Monitor at 9600 baud to see readings
```

---

## 🔍 Troubleshooting

### Common Issues and Solutions

#### 1. **Robot Not Following Line**
- **Check sensor calibration**: Ensure sensors are calibrated on both black line and white background
- **Adjust sensor height**: Sensors should be 2-5mm above the surface
- **Verify sensor readings**: Use Serial.print() to debug sensor values
- **Tune PID values**: Start with lower Kp and gradually increase

#### 2. **Motors Not Running**
- **Check power supply**: Ensure battery is charged (>6V for motors)
- **Verify connections**: Confirm all motor driver pins are correctly wired
- **Test motor driver**: Check if L298N enable pins are HIGH
- **Check PWM pins**: Ensure ENA/ENB are connected to PWM-capable pins

#### 3. **Erratic Behavior**
- **Power issues**: Separate power for motors and microcontroller (use common ground)
- **Sensor noise**: Add 100nF capacitors near sensors
- **Loose connections**: Check all jumper wires and solder joints

#### 4. **ESP8266 Not Uploading**
- **Correct board selection**: Tools → Board → ESP8266 → Your specific board
- **Upload speed**: Try reducing to 115200 baud
- **Flash mode**: Ensure GPIO0 is not pulled low during normal operation

#### 5. **RF Remote Not Working**
- **Frequency match**: Ensure TX and RX modules are on the same frequency (433MHz)
- **Antenna orientation**: Keep antennas perpendicular to each other
- **Power supply**: RF modules need stable 5V supply
- **Range testing**: Start with close range (1-2m) before going farther

#### 6. **Compilation Errors**
- **Missing libraries**: Install all required libraries via Library Manager
- **Board definition**: Ensure correct board is selected in Tools → Board
- **Syntax errors**: Check for missing semicolons, brackets, or typos

---

## 🤝 Contribution Guidelines

We welcome contributions from the community! Here's how you can help:

### How to Contribute

1. **Fork the repository**
   ```bash
   # Click 'Fork' button on GitHub
   git clone https://github.com/YOUR_USERNAME/Microcontroller-prj.git
   ```

2. **Create a new branch**
   ```bash
   git checkout -b feature/your-feature-name
   # or
   git checkout -b bugfix/issue-description
   ```

3. **Make your changes**
   - Write clean, well-commented code
   - Follow existing code style and conventions
   - Test your changes thoroughly

4. **Commit your changes**
   ```bash
   git add .
   git commit -m "Add: Brief description of changes"
   # Use prefixes: Add, Fix, Update, Remove, Refactor
   ```

5. **Push to your fork**
   ```bash
   git push origin feature/your-feature-name
   ```

6. **Create a Pull Request**
   - Go to the original repository
   - Click "New Pull Request"
   - Describe your changes clearly
   - Reference any related issues

### Contribution Ideas

- 📝 **Documentation**: Improve README, add tutorials, create circuit diagrams
- 🐛 **Bug Fixes**: Fix issues, improve error handling
- ✨ **New Features**: Add new projects, sensors, or algorithms
- 🔧 **Optimization**: Improve PID tuning, optimize algorithms
- 🧪 **Testing**: Add test sketches, validation scripts
- 📊 **Examples**: Provide more usage examples and templates

### Code Standards

- Use meaningful variable names
- Comment complex logic
- Follow Arduino IDE conventions
- Include pin definitions at the top
- Add hardware requirements in comments
- Keep functions modular and reusable

Please follow the coding standards and best practices mentioned in [CONTRIBUTING.md](CONTRIBUTING.md) (if available).

---

## 💬 Support

Need help or have questions?

- **Issues**: Open an issue on [GitHub Issues](https://github.com/beastbroak30/Microcontroller-prj/issues)
- **Discussions**: Start a discussion on GitHub Discussions
- **Email**: Contact the repository owner via GitHub profile

### Before Asking for Help

1. Check existing issues and documentation
2. Verify your wiring and connections
3. Test with simple example code first
4. Provide error messages and code snippets when asking questions

---

## 📜 License

This repository is licensed under the **MIT License**. See the [LICENSE](LICENSE) file for more information.

**You are free to**:
- ✅ Use this code for personal and commercial projects
- ✅ Modify and adapt the code
- ✅ Share and distribute the code
- ✅ Use in competitions and academic projects

**With the condition that**:
- ℹ️ You include the original license and copyright notice
- ℹ️ You acknowledge the original author

---

## 🌟 Star This Repository

If you find this repository helpful, please consider giving it a ⭐ star! It helps others discover these projects.

---

## 📊 Repository Stats

- **Total Projects**: 40+ Arduino/microcontroller projects
- **Main Categories**: 6 major project categories
- **Boards Supported**: 6 different microcontroller platforms
- **Competition Ready**: Multiple IIT and international competition-grade robots

---

**Happy Building! 🤖⚡**

*Built with ❤️ by Beastbroak30*
