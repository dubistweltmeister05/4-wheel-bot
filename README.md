# 4-Wheel Bot Firmware (STM32F407VG)

Welcome to the 4-Wheel Bot project! This repository contains firmware for a four-wheel robot chassis powered by the STM32F407VG microcontroller. The codebase is designed for robust sensor integration, modularity, and ease of experimentation—perfect for robotics enthusiasts, students, and developers.

---

## 🚗 Project Overview

This firmware enables a four-wheel robot to sense, move, and interact with its environment. It features:

- Real-time orientation and motion sensing (BNO055 IMU)
- PWM-based motor control
- I2C communication for sensors and controllers
- Example code for PS4 controller interfacing via Arduino
- Modular drivers for STM32 peripherals

---

## ✨ Features

- **4-Wheel Chassis Control**: Smooth PWM motor control using TIM4
- **BNO055 IMU Integration**: Accurate orientation (Euler angles, quaternion)
- **I2C Communication**: For sensor and controller interfacing
- **PS4 Controller Support**: Receives commands via I2C from an Arduino bridge
- **Self-Test & Diagnostics**: Example code for sensor and system validation
- **STM32 HAL & CMSIS**: Leverages official drivers for reliability

---

## 🛠️ Hardware Requirements

- **MCU**: STM32F407VG (tested on STM32F407G-DISC1 board)
- **IMU**: BNO055 (I2C1: SCL=PB8, SDA=PB9)
- **Motors**: 4 DC motors (TIM4 PWM: PD12–PD15)
- **Optional**: Arduino for PS4 controller bridge (I2C2)
- **Power**: As required by your motors and board

---

## 📁 Directory Structure

```
.
├── Arduino_code/
│   ├── Ps4_arduino_slave_sender_5bytes/
│   │   └── Ps4_arduino_slave_sender_5bytes.ino
│   └── Ps4_arduino_slave_sender_7bytes/
│       └── Ps4_arduino_slave_sender_7bytes.ino
├── 4_wheel_chassis/
│   ├── Core/
│   │   ├── Inc/           # Main header files
│   │   │   └── Backup/    # Backup/legacy header files
│   │   ├── Src/           # Main source files
│   │   │   ├── src/       # Custom STM32F407 peripheral drivers (GPIO, I2C, SPI, RCC)
│   │   │   └── Backup/    # Backup/legacy source files
│   │   └── Startup/       # (If present) Startup code
│   ├── Drivers/
│   │   ├── STM32F4xx_HAL_Driver/
│   │   │   ├── Inc/       # STM32 HAL driver headers
│   │   │   └── Src/       # STM32 HAL driver sources
│   │   └── CMSIS/
│   │       ├── Include/   # ARM CMSIS core headers
│   │       └── Device/
│   │           └── ST/
│   │               └── STM32F4xx/
│   │                   ├── Include/   # STM32F4xx device headers
│   │                   └── Source/    # STM32F4xx device startup/system files
│   ├── Debug/             # IDE-generated debug output
│   ├── .settings/         # IDE settings
│   ├── .cproject, .project, .mxproject  # STM32CubeIDE project files
│   ├── *.launch, *.cfg, *.ioc           # IDE and CubeMX config files
```

---

## 🕹️ Arduino PS4 Controller Bridge

**Location:** `Arduino_code/`

This folder contains Arduino sketches that turn an Arduino (with a USB Host Shield) into a PS4 controller-to-I2C bridge. The STM32 board can then read controller data over I2C for remote robot control.

- **Ps4_arduino_slave_sender_5bytes.ino**: Sends left stick X/Y, trigger difference (W), and button states (5 bytes total).
- **Ps4_arduino_slave_sender_7bytes.ino**: Sends left stick X/Y, trigger difference (W), button states, and touchpad X/Y (7 bytes total).

**How to use:**

1. Flash the desired `.ino` file to your Arduino (with USB Host Shield).
2. Pair your PS4 controller with the Arduino.
3. Connect Arduino's I2C (SDA/SCL) to the STM32 board (I2C2).
4. The STM32 firmware can now receive controller data for robot control.

---

## 🚀 Getting Started

### 1. Clone the Repository

```sh
git clone <repo-url>
cd 4_wheel_chassis
```

### 2. Open in STM32CubeIDE

- Open STM32CubeIDE.
- Select `File > Open Projects from File System...` and import the `4_wheel_chassis` folder.

### 3. Hardware Setup

- Connect the BNO055 IMU to I2C1 (PB8=SCL, PB9=SDA).
- Connect your four DC motors to PD12–PD15 (TIM4 PWM outputs).
- Set up an Arduino as a PS4 controller bridge and connect via I2C2.

### 4. Build and Flash

- Select the build configuration (Debug/Release).
- Click the build button.
- Connect your STM32 board and flash the firmware.

---

## 🧩 Main Files

- **main.c**: Entry point; initializes peripherals, reads BNO055 data, prints orientation.
- **bno055.c/h**: Driver for the BNO055 IMU.
- **test.c**: Example/test code for I2C, PWM, and BNO055.
- **Ps4_Interfacing_Arduino_I2C2.c**: Example for PS4 controller input via Arduino/I2C.
- **self.c**: Minimal self-test for BNO055 and system clock.

---

## 📝 Example: Reading Orientation

The main loop in `main.c`:

```c
while (1) {
    bno055_vector_t v = bno055_getVectorEuler();
    printf("Heading: %.2f Roll: %.2f Pitch: %.2f\r\n", v.x, v.y, v.z);
    v = bno055_getVectorQuaternion();
    printf("W: %.2f X: %.2f Y: %.2f Z: %.2f\r\n", v.w, v.x, v.y, v.z);
    HAL_Delay(1000);
}
```

---

## ⚙️ Customization

- **PID Tuning**: Adjust `KP` and `KD` in the movement functions for your chassis.
- **Controller Mapping**: Modify `Ps4_Interfacing_Arduino_I2C2.c` for your controller protocol.
- **Sensor Fusion**: Use different BNO055 operation modes as needed.

---

## 🛠️ Troubleshooting

- **No IMU Data**: Check I2C wiring and address (default 0x28 for BNO055).
- **No Motor Movement**: Verify PWM outputs and motor driver connections.
- **CubeIDE Build Errors**: Ensure STM32 HAL and CMSIS drivers are present in `Drivers/`.
- **PS4 Controller Not Detected**: Confirm Arduino bridge is running and I2C2 is connected.

---

## 🤝 Contributing

Pull requests are welcome! For major changes, please open an issue first to discuss what you would like to change or add. Contributions for new sensors, controller support, or improved algorithms are especially appreciated.

---

## 📜 License

This project uses STM32 HAL and CMSIS drivers. See the LICENSE file (if present) or refer to STMicroelectronics' licensing terms. All original code is provided AS-IS for educational and non-commercial use.

---

## 🙏 Credits & Acknowledgments

- Developed by wardawg, 2023–2024.
- Built with STM32CubeIDE and STM32 HAL.
- Thanks to the open-source community and STMicroelectronics for their tools and libraries.

---

## 💡 Inspiration

> "Are your dreams really worth trying if they don't let you sleep?"

This project is a labor of love for robotics, learning, and late-night tinkering. Happy hacking!
