# 4-Wheel Bot: Next-Gen Embedded Robotics Platform 🚗✨

Welcome to the future of embedded robotics! This project is a showcase of advanced real-time control, sensor fusion, and modular firmware engineering—built for the STM32F407VG and designed to impress engineers, makers, and recruiters alike.

---

## 🚀 Project Highlights

- **Real-Time 4-Wheel Chassis Control**: Precision PWM motor control for omnidirectional movement.
- **Sensor Fusion**: BNO055 IMU integration for orientation, heading, and stabilization.
- **PS4 Wireless Control**: Seamless remote operation via a custom Arduino I2C bridge.
- **Custom STM32 Drivers**: Handcrafted low-level drivers for GPIO, I2C, SPI, and more.
- **Modular, Extensible Codebase**: Clean separation of logic, drivers, and hardware abstraction.
- **Recruiter-Ready**: Modern C, robust architecture, and real-world robotics application.

---

## 🏗️ Directory Structure

```
.
├── Arduino_code/
│   ├── Ps4_arduino_slave_sender_5bytes/
│   │   └── Ps4_arduino_slave_sender_5bytes.ino
│   └── Ps4_arduino_slave_sender_7bytes/
│       └── Ps4_arduino_slave_sender_7bytes.ino
├── 4_wheel_chassis/
│   ├── Core/
│   │   ├── Inc/
│   │   │   └── Backup/
│   │   ├── Src/
│   │   │   ├── Navigation.c      # Main navigation/control logic
│   │   │   ├── test.c
│   │   │   ├── self.c
│   │   │   ├── self_msp.c
│   │   │   ├── tim.c
│   │   │   ├── gpio.c
│   │   │   ├── i2c.c
│   │   │   ├── bno055.c
│   │   │   ├── sysmem.c
│   │   │   ├── stm32f4xx_it.c
│   │   │   ├── system_stm32f4xx.c
│   │   │   ├── src/              # Custom STM32F407 peripheral drivers
│   │   │   └── Backup/           # Backup/legacy source files
│   │   └── Startup/
│   ├── Drivers/
│   │   ├── STM32F4xx_HAL_Driver/
│   │   └── CMSIS/
│   ├── Debug/
│   ├── .settings/
│   ├── .cproject, .project, .mxproject
│   ├── *.launch, *.cfg, *.ioc
```

---

## 🕹️ Arduino PS4 Controller Bridge

**Location:** `Arduino_code/`

Turn any Arduino + USB Host Shield into a wireless PS4 controller-to-I2C bridge. This lets you drive the robot in real time with a PS4 controller—no wires, no lag.

- **5-byte & 7-byte modes**: Choose your data granularity (basic movement or full touchpad support).
- **Plug-and-play**: Just flash, pair, and connect I2C to the STM32 board.

---

## 🧩 Main Firmware Files

- **Navigation.c**: The heart of the robot—handles I2C command reception, real-time kinematics, and motor control.
- **bno055.c/h**: IMU driver for orientation and stabilization.
- **test.c**: Standalone test routines for hardware bring-up.
- **self.c**: Minimal self-test and diagnostics.
- **src/**: Custom STM32F407 peripheral drivers (GPIO, I2C, SPI, RCC).

---

## 💡 Example: Real-Time Robot Control Loop

Here’s a glimpse of the core logic from `Navigation.c`—where PS4 commands become smooth, omnidirectional movement:

```c
while (1) {
    // Receive command from Arduino PS4 bridge
    commandcode = 0x05;
    I2C_MasterSendData(&I2C2Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR);
    I2C_MasterReceiveData(&I2C2Handle, rcv_buf, len, SLAVE_ADDR, I2C_DISABLE_SR);

    // Map and process joystick data
    w = map(rcv_buf[2], 0, 255, -127, 127);
    x = (rcv_buf[0] < 138 && rcv_buf[0] > 116) ? 0 : rcv_buf[0] - 127;
    y = (rcv_buf[1] < 138 && rcv_buf[1] > 116) ? 0 : 127 - rcv_buf[1];
    v = sqrt((pow(x, 2) + pow(y, 2)));
    v = map(v, 0, 150, 0, 100);
    v_w = map(w, 0, 255, 0, 43);
    angle = atan2(y, x);
    movement(v, v_w, angle, KP, KD);
    HAL_Delay(1);
}

void movement(float v, float v_w, float angle, float KP, float KD) {
    v_x = v * cos(angle);
    v_y = v * sin(angle);
    // ... PID and correction logic ...
    w1 = (0.70711 * (-v_x + v_y)) + w + correction;
    w2 = (0.70711 * (-v_x - v_y)) + w + correction;
    w3 = (0.70711 * (v_x - v_y)) + w + correction;
    w4 = (0.70711 * (v_x + v_y)) + w + correction;
    __HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_1, fabs(w1));
    __HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_2, fabs(w2));
    __HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_3, fabs(w3));
    __HAL_TIM_SET_COMPARE(&htimer4, TIM_CHANNEL_4, fabs(w4));
}
```

---

## 🌟 Why This Project Stands Out

- **End-to-End Robotics Stack**: From wireless gamepad to real-time motor control, every layer is engineered for performance and clarity.
- **Custom Drivers**: No black boxes—understand and extend every part of the hardware interface.
- **Recruiter-Ready Code**: Modern C, modular design, and clear documentation.
- **Plug-and-Play Demos**: Instantly show off wireless control, sensor fusion, and advanced kinematics.
- **Extensible**: Add new sensors, swap controllers, or port to other STM32 boards with ease.

---

## 🛠️ Getting Started

1. **Clone the repo** and open in STM32CubeIDE.
2. **Flash the Arduino bridge** with your preferred `.ino` sketch.
3. **Wire up** the BNO055 IMU, motors, and I2C connections.
4. **Build and flash** the STM32 firmware.
5. **Pair your PS4 controller** and drive your bot like a pro!

---

## 🤝 Contributing & License

Pull requests are welcome! For major changes, open an issue to discuss your ideas. All original code is provided AS-IS for educational and non-commercial use. See LICENSE and STMicroelectronics terms for dependencies.

---

## 🙏 Credits & Inspiration

- Developed by wardawg, 2023–2024.
- Built with STM32CubeIDE, STM32 HAL, and a passion for robotics.
- Thanks to the open-source community and STMicroelectronics.

> "Are your dreams really worth trying if they don't let you sleep?"

---

**Ready to build, learn, and impress? This is the robotics platform you’ve been looking for.**
