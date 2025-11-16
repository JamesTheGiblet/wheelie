
# Changelog: Week Ending 2025-10-26

## Assembly, Breadboard Integration, and Initial Wiring

**Step-by-step progress for this week:**

1. **Chassis & Body Assembly**

- Completed mechanical assembly of the chassis, including mounting the ESP32, motor driver, battery holder, and main PCB/breadboard.
- Ensured all mounting points were secure and accessible for wiring.

1. **Power System Setup**

- Wired the battery pack, power switch, and voltage divider for battery monitoring.
- Verified correct voltage readings on the breadboard before connecting to the ESP32.

1. **Motor & Driver Integration**

- Connected the MOSFET H-Bridge (or L298N, if used) to the ESP32 and motors.
- Tested motor direction and PWM control, confirming correct pin assignments.

1. **Encoder Installation**

- Wired LM393 H2010 encoders to the ESP32, ensuring correct pull-up configuration.
- Verified encoder counts and direction using manual wheel rotation.

1. **Body Sensor Integration**

- Installed and wired the following sensors:
- **VL53L0X ToF (front):** Connected to I2C bus (SDA/SCL), verified distance readings.
- **MPU6050 IMU:** Connected to I2C bus, confirmed angle and acceleration data.
- **KY-009 RGB LED:** Wired to PWM pins, tested color output.
- **Buzzer:** Connected to digital pin, tested with simple tone sequence.
- **H-1-0332 Sound Sensor:** Wired to digital input, confirmed sound detection.
- **(If present) HC-SR04 Ultrasonic:** Wired to trigger/echo pins, tested distance measurement.

1. **Breadboard Testing & Debugging**

- Powered up the full system on the breadboard.
- Sequentially tested each subsystem (motors, encoders, sensors, power) using test sketches and serial output.
- Debugged any wiring or pinout issues, updating the wiring guide as needed.

1. **Documentation & Confirmation**

- Updated assembly and wiring guides to reflect the breadboard-based build and pin assignments.
- Marked this Sunday as the milestone for successful integration and confirmation of all body sensors and core systems on the breadboard.

## Assembly & Wiring Guide Changes
