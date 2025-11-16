# HC-SR04 Ultrasonic Sensor Guide

## 1. Overview

The HC-SR04 is a low-cost, widely used ultrasonic distance measuring sensor. It determines the distance to an object by sending out a sound wave and measuring how long it takes for the echo to return.

In the Wheelie robot, this sensor is used as a **rear-facing obstacle detector**. It provides the "Body" (the `WheelieHAL`) with the necessary information to generate a repulsive force, pushing the robot away from objects behind it.

---

## 2. Key Specifications

- **Operating Voltage**: 5V DC
- **Detection Range**: 2cm to 400cm (4 meters)
- **Measuring Angle**: ~15 degrees
- **Operating Frequency**: 40 kHz
- **Interface**: 4 pins (VCC, Trig, Echo, GND)

---

## 3. Wiring

The sensor is connected directly to the ESP32. Since it requires 5V, it should be powered from the 5V rail of the buck converter, not the 3.3V pin of the ESP32.

```txt
HC-SR04 Pin -> ESP32 Pin
--------------------------
VCC         -> 5V Rail
Trig        -> GPIO 16
Echo        -> GPIO 32
GND         -> GND Rail
```

---

## 4. How It Works (Theory)

The sensor operates on a simple trigger-echo principle:

1. **Trigger**: We send a short, 10-microsecond high pulse to the `Trig` pin.
2. **Transmit**: The sensor responds by sending out a burst of 8 ultrasonic sound waves.
3. **Echo**: The sensor then raises the `Echo` pin to HIGH.
4. **Listen**: The `Echo` pin remains HIGH until the sound waves return after bouncing off an object.
5. **Measure**: We measure the duration (in microseconds) that the `Echo` pin was HIGH.
6. **Calculate**: The distance is calculated using the formula:
    `Distance (cm) = (Duration * Speed of Sound) / 2`

    We divide by 2 because the duration measures the time for the sound to go to the object *and* return.

---

## 5. Firmware Integration (`WheelieHAL`)

The logic for reading and interpreting the HC-SR04 sensor is located in `src/WheelieHAL.cpp`. It is designed to be robust and to avoid blocking the main loop.

### Reading the Sensor (`updateAllSensors`)

The reading process is handled in the `WheelieHAL::updateAllSensors()` function and involves two key features:

1. **Semi-Non-Blocking Read**: The firmware uses the `pulseIn()` function with a timeout.

    ```cpp
    long duration_us = pulseIn(FRONT_ULTRASONIC_ECHO_PIN, HIGH, 38000);
    ```

    - The `38000` microsecond (38ms) timeout is crucial. It corresponds to the sensor's maximum theoretical range.
    - If no echo is received within this time, `pulseIn()` returns 0 and stops blocking, preventing the main loop from freezing indefinitely.

2. **Moving Average Filter**: Raw ultrasonic readings can be noisy. To get a stable distance measurement, the firmware uses a **5-sample moving average filter**.

    ```cpp
    // In WheelieHAL.cpp
    const int ULTRASONIC_FILTER_SIZE = 5;
    float ultrasonicReadings[ULTRASONIC_FILTER_SIZE] = {0};
    ```

    - When a new, valid reading is received, it is added to a circular buffer (`ultrasonicReadings`).
    - The final distance stored in `sensors.rearDistanceCm` is the average of the last 5 valid readings.
    - This smoothing process filters out occasional spurious readings and provides a much more stable value for the navigation system.

### Usage in Navigation (`getObstacleRepulsion`)

The "Brain" (`PotentialFieldNavigator`) is only concerned with forces, not sensor types. The `WheelieHAL` is responsible for translating the sensor reading into a force.

1. The `getObstacleRepulsion()` function reads the filtered `sensors.rearDistanceCm`.
2. If the distance is within the `REAR_INFLUENCE_RADIUS` (30cm), it calculates a repulsive force.
3. Because the sensor is on the rear of the robot (the negative X-direction in the HAL's coordinate system), the repulsive force it generates is in the **positive X-direction** (pushing the robot forward).

    ```cpp
    // In WheelieHAL::getObstacleRepulsion()
    if (rearDistanceCm < REAR_INFLUENCE_RADIUS) {
        // Repulsion force is forward (HAL standard: X+).
        float strength = REAR_REPULSION_STRENGTH * (1.0f - (rearDistanceCm / REAR_INFLUENCE_RADIUS));
        totalRepulsionForce += Vector2D(strength, 0.0f);
    }
    ```

This allows the robot to fluidly move away from objects behind it without the Brain needing to know that an ultrasonic sensor was involved.

---

## 6. Troubleshooting

- **Readings are always 0 or max range**: Check your wiring. A loose `Trig` or `Echo` pin is the most common cause. Ensure the sensor is receiving a stable 5V.
- **Readings are noisy or erratic**: The moving average filter should handle most noise. If it's still an issue, ensure the sensor is mounted securely and is not picking up vibrations from the chassis. Hard, flat surfaces produce the best results. Soft or angled surfaces can absorb or deflect the sound waves.

---
