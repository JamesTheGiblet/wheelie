# Changelog: RobotForge Firmware - The "Swarm Intelligence" Update

This milestone update transitions the robot from a solitary agent to a collaborative member of a swarm. The navigation core has been upgraded to a learning-based system, and the firmware now includes a full suite of modern development and control tools, including P2P communication, OTA updates, and remote web-based telemetry.

## ✨ New Features

* **🧠 Reinforcement Learning Navigator (`LearningNavigator`)**: Replaced the previous simple navigator with an advanced brain that learns from its interactions. It now calculates a final drive vector by combining weighted inputs from a goal-seeking vector, a repulsive obstacle vector (from the HAL), and a new social `swarmForce` vector. This allows for more complex and adaptive navigation.
* **� Swarm Intelligence & Communication**:
  * **ESP-NOW Protocol**: Introduced the `SwarmCommunicator` singleton to manage peer-to-peer robot communication using the efficient, low-latency ESP-NOW protocol. This avoids the overhead of a traditional Wi-Fi network.
  * **Coordinated Movement**: The `LearningNavigator` now incorporates a `swarmForce`, allowing robots to influence each other's movements for emergent behaviors like flocking, formation-keeping, or coordinated exploration.
  * **State Broadcasting**: Robots now continuously broadcast their state (current position and velocity) to all peers in the swarm.
* **📡 Over-the-Air (OTA) Updates**: Integrated an `ota_manager` using the `ArduinoOTA` library. Firmware can now be updated wirelessly over the network, eliminating the need for a physical serial connection and dramatically speeding up the development cycle.
* **💻 Web Server & WebSocket Telemetry**: A web server (`web_server`) has been added for remote monitoring. It serves a web interface and pushes real-time telemetry (such as robot state, speed, and position) to connected clients via WebSockets for live visualization.
* **⌨️ Command-Line Interface (CLI)**: A serial-based CLI (`cli_manager`) has been implemented for direct, low-level interaction. This is essential for on-the-fly diagnostics, parameter tuning, and manual control without needing a network connection.

## 🚀 Improvements

* **⚡ Multi-Core Processing**: To guarantee real-time performance, data logging has been offloaded to a dedicated FreeRTOS task (`log_task`) pinned to Core 0. This prevents blocking I/O operations from impacting the performance of the main control and navigation loop, which runs exclusively on Core 1.
* **📊 Enhanced Diagnostics**:
  * **Periodic Reporting**: The main loop now prints a comprehensive diagnostic report every few seconds, including system status (memory, uptime), navigation vectors, learning statistics, and swarm health.
  * **On-Demand Status**: Added `printSystemInfo()` and `printNavigationStatus()` helper functions, callable via the CLI, for detailed, on-demand status checks.
* **🏗️ Architectural Refinement (HAL/Brain)**: Solidified the 2-layer architecture by strictly separating the hardware control (`WheelieHAL`, the "Body") from the navigation logic (`LearningNavigator`, the "Brain"). The Brain now operates on a generic `HAL` interface, making the core navigation logic portable to entirely new robot platforms with minimal code changes.

## 🛠️ Code Refactoring

* **Singleton Pattern**: The `SwarmCommunicator` is now implemented as a singleton (`SwarmCommunicator::getInstance()`) to ensure a single, globally accessible point for managing all swarm interactions.
* **State Management**: Centralized robot state control through a dedicated state machine (`RobotStateEnum`) and accessor functions (`setRobotState()`, `getCurrentState()`). This provides a clear and predictable lifecycle for the robot's operational modes (e.g., `IDLE`, `NAVIGATING`, `LEARNING`).
