# 🚀 Project Overview: Wheelie Autonomous Robot

## 1. Vision & Goal

An advanced, open-source autonomous robot built on the ESP32 platform. The project features a professional, layered software architecture designed for scalability, real-world performance, and community contribution. The core vision is to create a fleet of intelligent robots that can learn and evolve their behaviors.

---

## 2. Current Project Status (as of November 16, 2025)

- **Firmware**: The core firmware is now almost entirely non-blocking, including the animation system and sensor handling in the HAL. The `LearningNavigator` has been integrated into `main.cpp`, providing a foundation for experience-based learning.
- **Architecture**: The Hardware Abstraction Layer (HAL) architecture is fully implemented and documented, providing a robust and scalable foundation for adding new robot types.
- **Navigation**: The Potential Field Navigation "Brain" is implemented and integrated. The robot can perform fluid, continuous navigation and obstacle avoidance. The next major step is fine-tuning its parameters for optimal real-world performance.
- **Documentation**: The project's documentation is now comprehensive, consistent, and synchronized with the latest hardware and software. All major systems have dedicated guides.

---

## 3. Architectural Pillars

The project is built on three core design principles:

1. **Hardware Abstraction Layer (HAL)**: A formal C++ interface separates the robot's "Brain" (universal navigation logic) from its "Body" (hardware-specific implementation). This allows the same intelligence to run on different physical robots with minimal code changes.
2. **Potential Field Navigation (PFN)**: Instead of discrete start-stop logic, the robot navigates a continuous "pressure gradient" of attractive (goal) and repulsive (obstacle) forces. This results in smooth, fluid, and intelligent-looking movement.
3. **Non-Blocking, Event-Driven Code**: All operational code, from sensor reads to animations, is designed to be non-blocking. This ensures the main loop runs at a high, consistent frequency, making the robot highly responsive to its environment.

---

## 4. Key Implemented Features

- 🏛️ **Professional HAL Architecture**: Clean separation of Brain and Body.
- 🧠 **Intelligent Navigation**: Fluid movement with Potential Field Navigation.
- 🤖 **Autonomous Calibration**: One-time, self-calibrating routine for motors and sensors.
- 🔋 **Smart Power Management**: 5-level power modes with graceful degradation.
- 📡 **Over-the-Air (OTA) Updates**: Stable and reliable wireless firmware updates.
- 💡 **Generic Animation Engine**: A non-blocking system for creating complex LED and sound animations.
- 🤝 **Swarm Communication**: ESP-NOW mesh networking for multi-robot coordination.
- 🎓 **Foundational Learning**: A `LearningNavigator` is implemented, ready for experience-based improvement.

---

## 5. Roadmap: The Next Month

The immediate focus is on leveraging the robust platform we've built to enhance the robot's autonomy and intelligence.

- **Advanced Navigation**:
  - Fine-tune the Potential Field Navigation parameters for optimal real-world performance.
  - Implement and test the `FormationController` for coordinated swarm movement.

- **AI/Brain Expansion**:
  - Expand the `LearningNavigator` to allow the robot to adapt its navigation parameters based on its recorded experiences.
  - Begin implementation of the `TaskAllocator` for dynamic role assignment within the swarm.

- **Community & Contribution**:
  - Continue to refine documentation based on feedback.
  - Encourage new contributors to build their own robots and test the firmware.

---

*The Wheelie project has successfully transitioned from a hardware assembly project into a robust, extensible software platform ready for advanced robotics and AI experimentation. The next month will focus on making the robot smarter, more autonomous, and even easier for others to adopt and extend.*
