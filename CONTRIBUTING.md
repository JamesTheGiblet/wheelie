# 🤝 Contributing to Wheelie

First off, thank you for considering contributing to the Wheelie robot project! Your help is invaluable in making this project even better. Every contribution, from a bug report to a new feature, is greatly appreciated.

This document provides guidelines for contributing to the project. Please read it carefully to ensure a smooth and effective collaboration process.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [How Can I Contribute?](#how-can-i-contribute)
  - [Reporting Bugs](#reporting-bugs)
  - [Suggesting Enhancements](#suggesting-enhancements)
  - [Submitting Code Changes](#submitting-code-changes)
- [Development Setup](#development-setup)
- [Style Guides](#style-guides)
  - [Git Commit Messages](#git-commit-messages)
  - [C Code Style](#c-code-style)
- [Pull Request Process](#pull-request-process)

---

## Code of Conduct

This project and everyone participating in it is governed by our [Code of Conduct](CODE_OF_CONDUCT.md). By participating, you are expected to uphold this code. Please report unacceptable behavior.

---

## How Can I Contribute?

### Reporting Bugs

Bugs are tracked as [GitHub Issues](https://github.com/your-username/wheelie/issues). Before creating a bug report, please check the existing issues to see if someone has already reported it.

When creating a bug report, please include as many details as possible:

- **A clear and descriptive title.**
- **A detailed description** of the problem.
- **Steps to reproduce** the behavior.
- **Expected behavior** vs. **actual behavior.**
- **Your setup:**
  - Robot hardware configuration (e.g., any different sensors).
  - Version of the firmware you are running.
  - Any modifications you have made.
- **Logs or serial output** that demonstrate the issue.

### Suggesting Enhancements

Enhancement suggestions are also tracked as [GitHub Issues](https://github.com/your-username/wheelie/issues).

- Use a clear and descriptive title.
- Provide a step-by-step description of the suggested enhancement in as many details as possible.
- Explain why this enhancement would be useful to other users.

### Submitting Code Changes

We welcome pull requests! If you're planning to implement a new feature, it's a good idea to first open an issue to discuss your proposal.

---

## Development Setup

To get your development environment set up, please follow the instructions in the main [**README.md**](README.md) file under the "Getting Started" section. This will guide you through cloning the repository, configuring credentials, and performing the initial build and upload.

---

## Style Guides

### Git Commit Messages

Please follow the [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) specification. This makes the commit history easier to read and allows for automated changelog generation.

Each commit message should be in the format: `type(scope): description`

- **type**: `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`.
- **scope** (optional): The module affected (e.g., `nav`, `power`, `sensors`).
- **description**: A short, imperative-tense description of the change.

**Examples:**

```txt
feat(nav): add non-blocking obstacle avoidance state machine
fix(power): correct battery percentage calculation
docs(readme): update hardware and setup sections
```

### C Code Style

Consistency is key. Please adhere to the style used throughout the existing codebase.

- **Formatting**: Use the default PlatformIO code formatter (`clang-format`). You can format a file by right-clicking and selecting "Format Document".
- **Naming Conventions**:
  - **Functions & Variables**: `camelCase` (e.g., `initializeSensors`, `lastUpdate`).
  - **Structs, Enums, & Classes**: `PascalCase` (e.g., `SystemStatus`, `RobotState`).
  - **Constants & Macros**: `UPPER_SNAKE_CASE` (e.g., `OBSTACLE_DISTANCE`, `LED_RED_PIN`).
- **Modularity**: Respect the existing layered architecture (HAL, Core Services, Application, UI). New functionality should be placed in the appropriate module. For example, direct hardware interactions belong in the HAL (`sensors.cpp`, `motors.cpp`), while high-level decision-making belongs in the Application layer (`navigation.cpp`, `robot.cpp`).
- **Comments**:
  - Use comments to explain the *why*, not the *what*.
  - Use Doxygen-style comments for function headers to describe their purpose, parameters, and return values.

    ```cpp
    /**
     * @brief Calculates the remaining battery percentage.
     * @param voltage The current battery voltage.
     * @return The estimated percentage (0.0 to 100.0).
     */
    float calculateBatteryPercentage(float voltage);
    ```

---

## Pull Request Process

1. **Fork the repository** and create your branch from `main`.
2. Make your changes, adhering to the style guides.
3. **Test your changes thoroughly** on the physical robot to ensure they work as expected and do not introduce regressions.
4. **Update documentation**. If you are adding a new feature or changing existing behavior, update the relevant documentation in the `docs/` directory and the `README.md`.
5. **Submit your Pull Request** to the `main` branch of the main repository.
6. In your PR description:
    - Provide a clear explanation of the changes.
    - Link to any related issues (e.g., "Closes #123").
    - Describe the testing you have performed.
7. Your PR will be reviewed, and you may be asked to make changes before it can be merged.

Thank you for your contribution!

---

### New Code of Conduct File

To support the `CONTRIBUTING.md`, I'll also create a standard `CODE_OF_CONDUCT.md` file.

```diff
--- /dev/null
 b/CODE_OF_CONDUCT.md
@@ -0,0 1,54 @@
# Contributor Covenant Code of Conduct

## Our Pledge

We as members, contributors, and leaders pledge to make participation in our
community a harassment-free experience for everyone, regardless of age, body
size, visible or invisible disability, ethnicity, sex characteristics, gender
identity and expression, level of experience, education, socio-economic status,
nationality, personal appearance, race, religion, or sexual identity
and orientation.

We pledge to act and interact in ways that contribute to an open, welcoming,
diverse, inclusive, and healthy community.

## Our Standards

Examples of behavior that contributes to a positive environment for our
community include:

*   Demonstrating empathy and kindness toward other people
*   Being respectful of differing opinions, viewpoints, and experiences
*   Giving and gracefully accepting constructive feedback
*   Accepting responsibility and apologizing to those affected by our mistakes,
    and learning from the experience
*   Focusing on what is best not just for us as individuals, but for the
    overall community

Examples of unacceptable behavior include:

*   The use of sexualized language or imagery, and sexual attention or
    advances of any kind
*   Trolling, insulting or derogatory comments, and personal or political attacks
*   Public or private harassment
*   Publishing others' private information, such as a physical or email
    address, without their explicit permission
*   Other conduct which could reasonably be considered inappropriate in a
    professional setting

## Enforcement

Community leaders are responsible for clarifying and enforcing our standards of
acceptable behavior and will take appropriate and fair corrective action in
response to any behavior that they deem inappropriate, threatening, offensive,
or harmful.

Community leaders have the right and responsibility to remove, edit, or reject
comments, commits, code, wiki edits, issues, and other contributions that are
not aligned to this Code of Conduct, and will communicate reasons for moderation
decisions when appropriate.

## Attribution

This Code of Conduct is adapted from the [Contributor Covenant][homepage],
version 2.0, available at
[https://www.contributor-covenant.org/version/2/0/code_of_conduct.html][v2.0].

[homepage]: https://www.contributor-covenant.org
[v2.0]: https://www.contributor-covenant.org/version/2/0/code_of_conduct.html

```

This provides a solid foundation for managing contributions and maintaining a healthy, collaborative environment for your project.

<!--
[PROMPT_SUGGESTION]Refactor the `main.cpp` file to be the main entry point for the full robot application.[/PROMPT_SUGGESTION]
[PROMPT_SUGGESTION]Analyze the `platformio.ini` file and suggest improvements or optimizations.[/PROMPT_SUGGESTION]
-->
