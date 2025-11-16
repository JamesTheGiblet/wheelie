# 🌐 Web Dashboard Design Document

## 1. Overview

This document outlines the plan for a web-based dashboard, served directly from the ESP32, to monitor and control the Wheelie robot in real-time. The dashboard will provide a user-friendly graphical interface accessible from any web browser on the same local network, serving as a powerful alternative to the serial monitor CLI.

---

## 2. Technology Stack

- **Backend (ESP32)**:
  - **Web Server**: `ESPAsyncWebServer`. This library is asynchronous and non-blocking, making it a perfect fit for our existing firmware architecture. It can handle multiple connections without disrupting the main navigation loop.
  - **Real-time Communication**: `WebSockets`. We will use a WebSocket connection to push live data (e.g., pose, sensor readings, state changes) from the robot to the web browser, eliminating the need for constant HTTP polling.
  - **Filesystem**: `LittleFS`. The HTML, CSS, and JavaScript files for the dashboard will be stored on the ESP32's flash memory using the LittleFS filesystem.

- **Frontend (Browser)**:
  - **Structure**: Plain HTML5.
  - **Styling**: Simple, clean CSS for a responsive layout.
  - **Logic**: Vanilla JavaScript to handle WebSocket communication, update UI elements, and send commands back to the robot.
  - **Visualization (Optional)**: A lightweight library like `Chart.js` could be used to create live graphs for sensor data or battery voltage over time.

---

## 3. UI Layout & Features

The dashboard will be a single-page application with a responsive grid layout, making it usable on both desktop and mobile devices.

### Mockup

```txt
┌──────────────────────────────────────────────────────────────────────────┐
│ 🤖 Wheelie Dashboard | WiFi: 📶 Strong | State: EXPLORING                 │
├──────────────────────────────────────────────────────────────────────────┤
│ 📊 SYSTEM STATUS                  | 🧭 NAVIGATION CONTROL                │
│ --------------------------------- | ------------------------------------ │
│ 🔋 Battery: 8.1V (95%)            | Pose: (150.2, 50.1) | H: 45.2°     │
│ ⏱️ Uptime: 1h 15m 30s              | Goal: (1000.0, 0.0)                  │
│ 🧠 Free Heap: 245 KB              | Velocity: (35.0, 5.0) mm/s           │
│                                   | [Set New Goal] [Emergency Stop]      │
├───────────────────────────────────┼──────────────────────────────────────┤
│ 👁️ LIVE SENSOR DATA                | 🤝 SWARM VIEW                          │
│ --------------------------------- | ------------------------------------ │
│ ToF Front: 35.4 cm                | (A simple 2D plot showing the        │
│ Ultrasonic Rear: 120.1 cm         | position of this robot and any       │
│ IMU Tilt: (1.2°, -0.5°)           | other robots in the swarm)           │
│                                   |                                      │
├───────────────────────────────────┴──────────────────────────────────────┤
│ 📜 LIVE LOGS                                                             │
│ ------------------------------------------------------------------------ │
│ [NAV] New goal set: (1000, 0)                                            │
│ [HAL] Obstacle detected: 35.4 cm                                         │
│ [BRAIN] Applying repulsive force: (-15.2, 0.0)                           │
└──────────────────────────────────────────────────────────────────────────┘
```

### Key Features

1. **Real-time Monitoring**:
    - Display current robot state (e.g., `EXPLORING`, `AVOIDING`).
    - Show live battery voltage and calculated percentage.
    - Visualize robot's position and heading.
    - Display live data from key sensors (ToF, Ultrasonic, IMU).

2. **Interactive Control**:
    - An "Emergency Stop" button that immediately halts the robot.
    - A "Set New Goal" button that opens a prompt to enter new X/Y coordinates for the navigator.
    - A "Force Recalibration" button to trigger the autonomous calibration sequence on the next reboot.

3. **Swarm Visualization**:
    - A simple 2D canvas that plots the current robot's position and the last known positions of any other robots detected via ESP-NOW.

4. **Log Streaming**:
    - A text area that displays the most recent log messages streamed from the robot over the WebSocket.

---

## 4. API & Data Flow

The communication between the dashboard and the robot will use a combination of HTTP for initial setup and WebSockets for continuous data flow.

### HTTP Endpoints

- `GET /`: Serves the main `index.html` file from LittleFS.
- `GET /style.css`, `GET /app.js`: Serves the static assets.
- `GET /status`: Returns a JSON object with the robot's static configuration and current status (called once on page load).

    ```json
    {
      "robotName": "Wheelie-A1",
      "firmwareVersion": "v1.3.0",
      "isCalibrated": true
    }
    ```

- `POST /command`: An endpoint to send one-off commands to the robot.

    ```json
    // Request Body
    { "command": "set_goal", "x": 1000, "y": 500 }
    ```

### WebSocket Communication (`/ws`)

- **Server -> Client (Push)**: The ESP32 will periodically (e.g., 5 times per second) push a JSON message to all connected clients with the latest real-time data.

    ```json
    {
      "type": "telemetry",
      "state": "Exploring",
      "pose": { "x": 150.2, "y": 50.1, "h": 45.2 },
      "battery": { "v": 8.1, "p": 95 },
      "sensors": { "tof": 35.4, "ultrasonic": 120.1 },
      "swarm": [
        { "id": "A1", "x": 150.2, "y": 50.1 },
        { "id": "B2", "x": 300.0, "y": -25.5 }
      ]
    }
    ```

- **Client -> Server (Push)**: The web client can send simple JSON messages to the server for actions that don't fit the `POST /command` model.

---

## 5. Implementation Roadmap

### Phase 1: Backend Foundation

1. **Integrate Libraries**: Add `ESPAsyncWebServer` and its dependencies to `platformio.ini`.
2. **Setup Web Server**: In a new `web_server.cpp` module, initialize the async web server.
3. **Filesystem Integration**: Create a `data` directory for the web files and configure PlatformIO to build and upload a LittleFS image.
4. **Basic Endpoints**: Implement the `GET /` and `GET /status` endpoints.
5. **WebSocket Server**: Set up the WebSocket server at `/ws` and implement a simple `onConnect` handler.

### Phase 2: Frontend & Real-time Data

1. **Create `index.html`**: Build the basic HTML structure for the dashboard layout.
2. **Create `app.js`**: Write JavaScript to connect to the WebSocket.
3. **Implement Data Push**: In `main.cpp`, create a function that gathers all telemetry data, formats it as JSON, and pushes it to all connected WebSocket clients.
4. **Implement Data Display**: In `app.js`, write the logic to parse the incoming JSON messages and update the corresponding HTML elements on the dashboard.

### Phase 3: Interactive Controls

1. **Implement `POST /command`**: Create the handler in `web_server.cpp` to parse incoming command JSON and call the appropriate functions (e.g., `navigator.setGoal()`, `hal.emergencyStop()`).
2. **Add Frontend Controls**: Add buttons and input fields to the HTML and write the JavaScript `fetch` requests to send commands to the server.

### Phase 4: Advanced Visualization

1. **Swarm View**: Use an HTML `<canvas>` element to draw the robot and peer positions.
2. **Live Charts**: Integrate a library like `Chart.js` to plot sensor data or battery voltage over time.
