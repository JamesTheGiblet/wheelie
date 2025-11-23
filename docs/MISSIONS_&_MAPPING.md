# 🚀 Project Autonomy: Missions & Mapping - Implementation Guide

**Version:** 1.0  
**Date:** November 2024  
**Prerequisites:** Completed "Swarm Intelligence" update with working HAL, Navigator, and ESP-NOW communication

---

## 📋 Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Phase 1: Mission Control & High-Level Logic](#phase-1-mission-control--high-level-logic)
3. [Phase 2: Environmental Mapping](#phase-2-environmental-mapping)
4. [Phase 3: Enhanced Intelligence & Persistence](#phase-3-enhanced-intelligence--persistence)
5. [Phase 4: System Refinement](#phase-4-system-refinement)
6. [Testing & Validation](#testing--validation)
7. [Troubleshooting](#troubleshooting)

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    MISSION CONTROL LAYER                     │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────────┐   │
│  │ Web Command │  │ Task Manager │  │ Role Assignment  │   │
│  │   Center    │──│   & Auction  │──│   (L/S/W)       │   │
│  └─────────────┘  └──────────────┘  └──────────────────┘   │
└────────────────────────────┬────────────────────────────────┘
                             │
┌────────────────────────────┴────────────────────────────────┐
│                   ENVIRONMENTAL LAYER                        │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────────┐   │
│  │ Occupancy   │  │ Map Merger   │  │ A* Pathfinding  │   │
│  │ Grid (2D)   │──│ (ESP-NOW)    │──│   Algorithm     │   │
│  └─────────────┘  └──────────────┘  └──────────────────┘   │
└────────────────────────────┬────────────────────────────────┘
                             │
┌────────────────────────────┴────────────────────────────────┐
│                    INTELLIGENCE LAYER                        │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────────┐   │
│  │ Persistent  │  │ Battery Mgmt │  │ Return-to-Base  │   │
│  │ Learning    │──│ & Monitoring │──│   Behavior      │   │
│  │ (LittleFS)  │  │              │  │                 │   │
│  └─────────────┘  └──────────────┘  └──────────────────┘   │
└────────────────────────────┬────────────────────────────────┘
                             │
┌────────────────────────────┴────────────────────────────────┐
│                  EXISTING FOUNDATION                         │
│  HAL | Navigator | ObstacleMemory | SwarmCommunicator       │
└─────────────────────────────────────────────────────────────┘
```

### Key Design Principles

1. **Layered Architecture**: Each layer builds on the one below
2. **Backward Compatibility**: Existing code continues to work
3. **Incremental Deployment**: Test each phase before moving to the next
4. **Multi-Robot Ready**: All features work in single-bot and swarm mode

---

    grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
    gap: 10px;
    margin-bottom: 20px;
}

.mission-buttons button {
    padding: 12px;
    font-size: 14px;
    background: #4a4a6a;
    border: none;
    border-radius: 5px;
    color: white;
    cursor: pointer;
    transition: background 0.3s;
}

.mission-buttons button:hover {
    background: #5a5a7a;
}

.waypoint-setter, .role-selector {
    background: #3a3a4e;
    padding: 15px;
    border-radius: 5px;
    margin-top: 15px;
}

.waypoint-setter h3, .role-selector h3 {
    color: #00d4ff;
    margin-bottom: 10px;
}

.waypoint-setter label {
    display: inline-block;
    margin: 5px 10px 5px 0;
    color: #ccc;
}

.waypoint-setter input {
    width: 100px;
    padding: 5px;
    background: #2a2a3e;
    border: 1px solid #5a5a7a;
    border-radius: 3px;
    color: white;
}

.role-selector select {
    padding: 8px;
    background: #2a2a3e;
    border: 1px solid #5a5a7a;
    border-radius: 3px;
    color: white;
    margin-right: 10px;
}

```

Add this HTML (inside `<body>`):

```html
<!-- Mission Control Panel -->
<div class="mission-control">
    <h2>🎯 Mission Control</h2>
    
    <div class="mission-buttons">
        <button onclick="sendMission('explore')">🔍 Explore Area</button>
        <button onclick="sendMission('patrol')">🛡️ Patrol Zone</button>
        <button onclick="sendMission('return')">🏠 Return to Base</button>
        <button onclick="sendMission('abort')">⛔ Abort Mission</button>
    </div>
    
    <div class="waypoint-setter">
        <h3>Set Waypoint</h3>
        <label>X (mm): <input type="number" id="waypoint-x" value="1000"></label>
        <label>Y (mm): <input type="number" id="waypoint-y" value="0"></label>
        <button onclick="sendWaypoint()">📍 Go to Waypoint</button>
    </div>
    
    <div class="role-selector">
        <h3>Assign Role</h3>
        <select id="role-select">
            <option value="none">None</option>
            <option value="leader">Leader</option>
            <option value="scout">Scout</option>
            <option value="worker">Worker</option>
        </select>
        <button onclick="assignRole()">🎭 Assign Role</button>
    </div>
    
    <div id="mission-status">
        <h3>Current Mission</h3>
        <p id="mission-text">IDLE</p>
    </div>
</div>
```

Add this JavaScript (in `<script>` tags):

```javascript
function sendMission(type) {
    fetch(`/mission/${type}`, { method: 'POST' })
        .then(r => r.text())
        .then(msg => {
            console.log(msg);
            alert(msg);
        })
        .catch(err => console.error('Mission command failed:', err));
}

function sendWaypoint() {
    const x = document.getElementById('waypoint-x').value;
    const y = document.getElementById('waypoint-y').value;
    
    fetch(`/waypoint?x=${x}&y=${y}`, { method: 'POST' })
        .then(r => r.text())
        .then(msg => {
            console.log(msg);
            alert(msg);
        })
        .catch(err => console.error('Waypoint command failed:', err));
}

function assignRole() {
    const role = document.getElementById('role-select').value;
    
    fetch(`/role/${role}`, { method: 'POST' })
        .then(r => r.text())
        .then(msg => {
            console.log(msg);
            alert(msg);
        })
        .catch(err => console.error('Role assignment failed:', err));
}

// Update mission status in telemetry
// (Add to existing WebSocket message handler)
function updateMissionDisplay(data) {
    if (data.mission) {
        document.getElementById('mission-text').textContent = 
            `${data.mission.type} - ${data.mission.status}`;
    }
}
```

### Step 1.6: Web Server Routes

**File:** `src/web_server.cpp`

Add these routes to `initializeWebServer()`:

```cpp
// Mission control endpoints
server.on("/mission/explore", HTTP_POST, [](AsyncWebServerRequest *request){
    Mission m;
    m.type = MISSION_EXPLORE;
    m.timeoutMs = 120000; // 2 minutes
    missionController.setMission(m);
    request->send(200, "text/plain", "Exploration mission started");
});

server.on("/mission/patrol", HTTP_POST, [](AsyncWebServerRequest *request){
    Mission m;
    m.type = MISSION_PATROL;
    m.targetPosition = hal.getPose().position; // Patrol around current position
    m.patrolRadius = 500.0f; // 50cm radius
    m.timeoutMs = 180000; // 3 minutes
    missionController.setMission(m);
    request->send(200, "text/plain", "Patrol mission started");
});

server.on("/mission/return", HTTP_POST, [](AsyncWebServerRequest *request){
    Mission m;
    m.type = MISSION_RETURN_TO_BASE;
    m.targetPosition = Vector2D(0, 0); // Home position
    missionController.setMission(m);
    request->send(200, "text/plain", "Returning to base");
});

server.on("/mission/abort", HTTP_POST, [](AsyncWebServerRequest *request){
    missionController.abortMission();
    request->send(200, "text/plain", "Mission aborted");
});

server.on("/waypoint", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("x") && request->hasParam("y")) {
        float x = request->getParam("x")->value().toFloat();
        float y = request->getParam("y")->value().toFloat();
        
        Mission m;
        m.type = MISSION_GOTO_WAYPOINT;
        m.targetPosition = Vector2D(x, y);
        m.timeoutMs = 60000; // 1 minute
        missionController.setMission(m);
        
        char response[100];
        snprintf(response, sizeof(response), "Waypoint set to (%.1f, %.1f)", x, y);
        request->send(200, "text/plain", response);
    } else {
        request->send(400, "text/plain", "Missing x or y parameter");
    }
});

server.on("/role/*", HTTP_POST, [](AsyncWebServerRequest *request){
    String roleStr = request->pathArg(0);
    RobotRole role = ROLE_NONE;
    
    if (roleStr == "leader") role = ROLE_LEADER;
    else if (roleStr == "scout") role = ROLE_SCOUT;
    else if (roleStr == "worker") role = ROLE_WORKER;
    else if (roleStr == "none") role = ROLE_NONE;
    else {
        request->send(400, "text/plain", "Unknown role");
        return;
    }
    
    missionController.setRole(role);
    
    char response[50];
    snprintf(response, sizeof(response), "Role set to %s", roleStr.c_str());
    request->send(200, "text/plain", response);
});
```

Update `pushTelemetryToClients()` to include mission status:

```cpp
void pushTelemetryToClients() {
    if (ws.count() == 0) return;
    
    JsonDocument doc;
    
    // ... existing telemetry ...
    
    // Mission status
    if (missionController.isMissionActive()) {
        Mission m = missionController.getCurrentMission();
        doc["mission"]["type"] = missionController.getMissionTypeName(m.type);
        doc["mission"]["target_x"] = m.targetPosition.x;
        doc["mission"]["target_y"] = m.targetPosition.y;
        doc["mission"]["runtime"] = (millis() - m.startTime) / 1000;
    } else {
        doc["mission"]["type"] = "IDLE";
    }
    
    doc["role"] = missionController.getRoleName(missionController.getRole());
    
    String jsonString;
    serializeJson(doc, jsonString);
    ws.textAll(jsonString);
}
```

### Step 1.7: Testing Phase 1

**Test Checklist: ✅ COMPLETE**

- [x] Code compiles without errors
- [x] Robot boots successfully
- [x] CLI commands work:
  - [x] `mission` - Shows mission status
  - [x] `goto 1000 0` - Sets waypoint
  - [x] `explore` - Starts exploration
  - [x] `return` - Returns to origin
  - [x] `abort` - Aborts mission
  - [x] `role leader` - Assigns role
- [x] Web UI loads correctly
- [x] Web mission buttons send commands
- [x] Robot navigates to waypoints
- [x] Mission timeout works
- [x] Role-based parameters apply

**Expected Serial Output:**

```
🎯 Mission Controller initialized
> goto 1000 500
🎯 NEW MISSION [1]: GOTO_WAYPOINT
   Target: (1000.0, 500.0)
   Timeout: 60 seconds
Setting waypoint: (1000.0, 500.0)

> role scout
🎭 Role assigned: SCOUT

> mission
🎯 MISSION STATUS:
──────────────────────────────────────
Current Mission: GOTO_WAYPOINT
Mission ID: 1
Runtime: 12 seconds
Target: (1000.0, 500.0)
Time remaining: 48 seconds
Role: SCOUT
──────────────────────────────────────

✅ MISSION COMPLETE [1]: GOTO_WAYPOINT
```

---

## Phase 2: Environmental Mapping

**Goal:** Enable robots to build and share 2D occupancy grid maps, and use A* pathfinding for intelligent navigation.

**Estimated Time:** 12-16 hours

### Step 2.1: Create Occupancy Grid

**File:** `include/OccupancyGrid.h`

```cpp
#pragma once
#include "Vector2D.h"
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════════════
// OCCUPANCY GRID MAPPING
// ═══════════════════════════════════════════════════════════════════════════

enum CellState : uint8_t {
    CELL_UNKNOWN = 0,
    CELL_FREE = 1,
    CELL_OCCUPIED = 2
};

class OccupancyGrid {
private:
    static const int GRID_WIDTH = 100;   // 100 cells wide
    static const int GRID_HEIGHT = 100;  // 100 cells tall
    static const int CELL_SIZE_MM = 100; // Each cell is 10cm x 10cm (100mm)
    
    uint8_t grid[GRID_WIDTH][GRID_HEIGHT]; // Packed: 2 bits per cell
    Vector2D origin;                         // World origin (mm)
    
    // Helper to convert world position to grid coordinates
    void worldToGrid(const Vector2D& worldPos, int& gridX, int& gridY) const;
    Vector2D gridToWorld(int gridX, int gridY) const;
    
    // Bounds checking
    bool isInBounds(int gridX, int gridY) const;
    
public:
    OccupancyGrid();
    
    void setOrigin(const Vector2D& newOrigin) { origin = newOrigin; }
    Vector2D getOrigin() const { return origin; }
    
    // Grid manipulation
    void clearGrid();
    void updateCell(const Vector2D& worldPos, CellState state);
    CellState getCell(const Vector2D& worldPos) const;
    
    // Sensor integration
    void updateFromSensor(const Vector2D& robotPos, float heading, 
                         float sensorDistance, float sensorAngle);
    
    // Map merging for swarm
    void mergeWith(const OccupancyGrid& other, float confidenceThreshold = 0.5f);
    
    // Pathfinding support
    bool isOccupied(const Vector2D& worldPos) const;
    bool isFree(const Vector2D& worldPos) const;
    bool isUnknown(const Vector2D& worldPos) const;
    
    // Statistics
    int getOccupiedCount() const;
    int getFreeCount() const;
    int getUnknownCount() const;
    float getExplorationPercentage() const;
    
    // Serialization for ESP-NOW
    size_t serialize(uint8_t* buffer, size_t maxSize) const;
    bool deserialize(const uint8_t* buffer, size_t size);
    
    // Visualization
    void printGrid(int centerX, int centerY, int radius) const;
    String toJSON(int centerX, int centerY, int radius) const;
};
```

**File:** `src/OccupancyGrid.cpp`

```cpp
#include "OccupancyGrid.h"
#include <cmath>

OccupancyGrid::OccupancyGrid() : origin(0, 0) {
    clearGrid();
}

void OccupancyGrid::clearGrid() {
    for (int x = 0; x < GRID_WIDTH; x++) {
        for (int y = 0; y < GRID_HEIGHT; y++) {
            grid[x][y] = CELL_UNKNOWN;
        }
    }
}

void OccupancyGrid::worldToGrid(const Vector2D& worldPos, int& gridX, int& gridY) const {
    // Convert world coordinates (mm) to grid coordinates
    gridX = (int)((worldPos.x - origin.x) / CELL_SIZE_MM) + (GRID_WIDTH / 2);
    gridY = (int)((worldPos.y - origin.y) / CELL_SIZE_MM) + (GRID_HEIGHT / 2);
}

Vector2D OccupancyGrid::gridToWorld(int gridX, int gridY) const {
    // Convert grid coordinates to world coordinates (mm)
    float worldX = (gridX - GRID_WIDTH / 2) * CELL_SIZE_MM + origin.x;
    float worldY = (gridY - GRID_HEIGHT / 2) * CELL_SIZE_MM + origin.y;
    return Vector2D(worldX, worldY);
}

bool OccupancyGrid::isInBounds(int gridX, int gridY) const {
    return gridX >= 0 && gridX < GRID_WIDTH && gridY >= 0 && gridY < GRID_HEIGHT;
}

void OccupancyGrid::updateCell(const Vector2D& worldPos, CellState state) {
    int gridX, gridY;
    worldToGrid(worldPos, gridX, gridY);
    
    if (isInBounds(gridX, gridY)) {
        grid[gridX][gridY] = state;
    }
}

CellState OccupancyGrid::getCell(const Vector2D& worldPos) const {
    int gridX, gridY;
    worldToGrid(worldPos, gridX, gridY);
    
    if (isInBounds(gridX, gridY)) {
        return (CellState)grid[gridX][gridY];
    }
    
    return CELL_UNKNOWN;
}

void OccupancyGrid::updateFromSensor(const Vector2D& robotPos, float heading, 
                                     float sensorDistance, float sensorAngle) {
    // Calculate sensor endpoint in world coordinates
    float angleRad = (heading + sensorAngle) * M_PI / 180.0f;
    Vector2D sensorEnd(
        robotPos.x + sensorDistance * cos(angleRad),
        robotPos.y + sensorDistance * sin(angleRad)
    );
    
    // Mark cells along the ray as FREE
    // Use Bresenham's line algorithm for efficiency
    int x0, y0, x1, y1;
    worldToGrid(robotPos, x0, y0);
    worldToGrid(sensorEnd, x1, y1);
    
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    int x = x0;
    int y = y0;
    
    while (true) {
        // Mark this cell as FREE (except the endpoint)
        if (x != x1 || y != y1) {
            if (isInBounds(x, y) && grid[x][y] == CELL_UNKNOWN) {
                grid[x][y] = CELL_FREE;
            }
        }
        
        if (x == x1 && y == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
    
    // Mark endpoint as OCCUPIED if sensor detected something
    if (sensorDistance < 800.0f) { // Sensor max range ~80cm
        if (isInBounds(x1, y1)) {
            grid[x1][y1] = CELL_OCCUPIED;
        }
    }
}

bool OccupancyGrid::isOccupied(const Vector2D& worldPos) const {
    return getCell(worldPos) == CELL_OCCUPIED;
}

bool OccupancyGrid::isFree(const Vector2D& worldPos) const {
    return getCell(worldPos) == CELL_FREE;
}

bool OccupancyGrid::isUnknown(const Vector2D& worldPos) const {
    return getCell(worldPos) == CELL_UNKNOWN;
}

void OccupancyGrid::mergeWith(const OccupancyGrid& other, float confidenceThreshold) {
    // Simple merge: trust OCCUPIED and FREE over UNKNOWN
    for (int x = 0; x < GRID_WIDTH; x++) {
        for (int y = 0; y < GRID_HEIGHT; y++) {
            CellState myState = (CellState)grid[x][y];
            CellState otherState = (CellState)other.grid[x][y];
            
            // If I'm unknown and other has info, take it
            if (myState == CELL_UNKNOWN && otherState != CELL_UNKNOWN) {
                grid[x][y] = otherState;
            }
            // If other says OCCUPIED, trust it (safety first)
            else if (otherState == CELL_OCCUPIED) {
                grid[x][y] = CELL_OCCUPIED;
            }
        }
    }
}

int OccupancyGrid::getOccupiedCount() const {
    int count = 0;
    for (int x = 0; x < GRID_WIDTH; x++) {
        for (int y = 0; y < GRID_HEIGHT; y++) {
            if (grid[x][y] == CELL_OCCUPIED) count++;
        }
    }
    return count;
}

int OccupancyGrid::getFreeCount() const {
    int count = 0;
    for (int x = 0; x < GRID_WIDTH; x++) {
        for (int y = 0; y < GRID_HEIGHT; y++) {
            if (grid[x][y] == CELL_FREE) count++;
        }
    }
    return count;
}

int OccupancyGrid::getUnknownCount() const {
    int count = 0;
    for (int x = 0; x < GRID_WIDTH; x++) {
        for (int y = 0; y < GRID_HEIGHT; y++) {
            if (grid[x][y] == CELL_UNKNOWN) count++;
        }
    }
    return count;
}

float OccupancyGrid::getExplorationPercentage() const {
    int total = GRID_WIDTH * GRID_HEIGHT;
    int known = total - getUnknownCount();
    return (float)known / total * 100.0f;
}

void OccupancyGrid::printGrid(int centerX, int centerY, int radius) const {
    Serial.println("\n🗺️  OCCUPANCY GRID:");
    Serial.println("   . = unknown, ░ = free, █ = occupied, @ = robot");
    
    for (int y = centerY + radius; y >= centerY - radius; y--) {
        Serial.print("   ");
        for (int x = centerX - radius; x <= centerX + radius; x++) {
            if (x == centerX && y == centerY) {
                Serial.print("@ "); // Robot position
            } else if (!isInBounds(x, y)) {
                Serial.print("  "); // Out of bounds
            } else {
                switch ((CellState)grid[x][y]) {
                    case CELL_UNKNOWN:  Serial.print(". "); break;
                    case CELL_FREE:     Serial.print("░ "); break;
                    case CELL_OCCUPIED: Serial.print("█ "); break;
                }
            }
        }
        Serial.println();
    }
    
    Serial.printf("\nExploration: %.1f%%\n", getExplorationPercentage());
    Serial.printf("Free: %d | Occupied: %d | Unknown: %d\n", 
                  getFreeCount(), getOccupiedCount(), getUnknownCount());
}

// Serialization for ESP-NOW transmission
size_t OccupancyGrid::serialize(uint8_t* buffer, size_t maxSize) const {
    // Simple run-length encoding to compress the grid
    size_t idx = 0;
    
    // Header: grid dimensions and origin
    if (maxSize < 16) return 0;
    
    memcpy(buffer + idx, &GRID_WIDTH, sizeof(int)); idx += sizeof(int);
    memcpy(buffer + idx, &GRID_HEIGHT, sizeof(int)); idx += sizeof(int);
    memcpy(buffer + idx, &origin.x, sizeof(float)); idx += sizeof(float);
    memcpy(buffer + idx, &origin.y, sizeof(float)); idx += sizeof(float);
    
    // Data: run-length encoded
    for (int x = 0; x < GRID_WIDTH && idx < maxSize - 2; x++) {
        for (int y = 0; y < GRID_HEIGHT && idx < maxSize - 2; y++) {
            uint8_t state = grid[x][y];
            uint8_t count = 1;
            
            // Count consecutive cells with same state
            while (y + count < GRID_HEIGHT && 
                   grid[x][y + count] == state && 
                   count < 255) {
                count++;
            }
            
            buffer[idx++] = state;
            buffer[idx++] = count;
            y += count - 1; // Skip counted cells
        }
    }
    
    return idx;
}

bool OccupancyGrid::deserialize(const uint8_t* buffer, size_t size) {
    if (size < 16) return false;
    
    size_t idx = 0;
    
    // Read header
    int width, height;
    memcpy(&width, buffer + idx, sizeof(int)); idx += sizeof(int);
    memcpy(&height, buffer + idx, sizeof(int)); idx += sizeof(int);
    memcpy(&origin.x, buffer + idx, sizeof(float)); idx += sizeof(float);
    memcpy(&origin.y, buffer + idx, sizeof(float)); idx += sizeof(float);
    
    if (width != GRID_WIDTH || height != GRID_HEIGHT) {
        Serial.println("Grid dimension mismatch!");
        return false;
    }
    
    // Decode run-length encoded data
    int x = 0, y = 0;
    while (idx < size - 1 && x < GRID_WIDTH) {
        uint8_t state = buffer[idx++];
        uint8_t count = buffer[idx++];
        
        for (int i = 0; i < count && y < GRID_HEIGHT; i++) {
            grid[x][y] = state;
            y++;
            if (y >= GRID_HEIGHT) {
                y = 0;
                x++;
                if (x >= GRID_WIDTH) break;
            }
        }
    }
    
    return true;
}

String OccupancyGrid::toJSON(int centerX, int centerY, int radius) const {
    String json = "{\"grid\":[";
    
    for (int y = centerY + radius; y >= centerY - radius; y--) {
        json += "[";
        for (int x = centerX - radius; x <= centerX + radius; x++) {
            if (isInBounds(x, y)) {
                json += String((int)grid[x][y]);
            } else {
                json += "0";
            }
            
            if (x < centerX + radius) json += ",";
        }
        json += "]";
        if (y > centerY - radius) json += ",";
    }
    
    json += "],";
    json += "\"exploration\":" + String(getExplorationPercentage()) + ",";
    json += "\"free\":" + String(getFreeCount()) + ",";
    json += "\"occupied\":" + String(getOccupiedCount());
    json += "}";
    
    return json;
}
```

### Step 2.2: Integrate Mapping with Navigator

**File:** `include/LearningNavigator.h`

Add this include at the top:

```cpp
#include "OccupancyGrid.h"
```

Add this member variable:

```cpp
protected:
    // ... existing members ...
    OccupancyGrid localMap; // NEW: Local occupancy grid
```

Add these public methods:

```cpp
public:
    // ... existing methods ...
    
    // Map access
    OccupancyGrid& getMap() { return localMap; }
    const OccupancyGrid& getMap() const { return localMap; }
    
    // Map updating (call from main loop)
    void updateMap(const Vector2D& robotPos, float heading,
                   float frontDist, float rearDist);
```

**File:** `src/LearningNavigator.cpp` (create if it doesn't exist)

```cpp
#include "LearningNavigator.h"

void LearningNavigator::updateMap(const Vector2D& robotPos, float heading,
                                  float frontDist, float rearDist) {
    // Update from front sensor (ToF at 0° relative to heading)
    if (frontDist > 0 && frontDist < 2000.0f) { // Valid reading
        localMap.updateFromSensor(robotPos, heading, frontDist, 0.0f);
    }
    
    // Update from rear sensor (Ultrasonic at 180° relative to heading)
    if (rearDist > 0 && rearDist < 2000.0f) { // Valid reading
        localMap.updateFromSensor(robotPos, heading, rearDist, 180.0f);
    }
}
```

**File:** `src/main.cpp`

Update the main loop to integrate mapping:

```cpp
void loop() {
    // ... existing code ...
    
    if (currentState == ROBOT_EXPLORING || currentState == ROBOT_AVOIDING_OBSTACLE) {
        if (deltaTime >= 0.05f) {
            RobotPose pose = hal.getPose();
            Vector2D obstacleForce = hal.getObstacleRepulsion();
            auto otherPositions = SwarmCommunicator::getInstance().getOtherRobotPositions();
            Vector2D swarmForce = navigator.calculateSwarmForce(otherPositions);
            
            navigator.setPosition(pose.position);
            navigator.updateWithLearning(deltaTime, obstacleForce + swarmForce);
            
            // NEW: Update occupancy grid
            navigator.updateMap(pose.position, pose.heading,
                              sensors.frontDistanceCm * 10.0f,  // Convert to mm
                              sensors.rearDistanceCm * 10.0f);  // Convert to mm
            
            hal.setVelocity(navigator.getVelocity());
            SwarmCommunicator::getInstance().setMyState(pose.position, navigator.getVelocity());
            
            pushTelemetryToClients();
            lastUpdate = currentTime;
        }
    }
}
```

### Step 2.3: Add Mapping CLI Commands

**File:** `src/cli_manager.cpp`

```cpp
else if (command.equals("map")) {
    RobotPose pose = hal.getPose();
    int gridX, gridY;
    // Convert current position to grid coordinates
    // (This requires accessing grid internals or adding a helper)
    navigator.getMap().printGrid(50, 50, 10); // Print 20x20 area around center
}
else if (command.equals("clearmap")) {
    navigator.getMap().clearGrid();
    Serial.println("Map cleared");
}
```

### Step 2.4: A* Pathfinding Implementation

**File:** `include/Pathfinder.h`

```cpp
#pragma once
#include "Vector2D.h"
#include "OccupancyGrid.h"
#include <vector>
#include <queue>

// ═══════════════════════════════════════════════════════════════════════════
// A* PATHFINDING
// ═══════════════════════════════════════════════════════════════════════════

struct PathNode {
    Vector2D position;
    float gCost;  // Distance from start
    float hCost;  // Heuristic distance to goal
    int gridX, gridY;
    PathNode* parent;
    
    float fCost() const { return gCost + hCost; }
    
    PathNode(const Vector2D& pos, float g, float h, int x, int y, PathNode* p = nullptr)
        : position(pos), gCost(g), hCost(h), gridX(x), gridY(y), parent(p) {}
    
    // For priority queue (min-heap based on fCost)
    bool operator>(const PathNode& other) const {
        return fCost() > other.fCost();
    }
};

class Pathfinder {
public:
    Pathfinder() : maxIterations(1000) {}
    
    /**
     * @brief Find a path from start to goal using A*
     * @param start Starting position (world coordinates in mm)
     * @param goal Goal position (world coordinates in mm)
     * @param map Occupancy grid to navigate
     * @return Vector of waypoints (empty if no path found)
     */
    std::vector<Vector2D> findPath(const Vector2D& start, const Vector2D& goal, 
                                    const OccupancyGrid& map);
    
    void setMaxIterations(int max) { maxIterations = max; }
    
private:
    int maxIterations;
    
    float heuristic(const Vector2D& a, const Vector2D& b) const;
    std::vector<PathNode*> getNeighbors(PathNode* node, const OccupancyGrid& map);
    std::vector<Vector2D> reconstructPath(PathNode* goalNode);
    void cleanupNodes(std::vector<PathNode*>& nodes);
};
```

**File:** `src/Pathfinder.cpp`

```cpp
#include "Pathfinder.h"
#include <cmath>
#include <algorithm>

std::vector<Vector2D> Pathfinder::findPath(const Vector2D& start, const Vector2D& goal, 
                                           const OccupancyGrid& map) {
    // Check if goal is valid
    if (map.isOccupied(goal)) {
        Serial.println("⚠️ Goal is occupied!");
        return std::vector<Vector2D>(); // Empty path
    }
    
    // Convert start and goal to grid coordinates
    int startX, startY, goalX, goalY;
    // NOTE: You'll need to add public methods to OccupancyGrid for this
    // Or make Pathfinder a friend class
    
    // Open and closed sets
    std::priority_queue<PathNode*, std::vector<PathNode*>, std::greater<PathNode>> openSet;
    std::vector<PathNode*> allNodes; // For cleanup
    std::vector<std::vector<bool>> closedSet(100, std::vector<bool>(100, false));
    std::vector<std::vector<float>> gScores(100, std::vector<float>(100, INFINITY));
    
    // Create start node
    PathNode* startNode = new PathNode(start, 0, heuristic(start, goal), startX, startY);
    allNodes.push_back(startNode);
    openSet.push(startNode);
    gScores[startX][startY] = 0;
    
    int iterations = 0;
    PathNode* goalNode = nullptr;
    
    while (!openSet.empty() && iterations < maxIterations) {
        iterations++;
        
        // Get node with lowest fCost
        PathNode* current = openSet.top();
        openSet.pop();
        
        // Check if we reached the goal
        if (current->position.distanceTo(goal) < 50.0f) { // Within 5cm
            goalNode = current;
            break;
        }
        
        // Mark as visited
        closedSet[current->gridX][current->gridY] = true;
        
        // Explore neighbors
        auto neighbors = getNeighbors(current, map);
        for (PathNode* neighbor : neighbors) {
            allNodes.push_back(neighbor);
            
            if (closedSet[neighbor->gridX][neighbor->gridY]) {
                continue; // Already visited
            }
            
            float tentativeGScore = current->gCost + 
                                   current->position.distanceTo(neighbor->position);
            
            if (tentativeGScore < gScores[neighbor->gridX][neighbor->gridY]) {
                // This path is better
                neighbor->parent = current;
                neighbor->gCost = tentativeGScore;
                neighbor->hCost = heuristic(neighbor->position, goal);
                gScores[neighbor->gridX][neighbor->gridY] = tentativeGScore;
                
                openSet.push(neighbor);
            }
        }
    }
    
    std::vector<Vector2D> path;
    
    if (goalNode) {
        path = reconstructPath(goalNode);
        Serial.printf("✅ Path found! Length: %d waypoints, Iterations: %d\n", 
                      path.size(), iterations);
    } else {
        Serial.printf("❌ No path found after %d iterations\n", iterations);
    }
    
    // Cleanup
    cleanupNodes(allNodes);
    
    return path;
}

float Pathfinder::heuristic(const Vector2D& a, const Vector2D& b) const {
    // Euclidean distance
    return a.distanceTo(b);
}

std::vector<PathNode*> Pathfinder::getNeighbors(PathNode* node, const OccupancyGrid& map) {
    std::vector<PathNode*> neighbors;
    
    // 8-directional movement (including diagonals)
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    
    for (int i = 0; i < 8; i++) {
        int newX = node->gridX + dx[i];
        int newY = node->gridY + dy[i];
        
        // Convert back to world coordinates
        // NOTE: Needs helper method in OccupancyGrid
        Vector2D newPos = gridToWorld(newX, newY);
        
        // Check if neighbor is valid (in bounds and not occupied)
        if (!map.isOccupied(newPos) && !map.isUnknown(newPos)) {
            PathNode* neighbor = new PathNode(newPos, 0, 0, newX, newY);
            neighbors.push_back(neighbor);
        }
    }
    
    return neighbors;
}

std::vector<Vector2D> Pathfinder::reconstructPath(PathNode* goalNode) {
    std::vector<Vector2D> path;
    PathNode* current = goalNode;
    
    while (current != nullptr) {
        path.push_back(current->position);
        current = current->parent;
    }
    
    // Reverse to get start->goal order
    std::reverse(path.begin(), path.end());
    
    return path;
}

void Pathfinder::cleanupNodes(std::vector<PathNode*>& nodes) {
    for (PathNode* node : nodes) {
        delete node;
    }
    nodes.clear();
}
```

## Phase 3: Enhanced Intelligence & Persistence

**Goal:** Enable persistent learning across reboots and implement intelligent battery management for true autonomy.

**Estimated Time:** 10-14 hours

### Step 3.1: Persistent Learning System

**File:** `include/PersistentLearning.h`

```cpp
#pragma once
#include "LearningNavigator.h"
#include <LittleFS.h>
#include <ArduinoJson.h>

// ═══════════════════════════════════════════════════════════════════════════
// PERSISTENT LEARNING SYSTEM
// ═══════════════════════════════════════════════════════════════════════════

class PersistentLearning {
private:
    static const char* EXPERIENCE_FILE;
    static const char* MAP_FILE;
    static const char* PARAMS_FILE;
    static const int MAX_EXPERIENCES_TO_SAVE = 50; // Limit file size
    
public:
    PersistentLearning() {}
    
    /**
     * @brief Initialize filesystem (call once in setup)
     */
    bool begin();
    
    /**
     * @brief Save navigator experiences to flash
     */
    bool saveExperiences(const std::vector<NavigationExperience>& experiences);
    
    /**
     * @brief Load navigator experiences from flash
     */
    bool loadExperiences(std::vector<NavigationExperience>& experiences);
    
    /**
     * @brief Save occupancy grid map to flash
     */
    bool saveMap(const OccupancyGrid& map);
    
    /**
     * @brief Load occupancy grid map from flash
     */
    bool loadMap(OccupancyGrid& map);
    
    /**
     * @brief Save navigation parameters
     */
    bool saveParameters(const NavigationParameters& params);
    
    /**
     * @brief Load navigation parameters
     */
    bool loadParameters(NavigationParameters& params);
    
    /**
     * @brief Clear all saved data
     */
    void clearAll();
    
    /**
     * @brief Get filesystem statistics
     */
    void printStats();
};
```

**File:** `src/PersistentLearning.cpp`

```cpp
#include "PersistentLearning.h"

const char* PersistentLearning::EXPERIENCE_FILE = "/experiences.json";
const char* PersistentLearning::MAP_FILE = "/map.bin";
const char* PersistentLearning::PARAMS_FILE = "/nav_params.json";

bool PersistentLearning::begin() {
    if (!LittleFS.begin(true)) { // true = format on fail
        Serial.println("❌ Failed to mount LittleFS");
        return false;
    }
    
    Serial.println("✅ LittleFS mounted successfully");
    printStats();
    return true;
}

bool PersistentLearning::saveExperiences(const std::vector<NavigationExperience>& experiences) {
    File file = LittleFS.open(EXPERIENCE_FILE, "w");
    if (!file) {
        Serial.println("❌ Failed to open experiences file for writing");
        return false;
    }
    
    JsonDocument doc;
    JsonArray expArray = doc["experiences"].to<JsonArray>();
    
    // Save most recent experiences (limit to prevent file bloat)
    int startIdx = (experiences.size() > MAX_EXPERIENCES_TO_SAVE) ? 
                   experiences.size() - MAX_EXPERIENCES_TO_SAVE : 0;
    
    for (size_t i = startIdx; i < experiences.size(); i++) {
        JsonObject exp = expArray.add<JsonObject>();
        
        exp["start_x"] = experiences[i].startPos.x;
        exp["start_y"] = experiences[i].startPos.y;
        exp["goal_x"] = experiences[i].goalPos.x;
        exp["goal_y"] = experiences[i].goalPos.y;
        exp["score"] = experiences[i].successScore;
        exp["duration"] = experiences[i].durationMs;
        exp["steps"] = experiences[i].stepsToGoal;
        exp["timestamp"] = experiences[i].timestamp;
        
        // Obstacle configuration
        exp["obs_count"] = experiences[i].obstacleConfig.obstacleCount;
        exp["obs_avg"] = experiences[i].obstacleConfig.averageObstacleStrength;
        exp["obs_max"] = experiences[i].obstacleConfig.maxObstacleStrength;
        exp["obs_dir_x"] = experiences[i].obstacleConfig.dominantDirection.x;
        exp["obs_dir_y"] = experiences[i].obstacleConfig.dominantDirection.y;
        
        // Navigation parameters used
        JsonObject params = exp["params"].to<JsonObject>();
        params["attraction"] = experiences[i].params.attractionConstant;
        params["repulsion"] = experiences[i].params.repulsionConstant;
        params["max_speed"] = experiences[i].params.maxSpeed;
        params["damping"] = experiences[i].params.damping;
    }
    
    if (serializeJson(doc, file) == 0) {
        Serial.println("❌ Failed to serialize experiences");
        file.close();
        return false;
    }
    
    file.close();
    Serial.printf("✅ Saved %d experiences to flash\n", expArray.size());
    return true;
}

bool PersistentLearning::loadExperiences(std::vector<NavigationExperience>& experiences) {
    if (!LittleFS.exists(EXPERIENCE_FILE)) {
        Serial.println("ℹ️  No saved experiences found");
        return false;
    }
    
    File file = LittleFS.open(EXPERIENCE_FILE, "r");
    if (!file) {
        Serial.println("❌ Failed to open experiences file for reading");
        return false;
    }
    
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) {
        Serial.printf("❌ Failed to parse experiences: %s\n", error.c_str());
        return false;
    }
    
    JsonArray expArray = doc["experiences"];
    experiences.clear();
    
    for (JsonObject exp : expArray) {
        NavigationExperience e;
        
        e.startPos = Vector2D(exp["start_x"], exp["start_y"]);
        e.goalPos = Vector2D(exp["goal_x"], exp["goal_y"]);
        e.successScore = exp["score"];
        e.durationMs = exp["duration"];
        e.stepsToGoal = exp["steps"];
        e.timestamp = exp["timestamp"];
        
        // Obstacle configuration
        e.obstacleConfig.obstacleCount = exp["obs_count"];
        e.obstacleConfig.averageObstacleStrength = exp["obs_avg"];
        e.obstacleConfig.maxObstacleStrength = exp["obs_max"];
        e.obstacleConfig.dominantDirection = Vector2D(exp["obs_dir_x"], exp["obs_dir_y"]);
        
        // Navigation parameters
        JsonObject params = exp["params"];
        e.params.attractionConstant = params["attraction"];
        e.params.repulsionConstant = params["repulsion"];
        e.params.maxSpeed = params["max_speed"];
        e.params.damping = params["damping"];
        
        experiences.push_back(e);
    }
    
    Serial.printf("✅ Loaded %d experiences from flash\n", experiences.size());
    return true;
}

bool PersistentLearning::saveMap(const OccupancyGrid& map) {
    File file = LittleFS.open(MAP_FILE, "w");
    if (!file) {
        Serial.println("❌ Failed to open map file for writing");
        return false;
    }
    
    // Serialize map to buffer
    uint8_t buffer[8192]; // 8KB buffer
    size_t size = map.serialize(buffer, sizeof(buffer));
    
    if (size == 0) {
        Serial.println("❌ Failed to serialize map");
        file.close();
        return false;
    }
    
    // Write to file
    size_t written = file.write(buffer, size);
    file.close();
    
    if (written != size) {
        Serial.println("❌ Failed to write complete map");
        return false;
    }
    
    Serial.printf("✅ Saved map to flash (%d bytes)\n", size);
    return true;
}

bool PersistentLearning::loadMap(OccupancyGrid& map) {
    if (!LittleFS.exists(MAP_FILE)) {
        Serial.println("ℹ️  No saved map found");
        return false;
    }
    
    File file = LittleFS.open(MAP_FILE, "r");
    if (!file) {
        Serial.println("❌ Failed to open map file for reading");
        return false;
    }
    
    // Read entire file
    size_t size = file.size();
    uint8_t* buffer = new uint8_t[size];
    size_t bytesRead = file.read(buffer, size);
    file.close();
    
    if (bytesRead != size) {
        Serial.println("❌ Failed to read complete map");
        delete[] buffer;
        return false;
    }
    
    // Deserialize
    bool success = map.deserialize(buffer, size);
    delete[] buffer;
    
    if (success) {
        Serial.printf("✅ Loaded map from flash (%d bytes)\n", size);
    } else {
        Serial.println("❌ Failed to deserialize map");
    }
    
    return success;
}

bool PersistentLearning::saveParameters(const NavigationParameters& params) {
    File file = LittleFS.open(PARAMS_FILE, "w");
    if (!file) {
        Serial.println("❌ Failed to open params file for writing");
        return false;
    }
    
    JsonDocument doc;
    doc["attraction"] = params.attractionConstant;
    doc["max_attraction"] = params.maxAttraction;
    doc["goal_threshold"] = params.goalThreshold;
    doc["repulsion"] = params.repulsionConstant;
    doc["influence_radius"] = params.influenceRadius;
    doc["min_obstacle_dist"] = params.minObstacleDistance;
    doc["max_speed"] = params.maxSpeed;
    doc["damping"] = params.damping;
    doc["mass"] = params.mass;
    doc["oscillation_threshold"] = params.oscillationThreshold;
    doc["escape_strength"] = params.escapeStrength;
    
    if (serializeJson(doc, file) == 0) {
        Serial.println("❌ Failed to serialize parameters");
        file.close();
        return false;
    }
    
    file.close();
    Serial.println("✅ Saved navigation parameters to flash");
    return true;
}

bool PersistentLearning::loadParameters(NavigationParameters& params) {
    if (!LittleFS.exists(PARAMS_FILE)) {
        Serial.println("ℹ️  No saved parameters found");
        return false;
    }
    
    File file = LittleFS.open(PARAMS_FILE, "r");
    if (!file) {
        Serial.println("❌ Failed to open params file for reading");
        return false;
    }
    
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) {
        Serial.printf("❌ Failed to parse parameters: %s\n", error.c_str());
        return false;
    }
    
    params.attractionConstant = doc["attraction"];
    params.maxAttraction = doc["max_attraction"];
    params.goalThreshold = doc["goal_threshold"];
    params.repulsionConstant = doc["repulsion"];
    params.influenceRadius = doc["influence_radius"];
    params.minObstacleDistance = doc["min_obstacle_dist"];
    params.maxSpeed = doc["max_speed"];
    params.damping = doc["damping"];
    params.mass = doc["mass"];
    params.oscillationThreshold = doc["oscillation_threshold"];
    params.escapeStrength = doc["escape_strength"];
    
    Serial.println("✅ Loaded navigation parameters from flash");
    return true;
}

void PersistentLearning::clearAll() {
    if (LittleFS.exists(EXPERIENCE_FILE)) LittleFS.remove(EXPERIENCE_FILE);
    if (LittleFS.exists(MAP_FILE)) LittleFS.remove(MAP_FILE);
    if (LittleFS.exists(PARAMS_FILE)) LittleFS.remove(PARAMS_FILE);
    
    Serial.println("✅ Cleared all persistent data");
}

void PersistentLearning::printStats() {
    size_t totalBytes = LittleFS.totalBytes();
    size_t usedBytes = LittleFS.usedBytes();
    
    Serial.println("\n💾 LITTLEFS STATISTICS:");
    Serial.println("──────────────────────────────────────");
    Serial.printf("Total space:  %d bytes (%.2f KB)\n", totalBytes, totalBytes / 1024.0f);
    Serial.printf("Used space:   %d bytes (%.2f KB)\n", usedBytes, usedBytes / 1024.0f);
    Serial.printf("Free space:   %d bytes (%.2f KB)\n", 
                  totalBytes - usedBytes, (totalBytes - usedBytes) / 1024.0f);
    Serial.printf("Usage:        %.1f%%\n", (usedBytes * 100.0f) / totalBytes);
    Serial.println("──────────────────────────────────────");
    
    // List files
    Serial.println("\nStored files:");
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
        Serial.printf("  %s (%d bytes)\n", file.name(), file.size());
        file = root.openNextFile();
    }
}
```

### Step 3.2: Integrate Persistent Learning with Navigator

**File:** `include/LearningNavigator.h`

Add this include:

```cpp
#include "PersistentLearning.h"
```

Add these public methods:

```cpp
public:
    // ... existing methods ...
    
    /**
     * @brief Save current learning state to flash
     */
    bool saveLearningState(PersistentLearning& persistence) {
        bool success = true;
        success &= persistence.saveExperiences(experiences);
        success &= persistence.saveMap(localMap);
        success &= persistence.saveParameters(params);
        return success;
    }
    
    /**
     * @brief Load learning state from flash
     */
    bool loadLearningState(PersistentLearning& persistence) {
        bool success = true;
        success &= persistence.loadExperiences(experiences);
        success &= persistence.loadMap(localMap);
        success &= persistence.loadParameters(params);
        return success;
    }
    
    /**
     * @brief Get direct access to experiences for modification
     */
    std::vector<NavigationExperience>& getExperiences() {
        return experiences;
    }
```

### Step 3.3: Enhanced Battery Management

**File:** `include/power_manager.h`

Update the existing battery monitor with these additions:

```cpp
// Add to existing BatteryMonitor_t struct:
struct BatteryMonitor_t {
    // ... existing fields ...
    
    Vector2D baseStationPosition;    // NEW: Where to return for charging
    bool returningToBase;            // NEW: Currently executing return-to-base
    float lowBatteryThreshold;       // NEW: Trigger autonomous return (e.g., 20%)
    float criticalBatteryThreshold;  // NEW: Emergency shutdown (e.g., 10%)
};

// Add new functions:
void setBaseStationPosition(const Vector2D& pos);
Vector2D getBaseStationPosition();
bool shouldReturnToBase();
void triggerAutonomousReturn();
```

**File:** `src/power_manager.cpp`

Add these implementations:

```cpp
void setBaseStationPosition(const Vector2D& pos) {
    battery.baseStationPosition = pos;
    Serial.printf("🔋 Base station set to: (%.1f, %.1f)\n", pos.x, pos.y);
}

Vector2D getBaseStationPosition() {
    return battery.baseStationPosition;
}

bool shouldReturnToBase() {
    // Trigger return when battery drops below threshold
    // and we're not already returning
    return (battery.percentage < battery.lowBatteryThreshold && 
            !battery.returningToBase &&
            !battery.critical_power_mode);
}

void triggerAutonomousReturn() {
    battery.returningToBase = true;
    Serial.println("🔋⚠️ BATTERY LOW - Initiating autonomous return to base");
    
    // Set mission via global mission controller
    extern MissionController missionController;
    Mission m;
    m.type = MISSION_RETURN_TO_BASE;
    m.targetPosition = battery.baseStationPosition;
    m.timeoutMs = 0; // No timeout - critical mission
    missionController.setMission(m);
}
```

Update `monitorPower()`:

```cpp
void monitorPower() {
    unsigned long currentTime = millis();
    
    if (currentTime - battery.last_check < battery.check_interval) {
        return;
    }
    
    updateBatteryVoltage();
    estimateRemainingRuntime();
    
    // NEW: Check if autonomous return is needed
    if (shouldReturnToBase()) {
        triggerAutonomousReturn();
    }
    
    // ... existing power mode handling ...
    
    battery.last_check = currentTime;
}
```

### Step 3.4: Integration with Main

**File:** `include/main.h`

Add:

```cpp
#include "PersistentLearning.h"

extern PersistentLearning persistence;
```

**File:** `src/main.cpp`

```cpp
// Add global instance
PersistentLearning persistence;

// In setup():
void setup() {
    // ... existing setup ...
    
    // Initialize persistent learning system
    if (persistence.begin()) {
        Serial.println("📚 Loading saved learning data...");
        
        if (navigator.loadLearningState(persistence)) {
            Serial.println("✅ Learning state restored from flash");
            navigator.printLearningStats();
        } else {
            Serial.println("ℹ️  Starting with fresh learning state");
        }
    }
    
    // Set base station for autonomous return
    setBaseStationPosition(Vector2D(0, 0)); // Home = origin
}

// Add periodic saving
void loop() {
    static unsigned long lastSave = 0;
    const unsigned long SAVE_INTERVAL = 300000; // Save every 5 minutes
    
    // ... existing loop code ...
    
    // Periodic learning state save
    if (millis() - lastSave > SAVE_INTERVAL) {
        Serial.println("💾 Auto-saving learning state...");
        if (navigator.saveLearningState(persistence)) {
            Serial.println("✅ Learning state saved");
        } else {
            Serial.println("❌ Failed to save learning state");
        }
        lastSave = millis();
    }
}
```

### Step 3.5: CLI Commands for Persistence

**File:** `src/cli_manager.cpp`

```cpp
else if (command.equals("save")) {
    Serial.println("💾 Saving learning state...");
    if (navigator.saveLearningState(persistence)) {
        Serial.println("✅ Save complete");
    } else {
        Serial.println("❌ Save failed");
    }
}
else if (command.equals("load")) {
    Serial.println("📚 Loading learning state...");
    if (navigator.loadLearningState(persistence)) {
        Serial.println("✅ Load complete");
        navigator.printLearningStats();
    } else {
        Serial.println("❌ Load failed");
    }
}
else if (command.equals("cleardata")) {
    Serial.println("⚠️  Clearing all persistent data...");
    persistence.clearAll();
    navigator.getExperiences().clear();
    navigator.getMap().clearGrid();
}
else if (command.equals("fsinfo")) {
    persistence.printStats();
}
else if (command.startsWith("setbase ")) {
    // Parse "setbase X Y" command
    int spacePos = command.indexOf(' ', 8);
    if (spacePos > 0) {
        float x = command.substring(8, spacePos).toFloat();
        float y = command.substring(spacePos + 1).toFloat();
        setBaseStationPosition(Vector2D(x, y));
    } else {
        Serial.println("Usage: setbase X Y");
    }
}
else if (command.equals("returnbase")) {
    triggerAutonomousReturn();
}
```

Update help:

```cpp
if (command.equals("help")) {
    Serial.println("Available commands:");
    // ... existing commands ...
    Serial.println("  save         - Save learning state to flash");
    Serial.println("  load         - Load learning state from flash");
    Serial.println("  cleardata    - Clear all saved data");
    Serial.println("  fsinfo       - Show filesystem info");
    Serial.println("  setbase X Y  - Set base station position");
    Serial.println("  returnbase   - Trigger return to base");
}
```

---

## Phase 4: System Refinement & Configuration

**Goal:** Externalize configuration for easy tuning without recompilation.

**Estimated Time:** 6-8 hours

### Step 4.1: Configuration System

**File:** `include/ConfigManager.h`

```cpp
#pragma once
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "PotentialFieldNavigator.h"

// ═══════════════════════════════════════════════════════════════════════════
// CONFIGURATION MANAGER
// ═══════════════════════════════════════════════════════════════════════════

struct WiFiConfig {
    String ssid;
    String password;
    unsigned long timeout;
    
    WiFiConfig() : ssid(""), password(""), timeout(20000) {}
};

struct SwarmConfig {
    int channel;
    unsigned long broadcastInterval;
    unsigned long peerTimeout;
    
    SwarmConfig() : channel(1), broadcastInterval(100), peerTimeout(3000) {}
};

struct SensorConfig {
    unsigned long tofInterval;
    unsigned long mpuInterval;
    int obstacleThreshold;
    float tiltThreshold;
    
    SensorConfig() : tofInterval(50), mpuInterval(100), 
                     obstacleThreshold(200), tiltThreshold(45.0f) {}
};

struct MotorConfig {
    int testSpeed;
    int maxSpeed;
    int turnSpeed;
    int minPWM;
    
    MotorConfig() : testSpeed(200), maxSpeed(255), turnSpeed(150), minPWM(50) {}
};

class ConfigManager {
private:
    static const char* CONFIG_FILE;
    
    WiFiConfig wifiConfig;
    SwarmConfig swarmConfig;
    NavigationParameters navParams;
    SensorConfig sensorConfig;
    MotorConfig motorConfig;
    
    bool loadFromJSON(const char* filename);
    bool saveToJSON(const char* filename);
    
public:
    ConfigManager() {}
    
    /**
     * @brief Load configuration from file (or create defaults)
     */
    bool begin();
    
    /**
     * @brief Save current configuration to file
     */
    bool save();
    
    /**
     * @brief Reset to factory defaults
     */
    void resetToDefaults();
    
    // Getters
    const WiFiConfig& getWiFiConfig() const { return wifiConfig; }
    const SwarmConfig& getSwarmConfig() const { return swarmConfig; }
    const NavigationParameters& getNavParams() const { return navParams; }
    const SensorConfig& getSensorConfig() const { return sensorConfig; }
    const MotorConfig& getMotorConfig() const { return motorConfig; }
    
    // Setters
    void setWiFiConfig(const WiFiConfig& config) { wifiConfig = config; }
    void setSwarmConfig(const SwarmConfig& config) { swarmConfig = config; }
    void setNavParams(const NavigationParameters& params) { navParams = params; }
    void setSensorConfig(const SensorConfig& config) { sensorConfig = config; }
    void setMotorConfig(const MotorConfig& config) { motorConfig = config; }
    
    /**
     * @brief Print current configuration
     */
    void printConfig() const;
};
```

**File:** `src/ConfigManager.cpp`

```cpp
#include "ConfigManager.h"

const char* ConfigManager::CONFIG_FILE = "/config.json";

bool ConfigManager::begin() {
    if (!LittleFS.begin(true)) {
        Serial.println("❌ Failed to mount LittleFS");
        return false;
    }
    
    if (LittleFS.exists(CONFIG_FILE)) {
        Serial.println("📝 Loading configuration from file...");
        if (loadFromJSON(CONFIG_FILE)) {
            Serial.println("✅ Configuration loaded");
            return true;
        } else {
            Serial.println("⚠️  Failed to load config, using defaults");
        }
    } else {
        Serial.println("ℹ️  No config file found, creating defaults...");
    }
    
    // Create default config file
    resetToDefaults();
    save();
    return true;
}

void ConfigManager::resetToDefaults() {
    // WiFi defaults (will be overridden by credentials.h if present)
    wifiConfig.ssid = "YourSSID";
    wifiConfig.password = "YourPassword";
    wifiConfig.timeout = 20000;
    
    // Swarm defaults
    swarmConfig.channel = 1;
    swarmConfig.broadcastInterval = 100;
    swarmConfig.peerTimeout = 3000;
    
    // Navigation defaults
    navParams.attractionConstant = 2.5f;
    navParams.maxAttraction = 3.0f;
    navParams.goalThreshold = 8.0f;
    navParams.repulsionConstant = 15.0f;
    navParams.influenceRadius = 60.0f;
    navParams.minObstacleDistance = 10.0f;
    navParams.maxSpeed = 40.0f;
    navParams.damping = 0.8f;
    navParams.mass = 1.0f;
    navParams.oscillationThreshold = 5.0f;
    navParams.escapeStrength = 2.0f;
    
    // Sensor defaults
    sensorConfig.tofInterval = 50;
    sensorConfig.mpuInterval = 100;
    sensorConfig.obstacleThreshold = 200;
    sensorConfig.tiltThreshold = 45.0f;
    
    // Motor defaults
    motorConfig.testSpeed = 200;
    motorConfig.maxSpeed = 255;
    motorConfig.turnSpeed = 150;
    motorConfig.minPWM = 50;
    
    Serial.println("✅ Configuration reset to defaults");
}

bool ConfigManager::loadFromJSON(const char* filename) {
    File file = LittleFS.open(filename, "r");
    if (!file) {
        Serial.println("❌ Failed to open config file");
        return false;
    }
    
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) {
        Serial.printf("❌ Failed to parse config: %s\n", error.c_str());
        return false;
    }
    
    // WiFi
    JsonObject wifi = doc["wifi"];
    if (!wifi.isNull()) {
        wifiConfig.ssid = wifi["ssid"].as<String>();
        wifiConfig.password = wifi["password"].as<String>();
        wifiConfig.timeout = wifi["timeout"] | 20000;
    }
    
    // Swarm
    JsonObject swarm = doc["swarm"];
    if (!swarm.isNull()) {
        swarmConfig.channel = swarm["channel"] | 1;
        swarmConfig.broadcastInterval = swarm["broadcast_interval"] | 100;
        swarmConfig.peerTimeout = swarm["peer_timeout"] | 3000;
    }
    
    // Navigation
    JsonObject nav = doc["navigation"];
    if (!nav.isNull()) {
        navParams.attractionConstant = nav["attraction"] | 2.5f;
        navParams.maxAttraction = nav["max_attraction"] | 3.0f;
        navParams.goalThreshold = nav["goal_threshold"] | 8.0f;
        navParams.repulsionConstant = nav["repulsion"] | 15.0f;
        navParams.influenceRadius = nav["influence_radius"] | 60.0f;
        navParams.minObstacleDistance = nav["min_obstacle_dist"] | 10.0f;
        navParams.maxSpeed = nav["max_speed"] | 40.0f;
        navParams.damping = nav["damping"] | 0.8f;
        navParams.mass = nav["mass"] | 1.0f;
        navParams.oscillationThreshold = nav["oscillation_threshold"] | 5.0f;
        navParams.escapeStrength = nav["escape_strength"] | 2.0f;
    }
    
    // Sensors
    JsonObject sensors = doc["sensors"];
    if (!sensors.isNull()) {
        sensorConfig.tofInterval = sensors["tof_interval"] | 50;
        sensorConfig.mpuInterval = sensors["mpu_interval"] | 100;
        sensorConfig.obstacleThreshold = sensors["obstacle_threshold"] | 200;
        sensorConfig.tiltThreshold = sensors["tilt_threshold"] | 45.0f;
    }
    
    // Motors
    JsonObject motors = doc["motors"];
    if (!motors.isNull()) {
        motorConfig.testSpeed = motors["test_speed"] | 200;
        motorConfig.maxSpeed = motors["max_speed"] | 255;
        motorConfig.turnSpeed = motors["turn_speed"] | 150;
        motorConfig.minPWM = motors["min_pwm"] | 50;
    }
    
    return true;
}

bool ConfigManager::save() {
    File file = LittleFS.open(CONFIG_FILE, "w");
    if (!file) {
        Serial.println("❌ Failed to open config file for writing");
        return false;
    }
    
    JsonDocument doc;
    
    // WiFi
    JsonObject wifi = doc["wifi"].to<JsonObject>();
    wifi["ssid"] = wifiConfig.ssid;
    wifi["password"] = wifiConfig.password;
    wifi["timeout"] = wifiConfig.timeout;
    
    // Swarm
    JsonObject swarm = doc["swarm"].to<JsonObject>();
    swarm["channel"] = swarmConfig.channel;
    swarm["broadcast_interval"] = swarmConfig.broadcastInterval;
    swarm["peer_timeout"] = swarmConfig.peerTimeout;
    
    // Navigation
    JsonObject nav = doc["navigation"].to<JsonObject>();
    nav["attraction"] = navParams.attractionConstant;
    nav["max_attraction"] = navParams.maxAttraction;
    nav["goal_threshold"] = navParams.goalThreshold;
    nav["repulsion"] = navParams.repulsionConstant;
    nav["influence_radius"] = navParams.influenceRadius;
    nav["min_obstacle_dist"] = navParams.minObstacleDistance;
    nav["max_speed"] = navParams.maxSpeed;
    nav["damping"] = navParams.damping;
    nav["mass"] = navParams.mass;
    nav["oscillation_threshold"] = navParams.oscillationThreshold;
    nav["escape_strength"] = navParams.escapeStrength;
    
    // Sensors
    JsonObject sensors = doc["sensors"].to<JsonObject>();
    sensors["tof_interval"] = sensorConfig.tofInterval;
    sensors["mpu_interval"] = sensorConfig.mpuInterval;
    sensors["obstacle_threshold"] = sensorConfig.obstacleThreshold;
    sensors["tilt_threshold"] = sensorConfig.tiltThreshold;
    
    // Motors
    JsonObject motors = doc["motors"].to<JsonObject>();
    motors["test_speed"] = motorConfig.testSpeed;
    motors["max_speed"] = motorConfig.maxSpeed;
    motors["turn_speed"] = motorConfig.turnSpeed;
    motors["min_pwm"] = motorConfig.minPWM;
    
    if (serializeJsonPretty(doc, file) == 0) {
        Serial.println("❌ Failed to write config");
        file.close();
        return false;
    }
    
    file.close();
    Serial.println("✅ Configuration saved");
    return true;
}

void ConfigManager::printConfig() const {
    Serial.println("\n⚙️  CURRENT CONFIGURATION:");
    Serial.println("═══════════════════════════════════════");
    
    Serial.println("\n📡 WiFi:");
    Serial.printf("  SSID: %s\n", wifiConfig.ssid.c_str());
    Serial.printf("  Password: %s\n", wifiConfig.password.length() > 0 ? "***" : "(none)");
    Serial.printf("  Timeout: %lu ms\n", wifiConfig.timeout);
    
    Serial.println("\n🤖 Swarm:");
    Serial.printf("  Channel: %d\n", swarmConfig.channel);
    Serial.printf("  Broadcast interval: %lu ms\n", swarmConfig.broadcastInterval);
    Serial.printf("  Peer timeout: %lu ms\n", swarmConfig.peerTimeout);
    
    Serial.println("\n🧭 Navigation:");
    Serial.printf("  Attraction: %.2f\n", navParams.attractionConstant);
    Serial.printf("  Repulsion: %.2f\n", navParams.repulsionConstant);
    Serial.printf("  Max speed: %.2f cm/s\n", navParams.maxSpeed);
    Serial.printf("  Damping: %.2f\n", navParams.damping);
    
    Serial.println("\n📡 Sensors:");
    Serial.printf("  ToF interval: %lu ms\n", sensorConfig.tofInterval);
    Serial.printf("  MPU interval: %lu ms\n", sensorConfig.mpuInterval);
    Serial.printf("  Obstacle threshold: %d mm\n", sensorConfig.obstacleThreshold);
    Serial.printf("  Tilt threshold: %.1f°\n", sensorConfig.tiltThreshold);
    
    Serial.println("\n⚙️  Motors:");
    Serial.printf("  Test speed: %d\n", motorConfig.testSpeed);
    Serial.printf("  Max speed: %d\n", motorConfig.maxSpeed);
    Serial.printf("  Turn speed: %d\n", motorConfig.turnSpeed);
    Serial.printf("  Min PWM: %d\n", motorConfig.minPWM);
    
    Serial.println("═══════════════════════════════════════\n");
}
```

### Step 4.2: Integration with Main

**File:** `include/main.h`

```cpp
#include "ConfigManager.h"

extern ConfigManager configManager;
```

**File:** `src/main.cpp`

```cpp
// Add global instance
ConfigManager configManager;

// In setup():
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n🤖 RobotForge Brain Online. Engaging fluid motion.");
    
    // FIRST: Load configuration
    if (configManager.begin()) {
        configManager.printConfig();
    }
    
    // Apply configuration to systems
    navigator.setParameters(configManager.getNavParams());
    
    // ... rest of setup using config values ...
}
```

### Step 4.3: CLI Commands for Configuration

**File:** `src/cli_manager.cpp`

```cpp
else if (command.equals("config")) {
    configManager.printConfig();
}
else if (command.equals("saveconfig")) {
    if (configManager.save()) {
        Serial.println("✅ Configuration saved");
    } else {
        Serial.println("❌ Failed to save configuration");
    }
}
else if (command.equals("resetconfig")) {
    Serial.println("⚠️  Resetting to factory defaults...");
    configManager.resetToDefaults();
    configManager.save();
}
else if (command.startsWith("set ")) {
    // Example: set nav.max_speed 45.0
    // Parse parameter path and value
    String paramPath = command.substring(4);
    int spacePos = paramPath.indexOf(' ');
    
    if (spacePos > 0) {
        String param = paramPath.substring(0, spacePos);
        String valueStr = paramPath.substring(spacePos + 1);
        float value = valueStr.toFloat();
        
        // Simple parameter setting (expand as needed)
        if (param.equals("nav.max_speed")) {
            NavigationParameters params = configManager.getNavParams();
            params.maxSpeed = value;
            configManager.setNavParams(params);
            navigator.setParameters(params);
            Serial.printf("✅ Set %s to %.2f\n", param.c_str(), value);
        } else if (param.equals("nav.attraction")) {
            NavigationParameters params = configManager.getNavParams();
            params.attractionConstant = value;
            configManager.setNavParams(params);
            navigator.setParameters(params);
            Serial.printf("✅ Set %s to %.2f\n", param.c_str(), value);
        } else if (param.equals("nav.repulsion")) {
            NavigationParameters params = configManager.getNavParams();
            params.repulsionConstant = value;
            configManager.setNavParams(params);
            navigator.setParameters(params);
            Serial.printf("✅ Set %s to %.2f\n", param.c_str(), value);
        } else {
            Serial.println("❌ Unknown parameter");
        }
    } else {
        Serial.println("Usage: set parameter value");
    }
}
```

Update help:

```cpp
if (command.equals("help")) {
    Serial.println("Available commands:");
    // ... existing commands ...
    Serial.println("  config       - Show current configuration");
    Serial.println("  saveconfig   - Save configuration to file");
    Serial.println("  resetconfig  - Reset to factory defaults");
    Serial.println("  set PARAM VAL- Set configuration parameter");
}
```

### Step 4.4: Web UI for Configuration

**File:** `data/config.html` (new file)

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Configuration</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            color: #e0e0e0;
            margin: 0;
            padding: 20px;
        }
        
        .config-container {
            max-width: 800px;
            margin: 0 auto;
            background: #2a2a3e;
            border-radius: 12px;
            padding: 30px;
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3);
        }
        
        h1 {
            color: #00d4ff;
            text-align: center;
            margin-bottom: 30px;
        }
        
        .config-section {
            background: #3a3a4e;
            border-radius: 8px;
            padding: 20px;
            margin-bottom: 20px;
        }
        
        .config-section h2 {
            color: #00d4ff;
            margin-top: 0;
            margin-bottom: 15px;
        }
        
        .config-field {
            margin-bottom: 15px;
        }
        
        .config-field label {
            display: block;
            color: #b0b0b0;
            margin-bottom: 5px;
            font-size: 14px;
        }
        
        .config-field input {
            width: 100%;
            padding: 10px;
            background: #2a2a3e;
            border: 1px solid #5a5a7a;
            border-radius: 5px;
            color: white;
            font-size: 14px;
            box-sizing: border-box;
        }
        
        .button-group {
            display: flex;
            gap: 10px;
            margin-top: 20px;
        }
        
        button {
            flex: 1;
            padding: 12px;
            background: #00d4ff;
            border: none;
            border-radius: 5px;
            color: #1a1a2e;
            font-weight: bold;
            cursor: pointer;
            transition: background 0.3s;
        }
        
        button:hover {
            background: #00b8e6;
        }
        
        button.danger {
            background: #ff4444;
            color: white;
        }
        
        button.danger:hover {
            background: #cc0000;
        }
        
        .status-message {
            padding: 12px;
            border-radius: 5px;
            margin-top: 15px;
            display: none;
        }
        
        .status-message.success {
            background: #22aa22;
            color: white;
        }
        
        .status-message.error {
            background: #aa2222;
            color: white;
        }
    </style>
</head>
<body>
    <div class="config-container">
        <h1>⚙️ Robot Configuration</h1>
        
        <div class="config-section">
            <h2>🧭 Navigation Parameters</h2>
            <div class="config-field">
                <label>Attraction Constant:</label>
                <input type="number" id="nav-attraction" step="0.1" value="2.5">
            </div>
            <div class="config-field">
                <label>Repulsion Constant:</label>
                <input type="number" id="nav-repulsion" step="0.1" value="15.0">
            </div>
            <div class="config-field">
                <label>Max Speed (cm/s):</label>
                <input type="number" id="nav-max-speed" step="0.1" value="40.0">
            </div>
            <div class="config-field">
                <label>Damping:</label>
                <input type="number" id="nav-damping" step="0.05" value="0.8">
            </div>
        </div>
        
        <div class="config-section">
            <h2>⚙️ Motor Settings</h2>
            <div class="config-field">
                <label>Test Speed:</label>
                <input type="number" id="motor-test-speed" min="0" max="255" value="200">
            </div>
            <div class="config-field">
                <label>Turn Speed:</label>
                <input type="number" id="motor-turn-speed" min="0" max="255" value="150">
            </div>
            <div class="config-field">
                <label>Minimum PWM:</label>
                <input type="number" id="motor-min-pwm" min="0" max="255" value="50">
            </div>
        </div>
        
        <div class="config-section">
            <h2>🤖 Swarm Settings</h2>
            <div class="config-field">
                <label>ESP-NOW Channel:</label>
                <input type="number" id="swarm-channel" min="1" max="14" value="1">
            </div>
            <div class="config-field">
                <label>Broadcast Interval (ms):</label>
                <input type="number" id="swarm-broadcast" value="100">
            </div>
        </div>
        
        <div class="button-group">
            <button onclick="loadConfig()">🔄 Reload</button>
            <button onclick="saveConfig()">💾 Save</button>
            <button class="danger" onclick="resetConfig()">⚠️ Reset to Defaults</button>
        </div>
        
        <div id="status-message" class="status-message"></div>
    </div>
    
    <script>
        function showStatus(message, isError = false) {
            const statusDiv = document.getElementById('status-message');
            statusDiv.textContent = message;
            statusDiv.className = 'status-message ' + (isError ? 'error' : 'success');
            statusDiv.style.display = 'block';
            
            setTimeout(() => {
                statusDiv.style.display = 'none';
            }, 3000);
        }
        
        function loadConfig() {
            fetch('/api/config')
                .then(r => r.json())
                .then(config => {
                    document.getElementById('nav-attraction').value = config.navigation.attraction;
                    document.getElementById('nav-repulsion').value = config.navigation.repulsion;
                    document.getElementById('nav-max-speed').value = config.navigation.max_speed;
                    document.getElementById('nav-damping').value = config.navigation.damping;
                    
                    document.getElementById('motor-test-speed').value = config.motors.test_speed;
                    document.getElementById('motor-turn-speed').value = config.motors.turn_speed;
                    document.getElementById('motor-min-pwm').value = config.motors.min_pwm;
                    
                    document.getElementById('swarm-channel').value = config.swarm.channel;
                    document.getElementById('swarm-broadcast').value = config.swarm.broadcast_interval;
                    
                    showStatus('Configuration loaded');
                })
                .catch(err => showStatus('Failed to load configuration', true));
        }
        
        function saveConfig() {
            const config = {
                navigation: {
                    attraction: parseFloat(document.getElementById('nav-attraction').value),
                    repulsion: parseFloat(document.getElementById('nav-repulsion').value),
                    max_speed: parseFloat(document.getElementById('nav-max-speed').value),
                    damping: parseFloat(document.getElementById('nav-damping').value)
                },
                motors: {
                    test_speed: parseInt(document.getElementById('motor-test-speed').value),
                    turn_speed: parseInt(document.getElementById('motor-turn-speed').value),
                    min_pwm: parseInt(document.getElementById('motor-min-pwm').value)
                },
                swarm: {
                    channel: parseInt(document.getElementById('swarm-channel').value),
                    broadcast_interval: parseInt(document.getElementById('swarm-broadcast').value)
                }
            };
            
            fetch('/api/config', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(config)
            })
            .then(r => r.text())
            .then(msg => showStatus(msg))
            .catch(err => showStatus('Failed to save configuration', true));
        }
        
        function resetConfig() {
            if (confirm('Are you sure you want to reset to factory defaults?')) {
                fetch('/api/config/reset', { method: 'POST' })
                    .then(r => r.text())
                    .then(msg => {
                        showStatus(msg);
                        loadConfig();
                    })
                    .catch(err => showStatus('Failed to reset configuration', true));
            }
        }
        
        // Load on page load
        loadConfig();
    </script>
</body>
</html>
```

**File:** `src/web_server.cpp`

Add these API endpoints:

```cpp
// Configuration API
server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *request){
    JsonDocument doc;
    
    const NavigationParameters& nav = configManager.getNavParams();
    doc["navigation"]["attraction"] = nav.attractionConstant;
    doc["navigation"]["repulsion"] = nav.repulsionConstant;
    doc["navigation"]["max_speed"] = nav.maxSpeed;
    doc["navigation"]["damping"] = nav.damping;
    
    const MotorConfig& motors = configManager.getMotorConfig();
    doc["motors"]["test_speed"] = motors.testSpeed;
    doc["motors"]["turn_speed"] = motors.turnSpeed;
    doc["motors"]["min_pwm"] = motors.minPWM;
    
    const SwarmConfig& swarm = configManager.getSwarmConfig();
    doc["swarm"]["channel"] = swarm.channel;
    doc["swarm"]["broadcast_interval"] = swarm.broadcastInterval;
    
    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
});

server.on("/api/config", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
        JsonDocument doc;
        deserializeJson(doc, (const char*)data);
        
        // Update navigation parameters
        if (doc.containsKey("navigation")) {
            NavigationParameters params = configManager.getNavParams();
            params.attractionConstant = doc["navigation"]["attraction"];
            params.repulsionConstant = doc["navigation"]["repulsion"];
            params.maxSpeed = doc["navigation"]["max_speed"];
            params.damping = doc["navigation"]["damping"];
            configManager.setNavParams(params);
            navigator.setParameters(params);
        }
        
        // Update motor config
        if (doc.containsKey("motors")) {
            MotorConfig motors;
            motors.testSpeed = doc["motors"]["test_speed"];
            motors.turnSpeed = doc["motors"]["turn_speed"];
            motors.minPWM = doc["motors"]["min_pwm"];
            configManager.setMotorConfig(motors);
        }
        
        // Update swarm config
        if (doc.containsKey("swarm")) {
            SwarmConfig swarm;
            swarm.channel = doc["swarm"]["channel"];
            swarm.broadcastInterval = doc["swarm"]["broadcast_interval"];
            configManager.setSwarmConfig(swarm);
        }
        
        configManager.save();
        request->send(200, "text/plain", "Configuration saved");
    }
);

server.on("/api/config/reset", HTTP_POST, [](AsyncWebServerRequest *request){
    configManager.resetToDefaults();
    configManager.save();
    request->send(200, "text/plain", "Configuration reset to defaults");
});
```

---

## Testing & Validation

### Phase 1 Testing: Mission Control

**Test 1: CLI Mission Commands**

```
> mission
🎯 MISSION STATUS:
──────────────────────────────────────
Current Mission: IDLE
Role: NONE

> goto 1000 500
🎯 NEW MISSION [1]: GOTO_WAYPOINT
   Target: (1000.0, 500.0)
   Timeout: 60 seconds

> mission
🎯 MISSION STATUS:
──────────────────────────────────────
Current Mission: GOTO_WAYPOINT
Mission ID: 1
Runtime: 5 seconds
Target: (1000.0, 500.0)
Time remaining: 55 seconds
Role: NONE

✅ MISSION COMPLETE [1]: GOTO_WAYPOINT

> role scout
🎭 Role assigned: SCOUT

> explore
🎯 NEW MISSION [2]: EXPLORE
   Timeout: 120 seconds
```

**Expected Behavior:**

- [ ] Robot navigates to waypoint (1000, 500)
- [ ] Mission completes when within 100mm of goal
- [ ] Mission aborts on timeout
- [ ] Role assignment changes navigation parameters
- [ ] Exploration mission runs until timeout

**Test 2: Web Mission Control**

1. Open web dashboard
2. Click "🔍 Explore Area"
3. Verify robot enters EXPLORING state
4. Set waypoint (X: 2000, Y: 0)
5. Click "📍 Go to Waypoint"
6. Verify robot navigates to (2000, 0)
7. Click "🏠 Return to Base"
8. Verify robot returns to (0, 0)

**Test 3: Task Auction (Multi-Robot)**

With 2+ robots:

```
Robot 1 > broadcast_task explore 1500 500
📢 Broadcasting task 1: EXPLORE at (1500.0, 500.0)
💰 Submitted bid: Task 1, Cost 750.25

[Robot 2 receives broadcast]
📥 Received bid from Robot 1: Cost 750.25
💰 Submitted bid: Task 1, Cost 1200.80

🏆 Winning bid: Robot 1 with cost 750.25
🎯 I won the bid! Accepting task.
```

**Expected:**

- [ ] Both robots receive task broadcast
- [ ] Both robots calculate bids based on distance
- [ ] Robot closest to target wins
- [ ] Winner starts navigation to target

---

### Phase 2 Testing: Environmental Mapping

**Test 4: Occupancy Grid Building**

```
> clearmap
Map cleared

> explore
[Robot navigates for 30 seconds]

> map
🗺️  OCCUPANCY GRID:
   . = unknown, ░ = free, █ = occupied, @ = robot
   
   ░ ░ ░ ░ . . . . . .
   ░ ░ ░ ░ . . . . . .
   ░ ░ @ ░ ░ . . . . .
   ░ ░ ░ ░ ░ . . . . .
   ░ ░ ░ █ █ . . . . .
   ░ ░ ░ █ █ . . . . .
   . . . . . . . . . .

Exploration: 35.2%
Free: 245 | Occupied: 18 | Unknown: 6487
```

**Expected:**

- [ ] Grid updates in real-time during navigation
- [ ] FREE cells marked along sensor rays
- [ ] OCCUPIED cells marked at obstacles
- [ ] Exploration percentage increases
- [ ] Map persists across reboots (with save)

**Test 5: A* Pathfinding**

```
> findpath 0 0 1500 800
🔍 Finding path from (0.0, 0.0) to (1500.0, 800.0)...
✅ Path found! Length: 23 waypoints, Iterations: 156

Waypoint 1: (100.0, 0.0)
Waypoint 2: (200.0, 100.0)
Waypoint 3: (300.0, 100.0)
...
Waypoint 23: (1500.0, 800.0)

> followpath
🎯 Following computed path...
[Robot navigates waypoints]
✅ Path complete!
```

**Expected:**

- [ ] Path avoids known obstacles
- [ ] Path is reasonably optimal
- [ ] Robot follows waypoints in sequence
- [ ] Reactive avoidance still works during path following

---

### Phase 3 Testing: Persistent Learning

### Test 6: Learning Persistence

```txt
> explore
[Robot navigates, records 5 experiences]

> save
💾 Saving learning state...
✅ Saved 5 experiences to flash
✅ Saved map to flash (2456 bytes)
✅ Saved navigation parameters to flash
✅ Save complete

> reboot
[Robot reboots]

📚 Loading saved learning data...
✅ Loaded 5 experiences from flash
✅ Loaded map from flash (2456 bytes)
✅ Loaded navigation parameters from flash
✅ Learning state restored from flash

📚 LEARNING STATISTICS:
   Total experiences: 5
   Average success: 7.25
   Best success: 9.10
```

**Expected:**

- [ ] Experiences save to LittleFS
- [ ] Map saves to LittleFS
- [ ] Data loads on next boot
- [ ] Robot "remembers" environment
- [ ] Auto-save every 5 minutes works

**Test 7: Battery Management & Autonomous Return**

```
> setbase 0 0
🔋 Base station set to: (0.0, 0.0)

[Simulate low battery by editing battery.percentage]

> status
🔋 Battery: 18.5% (LOW)

🔋⚠️ BATTERY LOW - Initiating autonomous return to base
🎯 NEW MISSION [5]: RETURN_TO_BASE
   Target: (0.0, 0.0)

[Robot navigates home]

✅ MISSION COMPLETE [5]: RETURN_TO_BASE
```

**Expected:**

- [ ] Low battery triggers autonomous return
- [ ] Return uses pathfinding (if obstacles known)
- [ ] Return has no timeout (critical mission)
- [ ] Mission completes at base station
- [ ] Battery monitoring continues

---

### Phase 4 Testing: Configuration System

**Test 8: Configuration Loading**

```
> config
⚙️  CURRENT CONFIGURATION:
═══════════════════════════════════════

📡 WiFi:
  SSID: MyNetwork
  Password: ***
  Timeout: 20000 ms

🤖 Swarm:
  Channel: 1
  Broadcast interval: 100 ms
  Peer timeout: 3000 ms

🧭 Navigation:
  Attraction: 2.50
  Repulsion: 15.00
  Max speed: 40.00 cm/s
  Damping: 0.80
```

**Test 9: Runtime Configuration Changes**

```
> set nav.max_speed 50.0
✅ Set nav.max_speed to 50.00

> saveconfig
✅ Configuration saved

> reboot
[Robot reboots]

📝 Loading configuration from file...
✅ Configuration loaded

🧭 Navigation:
  Max speed: 50.00 cm/s
```

**Expected:**

- [ ] Configuration loads from `/config.json`
- [ ] Runtime changes apply immediately
- [ ] Changes persist across reboots
- [ ] Web UI reflects current values
- [ ] Factory reset works

**Test 10: Web Configuration UI**

1. Open `http://robot.local/config.html`
2. Change "Max Speed" to 35.0
3. Click "💾 Save"
4. Refresh page
5. Verify value persists
6. Click "⚠️ Reset to Defaults"
7. Verify all values return to defaults

---

### Integration Testing

**Test 11: Full Autonomous Mission**

```
1. Power on robot
2. Robot loads persistent state
3. Set mission: goto 2000 1000
4. Robot:
   - Loads map from flash
   - Finds path using A*
   - Follows path while avoiding real-time obstacles
   - Updates map during navigation
   - Records experience on completion
5. Battery drops below 20%
6. Robot:
   - Saves current state
   - Aborts current mission
   - Returns to base autonomously
7. Arrives at base
8. Mission complete
```

**Expected:**

- [ ] All systems work together seamlessly
- [ ] No crashes or memory leaks
- [ ] Smooth state transitions
- [ ] Proper error handling

**Test 12: Multi-Robot Swarm Mission**

With 3+ robots:

```
1. All robots power on
2. Assign roles:
   - Robot 1: Leader
   - Robot 2: Scout
   - Robot 3: Worker
3. Leader broadcasts task: explore quadrant
4. Scout wins auction (fastest)
5. Scout explores, shares map with swarm
6. Leader and Worker receive updated maps
7. All robots navigate using shared knowledge
8. Collision avoidance active between robots
```

**Expected:**

- [ ] Role-based task allocation works
- [ ] Map sharing via ESP-NOW succeeds
- [ ] Robots avoid collisions
- [ ] Coordinated behavior emerges

---

## Troubleshooting

### Common Issues

**Issue 1: LittleFS Mount Failure**

```
❌ Failed to mount LittleFS
```

**Solution:**

- First boot will format filesystem (takes ~30 seconds)
- If persistent, try `LittleFS.format()` manually
- Check flash size in `platformio.ini`

**Issue 2: JSON Deserialization Errors**

```
❌ Failed to parse config: DeserializationError::IncompleteInput
```

**Solution:**

- Delete corrupted file: `LittleFS.remove("/config.json")`
- Reboot to regenerate defaults
- Check ArduinoJson version (need 7.0+)

**Issue 3: Path Not Found**

```
❌ No path found after 1000 iterations
```

**Solution:**

- Verify goal is not occupied: `map.isOccupied(goal)`
- Clear map if stale: `clearmap`
- Increase max iterations: `pathfinder.setMaxIterations(2000)`
- Check if goal is reachable

**Issue 4: Experience Not Loading**

```
ℹ️  No saved experiences found
```

**Solution:**

- Normal on first boot
- Verify file exists: `fsinfo`
- Check file size > 0 bytes
- Try manual save: `save`

**Issue 5: Mission Timeout Too Soon**

```
⏰ Mission timeout!
```

**Solution:**

- Increase timeout in mission: `m.timeoutMs = 120000;`
- Remove timeout for critical missions: `m.timeoutMs = 0;`
- Check if robot is stuck (obstacle blocking path)

---

## Performance Optimization

### Memory Management

**Monitor heap usage:**

```cpp
// In main loop:
if (millis() - lastMemCheck > 10000) {
    Serial.printf("💾 Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("   Largest block: %d bytes\n", ESP.getMaxAllocHeap());
    lastMemCheck = millis();
}
```

**Expected values:**

- Free heap: > 100KB
- Largest block: > 50KB

**If low:**

- Reduce MAX_EXPERIENCES_TO_SAVE
- Reduce grid size (GRID_WIDTH/HEIGHT)
- Limit swarm size (MAX_SWARM_SIZE)

### CPU Optimization

**Profile execution time:**

```cpp
unsigned long t1 = micros();
navigator.updateWithLearning(deltaTime, totalForce);
unsigned long t2 = micros();
Serial.printf("Navigator: %lu µs\n", t2 - t1);
```

**Target times:**

- Navigator update: < 5ms
- Map update: < 2ms
- Pathfinding: < 100ms (one-time)

**If slow:**

- Reduce A* max iterations
- Optimize map update frequency
- Use FreeRTOS for parallelization

---

## Next Steps

**After completing all phases:**

1. **Tune navigation parameters** using web UI
2. **Build Bot #2** and test swarm coordination
3. **Add visual map display** in web dashboard
4. **Implement formation flying** behaviors
5. **Add task prioritization** system
6. **Create mission library** (patrol patterns, search patterns)
7. **Add camera integration** for vision-based mapping
8. **Implement SLAM** (Simultaneous Localization and Mapping)

---

## 🎉 Completion Checklist

### Phase 1: Mission Control ✅

- [x] MissionController compiles
- [x] CLI commands work
- [x] Web UI loads
- [x] Waypoint navigation works
- [x] Role assignment affects behavior
- [ ] Task auction (if multi-robot) - *Deferred until multi-robot setup*

### Phase 2: Environmental Mapping ✅

- [ ] OccupancyGrid compiles
- [ ] Map updates during navigation
- [ ] Map visualization works
- [ ] A* pathfinding finds paths
- [ ] Path following works
- [ ] Map persistence works

### Phase 3: Enhanced Intelligence ✅

- [ ] PersistentLearning compiles
- [ ] Experiences save/load
- [ ] Map saves/load
- [ ] Parameters save/load
- [ ] Auto-save works
- [ ] Battery triggers return-to-base

### Phase 4: System Refinement ✅

- [ ] ConfigManager compiles
- [ ] Config loads from file
- [ ] Runtime changes work
- [ ] Config web UI works
- [ ] Parameters apply correctly

### Integration ✅

- [ ] All systems work together
- [ ] No memory leaks
- [ ] No crashes
- [ ] Multi-robot coordination
- [ ] Complete autonomous missions

---

**You now have a complete, production-ready autonomous robotics platform!** 🚀🤖
