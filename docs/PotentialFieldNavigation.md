# Fluid Movement via Potential Field Navigation: Swarm Robot Navigation Theory

## Design Document for ESP32-Based Swarm Robot Navigation Systems

-----

> **Note:** Wheelie is the foundational and test bot for this series. The theory, architecture, and code described here were first validated on Wheelie, but are intended for all ESP32-based swarm robots in the collection.

-----

## 1\. Motivation: Beyond Start-Stop Robotics

Traditional obstacle avoidance is clunky and inefficient:

```cpp
// The classic approach
if (sensor.sees_wall_at < 10cm) {
    stop_motors();
    turn_motors(90);
    go_forward();
}
```

This creates robots that **react** rather than **flow**. They stop, think, turn, and move in jerky, interrupt-driven cycles. The goal is to move beyond this, achieving continuous, natural navigation for all robots in the swarm.

-----

## 2\. Theoretical Foundation: Potential Field Navigation

This series of ESP32-based robots will use Potential Field Navigation—a robotics concept where the environment is modeled as a continuous pressure gradient. Each robot "feels" the topology of forces and flows naturally along the gradient, never stopping to "think" in discrete steps.

### Core Principles

- **Goal (Attractive Force):** Pulls the robot toward its target (low pressure)
- **Obstacles (Repulsive Forces):** Push the robot away from collisions (high pressure)
- **Result:** The robot continuously moves "downhill" on the pressure gradient, creating smooth, optimal paths that emerge from the field dynamics

-----

## 3\. The Mavric Pattern: Emergent Navigation for Swarm Robots

### 3.1. Simple Known Variables (Adaptive Specialists)

- **Attractive Force:** Goal position creates a pull toward the target
- **Repulsive Forces:** Each obstacle creates a push away from collision
- **Magnitude Falloff:** Forces decrease with distance (inverse square or exponential)

### 3.2. Coordination Substrate

- **Pressure Gradient Field:** The combined force field at every point in space
- **Continuous Calculation:** Real-time summation of all forces based on sensor input
- **Vector Summation:** Addition of force vectors to determine resultant motion

### 3.3. Emergent Capability

- **Fluid Paths:** Smooth, efficient navigation that flows around obstacles
- **Dynamic Adaptation:** Real-time response to changing environments
- **Swarm Behavior:** Multiple robots naturally space themselves while flowing toward shared goals

-----

## 4\. Why This Works for Swarm Robots

### For Individual Robots

```cpp
// Continuous flow - no if-statements needed
Vector2D goalForce = (goal - position).normalize() * ATTRACTION_STRENGTH;
Vector2D obstacleForce = calculateRepulsion(sensorReadings);

Vector2D resultant = goalForce + obstacleForce;
motors.setVelocity(resultant); // Always moving, always adjusting
```

Each robot is **always in motion**, modulating its velocity vector. No interrupts, no decision trees, just pure fluid dynamics.

### For Swarms

- Each robot broadcasts its position
- Others treat these as repulsive forces
- The swarm naturally spaces itself while flowing toward shared goals

-----

## 5\. System Architecture

### 5.1. Hardware Requirements

- **ESP32 Microcontroller**: Field calculations and communication
- **Distance Sensors**: ToF, ultrasonic, or IR for obstacle detection
- **Motor Driver**: MOSFET H-Bridge (TB6612FNG or similar)
- **IMU (Optional)**: For heading and smooth motion

### 5.2. Core Algorithms

#### Attractive Force (Goal Pull)

```cpp
Vector2D calculateAttraction(Vector2D currentPos, Vector2D goalPos) {
    Vector2D direction = goalPos - currentPos;
    float distance = direction.magnitude();
    if (distance < GOAL_THRESHOLD) return Vector2D(0, 0);
    return direction.normalize() * ATTRACTION_CONSTANT;
}
```

#### Repulsive Force (Obstacle Push)

```cpp
Vector2D calculateRepulsion(float sensorDistance, float sensorAngle) {
    if (sensorDistance > INFLUENCE_RADIUS) return Vector2D(0, 0);
    float magnitude = REPULSION_CONSTANT / (sensorDistance * sensorDistance);
    Vector2D direction = Vector2D(cos(sensorAngle), sin(sensorAngle));
    return direction * -magnitude;
}
```

#### Field Summation

```cpp
void updateMotion() {
    Vector2D totalForce(0, 0);
    totalForce += calculateAttraction(currentPos, goalPos);
    for (auto sensor : sensors) {
        totalForce += calculateRepulsion(sensor.distance, sensor.angle);
    }
    // Add swarm repulsion from other robots
    setMotorsFromVector(totalForce);
}
```

-----

## 6\. Key Parameters

- **ATTRACTION\_CONSTANT**: Strength of goal pull (start: 1.0)
- **REPULSION\_CONSTANT**: Strength of obstacle push (start: 10.0)
- **INFLUENCE\_RADIUS**: Distance beyond which obstacles don't matter (start: 50cm)
- **GOAL\_THRESHOLD**: Distance to consider goal reached

### Falloff Functions

- **Linear**: `force = k * (max_dist - distance)`
- **Inverse**: `force = k / distance`
- **Inverse Square**: `force = k / (distance^2)`
- **Exponential**: `force = k * exp(-distance/scale)`

-----

## 7\. Implementation Roadmap

### Phase 1: Single Bot Field Navigation

1. Implement vector math library
2. Add goal attraction calculation
3. Add single-sensor obstacle repulsion
4. Test smooth approach and avoidance
5. Tune force constants for fluid motion

### Phase 2: Multi-Sensor Integration

1. Add multiple distance sensors
2. Calculate combined repulsive field
3. Implement sensor fusion and filtering

### Phase 3: Swarm Coordination

1. Set up ESP-NOW mesh network
2. Implement position broadcasting
3. Add inter-bot repulsive forces

### Phase 4: Advanced Behaviors

1. Dynamic goal reassignment
2. Formation control
3. Obstacle memory
4. Path prediction from velocity fields

-----

## 8\. Advantages

| Aspect | Traditional | Potential Field Swarm |
|--------|-------------|----------------------|
| **Motion** | Jerky, interrupt-driven | Smooth, continuous flow |
| **Decision Making** | Discrete logic | Emergent from field dynamics |
| **Obstacle Response** | Binary (stop/go) | Graceful deflection |
| **Swarm Coordination** | Complex protocols | Natural spacing |
| **Computational Load** | Event-driven spikes | Constant, predictable |
| **Adaptability** | State machine updates | Responds to new forces |

-----

## 9\. Common Challenges & Solutions

- **Local Minima**: Add random force or momentum to escape
- **Oscillation Near Goals**: Reduce attraction near goal or add damping
- **Sensor Noise**: Use moving average or Kalman filter
- **Narrow Passages**: Temporarily increase attraction or add path memory

-----

## 10\. Philosophy: Obstacles Define the Path

In this swarm, obstacles are not just problems—they define the solution space. Repulsive forces guide the robots, so they navigate *because of* obstacles, not despite them. Intelligence emerges from simple rules and substrates, not from complex decision trees.

-----

## 11\. Applications

- Warehouse and logistics robots
- Swarm exploration
- Educational demos
- Art installations

-----

## 12\. Getting Started

1. Clone the repository
2. Ensure hardware matches requirements
3. Implement and tune the navigation code (see this doc)
4. Test and iterate

-----

> "These robots don't avoid obstacles. They flow through the topology obstacles create."

-----

*This document is the theory and design foundation for all ESP32-based swarm robot navigation code in this series. All implementation should reference and align with these principles.*
