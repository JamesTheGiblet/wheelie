# Wheelie Robot - Shopping List

## Essential Components (Required)

### Chassis & Mechanical

- [ ] **DollaTek 2WD Smart Robot Car Chassis Kit** - ~$15-20
  - Includes: Chassis, 2x TT motors, wheels, encoders, battery box, casters
  - Alternative: Any 2WD robot chassis with TT motors

### Electronics - Core

- [ ] **ESP32 Development Board** - ~$8-15
  - Type-C USB, CH340C chip, 30-pin, 2.4GHz WiFi/Bluetooth
  - Alternative: ESP32-DevKitC-32E or ESP32-WROOM-32
- [ ] **ESP32 Breakout/Expansion Board** - ~$3-5
  - 30-pin breakout board for easier connections
  - Provides screw terminals and labeled pins
- [ ] **XL4015 Buck Converter Module** - ~$3-6
  - DC-DC step-down regulator (4-38V to 1.25-36V, 5A max)
  - Adjustable output voltage with constant current capability
- [ ] **MOSFET H-Bridge Motor Driver (TB6612FNG or similar)** - ~$3-6
  - Dual H-bridge PWM controller, high efficiency, 2V-13.5V supply
  - Compact, low heat, recommended for all new builds
- [ ] **Breadboard (400-point)** - ~$3-5
- [ ] **Jumper Wire Kit** (M-M, M-F, F-F) - ~$5-8
- [ ] **2x Li-Po Batteries 3.7V 2000mAh (903042)** - ~$15-25
  - Series connection for 7.4V 2000mAh battery pack
  - Rechargeable lithium polymer with high capacity
  - Perfect voltage for robot motors and extended runtime

### Sensors

- [ ] **VL53L0X GY-VL53L0XV2 Time-of-Flight Sensor** - ~$8-12
  - 940nm laser ranging sensor for precise distance measurement
  - I2C interface with 20-2000mm detection range
  - High accuracy obstacle detection and navigation support

- [ ] **Ultrasonic Sensor (HC-SR04)** - ~$2-3
  - Standard for rear obstacle detection
  - 5V operation, 2cm-400cm range
  - Trig: GPIO 16, Echo: GPIO 32
  - Use with VL53L0X for best coverage (both are now standard)
- [ ] **MPU6050 GY-521 6-Axis IMU** - ~$3-5
  - 3-axis accelerometer + 3-axis gyroscope module
  - 16-bit ADC with I2C interface for precise motion sensing
  - 6 DOF (Degrees of Freedom) for comprehensive tilt and motion detection
- [ ] **IR Edge/Cliff Sensor** - ~$2-4
- [ ] **H-1-0332 Sound Sensor Module** - ~$2-3
  - High-sensitivity microphone with adjustable trigger level
  - Digital output for sound detection and audio-reactive behaviors
- [ ] **2x LM393 H2010 Photoelectric Sensors** - ~$3-6
  - Opposite-type infrared photoelectric sensors for motor speed detection
  - Digital output for wheel encoder and rotation counting applications

### Indicators

- [ ] **KY-009 RGB LED Module** - ~$1-2
  - 3-color SMD LED board with built-in current limiting
  - DC 5V operation with red, green, blue color control
  - Compact module design for easy integration
- [ ] **KY-006 Piezoelectric Buzzer** - ~$1-3
  - 3-5V operating voltage, piezoelectric sound emitter
  - Compatible with Arduino/ESP32 digital output pins
- [ ] **2S Li-Po Charger Module (USB-C)** - ~$8-12
  - Multi-cell boost charger for 2S (7.4V) Li-Po batteries
  - USB-C input with 1A charging current for 2S configuration
  - Step-up boost charging to 8.4V (4.2V per cell)
- [ ] **Resistors (220Ω, pack)** - ~$2-3

### Hardware

- [ ] **M3 Screws & Nuts Kit** - ~$3-5
- [ ] **Double-sided Mounting Tape** - ~$2-3

### Total Estimated Cost: $65-110

---

## Optional Upgrades

### Enhanced Sensors

- [ ] **Ultrasonic Sensor (HC-SR04)** - ~$2-3
- [ ] **PIR Motion Sensor** - ~$2-3
- [ ] **Camera Module (ESP32-CAM)** - ~$8-12

### Power & Performance

- [ ] **18650 Battery Holder + Batteries** - ~$10-15
- [ ] **Lithium Battery Capacity Indicator Module** - ~$3-5
  - 3.7V Blue LED Display Board with 26AWG Wire
  - Shows real-time battery voltage/capacity
- [ ] **Buck Converter (LM2596)** - ~$2-3
- [ ] **Power Switch** - ~$1-2

### Mechanical Upgrades

- [ ] **Better Wheels (rubber tread)** - ~$5-8
- [ ] **Ball Casters (upgrade)** - ~$3-5
- [ ] **Mounting Brackets Set** - ~$5-8

### Tools (if needed)

- [ ] **Soldering Iron Kit** - ~$15-25
- [ ] **Wire Strippers** - ~$5-10
- [ ] **Multimeter** - ~$10-20
- [ ] **Small Screwdriver Set** - ~$5-10

---

## Where to Buy

### Online Retailers

**Amazon/eBay** - Complete kits and individual components
**AliExpress** - Cheaper options, longer shipping
**Adafruit** - High-quality components, good documentation
**SparkFun** - Educational focus, reliable parts
**Banggood** - Good for robot chassis and mechanical parts

### Local Options

**Electronics stores** - Immediate availability
**Maker spaces** - Often have components and tools
**University bookstores** - Educational discounts possible

---

## Money-Saving Tips

### Budget Build (~$50-65)

- Buy components from AliExpress (allow 2-4 weeks shipping)
- Use generic ESP32 boards instead of name-brand
- Skip optional sensors initially
- Use AA batteries instead of Li-Po

### Premium Build (~$90-110)

- Buy from local/fast shipping sources
- Get name-brand components (Adafruit, SparkFun)
- Include all sensors and upgrades
- Add professional mounting hardware

### DIY Alternatives

- **Chassis**: Build from acrylic sheets or 3D print
- **Sensors**: Use ultrasonic instead of VL53L0X for budget option
- **LEDs**: Individual LEDs instead of RGB module
- **Mounting**: Hot glue instead of proper brackets (temporary)

---

## Bulk Purchase Considerations

### If Building Multiple Robots

- ESP32 boards often cheaper in 3-5 packs
- Sensor modules available in bulk
- Consider educator discounts from major suppliers
- Share shipping costs with maker groups

### Recommended Quantities

- **Resistors**: Buy assortment pack (covers many projects)
- **Jumper wires**: Extra length/quantity useful for prototyping
- **Screws**: M3 variety pack covers most robot projects
- **Batteries**: Buy extras for continuous operation

---

## Assembly Timeline

### Phase 1: Order Components (Week 1)

- Place orders for all required components
- Download and set up software environment
- Review assembly documentation

### Phase 2: Mechanical Assembly (Week 2-3)

- Assemble chassis when parts arrive
- Test fit all components
- Plan wire routing

### Phase 3: Electronics Integration (Week 3-4)

- Wire and test each subsystem
- Upload and test software
- Calibrate sensors

### Phase 4: Testing & Tuning (Week 4+)

- Full system integration testing
- Behavior tuning and customization
- Add optional features

---

## Kit vs Individual Components

### Pre-made Robot Kits

**Pros:**

- All components guaranteed compatible
- Usually includes assembly instructions
- Often cheaper than individual parts
- Good for beginners

**Cons:**

- Less learning about component selection
- May include unnecessary components
- Harder to customize or upgrade
- Limited to kit specifications

### Individual Component Purchase

**Pros:**

- Learn about each component's function
- Can upgrade specific parts easily
- Choose quality level for each component
- Better understanding of system design

**Cons:**

- Risk of compatibility issues
- More research required
- Potentially higher total cost
- Need to source assembly instructions

---

## Compatibility Notes

### ESP32 Variants

- **ESP32 Type-C with CH340C** (30-pin): Used in this project, modern connector
- **ESP32-DevKitC**: Most common, micro-USB, well-supported
- **ESP32-WROOM-32**: Compact, built-in WiFi/BT
- **ESP32-S3**: Newer, more features (ensure code compatibility)

**Note**: The Type-C variant with CH340C chip offers the same functionality as other ESP32 boards but with a more durable USB connector and reliable serial communication.

### Motor Driver Alternatives

- **MOSFET H-Bridge (TB6612FNG or similar)** (Recommended): High efficiency, compact, low heat, supports higher current, best for ESP32 builds
- **Fasizi L298N** (Legacy/Alternative): Ultra-compact (5g), 2V-10V supply, built-in thermal protection
- **Standard L298N**: Most common alternative, slightly larger form factor  
- **DRV8833**: Lower voltage, good for smaller motors

### Sensor Substitutions

- **VL53L0X** ↔ **HC-SR04**: ToF vs Ultrasonic
- **MPU6050** ↔ **MPU9250**: 6-axis vs 9-axis IMU
- **Generic edge sensor** ↔ **Sharp IR sensor**: Different mounting

---

## Total Budget Range: $50-150 depending on options chosen

**Note:** Prices are estimates and may vary by region and supplier
