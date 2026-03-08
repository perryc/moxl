# MOXL Hardware TODO

Everything that needs wiring, verifying, or selecting before field deployment.

## MOXL HAT (custom Pi 5 I/O board)
See full spec: `docs/moxl-hat-spec.md`
- [ ] Schematic design (KiCad)
- [ ] PCB layout (2-layer, Pi HAT form factor)
- [ ] Order PCBs + parts
- [ ] Assemble and bench test
- Key ICs: MCP251863T (CAN), MAX31855 (CHT), PCA9685 (servos), ADS1115 (ADC)
- Opto-isolated: motor PWM pass-through (TLP2361 x8), relay drivers, RPM tach input
- 24V→5V buck converter powers Pi from battery bus

## Drive System (BTS7960)
- [ ] Wire left motor: RPWM→GPIO12, LPWM→GPIO16, R_EN→GPIO20, L_EN→GPIO21
- [ ] Wire right motor: RPWM→GPIO13, LPWM→GPIO26, R_EN→GPIO23, L_EN→GPIO24
- [ ] Verify 24V supply to both BTS7960 boards
- [ ] Verify PWM frequency (20 kHz) and duty cycle range works with motors
- [ ] Compile BTS7960 hw_interface for Pi (currently using mock_components on desktop)
- [ ] Investigate wheel encoders — motors confirmed 2 power + 2 brake, no encoder connector
  - Options: magnetic encoders on wheel hubs (AS5048A), optical slot sensors
  - 2 GPIO inputs for pulse counting (future, or via CAN motor nodes)
  - Chain drive: 19T motor → 22T (idler?) → 32T wheel, ratio 1.684:1
- [x] Motor brakes — removed (2 brake wires per motor disconnected)

## Engine Control (via MOXL HAT)
- [ ] Run relay (magneto kill + fuel solenoid) → HAT relay driver, GPIO 17
  - Fuel solenoid: 2-wire anti-afterfire solenoid under carb float bowl (energized = fuel flows)
  - Wired through run relay so magneto kill + fuel cutoff happen together
- [ ] Start relay (starter solenoid) → HAT relay driver, GPIO 27 — momentary pulse
- [ ] Choke/throttle servo → HAT PCA9685 CH0 — combined single lever
  - Positions: idle → half → full → choke (past full throttle detent)
  - Mount servo to combined choke/throttle cable/lever on engine
- [ ] RPM sensor → HAT opto-isolated input, GPIO 4 — hall sensor on flywheel or spark plug lead pickup
- [ ] CHT thermocouple → HAT MAX31855, SPI CS1 — K-type ring terminal under head bolt
- [ ] Implement engine_controller_node.py: cold/warm start sequence, CHT-driven choke logic

## Blade Engagement (via MOXL HAT)
- [ ] Blade engage relay → HAT relay driver, GPIO 25
  - Actuates belt tensioner lever (rod/black knob) that tensions engine→center spindle belt
  - Two-belt system: Belt #1 engine→center spindle (engagement tensioner), Belt #2 center→3 blades (permanent)
  - ~10 ft-lbs force, ~8" lever travel, spring return (disengage = safe default)
- [ ] Belt engagement actuator — 12V linear actuator, ~200mm stroke, ~50 lbs force
  - Pulls the engagement lever; spring return handles disengage
  - Driven via blade relay (GPIO 25)
- [ ] GPS work_state feedback via CAN (GPXDR MACHINE_WORK) confirms engagement

## Power System
- [ ] Install 2x 12V batteries in series (24V drive, 12V center tap for starter)
- [ ] MOXL HAT 12V→5V buck powers Pi (no separate Pi power supply needed)
- [ ] Pi 5 RTC battery — CR2032 coin cell on Pi 5 J5 header (JST-SH 2-pin)
  - Enables scheduled wake from shutdown (built-in RTC on RP1 chip)
  - Mission node sets RTC alarm before shutdown, Pi wakes at next mow time
  - Pi draws <1mA in halt — batteries last weeks between mows
- [ ] Direct-drive alternator coupler on blade pulley shaft
  - [ ] Measure blade pulley shaft diameter, keyway, protrusion above deck
  - [ ] Design/machine coupler to mount Denso alternator
  - [ ] Convert Denso alternator to 24V output
  - [ ] Mount alternator bracket above deck
- [ ] Fuel level sender — resistive float in plastic tank → HAT ADS1115 A2
  - Tank: B&S plastic, L-shaped, ~12" wide x ~7" tall, ~3.5 qt (3.3L), translucent
  - Filler cap on lower extension — sender through replacement cap or drilled/tapped fitting
  - Resistive float sender (10-180Ω or 0-90Ω), reference resistor on HAT PCB
- [ ] Battery voltage monitoring (2x lead-acid in series)
  - ADS1115 A0: bottom battery (GND to center tap) — 100K/27K divider
  - ADS1115 A3: full pack (GND to +24V) — 100K/10K divider
  - Top battery = pack - bottom; alert if imbalance > 0.5V
- [ ] Alternator current monitoring → HAT ADS1115 A1 (hall sensor or shunt)

## GPS (FarmTRX RTK)
- [ ] **Phase 1**: Serial `/dev/ttyUSB0` — current setup, NMEA0183
- [ ] **Phase 2**: Migrate to CAN bus via MOXL HAT MCP251863T
  - Position, heading, rate of turn, work state all over CAN
  - Eliminates serial NMEA parsing, uses structured PGNs
  - Perry defines PGNs on FarmTRX side (J1939 or NMEA 2000)
- [ ] Measure actual GPS antenna offset from base_link (drive axle center)
  - Currently estimated: `[0.0, 0.0, 0.50]` m in `config/hardware/moxl.yaml`
- [ ] Confirm magnetic declination 7.2° East for CDS2 site

## Safety
- [ ] LiDAR — **SLAMTEC RPLIDAR C1** (ordered, AliExpress)
  - 360° dTOF, 12m range (white), 6m (black), 5V/230mA via USB-A
  - Pi 5 USB-A → rplidar_ros2 driver → `/scan` → safety_monitor_node.py
  - Mount location TBD — needs clear forward arc
- [ ] Physical e-stop button — GPIO input (pin TBD), hardware interrupt to kill motors + blades
- [ ] RC manual override — **FrSky Taranis + X8R receiver**
  - X8R SBUS out → HAT J12 (PH 3p) → 74HC14 inverter → Pi UART0 RX (GPIO15)
  - UART0 shared with RS-232 (J11) via 0R DNP selector resistors
  - ROS2 SBUS parser node → `/rc/cmd_vel` → twist_mux (highest priority)
  - Dead-man switch on spare channel — RC only active when held
  - Also serves as wireless e-stop (release stick = stop)
- [ ] CHT overheat protection: >200°C reduce throttle, >250°C emergency shutdown

## Physical Measurements to Verify
See printable measurement sheets: `docs/measurement-sheets/`
- [x] Wheel separation — 52" center-to-center (1.321m), updated in config
- [x] Wheel radius — 14.5" loaded diameter (0.184m radius), updated in config
- [x] Chassis dimensions — 1.52m x 1.137m (from Swisher drawing, sufficient for URDF visual)
- [x] Caster geometry — 11.75" loaded dia, 4.5" wide, 3.625" swivel offset, 11" pivot, 3.25" trail, 40" separation
- [x] GPS antenna position — 5" forward of drive axle, centered laterally, height TBD
- [x] Engine spec plate — B&S I/C 500cc 14.5 HP, Family 9BSXS.5002VP, ~3600 RPM no-load
- [x] Belt routing — 2-belt system, all blade pulleys 5.5" (1:1), engage pulley 3.7"
- [x] Blade pulley shaft — threaded, 1/8" protrusion, hex nut
- [x] Choke mechanism — combined choke/throttle single lever (idle→half→full→choke)
- [x] Motor wiring — 2 power + 2 brake wires, no encoder connector, brakes removed
- [x] Fuel tank — B&S plastic, L-shaped, ~12" x 7", ~3.5 qt, translucent, filler cap on extension

## GPIO Pin Map (via MOXL HAT — all opto-isolated)

| Pin | Assignment | HAT Section |
|-----|-----------|-------------|
| GPIO 2 (SDA) | I2C data | PCA9685 + ADS1115 |
| GPIO 3 (SCL) | I2C clock | PCA9685 + ADS1115 |
| GPIO 4 | RPM tach input | Schmitt trigger + opto |
| GPIO 7 (CE1) | MAX31855 chip select | SPI |
| GPIO 8 (CE0) | MCP251863T chip select | SPI |
| GPIO 9 (MISO) | SPI data in | SPI |
| GPIO 10 (MOSI) | SPI data out | SPI |
| GPIO 11 (SCLK) | SPI clock | SPI |
| GPIO 12 | Left motor RPWM | Opto pass-through |
| GPIO 13 | Right motor RPWM | Opto pass-through |
| GPIO 16 | Left motor LPWM | Opto pass-through |
| GPIO 17 | Engine run relay | Opto relay driver |
| GPIO 20 | Left motor R_EN | Opto pass-through |
| GPIO 21 | Left motor L_EN | Opto pass-through |
| GPIO 22 | MCP251863T interrupt | SPI |
| GPIO 23 | Right motor R_EN | Opto pass-through |
| GPIO 24 | Right motor L_EN | Opto pass-through |
| GPIO 25 | Blade relay | Opto relay driver |
| GPIO 26 | Right motor LPWM | Opto pass-through |
| GPIO 27 | Engine start relay | Opto relay driver |
| GPIO 5 | *Available* | |
| GPIO 6 | *Available* | |
| GPIO 19 | *Available* | |
