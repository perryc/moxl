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
- [ ] Investigate wheel encoders — motors are plain DC (no built-in halls)
  - Options: magnetic encoders on wheel hubs (AS5048A), optical slot sensors
  - 2 GPIO inputs for pulse counting (future, or via CAN motor nodes)

## Engine Control (via MOXL HAT)
- [ ] Run relay (magneto kill) → HAT relay driver, GPIO 17
- [ ] Start relay (starter solenoid) → HAT relay driver, GPIO 27 — momentary pulse
- [ ] Choke servo → HAT PCA9685 CH0 — mount servo to choke cable/lever
- [ ] Throttle servo → HAT PCA9685 CH1 — mount servo to throttle cable/lever
- [ ] RPM sensor → HAT opto-isolated input, GPIO 4 — hall sensor on flywheel or spark plug lead pickup
- [ ] CHT thermocouple → HAT MAX31855, SPI CS1 — K-type ring terminal under head bolt
- [ ] Implement engine_controller_node.py: cold/warm start sequence, CHT-driven choke logic

## Blade Engagement (via MOXL HAT)
- [ ] Blade relay → HAT relay driver, GPIO 25
- [ ] Belt engagement actuator → HAT PCA9685 CH2 — **hardware not selected**
  - Options: linear actuator, solenoid, servo on engagement lever
  - Needs enough force to engage/disengage blade drive belt under tension
- [ ] GPS work_state feedback via CAN (GPXDR MACHINE_WORK) confirms engagement

## Power System
- [ ] Install 2x 12V batteries in series (24V drive, 12V center tap for starter)
- [ ] MOXL HAT 24V→5V buck powers Pi (no separate Pi power supply needed)
- [ ] Direct-drive alternator coupler on blade pulley shaft
  - [ ] Measure blade pulley shaft diameter, keyway, protrusion above deck
  - [ ] Design/machine coupler to mount Denso alternator
  - [ ] Convert Denso alternator to 24V output
  - [ ] Mount alternator bracket above deck
- [ ] Fuel level sender — resistive float in plastic tank → HAT ADS1115 A2
- [ ] Battery voltage monitoring → HAT ADS1115 A0 (24V via divider)
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
- [ ] LiDAR — **hardware not selected**, obstacle detection stubbed in `safety_monitor_node.py`
  - Options: RPLidar A1 (2D, cheap), RPLidar A2, Livox Mid-360
  - Need: forward arc obstacle detection, ~10m range minimum
- [ ] Physical e-stop button — GPIO input (pin TBD), hardware interrupt to kill motors + blades
- [ ] Wireless e-stop / RC kill switch (nice to have)
- [ ] CHT overheat protection: >200°C reduce throttle, >250°C emergency shutdown

## Physical Measurements to Verify
See printable measurement sheets: `docs/measurement-sheets/`
- [ ] Wheel separation — configured 1.19m (from engineering drawing, 51 3/4")
- [ ] Wheel radius — configured 0.165m (~13" diameter)
- [ ] Chassis dimensions — configured 1.52m x 1.137m
- [ ] Caster geometry — wheel diameter, trail, swivel offset, pivot height
- [ ] GPS antenna position relative to drive axle center
- [ ] Engine spec plate / rated RPM
- [ ] Belt routing + all pulley diameters
- [ ] Blade pulley shaft — diameter, end type, protrusion for alternator coupler
- [ ] Choke mechanism type (manual cable / auto / primer)
- [ ] Motor wiring (confirm no encoder connector)
- [ ] Fuel tank — size, sender mounting location

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
