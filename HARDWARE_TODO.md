# MOXL Hardware TODO

Everything that needs wiring, verifying, or selecting before field deployment.

## Drive System (BTS7960)
- [ ] Wire left motor: RPWM→GPIO12, LPWM→GPIO16, R_EN→GPIO20, L_EN→GPIO21
- [ ] Wire right motor: RPWM→GPIO13, LPWM→GPIO26, R_EN→GPIO23, L_EN→GPIO24
- [ ] Verify 24V supply to both BTS7960 boards
- [ ] Verify PWM frequency (20 kHz) and duty cycle range works with motors
- [ ] Compile BTS7960 hw_interface for Pi (currently using mock_components on desktop)

## Engine Control (all stubbed in `engine_controller_node.py`)
- [ ] Starter relay → GPIO 17 — wire relay module, implement GPIO toggle
- [ ] Choke servo → GPIO 18 (hardware PWM) — select servo, mount to choke lever, implement PWM control
- [ ] RPM sensor → GPIO 4 — install tach pickup on engine, implement edge counting
- [ ] CHT sensor → I2C ADC — select thermocouple/thermistor + ADC board, wire to I2C bus

## Blade Engagement (stubbed in `blade_controller_node.py`)
- [ ] Blade relay → GPIO 25 — wire relay to blade enable circuit
- [ ] Belt engagement actuator → GPIO 27 (reserved) — **hardware not selected**
  - Options: linear actuator, solenoid, servo on engagement lever
  - Needs enough force to engage/disengage blade drive belt under tension

## Power System
- [ ] Install 2x 12V batteries in series (24V drive, 12V center tap for starter/Pi)
- [ ] Install 24V alternator coupled to blade belt drive system
- [ ] Wire 12V center tap to starter motor and Pi power supply
- [ ] Verify alternator charges at sufficient rate for continuous operation

## GPS (FarmTRX RTK)
- [ ] Verify serial port — currently `/dev/ttyUSB0` in `config/gps.yaml`
- [ ] Verify baud rate — currently `115200` in `config/gps.yaml`
- [ ] Measure actual GPS antenna offset from base_link (drive axle center)
  - Currently estimated: `[0.0, 0.0, 0.50]` m in `config/hardware/moxl.yaml`
- [ ] Confirm magnetic declination 7.2° East for CDS2 site

## Safety
- [ ] LiDAR — **hardware not selected**, obstacle detection stubbed in `safety_monitor_node.py`
  - Options: RPLidar A1 (2D, cheap), RPLidar A2, Livox Mid-360
  - Need: forward arc obstacle detection, ~10m range minimum
- [ ] Physical e-stop button — GPIO input (pin TBD), hardware interrupt to kill motors + blades
- [ ] Wireless e-stop / RC kill switch (nice to have)

## Physical Measurements to Verify
- [ ] Wheel separation — configured 1.19m (from engineering drawing, 51 3/4")
- [ ] Wheel radius — configured 0.165m (~13" diameter)
- [ ] Chassis dimensions — configured 1.52m x 1.137m
- [ ] Caster wheel offset — configured 0.895m behind drive axle

## GPIO Pin Map
| Pin | Assignment | Status |
|-----|-----------|--------|
| GPIO 4 | RPM tach input | Reserved, not wired |
| GPIO 12 | Left motor RPWM | Needs wiring |
| GPIO 13 | Right motor RPWM | Needs wiring |
| GPIO 16 | Left motor LPWM | Needs wiring |
| GPIO 17 | Engine starter relay | Reserved, not wired |
| GPIO 18 | Choke servo PWM | Reserved, not wired |
| GPIO 20 | Left motor R_EN | Needs wiring |
| GPIO 21 | Left motor L_EN | Needs wiring |
| GPIO 23 | Right motor R_EN | Needs wiring |
| GPIO 24 | Right motor L_EN | Needs wiring |
| GPIO 25 | Blade relay | Reserved, not wired |
| GPIO 26 | Right motor LPWM | Needs wiring |
| GPIO 27 | Belt actuator (TBD) | Reserved, not wired |
| GPIO 5 | *Available* | |
| GPIO 6 | *Available* | |
| GPIO 19 | *Available* | |
| GPIO 22 | *Available* | |
