# Mo-XL — Mower eXtra Large

An autonomous mower for large-scale grass areas like airstrips, built with ROS2 and RTK GPS.

![Mo-XL — Swisher 60" deck with differential drive, hangar measurement session](photos/WhatsApp%20Image%202026-03-07%20at%2019.22.29.jpeg)

## The Problem

Mowing a grass airstrip is a big job and the guy with the plane is often not around to cut it. This project has been kicking around for nearly 10 years now, but with "spring is coming soon" optimism, a lot of Claude Code help, and a few CDS2-ers pitching in — maybe we can cut some grass this year. The donor mower was chosen simply because we had one sitting at the farm not getting used.

The initial deployment is at CDS2 (Disley, Saskatchewan) — a 2246 ft x 78 ft grass runway that needs regular cutting. That's nearly half a mile of back-and-forth passes. RTK GPS gives centimeter-level accuracy in an open field. A robot can do this.

## The Approach

The [OpenMower](https://openmower.de/) project proved that a consumer mower can be converted to autonomous operation using ROS2, GPS, and off-the-shelf electronics. [OpenMowerNext](https://github.com/jkaflik/OpenMowerNext) by Jakub Kaflik rebuilt the software stack on modern ROS2 Jazzy with Nav2 and ros2_control.

Mo-XL takes that same architecture and scales it up — from a backyard lawn mower to a 60-inch commercial deck that can handle an airstrip. The software is forked from OpenMowerNext; the hardware is entirely custom.

## What Changed from OpenMowerNext

The fork keeps the proven ROS2 infrastructure — ros2_control, Nav2 path following, robot_localization EKF, Foxglove monitoring — and replaces everything hardware-specific:

| OpenMowerNext | Mo-XL | Why |
|---|---|---|
| VESC motor controllers | BTS7960 43A H-bridges | Different motors, 24V system |
| u-blox F9P GPS | FarmTRX RTK (CAN GPS) | We make this stuff! Let's use it |
| NTRIP client | Removed | FarmTRX handles its own RTK corrections |
| micro-ROS + custom mainboard | Mo-XL Pi HAT (custom PCB) | Single board, opto-isolated, JLCPCB assembled |
| IMU (on mainboard) | GPS magnetometer + gyro | FarmTRX sends heading (HDT/HCHDG) and rate of turn (TIROT) |
| Lawn polygon map server | Airstrip toolpath planner | Runway data from surveyed coordinates |
| Docking station | GPS park position | No charging dock — gas engine with alternator |
| YardForce 500B URDF | Custom URDF from measurements | Completely different machine |

## Hardware

### The Machine

A Swisher 60" towable rough-cut mower deck, converted from tow-behind to self-propelled:

- **Cutting deck**: Swisher 60" (1.52m), 3-blade triangle layout, B&S I/C 500cc 14.5 HP gas engine with electric start
- **Drive system**: Two electric wheelchair motors on 14.5" turf tires, chain-driven (19T→32T, 1.684:1), differential steering
- **Motor controllers**: 2x BTS7960 43A dual H-bridge boards, 24V PWM, 3.3V logic compatible
- **GPS**: FarmTRX RTK rover — NMEA0183 serial output, magnetometer heading (HDT/HCHDG), rate of turn gyro (TIROT), cm-level accuracy
- **I/O Board**: Mo-XL Pi HAT v1.0 — custom 6-layer PCB (see below)
- **Radio**: RTL-SDR v3/v4 USB dongle — monitors 122.8 MHz CTAF for aviation traffic
- **Computer**: Raspberry Pi 5 (8GB) running ROS2 Jazzy
- **Power**: 2x 12V lead-acid batteries in series (24V drive, 12V center tap for starter and Pi HAT)
- **Charging**: 24V alternator belt-driven off blade engagement belt — charges while mowing

### Key Dimensions

All dimensions from hangar measurements (Mar 2026) — see `config/hardware/moxl.yaml`.

| Measurement | Value |
|---|---|
| Rear wheel separation (center-to-center) | 1.321 m (52") |
| Rear wheel radius (turf tires) | 0.184 m (14.5" diameter) |
| Wheelbase (caster pivot to drive axle) | 1.105 m (43.5") |
| Caster separation | 1.016 m (40") |
| Cutting width | 1.52 m (60") |
| Chassis length | 1.137 m (44 3/4") |
| GPS antenna position | 0.127m fwd of drive axle, 0.80m AGL |
| Chain drive ratio | 19T motor → 32T wheel (1.684:1) |
| Total mass (estimated) | ~140 kg |

```
                 1.016m (40")
          ┌──────────────────────┐
    caster ○                      ○ caster
          │                      │
          │    1.105m wheelbase   │
          │                      │
          │      ★ GPS antenna   │
          │     (5" fwd of axle) │
          │                      │
    drive ●──────────────────────● drive
          └──────────────────────┘
                 1.321m (52")

    ●/○ = wheel    ★ = GPS antenna
    Front casters, rear drive (diff steer)
```

### Mo-XL Pi HAT v1.0

Custom Raspberry Pi 5 I/O board — single PCB replaces all direct GPIO wiring with isolated, protected I/O. 6-layer ENIG, 65 x 56.5mm Pi HAT form factor, assembled by JLCPCB.

Full spec sheet: [`hardware/moxl-hat/Mo-XL_Hat_v1.0_Spec.pdf`](hardware/moxl-hat/Mo-XL_Hat_v1.0_Spec.pdf)

| Feature | Implementation |
|---|---|
| **Power** | Hardened 12V input (TVS + EMI filter + polyfuse), XL4015 buck to 5.88V, TPS7A7001 LDO to 5.05V. Powers Pi and all HAT circuitry. |
| **2x Motor Drive** | 8 opto-isolated signals (TLP293-4) to BTS7960 modules via IDC ribbon cable. 20 kHz PWM. |
| **CAN / CAN-FD** | MCP251863T integrated controller + transceiver (SPI). TVS protected. |
| **RS-232** | MAX3221 on UART0. TX always active. |
| **SBUS** | RC receiver input via 74HC14 inverter on UART0 RX. Hardware-selectable vs RS-232 RX. |
| **4x Servo PWM** | PCA9685 16-ch PWM driver (I2C). 5.88V servo power rail. |
| **4x Analog Input** | ADS1115 16-bit ADC (I2C). On-board dividers for 12V/24V battery monitoring. |
| **K-type Thermocouple** | MAX31855 (SPI). Cylinder head temperature, 0-1024C. |
| **4x Relay Driver** | ULN2003A with flyback clamp. Engine run, start, blade engage, aux. |
| **4x Schmitt Trigger Input** | 74HC14 conditioned: engine tach + 2 wheel encoder pulses + SBUS. |
| **HAT ID EEPROM** | CAT24C256 on I2C0. Pi auto-configuration per HAT spec. |

KiCad source: [`hardware/moxl-hat/`](hardware/moxl-hat/)

### Connectors

| Connector | Type | Signals |
|---|---|---|
| J1 | JST-VH 2p | GND, +12V_RAW |
| J2 | JST-XH 4p | CANH, CANL, CAN_12V, GND |
| J3 | IDC 2x4 | Left motor (RPWM, LPWM, R_EN, L_EN, R_IS, L_IS, VCC, GND) |
| J4 | IDC 2x4 | Right motor (same as J3) |
| J5 | JST-XH 5p | RLY_RUN, RLY_START, RLY_BLADE, RLY_AUX, GND |
| J6 | JST-XH 8p | 4x servo PWM + 6V power + GND |
| J7 | JST-PH 5p | BATT_V, ALT_I, FUEL, PACK_V, GND |
| J8 | JST-PH 5p | RPM_SIG, ENC_L, ENC_R, 5V, GND |
| J9 | JST-XH 2p | TC+, TC- |
| J11 | JST-PH 3p | RS232_TX, RS232_RX, GND |
| J12 | JST-PH 3p | SBUS_IN, 5V, GND |

Unique pin counts per connector family prevent cross-plugging.

## Software Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        moxl.launch.py                           │
├──────────┬──────────┬──────────┬──────────┬─────────────────────┤
│  URDF /  │   GPS    │  Local-  │  Nav2    │  Mission Control    │
│  ros2    │          │  ization │          │                     │
│  control │          │          │          │                     │
├──────────┼──────────┼──────────┼──────────┼─────────────────────┤
│ robot_   │ nmea_    │ heading  │ control  │ toolpath_node       │
│ state_   │ navsat_  │ _to_imu  │ _server  │ engine_controller   │
│ publisher│ driver   │ _node    │          │ blade_controller    │
│          │          │          │ planner  │ mission_node        │
│ controller         │ ekf_odom │ _server  │ safety_monitor      │
│ _manager │          │          │          │ sdr_detector        │
│          │ /gps/fix │ ekf_map  │ bt_nav   │                     │
│ diff_    │ /heading │          │          │                     │
│ drive_   │          │ navsat_  │ velocity │                     │
│ controller         │ transform│ _smoother│                     │
├──────────┴──────────┴──────────┴──────────┴─────────────────────┤
│                   twist_mux (with e-stop lock)                  │
├─────────────────────────────────────────────────────────────────┤
│                   Foxglove Bridge (web UI)                      │
└─────────────────────────────────────────────────────────────────┘
```

### ROS2 Nodes

**Localization** — GPS position and heading fused into odometry:
- `nmea_navsat_driver` — reads FarmTRX NMEA serial, publishes `/gps/fix` (NavSatFix), `/heading` (QuaternionStamped), and `/nmea_sentence` (raw NMEA for HCHDG/TIROT parsing)
- `heading_to_imu_node` — bridges GPS heading to `sensor_msgs/Imu` with speed-dependent covariance blending. Parses TIROT (rate of turn) for angular velocity and HCHDG as fallback heading source.
- `ekf_se_odom` — odom frame EKF: wheel odometry + magnetometer heading + gyro rate of turn
- `ekf_se_map` — map frame EKF: wheel odometry + GPS position + heading + gyro
- `navsat_transform_node` — converts GPS lat/lon to local map frame odometry

**Navigation** — Nav2 path following:
- `RegulatedPurePursuitController` — smooth path tracking tuned for large turning radius
- `SmacPlanner2D` — global path planning
- `VelocitySmoother` — acceleration limiting for smooth motion
- `twist_mux` — priority-based velocity command multiplexing (RC > navigation, e-stop locks all)

**Mission Control** — autonomous mowing orchestration:
- `toolpath_node` — generates headland-first mowing pattern: perimeter passes then parallel strips with step-in turns
- `engine_controller_node` — engine start/stop with CHT-driven choke logic (stub — HAT GPIO pending)
- `blade_controller_node` — blade engage/disengage via belt actuator relay (stub — HAT GPIO pending)
- `mission_node` — action server state machine: preflight (start engine, engage blades) → mow all strips via Nav2 FollowPath → return to park → shutdown. Monitors CTAF radio and evacuates when traffic is detected.
- `safety_monitor_node` — GPS fix watchdog (e-stop on RTK loss or timeout), future LiDAR obstacle detection
- `sdr_detector_node` — RTL-SDR radio traffic detector monitoring 122.8 MHz CTAF

### Custom ROS2 Interfaces

**Messages:**
- `EngineStatus.msg` — engine state (OFF/CRANKING/RUNNING/ERROR), RPM, cylinder head temp, choke
- `BladeStatus.msg` — blade state (DISENGAGED/ENGAGING/ENGAGED/DISENGAGING/ERROR)
- `MissionStatus.msg` — mission state, current strip, progress percentage
- `RadioDetection.msg` — SDR detection event: frequency, signal power, active flag, quiet duration

**Services:**
- `StartEngine.srv`, `StopEngine.srv` — engine control
- `SetBlades.srv` — blade engagement (engage/disengage)

**Actions:**
- `MowMission.action` — full autonomous mowing mission with feedback (strip progress, state)

### Toolpath Generation

The toolpath planner loads runway corner coordinates (RTK-surveyed at 0.02m accuracy), projects to UTM, and generates a headland-first mowing pattern:

1. Load runway polygon from `config/airstrips/CDS2.json`
2. Project WGS84 corners to UTM Zone 13N (EPSG:32613) via pyproj
3. Mow perimeter headland passes first (2 passes around the boundary)
4. Generate parallel strips for the interior with step-in turns
5. Densify each path with waypoints every 1.5m
6. All paths sent to Nav2 FollowPath (bypasses planner — open field needs no obstacle avoidance)

For CDS2 runway 11/29 (2246 ft x 78 ft) with a 60" deck and 5cm overlap: ~4.3 km total mowing distance.

### Aviation Radio Traffic Detector

You can't land if a mower is on the runway. Mo-XL monitors the airport traffic frequency with an RTL-SDR USB dongle (~$30) and automatically evacuates the runway when radio traffic is detected.

**How it works:**

1. `sdr_detector_node` tunes RTL-SDR to 122.8 MHz, reads IQ samples in a background thread, and computes signal power against a rolling noise floor
2. When a transmission is detected (power > noise floor + squelch threshold), the node publishes `radio_active=True` and asserts e-stop to immediately halt the mower
3. `mission_node` cancels the active FollowPath goal, navigates to the nearest pre-defined clear zone (off-runway safe area), and waits
4. After 5 minutes of radio silence, the mower resumes mowing from where it left off

**Clear zones** are defined per-runway in `CDS2.json` — three safe parking areas (north apron, midfield west, south holdshort) where the mower can wait without blocking the runway.

**Fail-open design:** If the RTL-SDR hardware isn't connected or `pyrtlsdr` isn't installed, the detector logs a warning and publishes `radio_active=False` — mowing continues unblocked. The GPS safety monitor remains the hard backstop.

## Target Airstrip

**CDS2** — Disley, Saskatchewan, Canada

| | |
|---|---|
| Runway | 11/29 |
| Dimensions | 2246 ft x 78 ft (685 m x 24 m) |
| Heading | ~117° true |
| Elevation | 1847–1861 ft |
| Magnetic declination | 7.2° East |
| UTM Zone | 13N (EPSG:32613) |

All four runway corners RTK-surveyed at 0.02m accuracy:

| Corner | Latitude | Longitude |
|---|---|---|
| NW | 50.63612190 | -105.03202723 |
| NE | 50.63603243 | -105.03175091 |
| SE | 50.63285006 | -105.03165067 |
| SW | 50.63288086 | -105.03194065 |

## Project Structure

```
moxl/
├── config/
│   ├── airstrips/CDS2.json          # RTK-surveyed runway corners
│   ├── controllers.yaml             # ros2_control diff_drive params
│   ├── gps.yaml                     # nmea_navsat_driver config
│   ├── hardware/moxl.yaml           # Physical dimensions (from hangar measurements)
│   ├── mowing_params.yaml           # Cutting width, overlap, speed
│   ├── nav2_params.yaml             # Nav2 path following tuning
│   ├── robot_localization.yaml      # Dual EKF + navsat_transform
│   └── twist_mux.yaml              # Velocity priority + e-stop lock
├── description/                     # URDF/xacro robot model
│   ├── robot.urdf.xacro            # Top-level URDF
│   ├── robot_core.xacro            # Chassis, wheels, casters, deck
│   ├── gps.xacro                   # GPS antenna link + visual
│   └── ros2_control.xacro          # BTS7960 hw_interface (mock on desktop)
├── docs/
│   ├── moxl-hat-spec.md            # Pi HAT full specification
│   └── wiring/moxl_harness.yml     # WireViz wiring harness diagram
├── hardware/moxl-hat/              # KiCad PCB design
│   ├── moxl-hat.kicad_sch          # Schematic (clean DRC)
│   ├── moxl-hat.kicad_pcb          # 6-layer PCB layout
│   ├── Mo-XL_Hat_v1.0_Spec.pdf     # Spec sheet with schematic
│   ├── pinout-table.dxf            # Bottom silk pinout (for KiCad import)
│   └── production/                 # JLCPCB fabrication files (BOM, positions, Gerbers)
├── launch/
│   ├── moxl.launch.py              # Full system launch
│   ├── gps.launch.py               # GPS driver
│   ├── localization.launch.py      # EKF + navsat_transform + heading bridge
│   ├── nav2.launch.py              # Navigation stack
│   ├── mission.launch.py           # Mission control nodes
│   ├── joystick.launch.py          # Manual teleop
│   └── sim.launch.py               # Gazebo simulation
├── src/
│   ├── msg/                         # EngineStatus, BladeStatus, MissionStatus, RadioDetection
│   ├── srv/                         # StartEngine, StopEngine, SetBlades
│   ├── action/                      # MowMission
│   ├── toolpath_planner/            # Python ROS2 package
│   │   ├── toolpath_planner/        # Core library
│   │   │   ├── coordinate_utils.py  # WGS84 ↔ UTM conversion
│   │   │   ├── polygon_loader.py    # Airport JSON → runway corners
│   │   │   └── strip_generator.py   # Headland-first pattern generation
│   │   ├── nodes/                   # ROS2 nodes
│   │   │   ├── toolpath_node.py
│   │   │   ├── heading_to_imu_node.py
│   │   │   ├── engine_controller_node.py
│   │   │   ├── blade_controller_node.py
│   │   │   ├── mission_node.py
│   │   │   ├── safety_monitor_node.py
│   │   │   └── sdr_detector_node.py
│   │   └── test/                    # 31 unit tests
│   └── lib/bts7960_hw_interface/    # C++ ros2_control plugin (TBD)
├── photos/                          # Build photos & measurements
├── HARDWARE_TODO.md                 # Hardware integration checklist
└── worlds/                          # Gazebo simulation worlds
```

## Getting Started

### Prerequisites

ROS2 Jazzy on Ubuntu 24.04 (or Raspberry Pi OS with ROS2):

```bash
sudo apt install \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-diff-drive-controller ros-jazzy-robot-localization \
  ros-jazzy-nmea-navsat-driver ros-jazzy-navigation2 \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-foxglove-bridge \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-twist-mux
```

Python dependencies:

```bash
pip install pyproj shapely numpy
```

### Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/perryc/moxl.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Run Tests

```bash
cd src/moxl/src/toolpath_planner
python3 -m pytest test/ -v
```

All 31 tests cover coordinate conversion accuracy, strip generation geometry, and polygon loading.

### Simulation

Runs Gazebo simulation with full Nav2 navigation, toolpath planning, and mission control.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws && colcon build --symlink-install
source install/setup.bash
ros2 launch moxl sim.launch.py

# Send a mowing mission (in a second terminal)
ros2 action send_goal /moxl/mission moxl/action/MowMission \
  '{airstrip_id: CDS2, runway_id: "11/29"}' --feedback
```

### Launch (Native ROS2)

```bash
# Full system (uses mock hardware interface on desktop)
ros2 launch moxl moxl.launch.py

# Manual teleop for testing
ros2 launch moxl joystick.launch.py
```

### Start a Mowing Mission

```bash
ros2 action send_goal /moxl/mission moxl/action/MowMission \
  "{airstrip_id: 'CDS2', runway_id: '11/29'}"
```

The mission sequences through: preflight (engine start, blade engage) → mow all strips → return to park → shutdown. Monitor progress via Foxglove at `ws://localhost:8765`.

## Status

This is an active build. The software architecture is complete; hardware integration is in progress. Pi HAT boards ordered from JLCPCB.

- [x] Fork and adapt OpenMowerNext
- [x] BTS7960 hardware interface (ros2_control plugin defined, mock for desktop)
- [x] URDF robot description (from hangar measurements, all dimensions verified)
- [x] GPS integration (nmea_navsat_driver + magnetometer heading + TIROT gyro bridge)
- [x] Toolpath planner (headland-first pattern, 31 tests passing)
- [x] Mission control (action server state machine, engine/blade stubs)
- [x] Safety monitor (GPS watchdog + e-stop)
- [x] Mo-XL Pi HAT v1.0 designed (KiCad, 6-layer, clean DRC, JLCPCB production files)
- [x] Wiring harness diagram (WireViz)
- [x] Gazebo simulation — full autonomous mowing mission completes
- [x] Aviation radio traffic detector — RTL-SDR monitors 122.8 MHz CTAF, auto-evacuates runway
- [ ] Pi HAT assembly and smoke test (boards ordered, 10 days out)
- [ ] BTS7960 GPIO driver (compile for Pi, wire motors via HAT J3/J4)
- [ ] Engine control via HAT (starter relay, choke servo, RPM tach, CHT thermocouple)
- [ ] Blade engagement via HAT (relay + belt actuator)
- [ ] Belt-driven alternator for 24V charging
- [ ] Wheel encoder pulse counters (via HAT 74HC14 inputs)
- [ ] LiDAR obstacle detection (RPLIDAR C1 ordered)
- [ ] RC manual override (SBUS via HAT)
- [ ] Field testing at CDS2

See [HARDWARE_TODO.md](HARDWARE_TODO.md) for the full hardware integration checklist.

## Acknowledgments

- [OpenMowerNext](https://github.com/jkaflik/OpenMowerNext) by Jakub Kaflik — the ROS2 foundation this project is built on
- [OpenMower](https://openmower.de/) by Clemens Elflein — the original open-source autonomous mower that proved the concept
- [Nav2](https://nav2.org/) — battle-tested robot navigation
- [robot_localization](https://docs.ros.org/en/jazzy/p/robot_localization/) — GPS + odometry fusion
- [WireViz](https://github.com/wireviz/WireViz) — wiring harness documentation
- Jack C — bled for the initial measurements so we didn't have to

## License

Apache 2.0 — see [LICENSE](LICENSE). Original OpenMowerNext code is also Apache 2.0.
