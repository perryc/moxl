# Mo-XL — Mower eXtra Large

An autonomous mower for large-scale grass areas like airstrips, built with ROS2 and RTK GPS.

![MOXL Build — Swisher 60" deck with differential drive](photos/WhatsApp%20Image%202024-12-12%20at%2014.28.09_95a63cdd.jpg)

## The Problem

Mowing a grass airstrip is big job and the guy with the plane is often not around to cut it. This is a very stale project nearing 10 years now but fully with spring is coming soon optimism and a lot of claude code help maybe we can cut some grass this year.  The donor mower was chosen simply because we had one kicking around at the farm not getting used.  The initial build the plan is to use at CDS2 (Disley, Saskatchewan) has a 2246 ft x 78 ft grass runway that needs regular cutting — that's nearly half a mile of back-and-forth passes with a mower. RTK GPS gives centimeter-level accuracy in an open field. A robot can do this.

## The Approach

The [OpenMower](https://openmower.de/) project proved that a consumer mower can be converted to autonomous operation using ROS2, GPS, and off-the-shelf electronics. [OpenMowerNext](https://github.com/jkaflik/OpenMowerNext) by Jakub Kaflik rebuilt the software stack on modern ROS2 Jazzy with Nav2 and ros2_control.

MOXL takes that same architecture and scales it up — from a backyard lawn mower to a 60-inch commercial deck that can handle an airstrip. The software is forked from OpenMowerNext; the hardware is entirely custom.

## What Changed from OpenMowerNext

The fork keeps the proven ROS2 infrastructure — ros2_control, Nav2 path following, robot_localization EKF, Foxglove monitoring — and replaces everything hardware-specific:

| OpenMowerNext | MOXL | Why |
|---|---|---|
| VESC motor controllers | BTS7960 43A H-bridges | Different motors, 24V system |
| u-blox F9P GPS | FarmTRX RTK (NMEA serial) | Already owned, corrections handled onboard |
| NTRIP client | Removed | FarmTRX handles its own RTK corrections |
| micro-ROS + custom mainboard | Raspberry Pi GPIO directly | No intermediate microcontroller needed |
| IMU (on mainboard) | GPS magnetometer heading | Open field, no tight spaces |
| Lawn polygon map server | Airstrip toolpath planner | Runway data from surveyed coordinates |
| Docking station | GPS park position | No charging dock — gas engine |
| YardForce 500B URDF | Custom URDF from measurements | Completely different machine |

## Hardware

### The Machine

A Swisher 60" towable rough-cut mower deck, converted from tow-behind to self-propelled:

- **Cutting deck**: Swisher 60" (1.52m), 3-blade, powered by a 16 HP gas engine with electric start
- **Drive system**: Two electric wheelchair motors on 13" turf tires, differential steering
- **Motor controllers**: 2x BTS7960 43A dual H-bridge boards, 24V PWM, 3.3V logic compatible
- **GPS**: FarmTRX RTK rover — NMEA0183 serial output, magnetometer heading (HDT), cm-level accuracy
- **Computer**: Raspberry Pi 4 running ROS2 Jazzy
- **Power**: 2x 12V lead-acid batteries in series (24V drive, 12V center tap for starter and Pi)
- **Charging**: 24V alternator belt-driven off the blade pulley — charges while mowing

### Key Dimensions

From the engineering drawing (all verified on the physical build):

| Measurement | Value |
|---|---|
| Wheel separation (center-to-center) | 1.19 m (51 3/4") |
| Wheel radius (turf tires) | 0.165 m (~13" diameter) |
| Cutting width | 1.52 m (60") |
| Chassis length | 1.137 m (44 3/4") |
| Chassis height | 0.394 m (15 1/2") |
| Caster offset behind drive axle | 0.895 m (35 1/4") |
| GPS antenna height | 0.50 m above drive axle |
| Total mass (estimated) | ~140 kg |

![Engineering drawing](photos/WhatsApp%20Image%202024-12-12%20at%2014.26.26_54bdfe53.jpg)

### GPIO Pin Map

| Pin | Function | Status |
|---|---|---|
| GPIO 4 | Engine RPM tach input | Reserved |
| GPIO 12, 16, 20, 21 | Left motor (RPWM, LPWM, R_EN, L_EN) | Needs wiring |
| GPIO 13, 26, 23, 24 | Right motor (RPWM, LPWM, R_EN, L_EN) | Needs wiring |
| GPIO 17 | Engine starter relay | Reserved |
| GPIO 18 | Choke servo PWM | Reserved |
| GPIO 25 | Blade engagement relay | Reserved |
| GPIO 27 | Belt actuator (TBD) | Reserved |

Full wiring harness diagram in [WireViz format](docs/wiring/moxl_harness.yml) — generate with `wireviz docs/wiring/moxl_harness.yml`.

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
│ _manager │          │          │          │                     │
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
- `nmea_navsat_driver` — reads FarmTRX NMEA serial, publishes `/gps/fix` (NavSatFix) and `/heading` (QuaternionStamped)
- `heading_to_imu_node` — bridges GPS magnetometer heading to `sensor_msgs/Imu` with speed-dependent covariance blending (trusts magnetometer at standstill, backs off at speed so GPS track heading dominates)
- `ekf_se_odom` — odom frame EKF: wheel odometry + magnetometer heading
- `ekf_se_map` — map frame EKF: wheel odometry + GPS position + heading
- `navsat_transform_node` — converts GPS lat/lon to local map frame odometry

**Navigation** — Nav2 path following:
- `RegulatedPurePursuitController` — smooth path tracking tuned for large turning radius
- `NavfnPlanner` — global path planning
- `VelocitySmoother` — acceleration limiting for smooth motion
- `twist_mux` — priority-based velocity command multiplexing (joystick > navigation, e-stop locks all)

**Mission Control** — autonomous mowing orchestration:
- `toolpath_node` — generates parallel boustrophedon mowing strips from RTK-surveyed runway corners
- `engine_controller_node` — engine start/stop services (stub — GPIO implementation pending)
- `blade_controller_node` — blade engage/disengage services (stub — GPIO implementation pending)
- `mission_node` — action server that sequences: preflight (start engine, engage blades) → mow all strips via Nav2 → return to park position → shutdown
- `safety_monitor_node` — GPS fix watchdog (e-stop on RTK loss or timeout), future LiDAR obstacle detection

### Custom ROS2 Interfaces

**Messages:**
- `EngineStatus.msg` — engine state (OFF/CRANKING/RUNNING/ERROR), RPM, cylinder head temp, choke
- `BladeStatus.msg` — blade state (DISENGAGED/ENGAGING/ENGAGED/DISENGAGING/ERROR)
- `MissionStatus.msg` — mission state, current strip, progress percentage

**Services:**
- `StartEngine.srv`, `StopEngine.srv` — engine control
- `SetBlades.srv` — blade engagement (engage/disengage)

**Actions:**
- `MowMission.action` — full autonomous mowing mission with feedback (strip progress, state)

### Toolpath Generation

The toolpath planner loads runway corner coordinates (RTK-surveyed at 0.02m accuracy), projects to UTM, and generates parallel mowing strips:

1. Load runway polygon from `config/airstrips/CDS2.json`
2. Project WGS84 corners to UTM Zone 13N (EPSG:32613) via pyproj
3. Compute runway heading from the long axis
4. Step perpendicular to heading at `cutting_width - overlap` intervals
5. Clip each strip to the runway polygon (shapely)
6. Order strips in boustrophedon (alternating direction) pattern

For CDS2 runway 11/29 (2246 ft x 78 ft) with a 60" deck and 15cm overlap: ~17 strips, ~12 km total mowing distance.

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
│   ├── hardware/moxl.yaml           # Physical dimensions & GPIO pins
│   ├── mowing_params.yaml           # Cutting width, overlap, speed
│   ├── nav2_params.yaml             # Nav2 path following tuning
│   ├── robot_localization.yaml      # Dual EKF + navsat_transform
│   └── twist_mux.yaml              # Velocity priority + e-stop lock
├── description/                     # URDF/xacro robot model
│   ├── robot.urdf.xacro            # Top-level URDF
│   ├── robot_core.xacro            # Chassis, wheels, casters, deck
│   ├── gps.xacro                   # GPS antenna link + visual
│   └── ros2_control.xacro          # BTS7960 hw_interface (mock on desktop)
├── docs/wiring/
│   └── moxl_harness.yml            # WireViz wiring harness diagram
├── launch/
│   ├── moxl.launch.py              # Full system launch
│   ├── gps.launch.py               # GPS driver
│   ├── localization.launch.py      # EKF + navsat_transform + heading bridge
│   ├── nav2.launch.py              # Navigation stack
│   ├── mission.launch.py           # Mission control nodes
│   ├── joystick.launch.py          # Manual teleop
│   └── sim.launch.py               # Gazebo simulation
├── src/
│   ├── msg/                         # EngineStatus, BladeStatus, MissionStatus
│   ├── srv/                         # StartEngine, StopEngine, SetBlades
│   ├── action/                      # MowMission
│   ├── toolpath_planner/            # Python ROS2 package
│   │   ├── toolpath_planner/        # Core library
│   │   │   ├── coordinate_utils.py  # WGS84 ↔ UTM conversion
│   │   │   ├── polygon_loader.py    # Airport JSON → runway corners
│   │   │   └── strip_generator.py   # Boustrophedon strip generation
│   │   ├── nodes/                   # ROS2 nodes
│   │   │   ├── toolpath_node.py     # Strip generation + publishing
│   │   │   ├── heading_to_imu_node.py # GPS heading → IMU bridge
│   │   │   ├── engine_controller_node.py # Engine control (stub)
│   │   │   ├── blade_controller_node.py  # Blade control (stub)
│   │   │   ├── mission_node.py      # Mission orchestrator
│   │   │   └── safety_monitor_node.py # GPS watchdog + e-stop
│   │   └── test/                    # 31 unit tests
│   └── lib/bts7960_hw_interface/    # C++ ros2_control plugin (TBD)
├── photos/                          # Build photos & engineering drawing
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

### Launch (Desktop / Simulation)

```bash
# Full system (uses mock hardware interface on desktop)
ros2 launch moxl moxl.launch.py

# Gazebo simulation
ros2 launch moxl sim.launch.py

# Manual teleop for testing
ros2 launch moxl joystick.launch.py
```

### Start a Mowing Mission

```bash
# Trigger via ROS2 action
ros2 action send_goal /moxl/mission moxl/action/MowMission \
  "{airstrip_id: 'CDS2', runway_id: '11/29'}"
```

The mission sequences through: preflight (engine start, blade engage) → mow all strips → return to park → shutdown. Monitor progress via Foxglove at `ws://localhost:8765`.

## Status

This is an active build. The software architecture is complete; hardware integration is in progress.

- [x] Fork and adapt OpenMowerNext
- [x] BTS7960 hardware interface (ros2_control plugin defined, mock for desktop)
- [x] URDF robot description (from engineering drawing measurements)
- [x] GPS integration (nmea_navsat_driver + magnetometer heading bridge)
- [x] Toolpath planner (boustrophedon strip generation, 31 tests passing)
- [x] Mission control (action server state machine, engine/blade stubs)
- [x] Safety monitor (GPS watchdog + e-stop)
- [x] Wiring harness diagram (WireViz)
- [ ] BTS7960 GPIO driver (compile for Pi, wire motors)
- [ ] Engine control GPIO (starter relay, choke servo, RPM sensor)
- [ ] Blade engagement hardware (relay + belt actuator)
- [ ] Field testing at CDS2

See [HARDWARE_TODO.md](HARDWARE_TODO.md) for the full hardware integration checklist.

## Acknowledgments

- [OpenMowerNext](https://github.com/jkaflik/OpenMowerNext) by Jakub Kaflik — the ROS2 foundation this project is built on
- [OpenMower](https://openmower.de/) by Clemens Elflein — the original open-source autonomous mower that proved the concept
- [Nav2](https://nav2.org/) — battle-tested robot navigation
- [robot_localization](https://docs.ros.org/en/jazzy/p/robot_localization/) — GPS + odometry fusion
- [WireViz](https://github.com/wireviz/WireViz) — wiring harness documentation

## License

Apache 2.0 — see [LICENSE](LICENSE). Original OpenMowerNext code is also Apache 2.0.
