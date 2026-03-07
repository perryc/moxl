# MOXL HAT — Custom Raspberry Pi 5 I/O Board

Purpose-built HAT for the Mo-XL autonomous mower. Single board replaces
direct GPIO wiring with isolated, protected I/O. All commodity parts.

## Block Diagram

```
                        MOXL HAT
┌─────────────────────────────────────────────────────┐
│                                                     │
│  ┌──────────┐ SPI CS0    ┌──────────┐               │
│  │MCP251863T├────────────┤ SPI Bus  │  ┌──────────┐ │    24V Bus
│  │CAN + XCVR│            │          ├──┤ Pi 5 SPI │◄├───────────┐
│  └────┬─────┘            │          │  └──────────┘ │           │
│       │ CAN H/L          │          │               │   ┌───────┴──────┐
│       ▼                  ├──────────┤               │   │ 24V→5V Buck  │
│  ┌─────────┐             │          │  ┌──────────┐ │   │ Powers Pi    │
│  │ CAN     │  SPI CS1    │          ├──┤ Pi 5 I2C │ │   └──────────────┘
│  │ Connector│  ┌─────────┤          │  └────┬─────┘ │
│  └─────────┘  │MAX31855  │          │       │       │
│               │K-type TC ├──────────┘       │       │
│               └────┬─────┘                  │       │
│                    │ TC input               │       │
│               ┌────┴─────┐           ┌──────┴─────┐ │
│               │CHT Sensor│           │  I2C Bus   │ │
│               │Connector │           │            │ │
│               └──────────┘           ├────────────┤ │
│                                      │            │ │
│  ┌──────────┐ I2C addr 0x40   ┌──────┤            │ │
│  │ PCA9685  ├─────────────────┘      │            │ │
│  │ 16-ch PWM│                        │            │ │
│  └──┬───────┘                 ┌──────┤            │ │
│     │ Servo PWM x4            │      │            │ │
│     ├── CH0: Choke servo      │      └────────────┘ │
│     ├── CH1: Throttle servo   │                     │
│     ├── CH2: Belt actuator    │                     │
│     └── CH3: Spare            │                     │
│                               │                     │
│  ┌──────────┐ I2C addr 0x48   │                     │
│  │ ADS1115  ├─────────────────┘                     │
│  │ 16-bit   │                                       │
│  │ 4-ch ADC │                                       │
│  └──┬───────┘                                       │
│     │ Analog inputs                                 │
│     ├── A0: Battery voltage (24V via divider)       │
│     ├── A1: Alternator current (hall/shunt)         │
│     ├── A2: Fuel level (resistive float sender)     │
│     └── A3: Spare                                   │
│                                                     │
│  ┌───────────────────────────┐                      │
│  │ Opto-isolated relay       │  GPIO 17, 25, 27     │
│  │ drivers (ULN2003 + opto)  │                      │
│  └──┬────────────────────────┘                      │
│     ├── Relay 1: Engine run (magneto kill)           │
│     ├── Relay 2: Engine start (starter solenoid)     │
│     └── Relay 3: Blade relay                        │
│                                                     │
│  ┌───────────────────────────┐                      │
│  │ Opto-isolated PWM         │  GPIO 12,13,16,20,   │
│  │ pass-through (TLP2361 x8) │  21,23,24,26         │
│  └──┬────────────────────────┘                      │
│     ├── Left motor:  RPWM, LPWM, R_EN, L_EN        │
│     └── Right motor: RPWM, LPWM, R_EN, L_EN        │
│                                                     │
│  ┌───────────────────────────┐                      │
│  │ RPM tach input            │  GPIO 4 (interrupt)  │
│  │ Schmitt trigger + opto    │                      │
│  └───────────────────────────┘                      │
│                                                     │
│  ┌───────────────────────────┐                      │
│  │ Connectors                │                      │
│  ├── J1: CAN bus (4-pin)     │                      │
│  ├── J2: Motors (10-pin)     │                      │
│  ├── J3: Relays (6-pin)      │                      │
│  ├── J4: Servos (8-pin)      │                      │
│  ├── J5: Analog (8-pin)      │                      │
│  ├── J6: CHT thermocouple    │                      │
│  ├── J7: 24V power in        │                      │
│  └── J8: RPM tach (3-pin)    │                      │
└─────────────────────────────────────────────────────┘
```

## ICs — Bill of Materials

| IC | Function | Bus | Notes |
|----|----------|-----|-------|
| MCP251863T | CAN FD controller + transceiver | SPI CS0 | Same part used in FarmTRX ESP32 design |
| MAX31855 | K-type thermocouple digital converter | SPI CS1 | CHT 0-1024°C, cold junction compensated |
| PCA9685 | 16-channel 12-bit PWM driver | I2C 0x40 | 4 servos: choke, throttle, belt, spare |
| ADS1115 | 16-bit 4-channel ADC | I2C 0x48 | Battery V, alternator A, fuel level, spare |
| TLP2361 x8 | High-speed opto-isolators | GPIO pass-through | 20kHz PWM capable, BTS7960 motor signals |
| ULN2003A | Darlington relay driver | GPIO | 3 relay channels (run, start, blade) |
| LMR36520 or similar | 24V→5V/2A buck converter | — | Powers Pi from battery bus |
| 74HC14 | Schmitt trigger | — | Clean up RPM tach signal |

## Pin Mapping — Raspberry Pi 5

### SPI Bus (SPI0)
| Pi GPIO | Function |
|---------|----------|
| GPIO 10 (MOSI) | SPI data out |
| GPIO 9 (MISO) | SPI data in |
| GPIO 11 (SCLK) | SPI clock |
| GPIO 8 (CE0) | MCP251863T chip select |
| GPIO 7 (CE1) | MAX31855 chip select |
| GPIO 22 | MCP251863T interrupt |

### I2C Bus (I2C1)
| Pi GPIO | Function |
|---------|----------|
| GPIO 2 (SDA) | I2C data (PCA9685 + ADS1115) |
| GPIO 3 (SCL) | I2C clock |

### Opto-Isolated Motor PWM Pass-Through
| Pi GPIO | Signal | Motor |
|---------|--------|-------|
| GPIO 12 | RPWM | Left |
| GPIO 16 | LPWM | Left |
| GPIO 20 | R_EN | Left |
| GPIO 21 | L_EN | Left |
| GPIO 13 | RPWM | Right |
| GPIO 26 | LPWM | Right |
| GPIO 23 | R_EN | Right |
| GPIO 24 | L_EN | Right |

### Relay Drivers (via ULN2003A, opto-isolated)
| Pi GPIO | Function |
|---------|----------|
| GPIO 17 | Engine run relay (magneto kill) |
| GPIO 25 | Blade relay |
| GPIO 27 | Engine start relay (starter solenoid) |

### Inputs
| Pi GPIO | Function |
|---------|----------|
| GPIO 4 | RPM tach (via Schmitt trigger + opto) |

### Free GPIOs
| Pi GPIO | Status |
|---------|--------|
| GPIO 5 | Available |
| GPIO 6 | Available |
| GPIO 19 | Available |

## Analog Input Wiring

### Battery Voltage (ADS1115 A0)
- 24V max → 100K/33K voltage divider → 0-5.94V range
- ADS1115 gain = ±6.144V (FSR), 16-bit resolution

### Alternator Current (ADS1115 A1)
- ACS712 30A hall-effect current sensor or 0.001Ω shunt + INA219
- Monitor charging current to confirm alternator is working

### Fuel Level (ADS1115 A2)
- Resistive float sender (typical 10-180Ω or 0-90Ω)
- Voltage divider with known reference resistor
- Calibrate empty/full in software

## Connector Pinouts (TBD)

Use Molex Mini-Fit Jr or JST-VH for power connectors (24V, motor).
Use JST-XH or Dupont for signal connectors (servos, analog, tach).
Screw terminals for CAN bus (field-serviceable).

## Power Budget

| Load | Current (typ) |
|------|--------------|
| Raspberry Pi 5 | 1.5A @ 5V |
| PCA9685 + servos | 0.5A @ 5V (servos powered separately from 6V rail) |
| HAT logic | 0.1A @ 3.3V |
| Total from 24V bus | ~0.5A @ 24V (12W) |

Note: Servo power (6V) should come from a separate regulator, not the Pi 5V rail.

## Engine Control Sequence

### Cold Start (CHT < 30°C)
1. Choke servo → full choke
2. Throttle servo → half throttle
3. Run relay → close (magneto enabled)
4. Start relay → pulse 2-3 seconds
5. Monitor RPM — if no catch after 3s, release, wait 5s, retry (max 3 attempts)
6. RPM confirmed → gradually open choke over 10-20s as CHT rises
7. CHT > 60°C → choke fully open, throttle to mowing RPM

### Warm Start (CHT > 40°C)
1. Choke servo → open (no choke needed)
2. Throttle servo → half throttle
3. Run relay → close
4. Start relay → pulse 2-3 seconds
5. RPM confirmed → throttle to mowing RPM

### Shutdown
1. Throttle → idle
2. Wait 10s (cool-down idle)
3. Run relay → open (magneto kills engine)
4. Choke → open (reset for next start)

### Safety Limits
| CHT | Action |
|-----|--------|
| < 30°C | Cold start with choke |
| 30-60°C | Choke blending (proportional) |
| > 60°C | Normal operating temp |
| > 200°C | Warning — reduce throttle to idle |
| > 250°C | Emergency shutdown |

## Future CAN Migration

When motor control moves to CAN nodes (Pico/STM32 at each BTS7960):
- Remove 8x opto-isolator pass-throughs from HAT
- Motor commands go over CAN bus instead
- GPIO 12/13/16/20/21/23/24/26 become free
- HAT becomes: CAN + thermocouple + servos + relays + ADC + power
- Encoder feedback from motor nodes also comes back over CAN

## Design Notes

- 2-layer PCB, Pi HAT form factor (65mm x 56.5mm)
- 40-pin GPIO header pass-through (stackable)
- Ground plane on bottom layer — critical for CAN signal integrity
- Keep SPI traces short — MCP251863T and MAX31855 near Pi header
- TVS diodes on CAN bus, relay outputs, and 24V input
- All opto-isolators need isolated power on the field side (separate 3.3V or 5V rail from 24V buck)
- Mounting holes match Pi HAT spec (M2.5, 58mm x 49mm)
- **EDA**: KiCad, use JLCPCB standard parts library for SMT assembly
- **Fab**: JLCPCB — PCB + SMT assembly in one order
