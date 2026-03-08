# MOXL HAT — Custom Raspberry Pi 5 I/O Board

Purpose-built HAT for the Mo-XL autonomous mower. Single board replaces
direct GPIO wiring with isolated, protected I/O. All commodity parts.

## Block Diagram

```
                        MOXL HAT
┌─────────────────────────────────────────────────────┐
│                                                     │
│  ┌──────────┐ SPI CS0    ┌──────────┐               │
│  │MCP251863T├────────────┤ SPI Bus  │  ┌──────────┐ │    12V (center tap)
│  │CAN + XCVR│            │          ├──┤ Pi 5 SPI │◄├───────────┐
│  └────┬─────┘            │          │  └──────────┘ │           │
│       │ CAN H/L          │          │               │   ┌───────┴──────┐
│       ▼                  ├──────────┤               │   │ 12V→5V Buck  │
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
│     │ Servo PWM                │      │            │ │
│     ├── CH0: Choke/throttle   │      └────────────┘ │
│     │   (combined single lever)│                     │
│     ├── CH1: Spare            │                     │
│     ├── CH2: Spare            │                     │
│     └── CH3: Spare            │                     │
│                               │                     │
│  ┌──────────┐ I2C addr 0x48   │                     │
│  │ ADS1115  ├─────────────────┘                     │
│  │ 16-bit   │                                       │
│  │ 4-ch ADC │                                       │
│  └──┬───────┘                                       │
│     │ Analog inputs                                 │
│     ├── A0: Battery voltage (12V via divider)        │
│     ├── A1: Alternator current (hall/shunt)         │
│     ├── A2: Fuel level (resistive float sender)     │
│     └── A3: Pack voltage (24V via divider)           │
│                                                     │
│  ┌───────────────────────────┐                      │
│  │ Opto-isolated relay       │  GPIO 17, 25, 27 +   │
│  │ drivers (ULN2003 + opto)  │                      │
│  └──┬────────────────────────┘                      │
│     ├── Relay 1: Engine run (magneto + fuel solenoid)│
│     ├── Relay 2: Engine start (starter solenoid)     │
│     └── Relay 3: Blade engage (belt actuator)       │
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
│  │ Connectors (JST)          │                      │
│  ├── J1: 12V power (VH 2p)  │                      │
│  ├── J2: CAN bus (XH 4p)    │                      │
│  ├── J3: Left motor (XH 6p) │                      │
│  ├── J4: Right motor (XH 6p)│                      │
│  ├── J5: Relays (XH 5p)     │                      │
│  ├── J6: Servos (XH 8p)     │                      │
│  ├── J7: Analog (PH 5p)     │                      │
│  ├── J8: RPM tach (XH 3p)   │                      │
│  ├── J9: Thermocouple (XH 2p)│                     │
│  ├── J11: RS-232 (PH 3p)    │                     │
│  └── J12: SBUS RC (PH 3p)   │                     │
└─────────────────────────────────────────────────────┘
```

## ICs — Bill of Materials

| IC | Function | Bus | Notes |
|----|----------|-----|-------|
| MCP251863T | CAN FD controller + transceiver | SPI CS0 | Same part used in FarmTRX ESP32 design |
| MAX31855 | K-type thermocouple digital converter | SPI CS1 | CHT 0-1024°C, cold junction compensated |
| PCA9685 | 16-channel 12-bit PWM driver | I2C 0x40 | CH0: choke/throttle (combined lever), CH1-3: spare |
| ADS1115 | 16-bit 4-channel ADC | I2C 0x48 | Battery V, alternator A, fuel level, pack V |
| TLP293-4 x2 | Quad opto-isolators (SO16) | GPIO pass-through | 20kHz PWM capable, BTS7960 motor signals |
| ULN2003A | Darlington relay driver | GPIO | 4 relay channels (run+fuel sol, start, blade, belt) |
| LMR33630 or similar | 12V→6V/3A buck converter | — | Main power rail from 12V center tap |
| LDO 5V (≤0.7V dropout, 2A) | 6V→5V linear regulator | — | Pi + HAT logic power, SOT-223/DPAK (2W dissipation) |
| MAX3221 | RS-232 line driver/receiver | UART0 | 3V logic to RS-232 levels (via DNP selector resistors) |
| CAT24C256 | HAT ID EEPROM (DNP) | I2C0 | Pi HAT auto-configuration, optional |
| 74HC14 | Hex Schmitt trigger inverter | — | Gate 1: RPM tach cleanup, Gate 2: SBUS uninvert |

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
| GPIO 17 | Engine run relay (magneto kill + fuel solenoid) — ULN2003A IN1 |
| GPIO 25 | Blade engage relay (belt tensioner linear actuator) — ULN2003A IN3 |
| GPIO 27 | Engine start relay (starter solenoid, momentary) — ULN2003A IN2 |
| GPIO 5 | Aux relay — ULN2003A IN4 |

### Inputs
| Pi GPIO | Function |
|---------|----------|
| GPIO 4 | RPM tach (via Schmitt trigger + opto) |

### Free GPIOs
| Pi GPIO | Status |
|---------|--------|
| GPIO 6 | Available |
| GPIO 18 | Available (hardware PWM0 capable) |
| GPIO 19 | Available |

## Analog Input Wiring

### Bottom Battery Voltage (ADS1115 A0)
- GND to center tap (bottom 12V battery)
- 100K / 27K voltage divider (0.1% tolerance recommended)
- 12.0V nominal → 2.55V, 14.4V charging → 3.06V (under 3.3V VDD limit)
- V_battery = V_adc × (127K / 27K) = V_adc × 4.704
- ADS1115 gain = ±4.096V, 16-bit resolution

### Alternator Current (ADS1115 A1)
- ACS712 30A hall-effect current sensor or 0.001Ω shunt + INA219
- Monitor charging current to confirm alternator is working

### Fuel Level (ADS1115 A2)
- Resistive float sender (typical 10-180Ω or 0-90Ω)
- Voltage divider with known reference resistor + VREF (3.3V via polyfuse)
- Calibrate empty/full in software

### Full Pack Voltage (ADS1115 A3)
- GND to +24V (full series pack)
- 100K / 10K voltage divider (0.1% tolerance recommended)
- 24.0V nominal → 2.18V, 28.8V charging → 2.62V (under 3.3V VDD limit)
- V_pack = V_adc × (110K / 10K) = V_adc × 11.0
- Top battery voltage = V_pack - V_bottom
- Alert if |V_top - V_bottom| > 0.5V (imbalance)

## Connector Pinouts

All connectors JST-XH except J1 (JST-VH for power), J7/J11/J12 (JST-PH).
Every XH connector has a unique pin count so nothing fits where it shouldn't.
J3/J4 are the only match (both 6-pin) — identical mirror circuits, swap just
reverses L/R steering. PH connectors can't plug into any XH slot, and J7 (5-pin PH)
can't plug into J11/J12 (3-pin PH) either.

UART0 (GPIO14/15) is shared between RS-232 (J11) and SBUS (J12) via 0603 0R
DNP selector resistors. Populate R1+R3 for RS-232, R4 for SBUS. Never both.

| Connector | Pins | Family | Signals |
|-----------|------|--------|---------|
| J1 | 2-pin | JST-VH | GND, +12V_RAW |
| J2 | 4-pin | XH | CANH, CANL, CAN_12V, GND |
| J3 | 6-pin | XH | L_RPWM, L_LPWM, L_REN, L_LEN, FIELD_5V, FIELD_GND |
| J4 | 6-pin | XH | R_RPWM, R_LPWM, R_REN, R_LEN, FIELD_5V, FIELD_GND |
| J5 | 5-pin | XH | RELAY_RUN, RELAY_START, RELAY_BLADE, RELAY_BELT, GND |
| J6 | 8-pin | XH | SERVO_CHKTHRTL, SERVO1, SERVO2, SERVO3, 6V, 6V, GND, GND |
| J7 | 5-pin | JST-PH | BATT_V, ALT_I, FUEL, PACK_V, GND |
| J8 | 3-pin | XH | RPM_SIG, 5V, GND |
| J9 | 2-pin | XH | TC+, TC- |
| J11 | 3-pin | JST-PH | RS232_TX, RS232_RX, GND (via R1+R3) |
| J12 | 3-pin | JST-PH | SBUS_IN, 5V, GND (via R4, 74HC14 inverter) |

## Power Tree

```
12V (J1, battery center tap)
 │
 F1 (2A polyfuse)
 │
 └─ 6V/3A Buck (LMR33630 or similar)
     ├─ PCA9685 V+ (servo power, 6V direct, 0.5A)
     ├─ J6 servo connector power pins (6V)
     └─ 5V/2A LDO (≤0.7V dropout, SOT-223/DPAK)
          ├─ Pi 5V (GPIO header pins 2/4, 1.5A)
          ├─ HAT logic ICs (3.3V from Pi)
          └─ Field-side opto power (5V)
```

## Power Budget

| Load | Voltage | Current (typ) |
|------|---------|--------------|
| Raspberry Pi 5 | 5V (from LDO) | 1.5A |
| PCA9685 + servo | 6V (from buck) | 0.5A |
| HAT logic ICs | 3.3V (from Pi) | 0.1A |
| Opto field side | 5V (from LDO) | 0.1A |
| **Total on 6V rail** | | **~2.5A peak** |
| **Total from 12V bus** | | **~1.5A @ 12V (18W)** |

LDO dissipation: (6V - 5V) × 2A = 2W max — SOT-223 or DPAK on ground plane.

## Engine Details

- **Engine**: Briggs & Stratton I/C 500cc, 14.5 HP (SAE J1940), vertical shaft
- **Engine Family**: 9BSXS.5002VP (2009 model year)
- **Rated speed**: ~3600 RPM no-load, ~3060 RPM full-load
- **Choke/throttle**: Combined single lever — idle → half → full → choke (one servo, PCA9685 CH0)
- **Fuel solenoid**: Anti-afterfire solenoid under carb float bowl (2-wire, 12V)
  - Energized = fuel flows, de-energized = fuel cutoff
  - Wire through engine run relay so magneto kill + fuel cutoff happen together

## Engine Control Sequence

Choke/throttle servo positions (PCA9685 CH0, single lever):
- **CHOKE**: max position (past full throttle detent)
- **FULL**: mowing RPM ~3000
- **HALF**: starting RPM
- **IDLE**: min position

### Cold Start (CHT < 30°C)
1. Servo → choke position
2. Run relay → close (magneto enabled + fuel solenoid opens)
3. Start relay → pulse 2-3 seconds
4. Monitor RPM — if no catch after 3s, release, wait 5s, retry (max 3 attempts)
5. RPM confirmed → gradually move servo from choke → full over 10-20s as CHT rises
6. CHT > 60°C → servo at full throttle (mowing RPM)

### Warm Start (CHT > 40°C)
1. Servo → half throttle
2. Run relay → close (magneto + fuel solenoid)
3. Start relay → pulse 2-3 seconds
4. RPM confirmed → servo to full throttle (mowing RPM)

### Shutdown
1. Servo → idle
2. Wait 10s (cool-down idle)
3. Run relay → open (magneto kills spark + fuel solenoid closes)
4. Servo → half throttle (reset for next start)
5. Set RTC wake alarm → shutdown Pi

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
- TVS diodes on CAN bus, relay outputs, and 12V input
- All opto-isolators need isolated power on the field side (separate 3.3V or 5V rail from 12V buck)
- Mounting holes match Pi HAT spec (M2.5, 58mm x 49mm)
- **EDA**: KiCad, use JLCPCB standard parts library for SMT assembly
- **Fab**: JLCPCB — PCB + SMT assembly in one order
