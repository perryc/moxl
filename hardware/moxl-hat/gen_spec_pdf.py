#!/usr/bin/env python3
"""Generate Mo-XL Hat v1.0 spec sheet PDF."""

from fpdf import FPDF

# ── Data ──────────────────────────────────────────────────────────

FEATURES = [
    ("Power",       "Hardened 12V input (TVS + EMI filter + polyfuse) powers Pi 5 and all HAT "
                    "circuitry via 5.88V/3A buck + 5.05V/2A LDO. No external Pi power supply needed."),
    ("Motor Drive", "2 opto-isolated BTS7960 channels (8 signals: RPWM, LPWM, R_EN, L_EN per "
                    "channel). 20 kHz PWM capable. IDC ribbon cable to BTS7960 modules."),
    ("CAN / CAN-FD","MCP251863T integrated controller + transceiver on SPI. "
                    "ISO 11898 compliant, TVS protected, 4-pin JST-XH bus connector."),
    ("RS-232",      "MAX3221 level shifter on UART0. 3-pin JST-PH connector. "
                    "TX always active. RX shared with SBUS via DNP resistor selection."),
    ("SBUS",        "RC receiver input via 74HC14 inverter on UART0 RX. "
                    "3-pin JST-PH with 5V receiver power. Hardware selectable vs RS-232 RX "
                    "(DNP 0R resistors). Dead-man switch for manual override."),
    ("Servo PWM",   "PCA9685 16-channel 12-bit PWM driver (I2C). 4 channels broken out to "
                    "8-pin JST-XH with 5.88V servo power (2 power + 2 GND pins)."),
    ("Analog",      "ADS1115 16-bit 4-channel ADC (I2C). On-board resistor dividers sized for "
                    "12V and 24V battery monitoring. 2 channels for resistive/current sensors."),
    ("Thermocouple","MAX31855 K-type thermocouple interface on SPI. Cold-junction compensated. "
                    "0-1024C range. 2-pin JST-XH connector."),
    ("Relay Drivers","ULN2003A Darlington driver, 4 channels with flyback clamp to 12V. "
                    "500 mA per channel. 5-pin JST-XH connector."),
    ("Digital In",  "74HC14 hex Schmitt trigger with 4 conditioned inputs: "
                    "1 engine tach + 2 wheel encoder pulse counters + 1 SBUS. "
                    "GPIO interrupt capable. 5-pin JST-PH connector."),
    ("EEPROM",      "CAT24C256 HAT ID EEPROM on I2C0. Enables Pi auto-configuration "
                    "per Raspberry Pi HAT specification. Write-protect via solder jumper JP1."),
]

GPIO_PINS = [
    ( 2, "SDA (I2C)",   ""),
    ( 3, "SCL (I2C)",   ""),
    ( 4, "RPM TACH",    "J8"),
    ( 5, "RLY AUX",     "J5"),
    ( 6, "ENC LEFT",    "J8"),
    ( 7, "CS1 (TC)",    "J9"),
    ( 8, "CS0 (CAN)",   "J2"),
    ( 9, "MISO",        ""),
    (10, "MOSI",        ""),
    (11, "SCLK",        ""),
    (12, "L_RPWM",      "J3"),
    (13, "R_RPWM",      "J4"),
    (16, "L_LPWM",      "J3"),
    (17, "RLY RUN",     "J5"),
    (18, "(SPARE)",     ""),
    (19, "ENC RIGHT",   "J8"),
    (20, "L_REN",       "J3"),
    (21, "L_LEN",       "J3"),
    (22, "CAN INT",     ""),
    (23, "R_REN",       "J4"),
    (24, "R_LEN",       "J4"),
    (25, "RLY BLADE",   "J5"),
    (26, "R_LPWM",      "J4"),
    (27, "RLY START",   "J5"),
]

CONN_LEFT = [
    ("J1  VH 2p",      ["GND", "+12V_RAW"]),
    ("J2  XH 4p",      ["CANH", "CANL", "CAN_12V", "GND"]),
    ("J3  IDC 2x4",    ["L_RPWM", "L_LPWM", "L_REN", "L_LEN",
                         "L_RIS", "L_LIS", "VCC", "GND"]),
    ("J4  IDC 2x4",    ["R_RPWM", "R_LPWM", "R_REN", "R_LEN",
                         "R_RIS", "R_LIS", "VCC", "GND"]),
    ("J5  XH 5p",      ["RLY_RUN", "RLY_START", "RLY_BLADE", "RLY_AUX", "GND"]),
]

CONN_RIGHT = [
    ("J6  XH 8p",      ["CHKTHRTL", "SERVO1", "SERVO2", "SERVO3",
                         "6V", "6V", "GND", "GND"]),
    ("J7  PH 5p",      ["BATT_V", "ALT_I", "FUEL", "PACK_V", "GND"]),
    ("J8  PH 5p",      ["RPM_SIG", "ENC_L", "ENC_R", "5V", "GND"]),
    ("J9  XH 2p",      ["TC+", "TC-"]),
    ("J11 PH 3p",      ["RS232_TX", "RS232_RX", "GND"]),
    ("J12 PH 3p",      ["SBUS_IN", "5V", "GND"]),
]

BOM = [
    ("MCP251863T",   "CAN FD controller + transceiver",     "SPI CS0",  "SSOP-28"),
    ("MAX31855",     "K-type thermocouple ADC",              "SPI CS1",  "SOIC-8"),
    ("PCA9685PW",    "16-ch 12-bit PWM driver",              "I2C 0x40", "TSSOP-28"),
    ("ADS1115",      "16-bit 4-ch ADC",                      "I2C 0x48", "TSSOP-10"),
    ("TLP293-4 x2",  "Quad opto-isolators (motor PWM)",      "GPIO",     "SO-16"),
    ("ULN2003A",     "Darlington relay driver (4 ch)",        "GPIO",     "SOP-16"),
    ("XL4015",       "12V to 5.88V/3A buck converter",       "",         "TO-263-5"),
    ("TPS7A7001DDA", "5.88V to 5.05V/2A LDO",               "",         "SO-PowerPAD-8"),
    ("MAX3221",      "RS-232 line driver/receiver",           "UART0",   "TSSOP-16"),
    ("74HC14D",      "Hex Schmitt trigger inverter",          "",         "SOIC-14"),
    ("CAT24C256",    "HAT ID EEPROM",                          "I2C0",    "TSSOP-8"),
]

POWER_BUDGET = [
    ("Raspberry Pi 5",     "5V (LDO)",   "1.5A"),
    ("PCA9685 + servo",    "6V (buck)",   "0.5A"),
    ("HAT logic ICs",      "3.3V (Pi)",   "0.1A"),
    ("Opto field side",    "5V (LDO)",    "0.1A"),
    ("Relay coils (4x)",   "12V",         "0.8A"),
    ("CAN transceiver",    "5V",          "0.07A"),
    ("Total from 12V",     "",            "~1.8A"),
]

ANALOG = [
    ("A0", "Bottom battery", "100K/27K divider", "V x 4.704"),
    ("A1", "Alternator I",   "ACS712 or shunt",  ""),
    ("A2", "Fuel level",     "Resistive sender",  "Calibrate in SW"),
    ("A3", "Full pack 24V",  "100K/10K divider",  "V x 11.0"),
]

TRACE_WIDTHS = [
    ("J1",  "+12V_RAW, GND",         "3A",    "Pour",        "Copper flood"),
    ("J2",  "CANH, CANL",            "70mA",  "0.2mm (8mil)","Diff pair"),
    ("J2",  "CAN_12V",               "50mA",  "0.2mm (8mil)",""),
    ("J3/4","Motor opto signals",     "20mA",  "0.2mm (8mil)","x8 signals"),
    ("J3/4","VCC, GND",              "100mA", "0.25mm (10mil)","Opto field pwr"),
    ("J5",  "RELAY_RUN/START/BLADE", "200mA", "0.3mm (12mil)","ULN2003A out"),
    ("J5",  "GND (4 relay return)",  "800mA", "0.5mm (20mil)",""),
    ("J6",  "Servo PWM signals",     "20mA",  "0.2mm (8mil)","x4 channels"),
    ("J6",  "6V power (x2 pins)",    "500mA", "0.5mm (20mil)","Servo stall"),
    ("J6",  "GND (x2 pins)",         "500mA", "0.5mm (20mil)",""),
    ("J7",  "Analog sense lines",    "<1mA",  "0.2mm (8mil)",""),
    ("J8",  "RPM, ENC_L, ENC_R",    "<5mA",  "0.2mm (8mil)",""),
    ("J8",  "5V, GND",              "50mA",  "0.2mm (8mil)","Sensor excite"),
    ("J9",  "TC+, TC-",             "<1mA",  "0.2mm (8mil)","Keep pair tight"),
    ("J11", "RS232_TX/RX, GND",     "10mA",  "0.2mm (8mil)",""),
    ("J12", "SBUS_IN, 5V, GND",     "50mA",  "0.2mm (8mil)",""),
]


# ── PDF Builder ───────────────────────────────────────────────────

class SpecPDF(FPDF):
    def header(self):
        if self.page_no() > 1:
            self.set_font("Helvetica", "I", 8)
            self.cell(0, 5, "Mo-XL Hat v1.0 Spec Sheet", align="L")
            self.cell(0, 5, f"Page {self.page_no()}", align="R", new_x="LMARGIN", new_y="NEXT")
            self.line(10, 12, 200, 12)
            self.ln(3)

    def footer(self):
        self.set_y(-12)
        self.set_font("Helvetica", "I", 7)
        self.cell(0, 5, "Mo-XL Autonomous Airstrip Mower  |  github.com/perryc/moxl", align="C")

    def section_title(self, title):
        self.set_font("Helvetica", "B", 11)
        self.set_fill_color(30, 30, 30)
        self.set_text_color(255, 255, 255)
        self.cell(0, 7, f"  {title}", fill=True, new_x="LMARGIN", new_y="NEXT")
        self.set_text_color(0, 0, 0)
        self.ln(2)

    def sub_title(self, title):
        self.set_font("Helvetica", "B", 9)
        self.cell(0, 5, title, new_x="LMARGIN", new_y="NEXT")
        self.ln(1)

    def body_text(self, text):
        self.set_font("Helvetica", "", 8)
        self.multi_cell(0, 4, text)
        self.ln(1)

    def mono_text(self, text):
        self.set_font("Courier", "", 7)
        for line in text.split("\n"):
            self.cell(0, 3.5, line, new_x="LMARGIN", new_y="NEXT")
        self.ln(1)

    def table(self, headers, rows, col_widths, header_bg=(70, 70, 70),
              stripe=True, font_size=7):
        self.set_font("Helvetica", "B", font_size)
        self.set_fill_color(*header_bg)
        self.set_text_color(255, 255, 255)
        for i, h in enumerate(headers):
            self.cell(col_widths[i], 5, h, border=1, fill=True, align="C")
        self.ln()
        self.set_text_color(0, 0, 0)
        self.set_font("Courier", "", font_size)
        for ri, row in enumerate(rows):
            if stripe and ri % 2 == 0:
                self.set_fill_color(240, 240, 240)
                fill = True
            else:
                fill = False
            for i, cell in enumerate(row):
                self.cell(col_widths[i], 4.5, str(cell), border=1, fill=fill)
            self.ln()
        self.ln(2)

    def feature_list(self, features):
        """Two-column feature table: bold label + description."""
        for label, desc in features:
            self.set_font("Helvetica", "B", 8)
            self.cell(22, 4, label, new_x="END")
            self.set_font("Helvetica", "", 7.5)
            self.multi_cell(0, 4, desc)
            self.ln(0.5)
        self.ln(2)

    def connector_column(self, connectors, x0, y0, col_widths):
        """Draw a connector pinout column at a given position. Returns y after."""
        self.set_xy(x0, y0)
        headers = ["Connector", "Pin", "Signal"]
        self.set_font("Helvetica", "B", 6.5)
        self.set_fill_color(70, 70, 70)
        self.set_text_color(255, 255, 255)
        for i, h in enumerate(headers):
            self.cell(col_widths[i], 4.5, h, border=1, fill=True, align="C")
        self.ln()
        self.set_x(x0)
        self.set_text_color(0, 0, 0)

        row_h = 4.0
        for ci, (label, pins) in enumerate(connectors):
            bg = (230, 240, 250) if ci % 2 == 0 else (245, 245, 245)
            self.set_fill_color(*bg)
            for pi, pin_name in enumerate(pins):
                if pi == 0:
                    self.set_font("Helvetica", "B", 6.5)
                    self.cell(col_widths[0], row_h, label, border=1, fill=True)
                else:
                    self.set_font("Courier", "", 6.5)
                    self.cell(col_widths[0], row_h, "", border=1, fill=True)
                self.set_font("Courier", "", 6.5)
                self.cell(col_widths[1], row_h, str(pi + 1), border=1, fill=True, align="C")
                self.cell(col_widths[2], row_h, pin_name, border=1, fill=True)
                self.ln()
                self.set_x(x0)
        return self.get_y()


# ── Build PDF ─────────────────────────────────────────────────────

pdf = SpecPDF(orientation="P", unit="mm", format="letter")
pdf.set_auto_page_break(auto=True, margin=15)

# ── Page 1: Title + Features + Power ─────────────────────────────
pdf.add_page()
pdf.set_font("Helvetica", "B", 24)
pdf.cell(0, 12, "Mo-XL Hat v1.0", new_x="LMARGIN", new_y="NEXT")
pdf.set_font("Helvetica", "", 10)
pdf.cell(0, 6, "Raspberry Pi 5 I/O HAT for Autonomous Vehicle Control",
         new_x="LMARGIN", new_y="NEXT")
pdf.set_font("Helvetica", "I", 8)
pdf.set_text_color(100, 100, 100)
pdf.cell(0, 5, "Designed for the Mo-XL autonomous airstrip mower  |  65 x 56.5 mm  |  "
         "6-layer ENIG  |  JLCPCB assembly",
         new_x="LMARGIN", new_y="NEXT")
pdf.set_text_color(0, 0, 0)
pdf.ln(1)
pdf.set_draw_color(0)
pdf.line(10, pdf.get_y(), 200, pdf.get_y())
pdf.ln(3)

# Features
pdf.section_title("Features")
pdf.feature_list(FEATURES)

# Power tree
pdf.section_title("Power Architecture")
pdf.mono_text(
    "12V (J1, battery center tap)\n"
    " |                               XL4015 Feedback: R1=10K, R2=2.7K\n"
    " F1 (3A polyfuse, 30V)           Output: 1.25 x (1 + 10K/2.7K) = 5.88V\n"
    " |\n"
    " +-- 5.88V / 3A Buck (XL4015, TO-263-5)\n"
    "      |-- PCA9685 servo power (5.88V direct, 0.5A)\n"
    "      |-- J6 servo connector power pins\n"
    "      +-- 5.05V / 2A LDO (TPS7A7001DDA)\n"
    "           |     Feedback: R27=274K, R26=30.1K\n"
    "           |     Output: 0.5 x (1 + 274K/30.1K) = 5.05V\n"
    "           |-- Pi 5V (GPIO header pins 2/4, 1.5A)\n"
    "           |-- HAT logic ICs (3.3V from Pi regulator)\n"
    "           +-- Field-side opto power (5V)"
)

# ── Page 2: BOM + Power Budget ───────────────────────────────────
pdf.add_page()

pdf.section_title("Bill of Materials  -  Key ICs")
pdf.table(
    ["IC", "Function", "Bus", "Package"],
    BOM,
    [32, 62, 25, 25],
    font_size=6.5,
)

pdf.section_title("Power Budget")
pdf.table(
    ["Load", "Rail", "Current"],
    POWER_BUDGET,
    [70, 40, 30],
)

pdf.section_title("GPIO Pin Map")
pdf.table(
    ["GPIO", "Function", "Connector"],
    [(str(g), f, c) for g, f, c in GPIO_PINS],
    [18, 55, 25],
)

# ── Page 3: Connector Pinouts (two columns) ──────────────────────
pdf.add_page()

pdf.section_title("Connector Pinouts")
pdf.body_text(
    "Connector families: J1 JST-VH (power), J3/J4 shrouded IDC 2x4 2.54mm "
    "(ribbon cable to BTS7960), J7/J8/J11/J12 JST-PH, all others JST-XH. "
    "Unique pin counts per family prevent cross-plugging. "
    "UART0 TX always active on RS-232 (J11). RX hardware-selectable between "
    "RS-232 RX and SBUS (J12) via DNP 0R resistors."
)

# Two columns side by side
col_w_left = [22, 8, 22]   # 52mm wide
col_w_right = [22, 8, 22]  # 52mm wide
col_start_y = pdf.get_y()
left_x = 10
right_x = 10 + sum(col_w_left) + 6  # 6mm gap

y_left = pdf.connector_column(CONN_LEFT, left_x, col_start_y, col_w_left)
y_right = pdf.connector_column(CONN_RIGHT, right_x, col_start_y, col_w_right)

pdf.set_y(max(y_left, y_right) + 2)

# ── Page 4: Analog + Trace Widths ────────────────────────────────
pdf.add_page()

pdf.section_title("Analog Inputs  (ADS1115, I2C 0x48)")
pdf.table(
    ["Channel", "Signal", "Interface", "Scale Factor"],
    ANALOG,
    [18, 35, 45, 35],
)

pdf.section_title("Trace Width Table  (1 oz Cu, 10C rise, outer layer)")
pdf.body_text("Power supply rails (12V, 5.88V, 5V) use copper pours. "
              "Traces below are for output connector routing only.")
pdf.table(
    ["Conn", "Signals", "I max", "Width", "Notes"],
    TRACE_WIDTHS,
    [14, 48, 18, 30, 30],
    font_size=6,
)

# ── Page 5: Engine Control ───────────────────────────────────────
pdf.add_page()

pdf.section_title("Engine Control Sequence")
pdf.body_text(
    "Engine: Briggs & Stratton I/C 500cc, 14.5 HP, 3600 RPM no-load. "
    "Combined choke/throttle single lever controlled by PCA9685 CH0 servo."
)

pdf.sub_title("Servo Positions (PCA9685 CH0)")
pdf.table(
    ["Position", "Description"],
    [("CHOKE", "Max position (past full throttle detent)"),
     ("FULL",  "Mowing RPM ~3000"),
     ("HALF",  "Starting RPM"),
     ("IDLE",  "Min position")],
    [30, 100],
)

pdf.sub_title("Cold Start (CHT < 30C)")
pdf.body_text(
    "1. Servo -> choke\n"
    "2. Run relay -> close (magneto + fuel solenoid)\n"
    "3. Start relay -> pulse 2-3s\n"
    "4. Monitor RPM - if no catch after 3s, wait 5s, retry (max 3)\n"
    "5. RPM confirmed -> choke to full over 10-20s as CHT rises\n"
    "6. CHT > 60C -> full throttle (mowing RPM)"
)

pdf.sub_title("Warm Start (CHT > 40C)")
pdf.body_text(
    "1. Servo -> half throttle\n"
    "2. Run relay -> close\n"
    "3. Start relay -> pulse 2-3s\n"
    "4. RPM confirmed -> full throttle"
)

pdf.sub_title("Shutdown")
pdf.body_text(
    "1. Servo -> idle\n"
    "2. Wait 10s (cool-down)\n"
    "3. Run relay -> open (kill spark + fuel cutoff)\n"
    "4. Servo -> half (reset for next start)\n"
    "5. Set RTC wake alarm -> shutdown Pi"
)

pdf.sub_title("CHT Safety Limits")
pdf.table(
    ["CHT", "Action"],
    [("< 30C",   "Cold start with choke"),
     ("30-60C",  "Choke blending (proportional)"),
     ("> 60C",   "Normal operating temperature"),
     ("> 200C",  "Warning: reduce to idle"),
     ("> 250C",  "Emergency shutdown")],
    [30, 100],
)

# ── Save spec pages to temp file, then merge with schematic ──────
import tempfile, os
from pypdf import PdfReader, PdfWriter

spec_tmp = tempfile.NamedTemporaryFile(suffix=".pdf", delete=False)
pdf.output(spec_tmp.name)
spec_pages = pdf.page_no()

# Merge: spec pages + schematic appendix
schematic_path = "/Users/4perr/documents/github/moxl/hardware/moxl-hat/Mo-XL_PiHat_Schematic.pdf"
out_path = "/Users/4perr/documents/github/moxl/hardware/moxl-hat/Mo-XL_Hat_v1.0_Spec.pdf"

writer = PdfWriter()

# Add all spec pages
spec_reader = PdfReader(spec_tmp.name)
for page in spec_reader.pages:
    writer.add_page(page)

# Add schematic page(s)
schem_reader = PdfReader(schematic_path)
for page in schem_reader.pages:
    writer.add_page(page)

writer.write(out_path)
os.unlink(spec_tmp.name)

total = len(spec_reader.pages) + len(schem_reader.pages)
print(f"Written {out_path}")
print(f"{spec_pages} spec pages + {len(schem_reader.pages)} schematic page = {total} total")
