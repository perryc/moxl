#!/usr/bin/env python3
"""Generate pinout tables DXF for MOXL HAT bottom silk layer.

Two tables side by side:
  Left:  GPIO map (pin → function → connector)
  Right: Connector pinouts (J# → pin1, pin2, ...)
"""

TEXT_H = 0.7       # mm text height
ROW_H  = 1.1       # mm row spacing
GAP    = 3.0       # mm gap between the two tables
MIRROR = False     # KiCad handles bottom silk mirroring

# ── GPIO table data ──────────────────────────────────────────────
GPIO_COLS = [5, 10, 5]  # GPIO | Function | Conn
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

# ── Connector table data ─────────────────────────────────────────
# Each entry: (connector_label, [pin1, pin2, ...])
CONN_COLS = [5, 4, 11]  # Conn | Pin# | Signal
CONNECTORS = [
    ("J1  VH 2p",  ["GND", "+12V_RAW"]),
    ("J2  XH 4p",  ["CANH", "CANL", "CAN_12V", "GND"]),
    ("J3  IDC 2x4", ["L_RPWM", "L_LPWM", "L_REN", "L_LEN",
                      "L_RIS", "L_LIS", "VCC", "GND"]),
    ("J4  IDC 2x4", ["R_RPWM", "R_LPWM", "R_REN", "R_LEN",
                      "R_RIS", "R_LIS", "VCC", "GND"]),
    ("J5  XH 5p",  ["RLY_RUN", "RLY_START", "RLY_BLADE", "RLY_AUX", "GND"]),
    ("J6  XH 8p",  ["CHKTHRTL", "SERVO1", "SERVO2", "SERVO3",
                     "6V", "6V", "GND", "GND"]),
    ("J7  PH 5p",  ["BATT_V", "ALT_I", "FUEL", "PACK_V", "GND"]),
    ("J8  PH 5p",  ["RPM_SIG", "ENC_L", "ENC_R", "5V", "GND"]),
    ("J9  XH 2p",  ["TC+", "TC-"]),
    ("J11 PH 3p",  ["RS232_TX", "RS232_RX", "GND"]),
    ("J12 PH 3p",  ["SBUS_IN", "5V", "GND"]),
]


# ── DXF helpers (R12 ASCII) ──────────────────────────────────────
def dxf_line(x1, y1, x2, y2):
    if MIRROR:
        x1, x2 = -x1, -x2
    return (f"0\nLINE\n8\n0\n"
            f"10\n{x1:.3f}\n20\n{y1:.3f}\n30\n0\n"
            f"11\n{x2:.3f}\n21\n{y2:.3f}\n31\n0\n")


def dxf_text(x, y, h, s):
    if MIRROR:
        x = -x
        gen_flag = "71\n2\n"  # backward text (mirrored X)
    else:
        gen_flag = ""
    return (f"0\nTEXT\n8\n0\n"
            f"10\n{x:.3f}\n20\n{y:.3f}\n30\n0\n"
            f"40\n{h:.3f}\n{gen_flag}1\n{s}\n")


def draw_table(out, x0, y_top, col_widths, headers, rows):
    """Draw a bordered table with header row.

    rows: list of tuples, each tuple has one string per column.
    Returns the total height consumed.
    """
    n_rows = len(rows) + 1  # +1 for header
    table_w = sum(col_widths)
    table_h = n_rows * ROW_H

    # Horizontal lines
    for i in range(n_rows + 1):
        y = y_top - i * ROW_H
        out.append(dxf_line(x0, y, x0 + table_w, y))

    # Double line under header
    out.append(dxf_line(x0, y_top - ROW_H + 0.2, x0 + table_w, y_top - ROW_H + 0.2))

    # Vertical lines
    x = x0
    for w in col_widths:
        out.append(dxf_line(x, y_top, x, y_top - table_h))
        x += w
    out.append(dxf_line(x0 + table_w, y_top, x0 + table_w, y_top - table_h))

    # Header text
    x = x0
    y = y_top - ROW_H + (ROW_H - TEXT_H) / 2
    for ci, hdr in enumerate(headers):
        out.append(dxf_text(x + 0.5, y, TEXT_H, hdr))
        x += col_widths[ci]

    # Data rows
    for ri, row in enumerate(rows):
        y = y_top - (ri + 2) * ROW_H + (ROW_H - TEXT_H) / 2
        x = x0
        for ci, cell in enumerate(row):
            if cell:
                out.append(dxf_text(x + 0.5, y, TEXT_H, cell))
            x += col_widths[ci]

    return table_h


# ── Build connector rows ─────────────────────────────────────────
# Split: J1-J3 go under GPIO table (left), J4-J12 on the right.
CONN_LEFT = CONNECTORS[:3]   # J1 (2p), J2 (4p), J3 (8p) = 14 rows
CONN_RIGHT = CONNECTORS[3:]  # J4-J12 = 39 rows


def flatten_connectors(conn_list):
    """Flatten connector list into table rows + separator indices."""
    rows = []
    separators = []
    for ci, (label, pins) in enumerate(conn_list):
        if ci > 0:
            separators.append(len(rows))
        for pi, pin_name in enumerate(pins):
            conn_label = label if pi == 0 else ""
            rows.append((conn_label, str(pi + 1), pin_name))
    return rows, separators


conn_left_rows, conn_left_seps = flatten_connectors(CONN_LEFT)
conn_right_rows, conn_right_seps = flatten_connectors(CONN_RIGHT)


# ── Assemble DXF ─────────────────────────────────────────────────
out = []
out.append("0\nSECTION\n2\nHEADER\n0\nENDSEC\n")
out.append("0\nSECTION\n2\nTABLES\n0\nENDSEC\n")
out.append("0\nSECTION\n2\nBLOCKS\n0\nENDSEC\n")
out.append("0\nSECTION\n2\nENTITIES\n")

# Compute heights
gpio_rows_data = [(str(g), f, c) for g, f, c in GPIO_PINS]
gpio_n_rows = len(gpio_rows_data) + 1
conn_left_n = len(conn_left_rows) + 1
conn_right_n = len(conn_right_rows) + 1

# Right column sets the overall height
right_h = conn_right_n * ROW_H
left_h = gpio_n_rows * ROW_H + 1.5 + conn_left_n * ROW_H  # GPIO + gap + J1-J3
y_top = max(right_h, left_h)

# Title
out.append(dxf_text(0, y_top + 1.5, 1.0, "Mo-XL Hat v1.0"))

# Top-left: GPIO map
gpio_h = draw_table(out, 0, y_top, GPIO_COLS,
                    ["GPIO", "FUNCTION", "J#"],
                    gpio_rows_data)

# Bottom-left: J1, J2, J3 (below GPIO table with small gap)
conn_left_y_top = y_top - gpio_h - 1.5
conn_left_h = draw_table(out, 0, conn_left_y_top, CONN_COLS,
                          ["CONN", "PIN", "SIGNAL"],
                          conn_left_rows)

# Separator lines in left connector table
for row_idx in conn_left_seps:
    y = conn_left_y_top - (row_idx + 1) * ROW_H
    out.append(dxf_line(0, y + 0.2, sum(CONN_COLS), y + 0.2))

# Right: J4-J12
conn_x0 = sum(GPIO_COLS) + GAP
conn_right_h = draw_table(out, conn_x0, y_top, CONN_COLS,
                           ["CONN", "PIN", "SIGNAL"],
                           conn_right_rows)

# Separator lines in right connector table
for row_idx in conn_right_seps:
    y = y_top - (row_idx + 1) * ROW_H
    out.append(dxf_line(conn_x0, y + 0.2, conn_x0 + sum(CONN_COLS), y + 0.2))

out.append("0\nENDSEC\n0\nEOF\n")

# Write file
dxf_path = "/Users/4perr/documents/github/moxl/hardware/moxl-hat/pinout-table.dxf"
with open(dxf_path, "w") as f:
    f.write("".join(out))

gpio_w = sum(GPIO_COLS)
conn_w = sum(CONN_COLS)
total_w = gpio_w + GAP + conn_w
left_total = gpio_h + 1.5 + conn_left_h
print(f"Written {dxf_path}")
print(f"GPIO table:     {gpio_w:.0f} x {gpio_h:.1f} mm  ({len(GPIO_PINS)} entries)")
print(f"Left connectors:  {conn_w:.0f} x {conn_left_h:.1f} mm  (J1-J3, {len(conn_left_rows)} pins)")
print(f"Right connectors: {conn_w:.0f} x {conn_right_h:.1f} mm  (J4-J12, {len(conn_right_rows)} pins)")
print(f"Left column:  {left_total:.1f} mm tall")
print(f"Right column: {conn_right_h:.1f} mm tall")
print(f"Total width:  {total_w:.0f} mm")
