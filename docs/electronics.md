# Bonus — Hardware integration (Raspberry Pi 5)

Answers to the electronics questions: how to wire the sensors to a Raspberry
Pi 5, how to power the system, and how to test each sensor outside ROS.

---

## 1. Connecting the sensors to a Raspberry Pi 5

The three sensor classes use three different interfaces. The Pi 5 exposes USB
3.0/2.0, a 40-pin GPIO header (UART/I²C/SPI), and two MIPI-CSI lanes.

| Sensor | Typical interface | Pi 5 connection |
|---|---|---|
| 2D lidar (e.g. RPLIDAR A1/A2) | USB serial (CP210x) or raw UART | **USB** is simplest: `/dev/ttyUSB0`. Or UART on GPIO14/15 (TXD/RXD) + 5 V/GND; the motor usually needs its own 5 V. |
| Depth camera (Orbbec Astra / RealSense) | USB 3.0 | **USB 3.0** port (blue). Needs the full 5 V/0.9 A USB budget — do not hang it off an unpowered hub. |
| GPS (u-blox NEO-M8N etc.) | UART (NMEA), sometimes I²C | **UART**: module TX→Pi RXD (GPIO15), module RX→Pi TXD (GPIO14), 3.3 V/5 V per module spec, GND common. PPS (if present) → a spare GPIO for precise timing. |

Notes / good practice:

- **One UART per device.** The Pi 5 has several PL011 UARTs; enable extra ones
  via `dtoverlay=uartN` in `/boot/firmware/config.txt` so the lidar (if UART)
  and GPS don't contend for the single primary UART. Prefer USB for the lidar to
  avoid this entirely.
- **Logic levels.** Pi GPIO is **3.3 V, not 5 V tolerant.** Use modules with
  3.3 V logic or a level shifter on any 5 V TX line feeding a Pi RX pin.
- **I²C** (if the GPS/IMU uses it): SDA=GPIO2, SCL=GPIO3, with the usual pull-ups
  (often on the breakout). Give each device a distinct address.
- **Grounding.** All devices must share a **common ground** with the Pi; a
  floating ground is the most common cause of flaky serial/USB data.
- **CSI camera alternative.** A Pi Camera (CSI) can emulate the RGB stream, but
  depth sensors (Astra/RealSense) are USB — use the USB 3.0 port for bandwidth.

---

## 2. Power architecture

The Pi 5 and the motors/sensors should be powered from **separate, regulated
rails off a common battery**, never by chaining the motors through the Pi.

```
            ┌──────────── Battery pack (e.g. 3S Li-ion / LiPo, 11.1 V nominal) ─────────────┐
            │                                                                                │
            │ main switch + fuse                                                             │
            ▼                                                                                ▼
   ┌─────────────────┐                                                        ┌───────────────────────┐
   │ Buck 5 V / 5 A  │  ── USB-C PD or GPIO 5 V ──►  Raspberry Pi 5           │ Motor driver (H-bridge│
   │ (logic rail)    │                               + USB sensors            │ / VESC) on raw battery│
   └─────────────────┘  ── 5 V / 3.3 V ──►  lidar, GPS, IMU                   │ voltage, own fuse     │
                                                                              └───────────────────────┘
```

Key points:

- **Dedicated 5 V buck for the Pi 5.** The Pi 5 wants **5 V @ 5 A (USB-C PD)**;
  under-powering it causes brownouts and SD corruption. Size the buck ≥ 5 A.
- **Separate motor rail.** Motors draw large, noisy, spiky currents. Feed the
  motor driver from the raw battery (or its own buck), **not** from the Pi's
  5 V rail, to keep logic-side voltage clean.
- **Common ground, star topology.** Tie all grounds at one point near the
  battery to avoid ground loops.
- **Protection.** Main fuse + per-branch fuses; a reverse-polarity diode/MOSFET;
  for LiPo, a low-voltage cutoff/alarm to protect cells.
- **Decoupling / inrush.** Bulk capacitors on the motor rail; a flyback path in
  the H-bridge; keep motor and signal wiring physically separated.
- **Clean shutdown.** Optionally a supercap/UPS HAT so the Pi can shut down
  gracefully on power loss (protects the filesystem).

Rough budget: Pi 5 ~5–12 W, lidar ~2.5 W, depth camera ~2–3 W, GPS <0.5 W, plus
motors (sized to the drivetrain). Pick the battery for runtime × total load with
margin.

---

## 3. Testing sensors standalone (outside ROS)

Validate each sensor at the OS level **before** bringing up ROS — it isolates
wiring/driver problems from ROS configuration problems.

**Lidar (USB serial)**
```bash
ls -l /dev/ttyUSB*                       # device present?
# Vendor SDK demo (RPLIDAR): prints distances/RPM
./ultra_simple /dev/ttyUSB0 115200
# Or raw bytes to confirm data flows:
cat /dev/ttyUSB0 | xxd | head
```

**GPS (UART/NMEA)**
```bash
# Raw NMEA sentences ($GPGGA, $GPRMC ...):
stty -F /dev/serial0 9600 && cat /dev/serial0
# Human-readable with gpsd:
sudo apt install gpsd gpsd-clients
gpsd /dev/serial0 -F /var/run/gpsd.sock
cgps -s                                  # live fix, satellites, HDOP
```

**Depth camera (USB)**
```bash
lsusb                                    # camera enumerated on the bus?
v4l2-ctl --list-devices                  # /dev/video* nodes
# Orbbec/RealSense vendor viewers:
OpenNI2 / astra viewer   # Astra
realsense-viewer         # Intel RealSense
```

**I²C / GPIO sanity (if used)**
```bash
sudo raspi-config          # enable I²C / SPI / serial interfaces
i2cdetect -y 1             # list devices + addresses on bus 1
pinout                     # confirm the Pi's physical pin map
```

**General checklist**
- Confirm the device **enumerates** (`dmesg | tail`, `lsusb`, `ls /dev/tty*`).
- Confirm **data flows** with a raw dump before trusting any driver.
- Check **permissions** — add the user to `dialout` (serial) / `video` (camera)
  groups, or data reads will fail silently.
- Verify **voltage at the connector** with a multimeter under load; brownouts
  look like intermittent sensor dropouts.
- Only after all three sensors pass standalone, bring up the ROS drivers and
  confirm topics with `ros2 topic hz /scan`, `ros2 topic echo /gps/fix`, etc.
