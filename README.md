# TC66 custom firmware

An open-source replacement firmware for the **RuiDeng TC66 / TC66C** USB-C
power meter, written from scratch in Rust. It installs through the meter's
own bootloader, which it never modifies, so you can always flash something
else, including the stock firmware.

> Unofficial. Not affiliated with or endorsed by RuiDeng. Flashing
> third-party firmware is at your own risk.

## What it does

- **Calibrated measurement.** Voltage, current, and power from the INA226,
  using your meter's own factory calibration. Sampling is paced by a
  hardware timer that reads nearly every conversion the INA226 makes, so how fast a host polls has no effect on how
  fresh the readings are. Also: NTC temperature, D+/D− voltages, and
  mAh/mWh energy counters with run time.
- **Screens.** Six screens, and K1 steps through them:
  1. V / A / W
  2. mAh, mWh, and run time
  3. temperature, D+, and D−
  4. a full-screen scrolling V/I plot
  5. USB-PD info: the charger's offers, the negotiated contract, live
     VBUS, the CC line and Type-C current, the cable's e-marker, and
     message counters. Short-press K2 for the next page (a bar on the
     right shows where you are).
  6. charger detection: short-press K2 to find out what the charger
     supports (see "Charger protocol detection" below). Results take a few
     pages; K2 steps through them, and K2 on the last page runs it again.

  Hold K2 for 1 s to reset the energy counters. The screen turns off
  after 15 s with no change in the readings, and any key wakes it.

  Switching to the PD screen starts PD sniffing (see below), and it keeps
  running until `tools/pd.py off`. While sniffing, the meter stops
  presenting itself as a USB-C sink, so a charger only supplies power while
  a device is plugged into the other side. If the meter starts up on the PD
  screen, sniffing stays off until you short-press K2.
- **Linux `sensors` support.** The meter identifies itself as a ChargerLAB
  POWER-Z KM003C, so the mainline Linux `powerz` hwmon driver binds to it.
  `sensors` then shows live V/I. It handles over 5000 reads/s.
- **Works with existing tools.** The stock USB serial protocol is
  supported: the encrypted `getva` packet, `gtrec` datalog download, and
  `rotat`. There is also a faster plaintext `rawva` command for streaming.
- **Bluetooth (TC66C).** The BLE module answers the stock app's
  measurement requests and our settings commands.
- **Settings stored in the stock EEPROM layout.** Includes brightness,
  rotation, screen timeout, plot scale, and datalog interval. Energy
  counters survive power loss. Stock and custom firmware can read each
  other's data.
- **Charge triggers.** USB-PD 2.0/3.0 sink with fixed PDOs and PPS,
  Quick Charge 2.0/3.0, and Samsung AFC. All of them are driven from the
  host: PD with `tools/pd.py`, QC with `tools/qc.py`, and AFC with the
  `afctr` command. **A granted request raises
  the voltage on the OUT port, up to 20 V on whatever is plugged in.**
  `pd.py` and `qc.py` refuse anything above 9 V unless you pass `--max-v`,
  and ask for confirmation unless you pass `--yes`.
- **Charger protocol detection.** Reports what the charger on the meter
  supports (Apple/Samsung dividers, QC, AFC, Type-C current, the full USB-PD
  offer) without ever raising the voltage — on the meter's detection screen
  (K2, ~10 s) or from a computer with `tools/detect.py` (`--device` runs
  the meter's own detection). Plug only the charger in: the meter refuses
  to run while current flows to the other port (over 30 mA) or a USB-C
  device is seen there, and it leaves PD off when done.
- **USB-PD sniffing.** Put a charger on one side and a phone or laptop on
  the other, and the meter records their whole PD conversation: offers,
  requests, accept/ready, vendor messages, and the charger's queries to an
  e-marked cable (whose vendor, current rating and speed get decoded). Plug
  and unplug events and corrupted packets are logged too, and the meter can
  make the pair renegotiate on command, so you don't have to re-plug. The
  PD screen shows the result live; `tools/pd.py info` prints it.
- **Firmware updates without the button.** Once this firmware is installed,
  it can reflash itself over USB.
- **Watchdog.** If the firmware hangs, the meter reboots.

### What it doesn't do (yet)

- There is no on-device menu. Settings are changed from a computer with
  `tools/cfg.py`.
- The on-device PD/QC trigger menu is missing. Triggers are host-driven
  only.
- RuiDeng's own PC and phone apps are only partly supported: live
  readings and screen rotation work, but page switching (next/previous
  screen) works over BLE only.

## Supported hardware

The RuiDeng **TC66C** (GD32F150G8 MCU) has been tested. The **TC66**
(without BLE) uses the same board design and should work, but nobody has
tested it yet. Other RuiDeng meters (UM25C, UM34C, TC64, …) use different
hardware and are **not** supported.

## Install

You need [uv](https://docs.astral.sh/uv/getting-started/installation/)
(it fetches Python and the dependencies itself) and a USB data cable to
the meter.

1. Download the latest `tc66fw-vN.enc.bin` from the
   [Releases](../../releases) page. Also download the flasher,
   [`tools/tc66flash.py`](tools/tc66flash.py).
2. Flash the image. The script walks you through putting the meter into
   its bootloader: unplug it, **hold K1** (the left button), plug it in,
   and release K1.
   ```
   uv run tc66flash.py flash tc66fw-vN.enc.bin
   ```
   Flashing takes about 10 s, and then the meter restarts. For about 2 s
   the boot splash shows `TC66` with the version number.
3. **Back up your EEPROM.** It holds your meter's factory calibration:
   ```
   uv run tc66flash.py backup-eeprom my-tc66-eeprom.bin
   ```
   The stock firmware has no way to read the EEPROM out, which is why this
   step comes after the first flash. This firmware never writes the
   calibration words; they are protected by a hard guard. Keep the file:
   it is specific to your meter.

`uv run tc66flash.py version` shows what the meter is running.

**Linux notes:** your user needs access to serial ports (usually the
`dialout` or `uucp` group). ModemManager can probe a newly connected
serial port in the middle of a flash and abort it. If a flash fails, run
`sudo systemctl stop ModemManager` and retry. Close any other program
that has the meter's port open.

## Going back to stock

The stock firmware is not redistributed here. The flasher downloads the
official image (V1.18) from RuiDeng's update server, checks its hash, and
flashes it the same way:

```
uv run tc66flash.py stock
```

Alternatively, use RuiDeng's Windows PC app. Hold K1 while you connect the
meter, then use its *firmware update* function.

## Recovery

The bootloader is never written, so a bad or interrupted flash can't brick
the meter. Hold K1 while you plug it in and flash again, either a release
image or stock.

## Using it

- **Settings:** `uv run tools/cfg.py` lists them, and
  `uv run tools/cfg.py set brightness 9` changes one. Changes apply
  immediately and are saved to the EEPROM about 2 s after the last change.
- **Live readings:** `sensors` (Linux `powerz` driver), or
  `tools/rawva.py` for lossless streaming.
- **Datalog:** `tools/gtrec.py` downloads the on-device datalog.
- **Charge triggers:** `tools/pd.py arm`, then `caps` and `req <n>`. QC
  uses `tools/qc.py 9`. Read the warnings in each tool's `--help` first.
- **Charger protocol detection:** `tools/detect.py` reports what the
  charger supports (Apple/Samsung dividers, QC, AFC, Type-C current,
  USB PD capabilities) without raising the voltage. Unplug the other port
  first.
- **PD sniffing** (the meter's PD switch must be on):
  ```
  uv run tools/pdcap.py 60
  ```
  then plug in the device. If the meter's PD engine is off, `pdcap.py`
  starts the sniffer itself (as `uv run tools/pd.py sniff` does). `uv run tools/pd.py hardreset` makes the charger
  and device renegotiate from scratch, and `uv run tools/pd.py off` ends
  sniffing. Notes:
  - **If `hardreset` does nothing,** check `pd.py status`. On our test unit
    it only worked with `cc=1`: that unit's CC2 contact on one port is
    barely connected (probably a bad solder joint). Flipping one of the
    cables moves the PD line to CC1.
  - **Permissions:** `pdcap.py` opens the meter's USB device directly.
    Install the udev rule once so it works without root:
    `sudo cp tools/udev/70-tc66.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger`
    (then replug the meter if needed). The rule gives access to the
    `plugdev` group and the logged-in desktop user.
  - **Missed messages:** the meter's PD chip can miss a frame when several
    arrive within about a millisecond, typically around the Accept.
  - **After a reboot:** the chip keeps its sniff setup through a meter
    reboot, so run `pd.py off` if the charger stops powering the other port.

The serial protocol, packet formats, and the full command list are in
[docs/HARDWARE.md](docs/HARDWARE.md).

## Building from source

See [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md). Short version: you need the
pinned Rust nightly (installed by rustup from `rust/rust-toolchain.toml`),
`arm-none-eabi` binutils, and Python 3 with `cryptography`. Then:

```
cd rust
make release   # -> tc66fw-release.enc.bin, flash with tools/tc66flash.py
make flash     # dev build, reflashes a meter already running this firmware
```

Every build runs static gates for the boot path, the stack budget, and code
placement. CI runs them on every push.

| Document | Contents |
|---|---|
| [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) | build, feature sets, versioning, debugging, status |
| [docs/HARDWARE.md](docs/HARDWARE.md) | board reference, memory map, peripherals, protocols, calibration |
| [docs/FIRMWARE.md](docs/FIRMWARE.md) | firmware internals and the boot-path rules |
| [docs/QUIRKS.md](docs/QUIRKS.md) | measured GD32F150 misbehavior |
| [docs/CHARGER_PROTOCOLS.md](docs/CHARGER_PROTOCOLS.md) | charger detection and PD sniffing: each protocol, what the meter can learn, tools |

## License

MIT; see [LICENSE](LICENSE). The bundled font
`tools/dev/font/inb.otf` (Inconsolata LGC Bold) is under the SIL Open Font
License; see [tools/dev/font/OFL.txt](tools/dev/font/OFL.txt). TC66 and RuiDeng are
trademarks of their owners.
