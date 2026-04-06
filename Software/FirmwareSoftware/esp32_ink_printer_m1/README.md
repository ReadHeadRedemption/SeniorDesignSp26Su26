# ESP32 Ink Printer Milestone 1 (ESP-IDF)

This is a **starter firmware** for the first milestone of your conductive-ink Ender-3 conversion:

- Home X/Y/Z using end switches
- Manually jog X/Y/Z/E from the serial console
- Control a bed heater with thermistor feedback and simple safety checks
- Mount an SD card over SPI and run a G-code file from it
- Execute a **small G-code subset**: `G0/G1/G28/G90/G91/G92/M105/M140/M190/M112/M18/M84`

## Important safety note

This code is a **starter**, not production firmware. Because your bed heater is mains-powered via an SSR, you should add:

- a thermal fuse or independent hardware cutoff
- proper mains isolation and creepage/clearance on the PCB
- a correctly characterized thermistor and resistor network
- final thermal-runaway testing before real printing

## What this code intentionally does not do yet

- Touchscreen UI
- Z-probe calibration routine
- RMT-based step generation
- Non-blocking motion planner / lookahead
- PID heater control (this uses starter bang-bang control)
- Pause/resume job recovery

## Assumptions you must review

### 1. Pin assignments
At the top of `main/app_main.c`, replace the GPIO pin definitions with your real hardware pins.

### 2. Thermistor network
The starter conversion assumes:

- 100K NTC thermistor
- Beta 3950
- 4.7K series resistor to 3.3V

If your bed sensor circuit differs, update `thermistor_c_from_adc_mv()`.

### 3. Switch polarity
The code assumes the end switches and probe are **active-low** with pull-ups enabled.

### 4. Axis scaling
Replace `STEPS_PER_MM_X/Y/Z/E` with your real values.

## Build

From your ESP-IDF shell:

```bash
idf.py set-target esp32
idf.py build
idf.py flash monitor
```

Use your actual ESP32 target if it is not the classic ESP32.

## Serial console commands

After boot, type these in the serial monitor:

```text
status
home
jog X10 F1200
jog Y-5 F1200
jog Z2 F600
jog E1.0 F300
temp 60
run /sdcard/test.gcode
stop
```

You can also send supported G-code directly, for example:

```text
G28
G90
G1 X10 Y10 F1200
M140 S60
M190 S60
M105
```

## Example G-code file

Save this as `/sdcard/test.gcode`:

```gcode
; simple motion test
G28
G90
M140 S50
M190 S50
G1 X10 Y10 Z2 F1200
G1 X100 Y10 F1800
G1 X100 Y100 F1800
G1 X10 Y100 F1800
G1 X10 Y10 F1800
M140 S0
M84
```

## Recommended next steps

1. Replace the blocking software step pulses with an RMT-based step generator.
2. Add a real UI task for your SPI TFT touchscreen.
3. Add a Z-probe calibration routine and store the offset on the SD card.
4. Move configuration into a file on SD instead of compile-time constants.
