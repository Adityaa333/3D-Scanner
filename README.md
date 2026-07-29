# DIY 3D Scanner

A low-cost rotary 3D scanner built on an Arduino Nano, two NEMA-17 steppers, and a Sharp-style analog IR distance sensor. Raw scan data is logged to an SD card, then post-processed in MATLAB/Octave into a printable STL mesh.

## How it works

1. **Scan** — `scannerCode.ino` runs on the Arduino. For each Z slice, it rotates the turntable through a full revolution, taking an averaged distance reading at every angular step and logging it to `SCAN.TXT` on the SD card. A `9999` delimiter line marks the end of each slice, then the sensor carriage steps up and the next slice begins.
2. **Process** — `processScanDistance.m` reads the raw text file, cleans outliers out of each slice, converts the polar (angle, radius) readings into XYZ points, lightly smooths/resamples the mesh, plots it, and exports it as an STL file via `surf2stl.m`.

## Hardware

- Arduino Nano
- 2x NEMA-17 stepper motors + A4988 driver modules
  - Motor A: turntable rotation (theta axis)
  - Motor B: sensor carriage height (Z axis)
- Sharp-style analog IR distance sensor (e.g. GP2Y0A21YK0F)
- SD card module (SPI)

### Pin mapping (`scannerCode.ino`)

| Function | Pin |
|---|---|
| Theta STEP | D3 |
| Theta DIR | D4 |
| Z STEP | D5 |
| Z DIR | D6 |
| Stepper enable (shared, active LOW) | D7 |
| SD card CS | D10 |
| IR sensor analog input | A0 |

## Files

| File | Purpose |
|---|---|
| `scannerCode.ino` | Arduino firmware: drives the scan and writes `SCAN.TXT` to the SD card |
| `processScanDistance.m` | Reads `SCAN.TXT`, filters/cleans it, builds the mesh, plots it, exports STL |
| `surf2stl.m` | Helper: writes an X/Y/Z surface grid out as an ASCII or binary STL file |


## Configuration

**Scan parameters** (`scannerCode.ino`):
- `STEPS_PER_REV` / `MICROSTEP_MULT` — angular resolution per revolution
- `Z_STEPS_PER_SLICE`, `NUM_SLICES` — vertical resolution and scan height
- `SAMPLES_PER_READING` — ADC samples averaged per distance reading
- `CAL_C3..CAL_C0` — cubic calibration coefficients mapping sensor voltage to distance (recalibrate for your specific sensor)

**Processing parameters** (`processScanDistance.m`):
- `zStepHeightMm` — physical height advanced per slice (should match the hardware's real-world Z step size)
- `outlierWindow` / `outlierThreshold` — median-filter window and rejection threshold for cleaning noisy readings
- `resampleAngles` — angular samples per slice after resampling
- `smoothSpan` — fraction of points used for mesh smoothing

## Notes

- The sensor calibration in `scannerCode.ino` (`CAL_C3`–`CAL_C0`) is specific to one sensor unit and distance range — refit with your own measurements (e.g. `polyfit` against known distances) for accurate results.
- `zStepHeightMm` in `processScanDistance.m` must match the physical distance the Z stepper actually travels per `Z_STEPS_PER_SLICE` — otherwise the exported model will be stretched or squashed vertically.
- `surf2stl.m` skips any grid cell touching a `NaN` vertex, so gaps in cleaned data simply produce holes in the mesh rather than crashing the export.

## References : 
1. Sensor Datasheet : https://www.alldatasheet.com/view.jsp?Searchword=Gp2y0a21yk0f
2. Code built upon : https://github.com/SuperMakeSomething/diy-3d-scanner
