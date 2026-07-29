/*
  scannerCode.ino
  ----------------
  Firmware for a low-cost 3D scanner built with:
    - Arduino Nano
    - 2x NEMA-17 stepper motors driven by A4988 modules
        - Motor A: rotates the turntable (theta axis)
        - Motor B: raises/lowers the IR sensor carriage (Z axis)
    - Sharp-style analog IR distance sensor (e.g. GP2Y0A21YK0F)
    - SD card module (SPI) for logging scan data

  Behaviour:
    - For every Z slice:
        - Step the turntable through a full revolution, pausing at each
          angular step to take an averaged distance reading and log it
        - After the full revolution, write a 9999 delimiter line to mark
          the end of that slice
        - Step the Z axis up by one slice height and repeat
    - The IR sensor is sampled several times per angular step and the
      analog readings are averaged before being converted to a distance
      using a cubic calibration fit (sensor voltage -> distance in cm).

  Output file format (SCAN.TXT on the SD card):
    distance_slice0_step0
    distance_slice0_step1
    ...
    distance_slice0_stepN
    9999
    distance_slice1_step0
    ...
    9999
    ...
*/

#include <SPI.h>
#include <SD.h>

// ---------------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------------

// Turntable stepper (A4988 #1)
const int THETA_STEP_PIN = 3;
const int THETA_DIR_PIN  = 4;

// Z-axis stepper (A4988 #2)
const int Z_STEP_PIN = 5;
const int Z_DIR_PIN  = 6;

// Shared enable line for both A4988 drivers (LOW = enabled)
const int STEPPER_ENABLE_PIN = 7;

// SD card chip-select
const int SD_CS_PIN = 10;

// IR distance sensor analog input
const int IR_SENSOR_PIN = A0;

// ---------------------------------------------------------------------
// Scan configuration
// ---------------------------------------------------------------------

const int STEPS_PER_REV       = 200;   // full steps per turntable revolution (1.8 deg/step)
const int MICROSTEP_MULT      = 1;     // set >1 if A4988 MS pins wired for microstepping
const int ANGULAR_STEPS       = STEPS_PER_REV * MICROSTEP_MULT;

const int Z_STEPS_PER_SLICE   = 20;    // Z steps to advance between slices
const int NUM_SLICES          = 50;    // total number of Z slices to scan

const int STEP_PULSE_US       = 800;   // delay between step pulses (speed control)
const int SETTLE_MS           = 40;    // pause after each move before sampling

const int SAMPLES_PER_READING = 25;    // number of ADC samples averaged per point

const char* OUTPUT_FILENAME   = "SCAN.TXT";
const long SLICE_DELIMITER    = 9999;

// ---------------------------------------------------------------------
// Sensor calibration
// ---------------------------------------------------------------------
// The Sharp-style IR sensor's output voltage relates to distance in a
// non-linear way. Distance (cm) is estimated from the averaged sensor
// voltage using a cubic polynomial fit obtained by measuring the sensor
// against known distances and fitting with polyfit() in MATLAB/Octave:
//
//   distance_cm = C3*V^3 + C2*V^2 + C1*V + C0
//
// Replace these coefficients with your own calibration for best accuracy.
const float CAL_C3 = 2.398f;
const float CAL_C2 = -18.567f;
const float CAL_C1 = 3.937f;
const float CAL_C0 = 68.219f;

const float ADC_REF_VOLTAGE = 5.0f;
const float ADC_MAX_COUNTS  = 1023.0f;

File scanFile;

// ---------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  pinMode(THETA_STEP_PIN, OUTPUT);
  pinMode(THETA_DIR_PIN, OUTPUT);
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);

  digitalWrite(STEPPER_ENABLE_PIN, LOW);   // enable both drivers
  digitalWrite(THETA_DIR_PIN, HIGH);       // rotation direction
  digitalWrite(Z_DIR_PIN, HIGH);           // "up" direction

  Serial.print(F("Initializing SD card..."));
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("failed."));
    while (true) { /* halt - no point scanning without storage */ }
  }
  Serial.println(F("done."));

  // Start with a fresh output file
  if (SD.exists(OUTPUT_FILENAME)) {
    SD.remove(OUTPUT_FILENAME);
  }

  runScan();

  Serial.println(F("Scan complete."));
  digitalWrite(STEPPER_ENABLE_PIN, HIGH);  // de-energize motors when done
}

void loop() {
  // Nothing to do after the scan finishes in setup().
}

// ---------------------------------------------------------------------
// Top-level scan routine: steps through all Z slices
// ---------------------------------------------------------------------
void runScan() {
  for (int slice = 0; slice < NUM_SLICES; slice++) {
    Serial.print(F("Scanning slice "));
    Serial.println(slice);

    scanOneRevolution();
    writeDelimiter();

    stepMotor(Z_STEP_PIN, Z_STEPS_PER_SLICE);
    delay(SETTLE_MS);
  }
}

// ---------------------------------------------------------------------
// Rotate the turntable through one full revolution, taking a reading
// at every angular step and logging it to the SD card
// ---------------------------------------------------------------------
void scanOneRevolution() {
  for (int step = 0; step < ANGULAR_STEPS; step++) {
    delay(SETTLE_MS);

    float distanceCm = readAveragedDistance();
    logValue(distanceCm);

    stepMotor(THETA_STEP_PIN, 1);
  }
}

// ---------------------------------------------------------------------
// Pulse a stepper's STEP pin a given number of times
// ---------------------------------------------------------------------
void stepMotor(int stepPin, int numSteps) {
  for (int i = 0; i < numSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_PULSE_US);
  }
}

// ---------------------------------------------------------------------
// Average several raw ADC samples, convert to voltage, then to a
// distance in cm using the cubic calibration fit
// ---------------------------------------------------------------------
float readAveragedDistance() {
  long total = 0;

  for (int i = 0; i < SAMPLES_PER_READING; i++) {
    total += analogRead(IR_SENSOR_PIN);
    delay(1);
  }

  float avgCounts = (float)total / (float)SAMPLES_PER_READING;
  float voltage = (avgCounts / ADC_MAX_COUNTS) * ADC_REF_VOLTAGE;

  float distanceCm = CAL_C3 * voltage * voltage * voltage
                    + CAL_C2 * voltage * voltage
                    + CAL_C1 * voltage
                    + CAL_C0;

  return distanceCm;
}

// ---------------------------------------------------------------------
// SD logging helpers
// ---------------------------------------------------------------------
void logValue(float value) {
  scanFile = SD.open(OUTPUT_FILENAME, FILE_WRITE);
  if (scanFile) {
    scanFile.println(value, 3);
    scanFile.close();
  } else {
    Serial.println(F("Error opening scan file for writing."));
  }
}

void writeDelimiter() {
  scanFile = SD.open(OUTPUT_FILENAME, FILE_WRITE);
  if (scanFile) {
    scanFile.println(SLICE_DELIMITER);
    scanFile.close();
  } else {
    Serial.println(F("Error opening scan file for writing."));
  }
}
