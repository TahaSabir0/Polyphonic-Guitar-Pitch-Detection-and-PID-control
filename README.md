# Automatic Guitar Tuning Stand

A standalone robotic system that automatically detects pitch and tunes guitar strings with precision. This project combines digital signal processing, motor control, and custom mechanical design to create a practical tool for musicians.

**Author:** Taha Sabir
**Institution:** Gettysburg College
**Program:** Digital Technology Summer Fellowship (DTSF) 2025

---

## Table of Contents

- [Introduction](#introduction)
- [Hardware & Circuits](#hardware--circuits)
  - [Audio Detection Circuit](#1-audio-detection-circuit)
  - [Automatic Strummer](#2-automatic-strummer)
  - [Motor Selection & Tuning Mechanisms](#3-motor-selection--tuning-mechanisms)
- [Software](#software)
  - [FFT-Based Pitch Detection](#fft-based-pitch-detection)
  - [PID Control System](#pid-control-system)
  - [Code Overview](#code-overview)
- [Getting Started](#getting-started)
- [Future Improvements](#future-improvements)
- [Acknowledgements](#acknowledgements)
- [References](#references)

---

## Introduction

This repository is the guide I wish I had while building this project. Whether you're looking to replicate this automatic guitar tuning stand or create something even better, I'll walk you through every component, circuit, and line of code that makes it work.

The system consists of three fundamental subsystems working together:

1. **Audio Detection** - Captures and analyzes string vibrations to determine pitch
2. **Tuning Mechanisms** - DC motors that physically turn the tuning pegs
3. **Automatic Strummer** - A motorized slider that strums strings for polyphonic tuning

Depending on your goals, you can build any combination of these systems. Want just pitch detection? Build the audio circuit. Need a single-string tuner? Add one motor. This modular design lets you scale the project to your needs.

---

## Hardware & Circuits

### 1. Audio Detection Circuit

The audio detection system is the foundation of the project. It captures string vibrations and amplifies them for the Arduino to analyze.

#### Components

- **Microphone:** [Korg CM200BK](https://www.korg.com/us/products/tuners/cm_200/) clip-on contact microphone

  - Why this mic? It picks up vibrations directly from the guitar body, eliminating ambient noise. Inspired by [Antonio Hinojosa Cabrera's work](https://upcommons.upc.edu/entities/publication/d1f17706-ed6d-4329-bab4-19746a667733).

- **Amplification Circuit:** Based on [Amanda Ghassaei's Arduino Audio Input circuit](https://www.instructables.com/Arduino-Audio-Input/)
  - Uses TL072CN operational amplifiers
  - All required components listed in Amanda's tutorial

#### Critical Modification

After weeks of debugging a signal degradation issue, I discovered that piezoelectric microphones require a DC bias path. **Add a 1 MΩ resistor** between the non-inverting input (+) of the op-amp and Arduino ground. Without this, the piezo element acts like a capacitor and slowly charges, causing the signal to fade.

**Circuit diagram:** [hardware/system_wiring/sound_amplification circuit_by_amanda_ghassaei.png](hardware/system_wiring/sound_amplification_circuit_by_amanda_ghassaei.png)

**Detailed breakdown:** [Week 3 Blog Post](https://dtsf.sites.gettysburg.edu/dtsf2025/uncategorized/week-3-algorithmic-pivots-torque-troubles-and-a-breakthrough/)

---

### 2. Automatic Strummer

For polyphonic pitch detection (tuning multiple strings simultaneously), you need to strum all strings at once. The automatic strummer handles this.

#### Components

- **Motor:** NEMA 17 stepper motor (doesn't require high torque)
- **Driver:** A4988 stepper motor driver
- **Linear Slide:** Salvaged from a 3D printer (any linear motion system works)
- **Pick Attachment:** [hardware/3D_sketches/slider_attachment.stl](hardware/3D_sketches/slider_attachment.stl)

#### Key Insight

Initial tests with a rigid 3D-printed pick produced too much string attack noise, interfering with FFT accuracy. **Solution:** Replace the plastic pick with a small piece of cardboard for a gentler, more natural strum that produces cleaner frequency data.

#### Circuit & Tutorial

Follow [Rachel De Barros' excellent stepper motor tutorial](https://www.youtube.com/watch?v=wcLeXXATCR4) for wiring the NEMA 17 with the A4988 driver.

**Circuit diagram:** [hardware/individual_parts/guitar_strummer_circuit.JPEG](hardware/individual_parts/guitar_strummer_circuit.JPEG)

---

### 3. Motor Selection & Tuning Mechanisms

This is the most critical hardware decision in the project.

#### Understanding Torque Requirements

The biggest challenge in automatic guitar tuning is motor torque. Through extensive testing with a DIY torque meter, I determined that acoustic guitar tuning pegs require approximately **0.3 N·m of dynamic torque** for the heaviest strings.

**Important distinction:** Static (holding) torque ≠ Dynamic (moving) torque. A stepper motor rated for 0.4 N·m static torque will only deliver about 40-60% of that while moving, which explains why the NEMA 17 (0.4 N·m static) couldn't handle acoustic guitar strings. ([Source](https://www.sciencedirect.com/topics/engineering/static-torque))

#### Motor Options

**Stepper Motors:**

- **Pros:** Precise control, easy to program, no gearbox needed
- **Cons:** More expensive, larger size relative to torque, dynamic torque drops significantly
- **Best for:** Single-string systems, electric guitars, lower-tension instruments
- **Recommendation:** NEMA 23 for acoustic guitars ([Auburn University reference project](https://www.youtube.com/watch?v=DU2-K5PHwyM&t=147s))

**DC Motors:**

- **Pros:** Higher torque-to-size ratio, cheaper, abundant gearbox options
- **Cons:** Less precise, more complex control
- **Best for:** Multi-string systems where space is constrained
- **Recommendation:** [Greartisan DC 12V 20RPM 37mm Gearmotor](https://www.amazon.com) (0.3+ N·m dynamic torque)

#### Why I Chose DC Motors

For a 6-string system with automatic strumming, motor size is crucial. The 37mm diameter of the Greartisan motors is the maximum that fits between guitar tuning pegs. Anything larger wouldn't work.

#### Mechanical Components

**3D Models:**

- **Motor enclosure:** [hardware/3D_sketches/motor_box.stl](hardware/3D_sketches/motor_box.stl)
- **Motor side holders:** [hardware/3D_sketches/motor_sideholder.stl](hardware/3D_sketches/motor_sideholder.stl) (prevents rotation inside enclosure)
- **DC motor peg attachment:** [hardware/3D_sketches/DC_peg_attachement.stl](hardware/3D_sketches/DC_peg_attachement.stl)
- **Stepper motor peg attachment:** [hardware/3D_sketches/stepper_peg_attachement.stl](hardware/3D_sketches/stepper_peg_attachement.stl)

**Motor Drivers:**

- **For DC motors:** L298N dual H-bridge motor drivers (can control 2 motors per driver)
- **For stepper motors:** A4988 or similar stepper driver

**Circuit diagrams:**

- [hardware/individual_parts/PID_control_circuit.JPEG](hardware/individual_parts/PID_control_circuit.JPEG)
- [hardware/system_wiring/](hardware/system_wiring/) folder contains complete system wiring photos

#### Critical Power Architecture Insight

**Problem:** Running the NEMA 17 strummer and audio circuit from the same 5V rail introduced electrical noise that disrupted the sensitive amplifier.

**Solution:** The L298N drivers have built-in 5V regulators! Power the entire system from a single 12V source:

- 12V feeds the L298N drivers (for DC motors)
- L298N's 5V output powers the Arduino
- A separate 5V source powers the audio circuit (isolated from motor noise)

This architecture eliminates electrical interference while simplifying the overall power system.

---

## Software

All code is written for the **Arduino Uno R4 WiFi** and is located in the [software/](software/) directory.

### FFT-Based Pitch Detection

After attempting autocorrelation methods (YIN algorithm), I pivoted to Fast Fourier Transform (FFT) for pitch detection. FFT converts time-domain audio signals into frequency-domain data, revealing the fundamental frequency of each string.

#### How It Works

1. **Signal Acquisition:** Sample audio at 4000 Hz with 1024 samples
2. **Windowing:** Apply Hamming window to reduce spectral leakage
3. **FFT Computation:** Transform time-domain signal to frequency spectrum
4. **Peak Detection:** For each string's frequency band (e.g., ±20 Hz around 82.41 Hz for low E), find the peak magnitude
5. **Quadratic Interpolation:** Refine frequency estimate using neighboring bins
6. **Filtering:** Apply SNR (signal-to-noise ratio) and magnitude thresholds to reject noise

#### Critical Discovery: Clock Drift

The Arduino R4 uses two internal clock sources:

- **USB-PLL:** Highly accurate when connected to a computer
- **HOCO:** Less accurate, temperature-sensitive clock when running standalone

My board's HOCO clock ran 2.5% fast, causing all frequency readings to be 2.5% too high. This explained why the system worked perfectly when connected to my laptop but failed when running on battery power. The solution was to measure the actual sampling rate during runtime and compensate accordingly.

**Implementation:** [software/six_string_FFT.ino:217-218](software/six_string_FFT.ino)

### PID Control System

PID (Proportional-Integral-Derivative) control optimally adjusts motor movements based on pitch error.

#### The Formula

```
control_output = Kp × error + Ki × ∫error dt + Kd × d(error)/dt
```

- **Kp (Proportional):** Immediate response proportional to error magnitude
- **Ki (Integral):** Corrects accumulated error over time
- **Kd (Derivative):** Dampens oscillations by considering rate of change

#### Tuning Constants

After extensive calibration:

- `Kp = 1.2`
- `Ki = 9.0`
- `Kd = 0.0` (derivative not needed for this application)

The error is calculated in **cents** (1 cent = 1/100th of a semitone) for musical accuracy:

```cpp
float cents = 1200.0f * log2f(targetFreq / measuredFreq);
```

The PID output is then clamped to ±20 units and mapped to motor burst durations (250-900 ms), preventing excessive overshooting.

**Implementation:** [software/Three_Strings_Final_Demo.ino:269-274](software/Three_Strings_Final_Demo.ino)

### Code Overview

#### [single_string_stepper.ino](software/single_string_stepper.ino)

**Purpose:** Tune a single string using a stepper motor
**Features:**

- FFT pitch detection (1024 samples @ 4000 Hz)
- PID control with integral wind-up prevention
- 0.6-second hold filter (rejects transient noise)
- Precision: ±1 cent tolerance

**Key functions:**

- `detectFundamental()`: Captures audio, performs FFT, returns frequency
- `pidDelta()`: Calculates motor steps needed based on pitch error
- `stepperStepBlocking()`: Executes precise stepper movements

---

#### [single_string_DC.ino](software/single_string_DC.ino)

**Purpose:** Tune a single string using a DC motor
**Features:**

- Identical FFT and PID logic to stepper version
- Burst-based motor control (maps PID output to timed motor pulses)
- Configurable direction sense (`DIR_SIGN`) for different tuning peg orientations

**Key differences from stepper:**

- `motorDrive()`: Sends PWM bursts instead of precise steps
- Faster tuning but less granular control

---

#### [six_string_FFT.ino](software/six_string_FFT.ino)

**Purpose:** Monitor and detect pitch for all six strings simultaneously
**Features:**

- Polyphonic pitch detection using frequency binning
- Individual SNR and magnitude thresholds per string
- Automatic strumming integration

**How polyphonic detection works:**
The FFT spectrum is divided into six non-overlapping frequency windows centered on standard tuning:

| String | Frequency | Detection Band |
| ------ | --------- | -------------- |
| E2     | 82.41 Hz  | 62-102 Hz      |
| A2     | 110.00 Hz | 90-130 Hz      |
| D3     | 146.83 Hz | 127-167 Hz     |
| G3     | 196.00 Hz | 176-216 Hz     |
| B3     | 246.94 Hz | 227-267 Hz     |
| E4     | 329.63 Hz | 310-350 Hz     |

Each band's peak is tracked independently, allowing simultaneous tuning of multiple strings.

**Challenge: Sympathetic Resonance**

When all strings are strummed together, harmonics from heavy strings interfere with lighter strings. For example, the 3rd harmonic of A2 (110 Hz) is 330 Hz, which overlaps with E4's fundamental (329.63 Hz). This required carefully tuned SNR thresholds and magnitude gates per string.

---

#### [Three_Strings_Final_Demo.ino](software/Three_Strings_Final_Demo.ino) ⭐

**Purpose:** Complete working prototype - automatically tunes three strings
**Features:**

- Integrated strumming, FFT detection, and concurrent motor control
- Median filtering (collects 0.2s of data, uses median frequency)
- Concurrent motor bursts (all three motors can move simultaneously)
- 4-second cycle period (strum → detect → tune → repeat)

**Control loop:**

1. Strum strings back and forth (2 seconds total movement)
2. Wait 0.2 seconds for string attack to settle
3. Collect FFT data for 0.2 seconds
4. Calculate median frequency for each string
5. Apply PID control and execute motor bursts concurrently
6. Check if all strings are within ±1 Hz tolerance
7. Repeat until tuned

**Why concurrent bursts?**
Motors operate simultaneously, dramatically reducing tuning time. Each motor independently calculates its burst duration based on PID output.

**Implementation highlights:**

- `startBurst()`: Initiates non-blocking motor movement
- `waitBursts()`: Polls motors until all complete
- `track()`: Per-string FFT peak detection with quadratic interpolation
- `median()`: Robust frequency estimation over multiple frames

---

## Getting Started

### Prerequisites

**Hardware:**

- Arduino Uno R4 WiFi (or Mega for 6-string system)
- Audio detection circuit components (see Hardware section)
- Motors (stepper or DC) based on your application
- Motor drivers (A4988 for stepper, L298N for DC)
- 12V power supply
- 3D printer for mechanical components

**Software:**

- [Arduino IDE](https://www.arduino.cc/en/software)
- [ArduinoFFT library](https://github.com/kosme/arduinoFFT)

### Assembly Steps

1. **Build the audio circuit** following Amanda Ghassaei's tutorial with the 1 MΩ resistor modification
2. **3D print mechanical components** from [hardware/3D_sketches/](hardware/3D_sketches/)
3. **Wire motors and drivers** using circuit diagrams in [hardware/individual_parts/](hardware/individual_parts/)
4. **Upload appropriate sketch** from [software/](software/) directory
5. **Calibrate** by adjusting FFT thresholds and PID gains for your specific guitar

### Quick Start: Single-String Tuner

For your first build, I recommend starting with the single-string stepper motor version:

1. Build the audio detection circuit
2. Connect a NEMA 17 motor with A4988 driver
3. Upload [single_string_stepper.ino](software/single_string_stepper.ino)
4. Adjust `TARGET_FREQ` to your desired string
5. Manually strum the string and watch it tune automatically!

---

## Future Improvements

This project has immense potential for expansion. Here are ideas for taking it further:

### 1. Comprehensive Torque Study

String instrument tuning torque requirements are severely under-documented in academic literature. A systematic study measuring torque across different instruments, string gauges, and materials would be invaluable for future automatic tuner designs.

### 2. Complete 6-String System

Due to time and budget constraints, I built a 3-string prototype. The complete vision involves:

- Two 3-motor modules (one fixed, one spring-loaded)
- Adjustable phone holder-style mechanism
- Clamps onto all six tuning pegs simultaneously
- Single-button operation

### 3. Linear Regression Tuning

The current system uses iterative "burst" adjustments. A smarter approach:

- Calibrate the relationship between motor rotation and pitch change for each string
- Use linear regression to predict exact motor movement needed
- Achieve perfect tuning in a single movement (~3 seconds total)

### 4. User Interface

Build a proper interface with:

- LCD display showing current pitch vs. target
- Tuning preset selection (standard, drop D, open G, etc.)
- Calibration buttons
- Status LEDs

### 5. Automatic FFT Calibration

Currently, FFT variables (SNR thresholds, magnitude gates, bin widths) require manual tweaking for hours. An automatic calibration protocol that adapts to different guitars would make the system truly universal.

### 6. Hybrid Initialization System

Current limitation: The polyphonic binning model only works if strings are initially close to the correct tuning. A robust solution:

- **Phase 1:** Individual string calibration (strum each string separately with high-precision strummer)
- **Phase 2:** Once all strings are within their bins, switch to concurrent tuning mode
- Combined with a good UI, this creates the blueprint for a commercializable product

---

## Acknowledgements

This project was sparked by [Tim Bell's 2011 automatic guitar tuner video](https://www.youtube.com/watch?v=ZahpROn3gm0), which proved this was possible to build. That single video flicked a switch in my mind and set me on this journey.

None of this would have been possible without the invaluable guidance and unwavering support of **Professor Eric Remy** and **Joshua Wagner**. Their mentorship through every challenge, from fried op-amps to motor torque mysteries, was instrumental in bringing this project to life.

---

## References

### Academic Papers

- Gylling, Martin, and Ruben Svensson. "Robotic Electric Guitar Tuner." Bachelor's thesis, 2020.
- Hinojosa Cabrera, Antonio. "Automatic Guitar Tuner." Bachelor's thesis, Universitat Politècnica de Catalunya, 2020. [Link](https://upcommons.upc.edu/entities/publication/d1f17706-ed6d-4329-bab4-19746a667733)
- Boestad, Erik, and Rudberg, Johan. "Piezoelectric Guitar Tuner." Research paper.

### Tutorials & Resources

- [Amanda Ghassaei - Arduino Audio Input](https://www.instructables.com/Arduino-Audio-Input/)
- [Rachel De Barros - Stepper Motor Control](https://www.youtube.com/watch?v=wcLeXXATCR4)
- [Auburn University MECH 5840 - Automatic Guitar Tuner](https://www.youtube.com/watch?v=DU2-K5PHwyM&t=147s)
- [ScienceDirect - Static vs Dynamic Torque](https://www.sciencedirect.com/topics/engineering/static-torque)

### Project Blog

For a week-by-week chronicle of the development process, including all the failures, breakthroughs, and lessons learned, visit:
[DTSF 2025 - Automatic Guitar Tuning Stand Blog](https://dtsf.sites.gettysburg.edu/dtsf2025/taha)

---

**Happy building! If you create your own version or make improvements, I'd love to hear about it.**

_This project was completed as part of the Digital Technology Summer Fellowship (DTSF) at Gettysburg College, Summer 2025._
