/*********************************************************************
 * Purpose: Single-string tuning with a stepper motor and PID control.
 * Board: Arduino Uno R4
 * Description: This sketch will tune a single guitar string using a
 * stepper motor and a PID controller for precise positioning.
 *********************************************************************/

#include <Arduino.h>
#include <arduinoFFT.h>
#include <algorithm>                 // std::nth_element


/*──────── USER CONSTANTS ───────*/
constexpr uint16_t FFT_SAMPLES   = 1024;
constexpr float    FS_HZ         = 4000.0f;
constexpr uint8_t  ADC_PIN       = A0;
constexpr float    DC_OFFSET     = 517.0f;         // bias midpoint (10-bit)


/* Signal-quality guards */
constexpr float    RMS_MIN       = 8.0f;           // min RMS to accept frame
constexpr float    MIN_SNR       = 5.0f;
constexpr uint8_t  N_STABLE      = 3;
constexpr float    STRUM_GATE    = 250.0f;


/* Search band */
constexpr float    FREQ_MIN_HZ   =  300.0f;
constexpr float    FREQ_MAX_HZ   = 400.0f;


/* Target (high-E) */
constexpr float    TARGET_FREQ   = 329.63f;
constexpr float    CENT_TOL      = 1.0f;


/* PID  */
constexpr float    Kp = 1.2f, Ki = 9.0f, Kd = 0.0f;
constexpr float    STEP_SCALE    = 1.2f;
constexpr int32_t  STEP_CLAMP    = 15;


/* Stepper rhythm */
constexpr uint16_t MOVE_TIME_MS  = 1000;
constexpr uint16_t MIN_PULSE_US  = 800;


/* Pins */
constexpr uint8_t  STEP_PIN = 3, DIR_PIN = 2, EN_PIN = 8;


/*──────── BUFFERS ───────*/
float vRe[FFT_SAMPLES], vIm[FFT_SAMPLES];
ArduinoFFT<float> FFT(vRe, vIm, FFT_SAMPLES, FS_HZ);
static float mags[FFT_SAMPLES/2];


const uint32_t SAMPLE_PERIOD_US = 1'000'000UL / uint32_t(FS_HZ);


/* PID / state */
float pidI = 0, lastErrHz = 0;
float lastPeakMag = 0, lastGoodFreq = 0;
uint8_t stableFrames = 0;


/* 0.6-s hold-filter state */
float    holdRefHz      = 0;
uint32_t holdStartMs    = 0;
bool     freqConfirmed  = false;


/*──────── STEPPER HELPERS ───────*/
inline void driverEnable(bool on) { digitalWrite(EN_PIN, on ? LOW : HIGH); }


/* fixed: works for both positive (tighten) AND negative (loosen) */
void stepperStepBlocking(int32_t steps)
{
  if (!steps) return;


  bool dirTighten = steps > 0;
  uint32_t stepCount = abs(steps);              // always non-negative
  uint32_t pulses    = 2UL * stepCount;         // HI + LO per µstep
  uint32_t pulse_us  = (MOVE_TIME_MS * 1000UL) / pulses;
  if (pulse_us < MIN_PULSE_US) pulse_us = MIN_PULSE_US;


  digitalWrite(DIR_PIN, dirTighten ? HIGH : LOW);


  for (uint32_t i = 0; i < stepCount; ++i) {    // deterministic finish
    digitalWrite(STEP_PIN, HIGH); delayMicroseconds(pulse_us);
    digitalWrite(STEP_PIN, LOW ); delayMicroseconds(pulse_us);
  }
}


/*──────── FFT FUNDAMENTAL DETECTOR ───────*/
float detectFundamental()
{
  /* 1 — acquire */
  uint32_t tNext = micros();
  float rmsSq = 0;
  for (uint16_t i = 0; i < FFT_SAMPLES; ++i) {
    while (micros() < tNext) {}
    float s = analogRead(ADC_PIN) - DC_OFFSET;
    vRe[i] = s;  vIm[i] = 0;  rmsSq += s * s;
    tNext += SAMPLE_PERIOD_US;
  }
  if (sqrtf(rmsSq / FFT_SAMPLES) < RMS_MIN) { lastPeakMag = 0; return -1; }


  /* 2 FFT */
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute   (FFTDirection::Forward);
  FFT.complexToMagnitude();


  const float BIN_HZ = FS_HZ / FFT_SAMPLES;
  uint16_t kL = max<uint16_t>(1, ceil (FREQ_MIN_HZ / BIN_HZ));
  uint16_t kH = min<uint16_t>(FFT_SAMPLES/2-1, floor(FREQ_MAX_HZ / BIN_HZ));


  /* 3 median floor */
  uint16_t n = 0;
  for (uint16_t k = kL; k <= kH; ++k) mags[n++] = vRe[k];
  std::nth_element(mags, mags + n/2, mags + n);
  float floorMag = mags[n/2];


  /* 4 peak & SNR */
  float pk = 0; uint16_t bin = 0;
  for (uint16_t k = kL; k <= kH; ++k)
    if (vRe[k] > pk) { pk = vRe[k]; bin = k; }
  if (pk < MIN_SNR * floorMag) return -1;
  lastPeakMag = pk;


  /* 5 quadratic interp */
  float f = bin * BIN_HZ;
  if (bin > 0 && bin < FFT_SAMPLES/2 - 1) {
    float m1 = vRe[bin-1], m2 = pk, m3 = vRe[bin+1];
    float d  = 2.0f * (2.0f*m2 - m1 - m3);
    float off = d ? (m3 - m1) / d : 0;
    f = (bin + off) * BIN_HZ;
  }
  return f;
}


/*──────── PID → µsteps ───────*/
int32_t pidDelta(float f)
{
  float errHz = TARGET_FREQ - f;
  float cent  = 1200.0f * log2f(TARGET_FREQ / f);
  if (fabsf(cent) < CENT_TOL) { pidI = 0; return 0; }


  pidI += errHz / FS_HZ;
  float pid = Kp*errHz + Ki*pidI + Kd*(errHz - lastErrHz)*FS_HZ;
  lastErrHz = errHz;


  int32_t steps = int32_t(cent * STEP_SCALE);
  return constrain(steps, -STEP_CLAMP, STEP_CLAMP);
}


/*──────── SETUP ───────*/
void setup()
{
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN , OUTPUT);
  pinMode(EN_PIN  , OUTPUT);
  driverEnable(true);


  analogReadResolution(10);           // keep 10-bit scale


  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("\nFFT tuner (fixed reverse-move bug + hold filter)");
}


/*──────── LOOP ───────*/
void loop()
{
  float f = detectFundamental();


  /* strum & reliability gates */
  if (lastPeakMag < STRUM_GATE)               { freqConfirmed = false; return; }
  if (f < 0)                                  { freqConfirmed = false; return; }


  /* ----- 0.6-s ±10 Hz hold filter ----- */
  uint32_t now = millis();
  if (!freqConfirmed) {
    if (holdStartMs == 0) {                   // first good read
      holdRefHz   = f;
      holdStartMs = now;
    } else if (fabsf(f - holdRefHz) <= 10.0f) {
      if (now - holdStartMs >= 600) freqConfirmed = true;
    } else {                                 // outside band → restart timer
      holdRefHz   = f;
      holdStartMs = now;
    }
  }


  Serial.print("Freq "); Serial.print(f, 2);
  Serial.print(" Hz  |X|max "); Serial.print(lastPeakMag, 0);


  if (!freqConfirmed) {
    Serial.println("  (waiting 0.6 s)");
    return;                                  // do not tune yet
  }


  /* Stability gate */
  if (lastGoodFreq > 0 && fabsf(f - lastGoodFreq) / f < 0.01f) {
    if (++stableFrames < N_STABLE) return;
  } else {
    stableFrames = 1;
  }
  lastGoodFreq = f;


  /* PID & move */
  int32_t d = pidDelta(f);
  if (d) {
    stepperStepBlocking(d);
    Serial.print("  move "); Serial.print(d); Serial.println(" µsteps");
  } else {
    Serial.println("  in-tune ✔");
  }
}



