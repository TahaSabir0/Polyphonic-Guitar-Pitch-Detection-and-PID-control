/*********************************************************************
 * Purpose: Single-string tuning with a DC motor and PID control.
 * Board: Arduino Uno R4
 * Description: This sketch will tune a single guitar string using a
 * DC motor with an encoder and a PID controller for precise positioning.
 *********************************************************************/

#include <Arduino.h>
#include <arduinoFFT.h>
#include <algorithm>                 // std::nth_element


/*──────── USER CONSTANTS ───────*/
constexpr uint16_t FFT_SAMPLES   = 1024;
constexpr float    FS_HZ         = 4000.0f;
constexpr uint8_t  ADC_PIN       = A0;
constexpr float    DC_OFFSET     = 517.0f;      // bias midpoint (10-bit)


/* Signal-quality guards */
constexpr float    RMS_MIN       = 8.0f;
constexpr float    MIN_SNR       = 5.0f;
constexpr uint8_t  N_STABLE      = 3;
constexpr float    STRUM_GATE    = 250.0f;


/* Search band */
constexpr float    FREQ_MIN_HZ   =  50.0f;
constexpr float    FREQ_MAX_HZ   = 400.0f;


/* Target (set per string before tuning) */
constexpr float    TARGET_FREQ   = 82.41f;     // high-E
constexpr float    CENT_TOL      = 0.30f;


/* PID gains */
constexpr float    Kp = 1.2f, Ki = 9.0f, Kd = 0.0f;
constexpr float    STEP_SCALE    = 1.2f;
constexpr int32_t  STEP_CLAMP    = 15;


/* ── DC-motor / L298N wiring ── */
constexpr uint8_t  M_IN1   = 2;
constexpr uint8_t  M_IN2   = 3;
constexpr uint8_t  M_PWM   = 9;
constexpr uint8_t  MAX_PWM = 255;     // full duty


/* Burst timing (maps 1…15 proxy-steps → 250…900 ms) */
constexpr uint16_t MIN_BURST_MS = 250;
constexpr uint16_t MAX_BURST_MS = 900;


/*  Direction sense  ------------------------------------------------
    DIR_SIGN = +1  → positive proxy-steps tighten (clockwise peg)
    DIR_SIGN = −1  → positive proxy-steps loosen  (flip motor sense)


    Set to −1 for strings whose tuners tighten in the opposite direction.
   ------------------------------------------------------------------*/
constexpr int8_t   DIR_SIGN = -1;     // change to −1 for reversed pegs


/*──────── BUFFERS ───────*/
float vRe[FFT_SAMPLES], vIm[FFT_SAMPLES];
ArduinoFFT<float> FFT(vRe, vIm, FFT_SAMPLES, FS_HZ);
static float mags[FFT_SAMPLES/2];


const uint32_t SAMPLE_PERIOD_US = 1'000'000UL / uint32_t(FS_HZ);


/* PID / state */
float pidI = 0, lastErrHz = 0;
float lastPeakMag = 0, lastGoodFreq = 0;
uint8_t stableFrames = 0;


/* 0.4-s hold filter */
float    holdRefHz     = 0;
uint32_t holdStartMs   = 0;
bool     freqConfirmed = false;


/*──────── DC-MOTOR HELPER ───────*/
void motorDrive(int32_t proxySteps)
{
  if (!proxySteps) return;                            // nothing to do


  int32_t signedSteps = proxySteps * DIR_SIGN;        // flip if needed
  bool tighten = signedSteps > 0;                     // final direction


  uint16_t burst = map(abs(signedSteps),
                       1, STEP_CLAMP,
                       MIN_BURST_MS, MAX_BURST_MS);


  digitalWrite(M_IN1, tighten ? HIGH : LOW);
  digitalWrite(M_IN2, tighten ? LOW  : HIGH);


  analogWrite(M_PWM, MAX_PWM);
  delay(burst);
  analogWrite(M_PWM, 0);                              // stop
}


/*──────── FFT FUNDAMENTAL DETECTOR ───────*/
float detectFundamental()
{
  uint32_t tNext = micros();
  float rmsSq = 0;
  for (uint16_t i = 0; i < FFT_SAMPLES; ++i) {
    while (micros() < tNext) {}
    float s = analogRead(ADC_PIN) - DC_OFFSET;
    vRe[i] = s;  vIm[i] = 0;  rmsSq += s * s;
    tNext += SAMPLE_PERIOD_US;
  }
  if (sqrtf(rmsSq / FFT_SAMPLES) < RMS_MIN) { lastPeakMag = 0; return -1; }


  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute   (FFTDirection::Forward);
  FFT.complexToMagnitude();


  const float BIN_HZ = FS_HZ / FFT_SAMPLES;
  uint16_t kL = max<uint16_t>(1, ceil (FREQ_MIN_HZ / BIN_HZ));
  uint16_t kH = min<uint16_t>(FFT_SAMPLES/2-1, floor(FREQ_MAX_HZ / BIN_HZ));


  uint16_t n = 0;
  for (uint16_t k = kL; k <= kH; ++k) mags[n++] = vRe[k];
  std::nth_element(mags, mags + n/2, mags + n);
  float floorMag = mags[n/2];


  float pk = 0; uint16_t bin = 0;
  for (uint16_t k = kL; k <= kH; ++k)
    if (vRe[k] > pk) { pk = vRe[k]; bin = k; }
  if (pk < MIN_SNR * floorMag) return -1;
  lastPeakMag = pk;


  float f = bin * BIN_HZ;
  if (bin > 0 && bin < FFT_SAMPLES/2 - 1) {
    float m1 = vRe[bin-1], m2 = pk, m3 = vRe[bin+1];
    float d  = 2.0f * (2.0f*m2 - m1 - m3);
    float off = d ? (m3 - m1) / d : 0;
    f = (bin + off) * BIN_HZ;
  }
  return f;
}


/*──────── PID → proxy-step (signed) ───────*/
int32_t pidProxy(float f)
{
  float errHz = TARGET_FREQ - f;
  float cent  = 1200.0f * log2f(TARGET_FREQ / f);
  if (fabsf(cent) < CENT_TOL) { pidI = 0; return 0; }


  pidI += errHz / FS_HZ;
  float pid = Kp*errHz + Ki*pidI + Kd*(errHz - lastErrHz)*FS_HZ;
  lastErrHz = errHz;


  int32_t s = int32_t(cent * STEP_SCALE);
  return constrain(s, -STEP_CLAMP, STEP_CLAMP);
}


/*──────── SETUP ───────*/
void setup()
{
  pinMode(M_IN1, OUTPUT);
  pinMode(M_IN2, OUTPUT);
  pinMode(M_PWM, OUTPUT);
  analogWrite(M_PWM, 0);


  analogReadResolution(10);


  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("\nFFT tuner – DC-motor version (DIR_SIGN variable)");
}


/*──────── LOOP ───────*/
void loop()
{
  float f = detectFundamental();


  if (lastPeakMag < STRUM_GATE)               { freqConfirmed=false; return; }
  if (f < 0)                                  { freqConfirmed=false; return; }


  /* Hold filter: ±10 Hz for ≥0.4 s */
  uint32_t now = millis();
  if (!freqConfirmed) {
    if (holdStartMs == 0) { holdRefHz = f; holdStartMs = now; }
    else if (fabsf(f - holdRefHz) <= 10.0f) {
      if (now - holdStartMs >= 400) freqConfirmed = true;   // 0.4 s
    } else { holdRefHz = f; holdStartMs = now; }
  }


  Serial.print("Freq "); Serial.print(f, 2);
  Serial.print(" Hz  |X|max "); Serial.print(lastPeakMag, 0);


  if (!freqConfirmed) { Serial.println("  (waiting 0.4 s)"); return; }


  /* Stability gate */
  if (lastGoodFreq>0 && fabsf(f-lastGoodFreq)/f < 0.01f) {
    if (++stableFrames < N_STABLE) return;
  } else stableFrames = 1;
  lastGoodFreq = f;


  int32_t proxy = pidProxy(f);
  if (proxy) {
    motorDrive(proxy);
    Serial.print("  burst "); Serial.print(proxy); Serial.println();
  } else {
    Serial.println("  in-tune ✔");
  }
}




