/*********************************************************************
 * Purpose: Monitor all six strings and measure fundamental frequency.
 * Board: Arduino Uno R4
 * Description: This sketch will monitor all six strings, detect which
 * one is played, and measure its fundamental frequency using FFT.
 *********************************************************************/


#include <Arduino.h>
#include <arduinoFFT.h>
#include <algorithm>




/* ---------- fixed FFT length ---------- */
constexpr uint16_t FFT_N = 1024;




/* ---------- analogue front-end ---------- */
constexpr uint8_t  ADC_PIN = A0;
constexpr float    DC_OFF  = 505.0f;      // bias midpoint
constexpr float    RMS_MIN = 8.0f;        // frame-level floor




/* ---------- per-string tunables ---------- */
struct StringTun {
  const char *name;
  float targetHz;
  float bandHalfHz;
  float minSNR;
  float minMag;
};




/* your new table */
StringTun STR[6] = {
/* name  Hz      band  SNR   mag */
 {"E2",  82.41f, 20,   5.0f,  500},
 {"A2", 110.00f, 20,   4.0f,  400},
 {"D3", 146.83f, 20,   3.0f,  300},
 {"G3", 196.00f, 20,   2.0f,  500},
 {"B3", 246.94f, 20,   3.0f,  200},
 {"E4", 329.63f, 20,   2.0f,  120}
};




/* ---------- FFT buffers ---------- */
float vRe[FFT_N], vIm[FFT_N];
ArduinoFFT<float> FFT(vRe, vIm, FFT_N, 4000.0f);   // Fs placeholder
static float mags[FFT_N/2];




/* ---------- helper: track one string ---------- */
float trackString(const float *spec,
                  const StringTun &st,
                  float binHz)              // actual Fs / 1024
{
  int kC = int(round(st.targetHz / binHz));
  int kL = max(1, kC - int(st.bandHalfHz / binHz));
  int kH = min(int(FFT_N/2-1), kC + int(st.bandHalfHz / binHz));




  /* median floor */
  uint16_t n=0; for(int k=kL;k<=kH;++k) mags[n++] = spec[k];
  std::nth_element(mags, mags+n/2, mags+n);
  float floorMag = mags[n/2];




  /* peak & SNR */
  float pk=0; int bin=kC;
  for(int k=kL;k<=kH;++k)
    if(spec[k] > pk){ pk = spec[k]; bin = k; }
  if(pk < st.minSNR * floorMag) return -1;
  if(pk < st.minMag)            return -1;




  /* quadratic interpolation */
  float f = bin * binHz;
  if(bin>0 && bin<FFT_N/2-1) {
    float m1=spec[bin-1], m2=pk, m3=spec[bin+1];
    float d = 2*(2*m2 - m1 - m3);
    float off = d ? (m3-m1)/d : 0;
    f = (bin + off) * binHz;
  }
  return f;
}




const uint8_t SLIDER_STEP_PIN = 0;   // STEP  ↔ driver
const uint8_t SLIDER_DIR_PIN  = 1;   // DIR   ↔ driver




/* Simple one-shot move executed in setup() */
void oneSecondStrum()                 // call this ONCE inside setup()
{
  const uint16_t PULSE_HZ = 800;               // 800 full-step pulses s-¹
  const uint16_t HALF_US  = 500000UL / PULSE_HZ;  // half-period (µs)








  pinMode(SLIDER_STEP_PIN, OUTPUT);
  pinMode(SLIDER_DIR_PIN,  OUTPUT);








  digitalWrite(SLIDER_DIR_PIN, HIGH);    // choose “strum” direction








  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {         // 1 000 ms = 1 s
    digitalWrite(SLIDER_STEP_PIN, HIGH);
    delayMicroseconds(HALF_US);
    digitalWrite(SLIDER_STEP_PIN, LOW);
    delayMicroseconds(HALF_US);
  }
}




/* Simple one-second move in the opposite direction */
void oneSecondStrumOpposite()        // call this ONCE inside setup()
{
  const uint16_t PULSE_HZ = 800;                 // 800 full-step pulses /s
  const uint16_t HALF_US  = 500000UL / PULSE_HZ; // half-period (µs)




  pinMode(SLIDER_STEP_PIN, OUTPUT);
  pinMode(SLIDER_DIR_PIN,  OUTPUT);




  digitalWrite(SLIDER_DIR_PIN, LOW); // **reverse** direction




  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {     // 1000 ms = 1 s
    digitalWrite(SLIDER_STEP_PIN, HIGH);
    delayMicroseconds(HALF_US);
    digitalWrite(SLIDER_STEP_PIN, LOW);
    delayMicroseconds(HALF_US);
  }
}




/* ---------- setup ---------- */
void setup()
{
  analogReadResolution(10);
  Serial.begin(115200);
  while(!Serial){}
  Serial.println("\nSix-string FFT monitor – measured Fs with tuned gates");
  oneSecondStrum();
  oneSecondStrumOpposite();
}




/* ---------- main loop ---------- */
void loop()
{
  /* 1 ── capture frame & time it */
  uint32_t t0 = micros();
  float rmsSq = 0;
  for(uint16_t i=0;i<FFT_N;++i){
    float s = analogRead(ADC_PIN) - DC_OFF;
    vRe[i] = s;  vIm[i] = 0;  rmsSq += s*s;
    delayMicroseconds(250);              // nominal 4 kHz pacing
  }
  uint32_t elapsed = micros() - t0;      // µs for 1024 samples
  if(elapsed == 0) return;               // safety
  float actualFs = 1.0e6f * FFT_N / elapsed;
  float binHz    = actualFs / FFT_N;




  if(sqrtf(rmsSq / FFT_N) < RMS_MIN) return;




  /* 2 ── FFT magnitude */
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute   (FFTDirection::Forward);
  FFT.complexToMagnitude();




  /* 3 ── scan & print */
  for(const auto &st : STR){
    float f = trackString(vRe, st, binHz);
    Serial.print(st.name); Serial.print(": ");
    if(f < 0) Serial.print("--");
    else { Serial.print(f,2); Serial.print(" Hz"); }
    Serial.print("   ");
  }
  Serial.println();
}












