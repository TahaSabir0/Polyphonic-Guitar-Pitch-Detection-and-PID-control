/*********************************************************************
 * Purpose: Automatic tuning for three strings with a hold mechanism.
 * Board: Arduino Uno R4
 * Description: This sketch will automatically tune three strings,
 * holding the tuner on the peg until the string is in tune.
 *********************************************************************/

 /*********************************************************************
 *  3-String Auto-Tuner – 1 Hz window, burst ±20, repeats every 4 s
 *********************************************************************/




#include <Arduino.h>
#include <arduinoFFT.h>
#include <vector>




/* ---------- FFT parameters ---------- */
constexpr uint16_t FFT_N   = 1024;          // 3.906 Hz bins
constexpr uint8_t  ADC_PIN = A0;
constexpr float    DC_OFF  = 506.0f;
constexpr float    RMS_MIN = 8.0f;




/* ---------- per-string gates ---------- */
struct StringTun { const char* name; float hz, band, snr, mag; };




StringTun STR_ALL[6] = {
 {"E2",  82.41f, 20, 5.0f, 500},
 {"A2", 110.00f, 20, 3.5f, 350},
 {"D3", 146.83f, 20, 3.0f, 300},
 {"G3", 196.00f, 20, 2.0f, 500},
 {"B3", 246.94f, 20, 3.0f, 200},
 {"E4", 329.63f, 20, 2.0f, 120}
};
constexpr uint8_t TUNE_IDX[3] = {0,1,2};    // E2-A2-D3




/* ---------- FFT buffers ---------- */
float vRe[FFT_N], vIm[FFT_N];
ArduinoFFT<float> FFT(vRe, vIm, FFT_N, 4000.0f);
static float mags[FFT_N/2];




/* ---------- motor map ---------- */
struct Motor { uint8_t in1,in2,pwm; int8_t dir; uint32_t stopMs=0; };
Motor M[3] = { {7,6,5,+1}, {2,4,3,+1}, {8,9,10,+1} };




/* ---------- burst constants ---------- */
constexpr int32_t  STEP_CLAMP = 20;                 // ±20 (was 15)
constexpr uint16_t MIN_BURST  = 250, MAX_BURST = 900;
constexpr uint8_t  MAX_PWM    = 255;




/* ---------- concurrent-burst helpers ---------- */
void startBurst(Motor& m,int32_t proxy)
{
  if(!proxy){ m.stopMs=0; return; }
  proxy*=m.dir;
  bool tighten = proxy>0;
  uint16_t dur = map(abs(proxy),1,STEP_CLAMP,MIN_BURST,MAX_BURST);




  digitalWrite(m.in1,tighten?HIGH:LOW);
  digitalWrite(m.in2,tighten?LOW:HIGH);
  analogWrite(m.pwm,MAX_PWM);
  m.stopMs = millis() + dur;
}
void waitBursts()
{
  bool any;
  do{
    any=false; uint32_t now=millis();
    for(auto& m:M){
      if(m.stopMs && now>=m.stopMs){ analogWrite(m.pwm,0); m.stopMs=0; }
      else if(m.stopMs) any=true;
    }
  }while(any);
}




/* ---------- tiny median ---------- */
template<typename T>
T median(std::vector<T>& v){
  size_t n=v.size(); if(!n) return T(-1);
  for(size_t i=1;i<n;++i){
    T key=v[i]; int j=i-1;
    while(j>=0 && v[j]>key){ v[j+1]=v[j]; --j; }
    v[j+1]=key;
  }
  return (n&1)? v[n/2] : 0.5*(v[n/2-1]+v[n/2]);
}




/* ---------- tracker ---------- */
float track(const float* s,const StringTun& st,float binHz)
{
  int kC=int(round(st.hz/binHz));
  int half=max(3,int(st.band/binHz));
  int kL=max(1,kC-half), kH=min(int(FFT_N/2-1),kC+half);




  /* floor */
  uint8_t n=0; for(int k=kL;k<=kH;++k) mags[n++]=s[k];
  for(uint8_t i=1;i<n;++i){
    float key=mags[i]; int8_t j=i-1;
    while(j>=0 && mags[j]>key){ mags[j+1]=mags[j]; --j; }
    mags[j+1]=key;
  }
  float floor=mags[n/2];




  /* peak & gates */
  float pk=0; int bin=kC;
  for(int k=kL;k<=kH;++k) if(s[k]>pk){ pk=s[k]; bin=k; }
  if(pk<st.snr*floor || pk<st.mag) return -1;




  /* quadratic interpolation */
  float f=bin*binHz;
  if(bin>0 && bin<FFT_N/2-1){
    float m1=s[bin-1],m2=pk,m3=s[bin+1];
    float d=2*(2*m2-m1-m3);
    float off=d? (m3-m1)/d:0;
    f=(bin+off)*binHz;
  }
  return f;
}




/* ---------- slider stepper (STEP=1 DIR=0) ---------- */
constexpr uint8_t SL_STEP=1, SL_DIR=0;
void strum(bool dir)
{
  const uint16_t PULSE_HZ=800, HALF_US=500000UL/PULSE_HZ;
  pinMode(SL_STEP,OUTPUT); pinMode(SL_DIR,OUTPUT);
  digitalWrite(SL_DIR,dir?HIGH:LOW);
  unsigned long t0=millis();
  while(millis()-t0<1000){
    digitalWrite(SL_STEP,HIGH); delayMicroseconds(HALF_US);
    digitalWrite(SL_STEP,LOW ); delayMicroseconds(HALF_US);
  }
}




/* ---------- setup ---------- */
void setup()
{
 
  Serial.begin(115200);
  while(!Serial){}
  Serial.println(F("\nAuto-tune – ±1 Hz, burst ±20, 4-s cycles"));




  for(auto& m:M){
    pinMode(m.in1,OUTPUT); pinMode(m.in2,OUTPUT); pinMode(m.pwm,OUTPUT);
    analogWrite(m.pwm,0);
  }
}




/* ---------- main loop ---------- */
void loop()
{
  uint32_t cycleStart=millis();
  static uint16_t cycle=0;
  Serial.print(F("\nCycle ")); Serial.println(++cycle);




  /* strum back and forth with no gap */
  strum(true);    // backward
  strum(false);   // forward
  delay(200);     // settle 0.20 s




  /* collect ≈0.20 s of frames */
  std::vector<float> buf[3];
  unsigned long tCollect=millis();
  while(millis()-tCollect<200){
    uint32_t t0=micros(); float rms=0;
    for(uint16_t i=0;i<FFT_N;++i){
      float s=analogRead(ADC_PIN)-DC_OFF;
      vRe[i]=s; vIm[i]=0; rms+=s*s;
      delayMicroseconds(250);
    }
    uint32_t dt=micros()-t0; if(!dt) continue;
    float Fs=1.0e6f*FFT_N/dt, binHz=Fs/FFT_N;
    if(sqrtf(rms/FFT_N)<RMS_MIN) continue;




    FFT.windowing(FFTWindow::Hamming,FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();




    for(uint8_t i=0;i<3;++i){
      float f=track(vRe,STR_ALL[TUNE_IDX[i]],binHz);
      if(f>0) buf[i].push_back(f);
    }
  }




  /* PID & bursts */
  int32_t proxy[3]={0};
  bool allTuned=true;




  Serial.print(F("Medians: "));
  for(uint8_t i=0;i<3;++i){
    const StringTun& st=STR_ALL[TUNE_IDX[i]];
    float med=median(buf[i]);
    Serial.print(st.name); Serial.print(' ');
    if(med<0){ Serial.print("--   "); allTuned=false; continue; }
    Serial.print(med,2); Serial.print(" Hz   ");




    float errHz=st.hz-med;
    if(fabsf(errHz)>1.0f){          // << tighter ±1 Hz window
      allTuned=false;
      float cents=1200.0f*log2f(st.hz/med);
      proxy[i]=constrain(int32_t(cents*1.2f),-STEP_CLAMP,STEP_CLAMP);
    }
  }
  Serial.println();




  if(allTuned){
    Serial.println(F("All strings in tune – done"));
    while(1);
  }




  /* launch bursts concurrently */
  for(uint8_t i=0;i<3;++i) startBurst(M[i],proxy[i]);
  waitBursts();




  Serial.print(F("Bursts : "));
  for(uint8_t i=0;i<3;++i)
    if(proxy[i]){ Serial.print(STR_ALL[TUNE_IDX[i]].name);
                  Serial.print(' '); Serial.print(proxy[i]); Serial.print("   "); }
  Serial.println();




  /* maintain 4-s cycle period */
  while(millis()-cycleStart<4000);
}















