// ===== Final v1.3-CL (Base: v1.0.0) [Index: LF,LR,RF,RR] =====
// Enc A map: LF:D18, LR:D19, RF:D2, RR:D3
// 保留原本馬達接線與開環方向函式（F/B/TL/TR）

#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// ───────────── Servo (保留 v1.0.0) ─────────────
Servo motorA, motorB, motorC, motorD;
int posA=90,posB=90,posC=90,posD=0, targetA=90,targetB=90,targetC=90,targetD=0;
const int everyA=25, everyB=15, everyC=15, everyD=15;
bool movingA=false,movingB=false,movingC=false,movingD=false;
int  stepNow=-1; bool armActive=false;

// ───────────── Chassis pins (沿用 v1.0.0) ─────────────
const int R1=22,R2=23,R3=24,R4=25, ENA1=5,ENB1=4;
const int L1=27,L2=26,L3=28,L4=29, ENA2=6,ENB2=7;

// 以側分組：0=LF, 1=LR, 2=RF, 3=RR
const uint8_t M_IN1[4] = { L1, L3, R3, R1 };
const uint8_t M_IN2[4] = { L2, L4, R4, R2 };
const uint8_t M_PWM[4] = { ENA2, ENB2, ENB1, ENA1 };

// ───────────── Comm & legacy ─────────────
const unsigned long RX_TIMEOUT=100;
unsigned long lastRx=0;
String incoming;

// ───────────── Encoder config ─────────────
// 指定 A 相外部中斷：LF:18, LR:19, RF:2, RR:3
const uint8_t ENC_A[4]={ 18, 19, 2, 3 };
// 若之後接 B 相（選用，僅作佈線參考）：LF:30, LR:32, RF:31, RR:33
const bool USE_B_PHASE=false;
const uint8_t ENC_B[4]={ 30, 32, 31, 33 };

// 計數策略：A 相上/下沿皆算 → ×2
const bool COUNT_BOTH_EDGES=true;
const int  PPR_MOTOR=3, GEAR=48;
const int  PPR_WHEEL = PPR_MOTOR*GEAR*(COUNT_BOTH_EDGES?2:1); // 288 或 144（你現在用 288）

// ISR 計數（方向由命令/目標符號推定）
volatile long encTick[4]={0,0,0,0};

// ───────────── 控制層參數 ─────────────
enum CtrlMode { OPEN_LOOP=0, SPEED_HOLD=1, MOVE_DIST=2, TURN_ANGLE=3 };
CtrlMode ctrlMode = OPEN_LOOP;

// 幾何參數（可動態指令更新）
float R_EFF_MM = 32.5f;   // 輪半徑(mm)；預 65mm 直徑
float LW_MM    = 120.0f;  // (L+W)(mm)

// 低階閉環（每 20ms）
const uint16_t CTRL_MS = 20;
unsigned long  lastCtrlMs=0;

// PI 參數
float KP=0.8f, KI=0.15f;
float integ[4]={0,0,0,0};
int   pwmCmd[4]={0,0,0,0};

// 目標/量測
float rpmRef[4]={0,0,0,0};
float rpmMeas[4]={0,0,0,0};

// 取樣快照
long prevTickCtrl[4]={0,0,0,0};
long prevTickStream[4]={0,0,0,0};

// RS 串流
bool streamRPM=false;
const uint16_t SAMPLE_MS=200;
unsigned long lastSampleMs=0;

// MOVE/TURN 狀態
float cruiseRPM = 180.0f;      // SETV 巡航
long  motionTargetPulses = 0;
long  motionStartTick[4]={0,0,0,0};

// ───────────── 小工具 ─────────────
inline int sgn(float x){ return (x>=0)?+1:-1; }
inline long atomicReadTick(uint8_t i){ noInterrupts(); long v=encTick[i]; interrupts(); return v; }

void applyWheel(uint8_t i, int pwm, int dir){
  pwm = constrain(pwm, 0, 255);
  if (dir>=0){ digitalWrite(M_IN1[i], HIGH); digitalWrite(M_IN2[i], LOW);  }
  else       { digitalWrite(M_IN1[i], LOW ); digitalWrite(M_IN2[i], HIGH); }
  analogWrite(M_PWM[i], pwm);
}

void stopAll(){
  for(int i=0;i<4;i++){
    digitalWrite(M_IN1[i], LOW);
    digitalWrite(M_IN2[i], LOW);
    analogWrite(M_PWM[i], 0);
  }
}

// ───────────── Open-loop（沿用 v1.0.0） ─────────────
void setOpenLoopForward(int pwm){
  digitalWrite(R1, LOW);  digitalWrite(R2, HIGH);
  digitalWrite(R3, HIGH); digitalWrite(R4, LOW);
  digitalWrite(L1, LOW);  digitalWrite(L2, HIGH);
  digitalWrite(L3, LOW);  digitalWrite(L4, HIGH);
  pwm = constrain(pwm,0,255);
  analogWrite(ENA1,pwm); analogWrite(ENB1,pwm);
  analogWrite(ENA2,pwm); analogWrite(ENB2,pwm);
}
void setOpenLoopBackward(int pwm){
  digitalWrite(R1, HIGH); digitalWrite(R2, LOW);
  digitalWrite(R3, LOW);  digitalWrite(R4, HIGH);
  digitalWrite(L1, HIGH); digitalWrite(L2, LOW);
  digitalWrite(L3, HIGH); digitalWrite(L4, LOW);
  pwm = constrain(pwm,0,255);
  analogWrite(ENA1,pwm); analogWrite(ENB1,pwm);
  analogWrite(ENA2,pwm); analogWrite(ENB2,pwm);
}
void setOpenLoopTurnLeft(int pwm){
  digitalWrite(R1, LOW);  digitalWrite(R2, HIGH);
  digitalWrite(R3, HIGH); digitalWrite(R4, LOW);
  digitalWrite(L1, HIGH); digitalWrite(L2, LOW);
  digitalWrite(L3, HIGH); digitalWrite(L4, LOW);
  pwm = constrain(pwm,0,255);
  analogWrite(ENA1,pwm); analogWrite(ENB1,pwm);
  analogWrite(ENA2,pwm); analogWrite(ENB2,pwm);
}
void setOpenLoopTurnRight(int pwm){
  digitalWrite(R1, HIGH); digitalWrite(R2, LOW);
  digitalWrite(R3, LOW);  digitalWrite(R4, HIGH);
  digitalWrite(L1, LOW);  digitalWrite(L2, HIGH);
  digitalWrite(L3, LOW);  digitalWrite(L4, HIGH);
  pwm = constrain(pwm,0,255);
  analogWrite(ENA1,pwm); analogWrite(ENB1,pwm);
  analogWrite(ENA2,pwm); analogWrite(ENB2,pwm);
}

// ───────────── Encoder ISR ─────────────
inline void _encEdge(uint8_t i){
  int dir = (rpmRef[i]>=0)? +1 : -1;
  if (!COUNT_BOTH_EDGES){
    if (digitalRead(ENC_A[i]) == HIGH) encTick[i] += dir;
  }else{
    encTick[i] += dir;
  }
}
void isr_LF(){ _encEdge(0); }  // LF (D18)
void isr_LR(){ _encEdge(1); }  // LR (D19)
void isr_RF(){ _encEdge(2); }  // RF (D2)
void isr_RR(){ _encEdge(3); }  // RR (D3)

// ───────────── 低階控制：每 20ms PI 一次 ─────────────
void controlStep(){
  long nowTick[4];
  for(int i=0;i<4;i++) nowTick[i]=atomicReadTick(i);
  for(int i=0;i<4;i++){
    long d = nowTick[i] - prevTickCtrl[i];
    prevTickCtrl[i] = nowTick[i];
    rpmMeas[i] = ( (double)d * 60000.0 ) / ( (double)PPR_WHEEL * CTRL_MS );
  }

  if (ctrlMode==OPEN_LOOP) return;

  for(int i=0;i<4;i++){
    float e = rpmRef[i] - rpmMeas[i];
    integ[i] += e * (CTRL_MS/1000.0f);
    integ[i] = constrain(integ[i], -500.0f, 500.0f);
    float u = KP*e + KI*integ[i];

    int dir = sgn(rpmRef[i]);
    int pwm = pwmCmd[i] + (int)u;
    pwm = constrain(pwm, 0, 255);
    pwmCmd[i] = pwm;
    applyWheel(i, pwm, dir);
  }

  if (ctrlMode==MOVE_DIST || ctrlMode==TURN_ANGLE){
    double avg = 0;
    for(int i=0;i<4;i++){
      long moved = llabs(nowTick[i] - motionStartTick[i]);
      avg += (double)moved;
    }
    avg /= 4.0;
    if (avg >= (double)motionTargetPulses){
      ctrlMode=OPEN_LOOP;
      for(int i=0;i<4;i++){ rpmRef[i]=0; integ[i]=0; pwmCmd[i]=0; }
      stopAll();
      Serial.println(F("DONE"));
    }
  }
}

// ───────────── RS 串流量測（200ms 視窗） ─────────────
void sampleRPMforStream(){
  long nowTick[4];
  for(int i=0;i<4;i++) nowTick[i]=atomicReadTick(i);
  for(int i=0;i<4;i++){
    long d = nowTick[i] - prevTickStream[i];
    prevTickStream[i] = nowTick[i];
    float rpm = ( (double)d * 60000.0 ) / ( (double)PPR_WHEEL * SAMPLE_MS );
    rpmMeas[i] = 0.6f*rpmMeas[i] + 0.4f*rpm;
  }
}
void printRPM(){
  Serial.print(F("RPM="));  Serial.println(rpmMeas[0],1);   // 相容舊：印 LF
  Serial.print(F("RPM1=")); Serial.print(rpmMeas[0],1);     // LF
  Serial.print(F(", RPM2=")); Serial.print(rpmMeas[1],1);   // LR
  Serial.print(F(", RPM3=")); Serial.print(rpmMeas[2],1);   // RF
  Serial.print(F(", RPM4=")); Serial.println(rpmMeas[3],1); // RR
}

// ───────────── Arm（保留） ─────────────
void checkServoA(){ static uint32_t t=0; if(millis()-t<everyA) return; t=millis();
  if      (posA<targetA) posA++;
  else if (posA>targetA) posA--;
  else { movingA=false; return; }
  motorA.write(posA);
}
void checkServoB(){ static uint32_t t=0; if(millis()-t<everyB) return; t=millis();
  if      (posB<targetB) posB++;
  else if (posB>targetB) posB--;
  else { movingB=false; return; }
  motorB.write(posB);
}
void checkServoC(){ static uint32_t t=0; if(millis()-t<everyC) return; t=millis();
  if      (posC<targetC) posC++;
  else if (posC>targetC) posC--;
  else { movingC=false; return; }
  motorC.write(posC);
}
void checkServoD(){ static uint32_t t=0; if(millis()-t<everyD) return; t=millis();
  if      (posD<targetD) posD++;
  else if (posD>targetD) posD--;
  else { movingD=false; return; }
  motorD.write(posD);
}
void executeSequence(){
  static unsigned long lastTime=0; static bool waiting=false;
  switch(stepNow){
    case 0:  targetC=180; movingC=true; stepNow++; break;
    case 1:  if(!movingC && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){ waiting=false; stepNow++; } break;
    case 2:  targetB=108; movingB=true; stepNow++; break;
    case 3:  if(!movingB && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){ waiting=false; stepNow++; } break;
    case 4:  targetA=60;  movingA=true; stepNow++; break;
    case 5:  if(!movingA && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){ waiting=false; targetC=90; movingC=true; targetB=180; movingB=true; stepNow++; } break;
    case 6:  if(!movingC && !movingB && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){ waiting=false; targetD=180; movingD=true; stepNow++; } break;
    case 7:  if(!movingD && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){ waiting=false; targetC=70; movingC=true; stepNow++; } break;
    case 8:  if(!movingC && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){ waiting=false; targetA=90; movingA=true; stepNow++; } break;
    case 9:  if(!movingA && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){ waiting=false; targetB=90; movingB=true; stepNow++; } break;
    case 10: if(!movingB && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){ waiting=false; targetD=0; movingD=true; stepNow++; } break;
    case 11: if(!movingD && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){ waiting=false; stepNow++; } break;
    case 12: Serial.println(F("✅ Arm sequence DONE")); stepNow=-1; armActive=false; break;
  }
}

// ───────────── 高階動作（閉環距離/角度） ─────────────
void startMoveByPulses(long pulses, int dirSign){
  ctrlMode = MOVE_DIST;
  motionTargetPulses = labs(pulses);
  for(int i=0;i<4;i++){
    motionStartTick[i]=atomicReadTick(i);
    rpmRef[i] = cruiseRPM * dirSign;  // 四輪同號
    integ[i]=0; pwmCmd[i]=120;
  }
}
void startTurnByPulses(long pulses, int sign){ // sign: +1 右、-1 左
  ctrlMode = TURN_ANGLE;
  motionTargetPulses = labs(pulses);
  const int pat[4] = { -1, -1, +1, +1 }; // 左側負、右側正
  for(int i=0;i<4;i++){
    motionStartTick[i]=atomicReadTick(i);
    rpmRef[i] = cruiseRPM * pat[i] * sign;
    integ[i]=0; pwmCmd[i]=120;
  }
}

void handleCommand(const String& s){
  String cmd=s; cmd.trim(); if(cmd.length()==0){ Serial.println(F("DONE")); return; }

  if(cmd=="S"){
    ctrlMode=OPEN_LOOP;
    for(int i=0;i<4;i++){ rpmRef[i]=0; integ[i]=0; pwmCmd[i]=0; }
    stopAll(); Serial.println(F("DONE")); return;
  }
  if(cmd=="G"){
    if(!armActive){ armActive=true; stepNow=0;
      movingA=movingB=movingC=movingD=false;
      targetA=posA; targetB=posB; targetC=posC; targetD=posD;
      Serial.println(F("Arm sequence START"));
    }
    Serial.println(F("DONE")); return;
  }
  if(cmd=="R"){ sampleRPMforStream(); printRPM(); return; }
  if(cmd=="RS"){ streamRPM=true;  Serial.println(F("DONE")); return; }
  if(cmd=="RX"){ streamRPM=false; Serial.println(F("DONE")); return; }

  int comma=cmd.indexOf(',');
  String op=(comma>0)?cmd.substring(0,comma):cmd;
  String arg=(comma>0)?cmd.substring(comma+1):"";
  op.trim(); arg.trim();

  // 開環維持
  if(op=="F"){ ctrlMode=OPEN_LOOP; setOpenLoopForward(arg.toInt());  Serial.println(F("DONE")); return; }
  if(op=="B"){ ctrlMode=OPEN_LOOP; setOpenLoopBackward(arg.toInt()); Serial.println(F("DONE")); return; }
  if(op=="TL"){ ctrlMode=OPEN_LOOP; setOpenLoopTurnLeft(arg.toInt());  Serial.println(F("DONE")); return; }
  if(op=="TR"){ ctrlMode=OPEN_LOOP; setOpenLoopTurnRight(arg.toInt()); Serial.println(F("DONE")); return; }

  // 閉環等速
  if(op=="EQ"){
    float r = arg.toFloat();
    ctrlMode = SPEED_HOLD;
    for(int i=0;i<4;i++){ rpmRef[i]=r; integ[i]=0; pwmCmd[i]=120; }
    Serial.println(F("DONE")); return;
  }

  // MOVE/TURN 參數
  if(op=="SETV"){ cruiseRPM = constrain(arg.toFloat(), -400.0f, 400.0f); Serial.println(F("DONE")); return; }

  if(op=="MOVE"){
    float mm = arg.toFloat();
    float C_mm = 2.0f*PI*R_EFF_MM;
    float pulses_per_mm = (float)PPR_WHEEL / C_mm;
    long  pulses = (long)fabs(mm * pulses_per_mm);
    startMoveByPulses(pulses, (mm>=0)?+1:-1);
    Serial.println(F("DONE")); return;
  }

  if(op=="TURN"){
    float deg = arg.toFloat();
    float rad = fabs(deg) * PI / 180.0f;
    float C_mm = 2.0f*PI*R_EFF_MM;
    float pulses_per_mm = (float)PPR_WHEEL / C_mm;
    float arc_mm = rad * LW_MM;  // 每側弧長
    long  pulses = (long)fabs(arc_mm * pulses_per_mm);
    startTurnByPulses(pulses, (deg>=0)?+1:-1); // 正=右轉
    Serial.println(F("DONE")); return;
  }

  if(op=="SETR"){ R_EFF_MM = max(1.0f, arg.toFloat()); Serial.println(F("DONE")); return; }
  if(op=="SETLW"){ LW_MM   = max(1.0f, arg.toFloat()); Serial.println(F("DONE")); return; }

  Serial.println(F("ERR"));
}

// ───────────── Setup / Loop ─────────────
void setup(){
  Serial.begin(57600);
  lastRx=millis();

  for(int i=0;i<4;i++){ pinMode(M_IN1[i],OUTPUT); pinMode(M_IN2[i],OUTPUT); pinMode(M_PWM[i],OUTPUT); }
  stopAll();

  // Servo 腳位（避開 4/5/6/7 馬達 PWM）
  motorA.attach(8, 544, 2400);
  motorB.attach(9);
  motorC.attach(10);
  motorD.attach(11);
  motorA.write(posA); motorB.write(posB); motorC.write(posC); motorD.write(posD);

  // Encoder A 相中斷
  for(int i=0;i<4;i++) pinMode(ENC_A[i], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), isr_LF, COUNT_BOTH_EDGES?CHANGE:RISING); // LF D18
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), isr_LR, COUNT_BOTH_EDGES?CHANGE:RISING); // LR D19
  attachInterrupt(digitalPinToInterrupt(ENC_A[2]), isr_RF, COUNT_BOTH_EDGES?CHANGE:RISING); // RF D2
  attachInterrupt(digitalPinToInterrupt(ENC_A[3]), isr_RR, COUNT_BOTH_EDGES?CHANGE:RISING); // RR D3

  if(USE_B_PHASE){ for(int i=0;i<4;i++) pinMode(ENC_B[i], INPUT_PULLUP); }

  lastCtrlMs=millis(); lastSampleMs=millis();

  Serial.println(F("READY v1.3-CL (Base v1.0.0 + Enc/PI/MOVE/TURN) [LF:18,LR:19,RF:2,RR:3]"));
  Serial.print (F("PPR_wheel=")); Serial.println(PPR_WHEEL);
  Serial.print (F("R_eff(mm)=")); Serial.print(R_EFF_MM);
  Serial.print (F(", L+W(mm)=")); Serial.println(LW_MM);
  Serial.println(F("Commands: F,B,TL,TR,<pwm> | S | G | R | RS | RX | EQ,<rpm> | SETV,<rpm> | MOVE,<mm> | TURN,<deg> | SETR,<mm> | SETLW,<mm>"));
}

void loop(){
  while(Serial.available()){
    char c=(char)Serial.read();
    if(c=='\n'){ handleCommand(incoming); incoming=""; lastRx=millis(); }
    else if(c!='\r'){ incoming+=c; }
  }

  if(millis()-lastRx > RX_TIMEOUT && ctrlMode==OPEN_LOOP){ stopAll(); }

  if(millis()-lastCtrlMs >= CTRL_MS){
    controlStep();
    lastCtrlMs += CTRL_MS;
  }

  if(streamRPM && millis()-lastSampleMs >= SAMPLE_MS){
    sampleRPMforStream();
    printRPM();
    lastSampleMs += SAMPLE_MS;
  }

  if(armActive){ checkServoA(); checkServoB(); checkServoC(); checkServoD(); executeSequence(); }
}
