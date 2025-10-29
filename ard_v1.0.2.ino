// ===== Final v1.0.0 → 4輪編碼器整合版 v1.2  =====
// 依原檔案 v1.0.0 保留：手臂 Servo、底盤腳位、指令協定（含 G 啟動手臂）
// 新增：四輪編碼器(A相中斷)、R/RS/RX 速度回傳
// -----------------------------------------------

#include <Arduino.h>
#include <Servo.h>

// ────────────────────────────────────────
//  Robotic Arm Servo Definitions (保留 v1.0.0)
// ────────────────────────────────────────
Servo motorA;   // SG90     - Base
Servo motorB;   // LD-220MG - Shoulder
Servo motorC;   // LD-220MG - Elbow
Servo motorD;   // MG996R   - Gripper

int posA = 90,  posB = 90,  posC = 90,  posD = 0;            // current angles
int targetA = 90, targetB = 90, targetC = 90, targetD = 0;   // target angles

const int everyA = 25;   // ms/deg for smooth motion
const int everyB = 15;
const int everyC = 15;
const int everyD = 15;

bool movingA = false, movingB = false, movingC = false, movingD = false;
int  stepNow  = -1;      // -1 idle, 0..N sequence steps
bool armActive = false;

// ────────────────────────────────────────
//  Chassis Motor Pins (沿用 v1.0.0 不變)
// ────────────────────────────────────────
const int R1   = 22; // right-rear IN1
const int R2   = 23; // right-rear IN2
const int R3   = 24; // right-front IN3
const int R4   = 25; // right-front IN4
const int ENA1 = 5;  // right-rear PWM
const int ENB1 = 4;  // right-front PWM

const int L1   = 27; // left-front IN1
const int L2   = 26; // left-front IN2
const int L3   = 28; // left-rear IN3
const int L4   = 29; // left-rear IN4
const int ENA2 = 6;  // left-front PWM
const int ENB2 = 7;  // left-rear PWM

// ────────────────────────────────────────
//  Communication & Timeout (沿用 v1.0.0)
// ────────────────────────────────────────
const unsigned long RX_TIMEOUT = 100;  // ms
unsigned long lastRx = 0;
String incoming;

// ────────────────────────────────────────
//  PWM Range
// ────────────────────────────────────────
const int PWM_MIN = 0;
const int PWM_MAX = 255;

// ────────────────────────────────────────
//  Encoder (新增；可只接 H1，也可 A中斷+B一般腳判向)
// ────────────────────────────────────────
static const bool USE_B_PHASE        = false;  // 接上 B 相要判方向就設 true
static const bool COUNT_BOTH_EDGES   = true;   // true: A相上下沿計數→288；false: 只上升沿→144
static const int  PPR_MOTOR = 3;              // 馬達軸每圈 3 週期
static const int  GEAR      = 48;             // 減速比
static const int  PPR_WHEEL = PPR_MOTOR * GEAR * (COUNT_BOTH_EDGES ? 2 : 1); // 288 或 144

// A 相接外部中斷 (Mega: 2,3,18,19)
const uint8_t ENC_A_PIN[4] = {2, 3, 18, 19};
// B 相接一般腳（可不接；USE_B_PHASE=true 才會用）
const uint8_t ENC_B_PIN[4] = {30, 31, 32, 33};

volatile long encCount[4] = {0,0,0,0};
volatile int  cmdDir  [4] = {0,0,0,0};   // 只接 H1 時用指令方向推定
float lastRPM[4] = {0,0,0,0};
bool  streamRPM  = false;
unsigned long lastSampleMs = 0;

inline void _encEdge(uint8_t i){
  uint8_t a = digitalRead(ENC_A_PIN[i]);
  int delta;
  if (USE_B_PHASE) {
    uint8_t b = digitalRead(ENC_B_PIN[i]);
    // 依接線可能顛倒，若方向相反再在 IN1/IN2 或此處調整
    delta = (a == b) ? +1 : -1;
  } else {
    delta = (cmdDir[i] >= 0) ? +1 : -1; // 單相靠命令方向推定
  }

  if (COUNT_BOTH_EDGES) {
    encCount[i] += delta;
  } else {
    if (a == HIGH) encCount[i] += delta; // 只上升沿
  }
}
void isrA0(){ _encEdge(0); }
void isrA1(){ _encEdge(1); }
void isrA2(){ _encEdge(2); }
void isrA3(){ _encEdge(3); }

void computeRPMs(uint16_t dt_ms){
  noInterrupts();
  long c0=encCount[0], c1=encCount[1], c2=encCount[2], c3=encCount[3];
  encCount[0]=encCount[1]=encCount[2]=encCount[3]=0;
  interrupts();
  const double k = 60000.0 / (double)(PPR_WHEEL * dt_ms);
  lastRPM[0] = c0 * k; lastRPM[1] = c1 * k; lastRPM[2] = c2 * k; lastRPM[3] = c3 * k;
}
void printRPM(){
  // 舊版相容：仍輸出一行 "RPM="
  Serial.print(F("RPM="));  Serial.println(lastRPM[0],1);
  // 四輪完整
  Serial.print(F("RPM1=")); Serial.print(lastRPM[0],1);
  Serial.print(F(", RPM2=")); Serial.print(lastRPM[1],1);
  Serial.print(F(", RPM3=")); Serial.print(lastRPM[2],1);
  Serial.print(F(", RPM4=")); Serial.println(lastRPM[3],1);
}

// ────────────────────────────────────────
//  Chassis Control Helpers（沿用 v1.0.0）
// ────────────────────────────────────────
void stopAll() {
  digitalWrite(R1, LOW);  digitalWrite(R2, LOW);
  digitalWrite(R3, LOW);  digitalWrite(R4, LOW);
  digitalWrite(L1, LOW);  digitalWrite(L2, LOW);
  digitalWrite(L3, LOW);  digitalWrite(L4, LOW);
  analogWrite(ENA1, 0);   analogWrite(ENB1, 0);
  analogWrite(ENA2, 0);   analogWrite(ENB2, 0);
}
void driveForward(int pwm) {
  digitalWrite(R1, LOW);  digitalWrite(R2, HIGH);
  digitalWrite(R3, HIGH); digitalWrite(R4, LOW);
  digitalWrite(L1, LOW);  digitalWrite(L2, HIGH);
  digitalWrite(L3, LOW);  digitalWrite(L4, HIGH);
  pwm = constrain(pwm, PWM_MIN, PWM_MAX);
  analogWrite(ENA1, pwm); analogWrite(ENB1, pwm);
  analogWrite(ENA2, pwm); analogWrite(ENB2, pwm);
  cmdDir[0]=cmdDir[1]=cmdDir[2]=cmdDir[3]=+1;
}
void driveBackward(int pwm) {
  digitalWrite(R1, HIGH); digitalWrite(R2, LOW);
  digitalWrite(R3, LOW);  digitalWrite(R4, HIGH);
  digitalWrite(L1, HIGH); digitalWrite(L2, LOW);
  digitalWrite(L3, HIGH); digitalWrite(L4, LOW);
  pwm = constrain(pwm, PWM_MIN, PWM_MAX);
  analogWrite(ENA1, pwm); analogWrite(ENB1, pwm);
  analogWrite(ENA2, pwm); analogWrite(ENB2, pwm);
  cmdDir[0]=cmdDir[1]=cmdDir[2]=cmdDir[3]=-1;
}
void turnLeft(int pwm) {   // 原地左轉：左退右進
  digitalWrite(R1, LOW);  digitalWrite(R2, HIGH);
  digitalWrite(R3, HIGH); digitalWrite(R4, LOW);
  digitalWrite(L1, HIGH); digitalWrite(L2, LOW);
  digitalWrite(L3, HIGH); digitalWrite(L4, LOW);
  pwm = constrain(pwm, PWM_MIN, PWM_MAX);
  analogWrite(ENA1, pwm); analogWrite(ENB1, pwm);
  analogWrite(ENA2, pwm); analogWrite(ENB2, pwm);
  cmdDir[0]=cmdDir[2]=-1; cmdDir[1]=cmdDir[3]=+1;
}
void turnRight(int pwm) {  // 原地右轉：左進右退
  digitalWrite(R1, HIGH); digitalWrite(R2, LOW);
  digitalWrite(R3, LOW);  digitalWrite(R4, HIGH);
  digitalWrite(L1, LOW);  digitalWrite(L2, HIGH);
  digitalWrite(L3, LOW);  digitalWrite(L4, HIGH);
  pwm = constrain(pwm, PWM_MIN, PWM_MAX);
  analogWrite(ENA1, pwm); analogWrite(ENB1, pwm);
  analogWrite(ENA2, pwm); analogWrite(ENB2, pwm);
  cmdDir[0]=cmdDir[2]=+1; cmdDir[1]=cmdDir[3]=-1;
}

// ────────────────────────────────────────
//  Smooth Servo Motion（保留）
// ────────────────────────────────────────
void checkServoA(){ static uint32_t t=0; if(millis()-t<everyA) return; t=millis();
  if      (posA < targetA) posA++;
  else if (posA > targetA) posA--;
  else { movingA=false; return; }
  motorA.write(posA);
}
void checkServoB(){ static uint32_t t=0; if(millis()-t<everyB) return; t=millis();
  if      (posB < targetB) posB++;
  else if (posB > targetB) posB--;
  else { movingB=false; return; }
  motorB.write(posB);
}
void checkServoC(){ static uint32_t t=0; if(millis()-t<everyC) return; t=millis();
  if      (posC < targetC) posC++;
  else if (posC > targetC) posC--;
  else { movingC=false; return; }
  motorC.write(posC);
}
void checkServoD(){ static uint32_t t=0; if(millis()-t<everyD) return; t=millis();
  if      (posD < targetD) posD++;
  else if (posD > targetD) posD--;
  else { movingD=false; return; }
  motorD.write(posD);
}

// ────────────────────────────────────────
//  Arm Sequence（保留：依 v1.0.0 的結構重寫步驟）
// ────────────────────────────────────────
void executeSequence(){
  static unsigned long lastTime=0; static bool waiting=false;
  switch(stepNow){
    case 0:  targetC=180; movingC=true; stepNow++; break;         // 肘部抬起
    case 1:  if(!movingC && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){ waiting=false; stepNow++; }
             break;
    case 2:  targetB=108; movingB=true; stepNow++; break;         // 肩下壓
    case 3:  if(!movingB && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){ waiting=false; stepNow++; }
             break;
    case 4:  targetA=60;  movingA=true; stepNow++; break;         // 底座旋轉
    case 5:  if(!movingA && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){
               waiting=false; targetC=90; movingC=true; targetB=180; movingB=true; stepNow++;
             }
             break;
    case 6:  if(!movingC && !movingB && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){ waiting=false; targetD=180; movingD=true; stepNow++; } // 夾爪收合
             break;
    case 7:  if(!movingD && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){ waiting=false; targetC=70; movingC=true; stepNow++; }  // 抬起
             break;
    case 8:  if(!movingC && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){ waiting=false; targetA=90; movingA=true; stepNow++; }  // 旋回
             break;
    case 9:  if(!movingA && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){ waiting=false; targetB=90; movingB=true; stepNow++; }  // 肩復位
             break;
    case 10: if(!movingB && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){ waiting=false; targetD=0; movingD=true; stepNow++; }   // 放下夾爪
             break;
    case 11: if(!movingD && !waiting){ lastTime=millis(); waiting=true; }
             if(waiting && millis()-lastTime>=1000){ waiting=false; stepNow++; }
             break;
    case 12: Serial.println(F("✅ Arm sequence DONE")); stepNow=-1; armActive=false; break;
  }
}

// ────────────────────────────────────────
//  Setup（含手臂保留 + 編碼器初始化）
// ────────────────────────────────────────
void setup(){
  Serial.begin(57600);
  lastRx = millis();

  // 底盤 GPIO
  pinMode(R1,OUTPUT); pinMode(R2,OUTPUT); pinMode(R3,OUTPUT); pinMode(R4,OUTPUT);
  pinMode(ENA1,OUTPUT); pinMode(ENB1,OUTPUT); pinMode(ENA2,OUTPUT); pinMode(ENB2,OUTPUT);
  pinMode(L1,OUTPUT); pinMode(L2,OUTPUT); pinMode(L3,OUTPUT); pinMode(L4,OUTPUT);
  stopAll();

  // 手臂 Servo（沿用 v1.0.0）
  motorA.attach(8, 544, 2400);
  motorB.attach(9);
  motorC.attach(10);
  motorD.attach(11);
  motorA.write(posA); motorB.write(posB); motorC.write(posC); motorD.write(posD);

  // 編碼器 A 相中斷、B 相一般腳
  for(int i=0;i<4;i++){
    pinMode(ENC_A_PIN[i], INPUT_PULLUP);
    if(USE_B_PHASE) pinMode(ENC_B_PIN[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[0]), isrA0, COUNT_BOTH_EDGES?CHANGE:RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[1]), isrA1, COUNT_BOTH_EDGES?CHANGE:RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[2]), isrA2, COUNT_BOTH_EDGES?CHANGE:RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[3]), isrA3, COUNT_BOTH_EDGES?CHANGE:RISING);

  lastSampleMs = millis();

  Serial.println(F("READY v1.2 (Base=v1.0.0, +4x encoders)"));
  Serial.print (F("PPR_wheel=")); Serial.println(PPR_WHEEL);
  Serial.println(F("Commands: F,B,TL,TR,<pwm> | S | G(arm) | R | RS | RX"));
}

// ────────────────────────────────────────
void loop(){
  // 1) Serial command
  while(Serial.available()){
    char c = (char)Serial.read();
    if(c=='\n'){
      lastRx = millis();
      String cmdline = incoming; incoming = "";
      cmdline.trim();
      if(cmdline.length()==0) { Serial.println(F("DONE")); continue; }

      if(cmdline=="S"){
        stopAll(); for(int i=0;i<4;i++) cmdDir[i]=0; Serial.println(F("DONE")); continue;
      }
      if(cmdline=="G"){  // 啟動手臂流程（保留）
        if(!armActive){ armActive=true; stepNow=0;
          movingA=movingB=movingC=movingD=false;
          targetA=posA; targetB=posB; targetC=posC; targetD=posD;
          Serial.println(F("Arm sequence START"));
        }
        Serial.println(F("DONE")); continue;
      }
      if(cmdline=="R"){
        unsigned long now=millis();
        uint16_t dt = (uint16_t)max( (unsigned long)50, now-lastSampleMs );
        computeRPMs(dt); lastSampleMs = now; printRPM(); continue;
      }
      if(cmdline=="RS"){ streamRPM=true;  Serial.println(F("DONE")); continue; }
      if(cmdline=="RX"){ streamRPM=false; Serial.println(F("DONE")); continue; }

      // 其餘: 有 <cmd,pwm>
      int comma = cmdline.indexOf(',');
      String op = (comma>0)? cmdline.substring(0,comma) : cmdline;
      int pwm   = (comma>0)? cmdline.substring(comma+1).toInt() : 0;

      if      (op=="F")  { driveForward(pwm);  Serial.println(F("DONE")); }
      else if (op=="B")  { driveBackward(pwm); Serial.println(F("DONE")); }
      else if (op=="TL") { turnLeft(pwm);      Serial.println(F("DONE")); }
      else if (op=="TR") { turnRight(pwm);     Serial.println(F("DONE")); }
      else               { Serial.println(F("ERR")); }
    }
    else if(c!='\r'){ incoming += c; }
  }

  // 2) 通訊逾時自動停車
  if (millis() - lastRx > RX_TIMEOUT) stopAll();

  // 3) 手臂序列運行（保留）
  if (armActive) {
    checkServoA();
    checkServoB();
    checkServoC();
    checkServoD();
    executeSequence();
  }

  // 4) RS 串流 RPM
  if (streamRPM && millis() - lastSampleMs >= 200) {
    computeRPMs(200);
    printRPM();
    lastSampleMs = millis();
  }
}
