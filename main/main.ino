// ================== Mapping: Arduino Mega 2560 (interrupt) ==================
#define PI1 2      // INT4
#define PI2 3      // INT5
#define PI3 18     // INT3
#define PI4 19     // INT2
#define PI5 20     // INT1
#define RUN_PIN 21 // INT0 - nút toggle (INPUT_PULLUP): nhấn = GND

#define PO1 8
#define PO2 9
#define PO3 10
#define PO4 11
#define POA 12   // ENA (PWM)
#define POB 13   // ENB (PWM)

// ========== Cấu hình logic cảm biến ==========
#define SENSOR_LEFT_IS_PI1  true   // nếu trái/phải ngược, đổi true/false

// ================== PID & Motion params ==================
float Kp = 55.0f;
float Ki = 0.0f;
float Kd = 20.0f;
float Kaw = 0.40f;

int   basePWM   = 15;
int   minPWM    = 0;
int   maxPWM    = 255;
int   maxTurn   = 255;

// Dynamic speed theo |u| (dùng ở RUN)
int   minBase   = 100;
float kSpeed    = 0.60f;

// ALIGN: dùng chung PID, chỉ thêm deadband nhỏ để đứng im khi đã vào line
float alignDeadbandE = 0.12f;   // 0.1–0.2 tuỳ track

// ====== Timing & options ======
#define PID_DT_MS 1
#define SERIAL_THROTTLE_MS 100
#define USE_SLEW 0
float U_SLEW_PER_SEC = 5000.0f;

const float I_MIN = -1000.0f, I_MAX = 1000.0f;

// ====== Derivative filter (low-pass) ======
float derFilt = 0.0f;
const float DER_ALPHA = 0.25f;

// ================== State ==================
volatile uint8_t g_sensors = 0;  // bit0..bit4 = PI1..PI5 (1 = line đen/LOW)
float lastError = 0.0f, integral = 0.0f, u_prev = 0.0f;
unsigned long lastPidMs = 0, lastPrintMs = 0;

// Chế độ điều khiển
enum Mode { MODE_AUTO = 0, MODE_ALIGN = 1, MODE_RUN = 2 };
Mode forceMode = MODE_AUTO;              // serial ép mode
volatile bool runBtnPressed = false;     // flag từ ISR
bool runEnabled = false;                 // trạng thái toggle (false=ALIGN, true=RUN)
unsigned long lastToggleMs = 0;
const unsigned long RUN_TOGGLE_DEBOUNCE_MS = 200;

// ================== Helpers & ISRs ==================
inline float clamp(float x,float a,float b){ return x<a?a:(x>b?b:x); }
inline int   clampi(int x,int a,int b){ return x<a?a:(x>b?b:x); }
inline float signf(float x){ return (x>0)-(x<0); }

// --- ISR cảm biến (LOW -> 1) ---
inline void isrUpdateBit(uint8_t bitIndex, uint8_t pin){
  uint8_t m = (1u << bitIndex);
  if (digitalRead(pin) == LOW) g_sensors |= m;
  else                         g_sensors &= ~m;
}
void isrPI1(){ isrUpdateBit(0, PI1); }
void isrPI2(){ isrUpdateBit(1, PI2); }
void isrPI3(){ isrUpdateBit(2, PI3); }
void isrPI4(){ isrUpdateBit(3, PI4); }
void isrPI5(){ isrUpdateBit(4, PI5); }

// --- ISR nút toggle ---
void isrRunBtn(){ runBtnPressed = true; }

// ==== Sensing & Motors ====
float computeLineError(){
  noInterrupts(); uint8_t s = g_sensors; interrupts();
  int b[5] = { (s>>0)&1, (s>>1)&1, (s>>2)&1, (s>>3)&1, (s>>4)&1 };
  int w[5];
  if (SENSOR_LEFT_IS_PI1) { int t[5]={-2,-1,0,1,2};  for(int i=0;i<5;i++) w[i]=t[i]; }
  else                    { int t[5]={ 2, 1,0,-1,-2}; for(int i=0;i<5;i++) w[i]=t[i]; }
  int sumB = b[0]+b[1]+b[2]+b[3]+b[4];
  if (sumB == 0) return lastError;              // lost-line: giữ hướng cũ
  int num=0; for(int i=0;i<5;i++) num += w[i]*b[i];
  return (float)num / (float)sumB;
}

void driveHBridge(int INx, int INy, int EN, int pwm){
  pwm = clampi(pwm, -maxPWM, maxPWM);
  if (pwm == 0){ digitalWrite(INx, LOW); digitalWrite(INy, LOW); analogWrite(EN, 0); return; }
  bool forward = (pwm > 0);
  int mag = abs(pwm); if (mag > 0) mag = max(mag, minPWM);  // cùng minPWM cho cả ALIGN/RUN
  digitalWrite(INx, forward ? HIGH : LOW);
  digitalWrite(INy, forward ? LOW  : HIGH);
  analogWrite(EN, mag);
}
void setMotors(int pwmL,int pwmR){
  driveHBridge(PO1, PO2, POA, pwmL);
  driveHBridge(PO4, PO3, POB, pwmR);
}
void motorsStop(){
  digitalWrite(PO1,LOW); digitalWrite(PO2,LOW);
  digitalWrite(PO3,LOW); digitalWrite(PO4,LOW);
  analogWrite(POA,0); analogWrite(POB,0);
}

// ================== Serial (rút gọn phần mode) ==================
String readLine;
String nextToken(String &s){
  s.trim(); if (!s.length()) return "";
  int sp = s.indexOf(' ');
  if (sp<0){ String t=s; s=""; return t; }
  String t = s.substring(0, sp); s = s.substring(sp+1); return t;
}
bool isNoChange(const String &t){ return (t.length()>0 && (t[0]=='n' || t[0]=='N')); }
bool applyFloatToken(float &var,const String &tok){
  if (!tok.length() || isNoChange(tok)) return false;
  char c=tok[0]; float val = tok.substring((c=='+'||c=='-'||c=='*'||c=='/')?1:0).toFloat();
  if (c=='+') var += val; else if (c=='-') var -= val;
  else if (c=='*') var *= (val==0 ? 1.0f : val);
  else if (c=='/') var /= (val==0 ? 1.0f : val);
  else var = tok.toFloat();
  return true;
}
bool applyIntToken(int &var,const String &tok,int lo,int hi){
  if (!tok.length() || isNoChange(tok)) return false;
  char c=tok[0]; int ival = tok.substring((c=='+'||c=='-'||c=='*'||c=='/')?1:0).toFloat();
  if (c=='+') var += ival; else if (c=='-') var -= ival;
  else if (c=='*') var = (int)(var * (ival==0 ? 1 : ival));
  else if (c=='/') var = (ival==0 ? var : (int)(var / ival));
  else var = (int)tok.toFloat();
  var = clampi(var, lo, hi);
  return true;
}
void printParams(){
  Serial.print("Kp="); Serial.print(Kp,1);
  Serial.print(" Ki="); Serial.print(Ki,2);
  Serial.print(" Kd="); Serial.print(Kd,1);
  Serial.print(" Kaw="); Serial.print(Kaw,2);
  Serial.print(" base="); Serial.print(basePWM);
  Serial.print(" min="); Serial.print(minPWM);
  Serial.print(" maxTurn="); Serial.print(maxTurn);
  Serial.print(" dyn[minBase="); Serial.print(minBase);
  Serial.print(" kSpeed="); Serial.print(kSpeed,2); Serial.print("]");
  Serial.print(" alignDB="); Serial.print(alignDeadbandE,2);
  Serial.print(" mode=");
  if (forceMode==MODE_ALIGN) Serial.print("ALIGN*");
  else if (forceMode==MODE_RUN) Serial.print("RUN*");
  else Serial.print(runEnabled ? "AUTO(RUN)" : "AUTO(ALIGN)");
  Serial.println();
}
void printHelp(){
  Serial.println(F("Commands: show, help, pid/kp/ki/kd/base/minpwm/maxturn/minbase/kspeed"));
  Serial.println(F("          mode align | mode run | mode auto"));
  Serial.println(F("          aligndb <f>"));
}
void handleSerial(){
  while (Serial.available()){
    char ch=Serial.read(); if (ch=='\r') continue;
    if (ch=='\n'){
      String cmd=readLine; readLine=""; cmd.trim(); if (!cmd.length()) return;
      String s=cmd; String op=nextToken(s); op.toLowerCase();
      if (op=="show"){ printParams(); }
      else if (op=="help"||op=="h"||op=="?"){ printHelp(); }
      else if (op=="pid"){
        String t1=nextToken(s), t2=nextToken(s), t3=nextToken(s);
        bool a1=applyFloatToken(Kp,t1);
        bool a2=applyFloatToken(Ki,t2);
        bool a3=applyFloatToken(Kd,t3);
        if (a1||a2||a3) integral=0;
        printParams();
      } else if (op=="kp"){ String t=nextToken(s); if (applyFloatToken(Kp,t)) integral=0; printParams(); }
      else if (op=="ki"){ String t=nextToken(s); if (applyFloatToken(Ki,t)) integral=0; printParams(); }
      else if (op=="kd"){ String t=nextToken(s); applyFloatToken(Kd,t); printParams(); }
      else if (op=="base"){ String t=nextToken(s); applyIntToken(basePWM,t,0,255); printParams(); }
      else if (op=="minpwm"){ String t=nextToken(s); applyIntToken(minPWM,t,0,255); printParams(); }
      else if (op=="maxturn"){ String t=nextToken(s); applyIntToken(maxTurn,t,0,255); printParams(); }
      else if (op=="minbase"){ String t=nextToken(s); applyIntToken(minBase,t,0,255); printParams(); }
      else if (op=="kspeed"){ String t=nextToken(s); applyFloatToken(kSpeed,t); kSpeed=clamp(kSpeed,0,1); printParams(); }
      else if (op=="aligndb"){ String t=nextToken(s); applyFloatToken(alignDeadbandE,t); printParams(); }
      else if (op=="mode"){
        String t=nextToken(s); t.toLowerCase();
        if (t=="align"){ forceMode=MODE_ALIGN; }
        else if (t=="run"){ forceMode=MODE_RUN; }
        else { forceMode=MODE_AUTO; }
        Serial.print(F("Mode set: ")); Serial.println(t);
      } else {
        Serial.print(F("Unknown cmd: ")); Serial.println(op);
        Serial.println(F("Type 'help'."));
      }
    } else {
      readLine += ch;
      if (readLine.length() > 120) readLine = "";
    }
  }
}

// ================== Setup ==================
void setup(){
  Serial.begin(115200);

  pinMode(PI1,INPUT_PULLUP); pinMode(PI2,INPUT_PULLUP);
  pinMode(PI3,INPUT_PULLUP); pinMode(PI4,INPUT_PULLUP); pinMode(PI5,INPUT_PULLUP);

  pinMode(RUN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RUN_PIN), isrRunBtn, FALLING); // nhấn -> LOW

  // init bitmask
  isrPI1(); isrPI2(); isrPI3(); isrPI4(); isrPI5();

  attachInterrupt(digitalPinToInterrupt(PI1), isrPI1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PI2), isrPI2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PI3), isrPI3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PI4), isrPI4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PI5), isrPI5, CHANGE);

  pinMode(PO1,OUTPUT); pinMode(PO2,OUTPUT);
  pinMode(PO3,OUTPUT); pinMode(PO4,OUTPUT);
  pinMode(POA,OUTPUT); pinMode(POB,OUTPUT);

  motorsStop();
  lastPidMs = millis();
  lastPrintMs = millis();

  runEnabled = false; // khởi động ở ALIGN
  Serial.println("Toggle D21: RUN<->ALIGN. Một luật PID duy nhất cho cả 2 chế độ.");
}

// ================== Loop ==================
void loop(){
  handleSerial();

  // Toggle RUN/ALIGN (debounce) khi MODE_AUTO
  if (forceMode == MODE_AUTO && runBtnPressed){
    unsigned long t = millis();
    if (t - lastToggleMs >= RUN_TOGGLE_DEBOUNCE_MS){
      lastToggleMs = t;
      runEnabled = !runEnabled;
      integral = 0; derFilt = 0; // reset nhẹ khi đổi mode
      Serial.print("Toggled -> "); Serial.println(runEnabled ? "RUN" : "ALIGN");
    }
    runBtnPressed = false;
  }

  unsigned long now = millis();
  if (now - lastPidMs >= PID_DT_MS){
    const float dt = (float)PID_DT_MS / 1000.0f;
    lastPidMs += PID_DT_MS;

    // Mode thực tế
    Mode modeNow = forceMode;
    if (modeNow == MODE_AUTO) modeNow = runEnabled ? MODE_RUN : MODE_ALIGN;

    // ======= PID duy nhất =======
    float e   = computeLineError();
    float rawDer = (e - lastError) / dt;
    derFilt = DER_ALPHA * rawDer + (1.0f - DER_ALPHA) * derFilt;

    float u_pid = Kp*e + Ki*integral + Kd*derFilt;
    float u_sat = clamp(u_pid, -maxTurn, +maxTurn);

    float I_dot = e + Kaw*(u_sat - u_pid);
    bool saturated = (u_sat != u_pid);
    if (saturated){
      if ((u_sat > 0 && I_dot > 0) || (u_sat < 0 && I_dot < 0)) I_dot = 0.0f;
    }
    integral += I_dot * dt;
    integral  = clamp(integral, I_MIN, I_MAX);

    float u_cmd = Kp*e + Ki*integral + Kd*derFilt;
    u_cmd = clamp(u_cmd, -maxTurn, +maxTurn);
#if USE_SLEW
    float u_max_step = U_SLEW_PER_SEC * dt;
    float du = u_cmd - u_prev;
    if (fabs(du) > u_max_step) u_cmd = u_prev + signf(du) * u_max_step;
#endif
    u_prev = u_cmd;

    // ======= Cùng cách trộn cho cả ALIGN & RUN =======
    int baseNow;
    if (modeNow == MODE_RUN){
      float scale = 1.0f - kSpeed * (fabs(u_cmd) / (float)maxTurn);
      baseNow = clampi((int)(basePWM * scale), minBase, basePWM);
    } else {
      baseNow = 0; // đứng yên xoay
    }

    int pwmL = baseNow - (int)u_cmd;
    int pwmR = baseNow + (int)u_cmd;

    // Deadband nhỏ khi ALIGN để không rung khi đã về trung tâm
    if (baseNow == 0 && fabs(e) <= alignDeadbandE){
      pwmL = 0; pwmR = 0;
    }

    setMotors(pwmL, pwmR);
    lastError = e;

    if (now - lastPrintMs >= SERIAL_THROTTLE_MS){
      lastPrintMs = now;
      noInterrupts(); uint8_t sb=g_sensors; interrupts();
      Serial.print((modeNow==MODE_ALIGN) ? "ALIGN " : "RUN   ");
      Serial.print("bits="); Serial.print(sb,BIN);
      Serial.print(" e="); Serial.print(e,2);
      Serial.print(" u="); Serial.print(u_cmd,0);
      Serial.print(" base="); Serial.print(baseNow);
      Serial.print(" L="); Serial.print(pwmL);
      Serial.print(" R="); Serial.println(pwmR);
    }
  }
}
