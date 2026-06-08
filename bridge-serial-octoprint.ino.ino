#include <AccelStepper.h>

/* ===========================
   BUILD MODE - REAL HARDWARE
   =========================== */
#define SIMULATION_MODE 0      // 0 = Real Elegoo Uno
#define DEFAULT_TERMINAL_UI 0

/* ===========================
   PINS (Elegoo Uno)
   =========================== */
const float STEPS_PER_MM_X = 80.0;
const float STEPS_PER_MM_Y = 80.0;
const float STEPS_PER_MM_Z = 400.0;
const float STEPS_PER_MM_E = 95.0;

const int X_STEP_PIN=2,  X_DIR_PIN=5;
const int Y_STEP_PIN=3,  Y_DIR_PIN=6;
const int Z_STEP_PIN=4,  Z_DIR_PIN=7;
const int E_STEP_PIN=12, E_DIR_PIN=13;

const int STEPPER_ENABLE_PIN = 8;
const int X_ENDSTOP_PIN=9, Y_ENDSTOP_PIN=10, Z_ENDSTOP_PIN=11;

const int FAN_PIN = A0;
const int HOTEND_PIN = 9;
const int BED_PIN = 10;
const int THERM_HOTEND = A1;
const int THERM_BED = A2;

/* ===========================
   STATE
   =========================== */
AccelStepper stepX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper stepE(AccelStepper::DRIVER, E_STEP_PIN, E_DIR_PIN);

float xPos=0, yPos=0, zPos=0, ePos=0;
bool absolutePositioning = true;
bool absoluteExtrusion = true;
float feedrate = 1500.0;
float speedFactor = 1.0;
float extrudeFactor = 1.0;

float hotendTemp = 25.0, bedTemp = 25.0;
float targetHotend = 0.0, targetBed = 0.0;

bool terminalUIEnabled = DEFAULT_TERMINAL_UI;
unsigned long lastUIUpdate = 0;
const unsigned long UI_INTERVAL = 500;

/* ===========================
   PARSER
   =========================== */
struct GCodeCommand {
  String cmd;
  float X=NAN, Y=NAN, Z=NAN, E=NAN, F=NAN, S=NAN, P=NAN, I=NAN, J=NAN;
};

GCodeCommand parseGCode(String line) {
  GCodeCommand g;
  line.trim();
  if (line.startsWith("N")) line = line.substring(line.indexOf(' ')+1);
  int star = line.indexOf('*'); if (star>0) line = line.substring(0,star);
  int comment = line.indexOf(';'); if (comment>=0) line = line.substring(0,comment);
  if (line.length() < 2) return g;

  int i=0;
  while (i<line.length() && (isalpha(line[i]) || isdigit(line[i]))) i++;
  g.cmd = line.substring(0,i); g.cmd.toUpperCase();

  for (int j=i; j<line.length(); j++) {
    char c = toupper(line[j]);
    if (strchr("XYZEFSPRIJ",c)) {
      float val = line.substring(j+1).toFloat();
      switch(c){
        case 'X': g.X=val; break; case 'Y': g.Y=val; break; case 'Z': g.Z=val; break;
        case 'E': g.E=val; break; case 'F': g.F=val; break; case 'S': g.S=val; break;
        case 'P': g.P=val; break; case 'I': g.I=val; break; case 'J': g.J=val; break;
      }
    }
  }
  return g;
}

/* ===========================
   UI
   =========================== */
void clearScreen() { Serial.write("\033[2J\033[H"); }

void drawDashboard() {
  clearScreen();
  Serial.println(F("============== ARDUKLIP FULL (Elegoo Uno) =============="));
  Serial.print(F("Status: ")); Serial.println((stepX.isRunning()||stepY.isRunning()||stepZ.isRunning()||stepE.isRunning()) ? "MOVING" : "IDLE");
  Serial.print(F("Pos X:")); Serial.print(xPos,2); Serial.print(F(" Y:")); Serial.print(yPos,2);
  Serial.print(F(" Z:")); Serial.print(zPos,2); Serial.print(F(" E:")); Serial.println(ePos,2);
  Serial.print(F("Hotend: ")); Serial.print(hotendTemp,1); Serial.print('/'); Serial.print(targetHotend,1);
  Serial.print(F("°C  Bed: ")); Serial.print(bedTemp,1); Serial.print('/'); Serial.print(targetBed,1); Serial.println("°C");
  Serial.print(F("Feedrate: ")); Serial.print(feedrate*speedFactor); Serial.println(" mm/min");
  Serial.println(F("M4000=UI ON | M4001=UI OFF"));
  Serial.println(F("======================================================="));
}

/* ===========================
   REPORTING
   =========================== */
void reportTemperatures() {
  Serial.print(F("ok T:")); Serial.print(hotendTemp,1);
  Serial.print(F(" /")); Serial.print(targetHotend,1);
  Serial.print(F(" B:")); Serial.print(bedTemp,1);
  Serial.print(F(" /")); Serial.print(targetBed,1);
  Serial.println(F(" @:0 B@:0"));
}

void reportPosition() {
  Serial.print(F("X:")); Serial.print(xPos,3);
  Serial.print(F(" Y:")); Serial.print(yPos,3);
  Serial.print(F(" Z:")); Serial.print(zPos,3);
  Serial.print(F(" E:")); Serial.println(ePos,3);
}

void sendOk() { Serial.println(F("ok")); }

/* ===========================
   HELPERS
   =========================== */
float readThermistor(int pin) {
  int raw = analogRead(pin);
  return (raw / 1023.0) * 300.0;   // TODO: Replace with real Steinhart-Hart table
}

void updateHeaters() {
  hotendTemp = readThermistor(THERM_HOTEND);
  bedTemp = readThermistor(THERM_BED);

  digitalWrite(HOTEND_PIN, (targetHotend > 0 && hotendTemp < targetHotend - 2) ? HIGH : LOW);
  digitalWrite(BED_PIN,    (targetBed > 0 && bedTemp < targetBed - 2) ? HIGH : LOW);
}

void setFan(int speed) { analogWrite(FAN_PIN, constrain(speed,0,255)); }

void homeAxis(AccelStepper &motor, int endstopPin, float &pos, float stepsPerMm) {
  motor.setMaxSpeed(2000); motor.setSpeed(-2000);
  while (digitalRead(endstopPin) == HIGH) {
    motor.runSpeed(); updateHeaters();
  }
  motor.move(10 * stepsPerMm);
  while (motor.distanceToGo() != 0) { motor.run(); updateHeaters(); }
  motor.setSpeed(-500);
  while (digitalRead(endstopPin) == HIGH) {
    motor.runSpeed(); updateHeaters();
  }
  motor.setCurrentPosition(0);
  pos = 0;
}

void executeLinearMove(GCodeCommand &g) {
  if (!isnan(g.F) && g.F > 0) feedrate = g.F;

  float newX = absolutePositioning ? (isnan(g.X)?xPos:g.X) : xPos + (isnan(g.X)?0:g.X);
  float newY = absolutePositioning ? (isnan(g.Y)?yPos:g.Y) : yPos + (isnan(g.Y)?0:g.Y);
  float newZ = absolutePositioning ? (isnan(g.Z)?zPos:g.Z) : zPos + (isnan(g.Z)?0:g.Z);
  float newE = absoluteExtrusion ? (isnan(g.E)?ePos:g.E) : ePos + (isnan(g.E)?0:g.E);

  xPos = newX; yPos = newY; zPos = newZ; ePos = newE * extrudeFactor;

  stepX.moveTo(xPos * STEPS_PER_MM_X);
  stepY.moveTo(yPos * STEPS_PER_MM_Y);
  stepZ.moveTo(zPos * STEPS_PER_MM_Z);
  stepE.moveTo(ePos * STEPS_PER_MM_E);

  float speed = (feedrate * speedFactor) / 60.0 * STEPS_PER_MM_X;
  stepX.setMaxSpeed(speed); stepY.setMaxSpeed(speed);
  stepZ.setMaxSpeed(speed * 0.6); stepE.setMaxSpeed(speed * 1.8);
}

/* ===========================
   SETUP
   =========================== */
void setup() {
  Serial.begin(115200);
  Serial.println(F("start"));
  Serial.println(F("FIRMWARE_NAME:ArduKlip_ElegooUno FIRMWARE_VERSION:2.5 PROTOCOL_VERSION:1.0 MACHINE_TYPE:Custom EXTRUDER_COUNT:1"));
  Serial.println(F("ok"));

  pinMode(STEPPER_ENABLE_PIN, OUTPUT); digitalWrite(STEPPER_ENABLE_PIN, LOW);
  pinMode(X_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(Y_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(Z_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(HOTEND_PIN, OUTPUT);
  pinMode(BED_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

  stepX.setAcceleration(3000);
  stepY.setAcceleration(3000);
  stepZ.setAcceleration(1500);
  stepE.setAcceleration(5000);

  if (terminalUIEnabled) drawDashboard();
}

/* ===========================
   MAIN LOOP - MAX G/M CODES
   =========================== */
void loop() {
  static String buffer = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buffer.length() > 0) {
        GCodeCommand g = parseGCode(buffer);
        buffer = "";
        bool sendOkFlag = true;

        // G-Codes
        if (g.cmd == "G0" || g.cmd == "G1") executeLinearMove(g);
        else if (g.cmd == "G2" || g.cmd == "G3") executeLinearMove(g); // approx
        else if (g.cmd == "G4" || g.cmd == "G04") {
          unsigned long ms = isnan(g.P) ? 0 : (unsigned long)(g.P * 1000);
          unsigned long start = millis();
          while (millis() - start < ms) { updateHeaters(); stepX.run(); stepY.run(); stepZ.run(); stepE.run(); }
        }
        else if (g.cmd == "G28") {
          if (isnan(g.X)&&isnan(g.Y)&&isnan(g.Z)) {
            homeAxis(stepX, X_ENDSTOP_PIN, xPos, STEPS_PER_MM_X);
            homeAxis(stepY, Y_ENDSTOP_PIN, yPos, STEPS_PER_MM_Y);
            homeAxis(stepZ, Z_ENDSTOP_PIN, zPos, STEPS_PER_MM_Z);
          } else {
            if (!isnan(g.X)) homeAxis(stepX, X_ENDSTOP_PIN, xPos, STEPS_PER_MM_X);
            if (!isnan(g.Y)) homeAxis(stepY, Y_ENDSTOP_PIN, yPos, STEPS_PER_MM_Y);
            if (!isnan(g.Z)) homeAxis(stepZ, Z_ENDSTOP_PIN, zPos, STEPS_PER_MM_Z);
          }
        }
        else if (g.cmd == "G90") absolutePositioning = true;
        else if (g.cmd == "G91") absolutePositioning = false;
        else if (g.cmd == "G92") {
          if (!isnan(g.X)) { xPos = g.X; stepX.setCurrentPosition(xPos*STEPS_PER_MM_X); }
          if (!isnan(g.Y)) { yPos = g.Y; stepY.setCurrentPosition(yPos*STEPS_PER_MM_Y); }
          if (!isnan(g.Z)) { zPos = g.Z; stepZ.setCurrentPosition(zPos*STEPS_PER_MM_Z); }
          if (!isnan(g.E)) { ePos = g.E; stepE.setCurrentPosition(ePos*STEPS_PER_MM_E); }
        }

        // M-Codes - Many as possible
        else if (g.cmd == "M104") if(!isnan(g.S)) targetHotend = g.S;
        else if (g.cmd == "M109") { if(!isnan(g.S)) targetHotend = g.S; while(hotendTemp < targetHotend-0.5){updateHeaters(); delay(80);} }
        else if (g.cmd == "M140") if(!isnan(g.S)) targetBed = g.S;
        else if (g.cmd == "M190") { if(!isnan(g.S)) targetBed = g.S; while(bedTemp < targetBed-0.5){updateHeaters(); delay(150);} }
        else if (g.cmd == "M105") { updateHeaters(); reportTemperatures(); sendOkFlag = false; }
        else if (g.cmd == "M106") setFan(isnan(g.S)?255:(int)g.S);
        else if (g.cmd == "M107") setFan(0);
        else if (g.cmd == "M82") absoluteExtrusion = true;
        else if (g.cmd == "M83") absoluteExtrusion = false;
        else if (g.cmd == "M84") {
          digitalWrite(STEPPER_ENABLE_PIN,HIGH);
          stepX.disableOutputs(); stepY.disableOutputs(); stepZ.disableOutputs(); stepE.disableOutputs();
        }
        else if (g.cmd == "M114") { reportPosition(); Serial.println(F("ok")); sendOkFlag = false; }
        else if (g.cmd == "M115") {
          Serial.println(F("FIRMWARE_NAME:ArduKlip_ElegooUno FIRMWARE_VERSION:2.5 PROTOCOL_VERSION:1.0 MACHINE_TYPE:Custom EXTRUDER_COUNT:1"));
          sendOkFlag = true;
        }
        else if (g.cmd == "M119") {
          Serial.print(F("X:")); Serial.print(digitalRead(X_ENDSTOP_PIN)==LOW?"TRIGGERED":"open");
          Serial.print(F(" Y:")); Serial.print(digitalRead(Y_ENDSTOP_PIN)==LOW?"TRIGGERED":"open");
          Serial.print(F(" Z:")); Serial.print(digitalRead(Z_ENDSTOP_PIN)==LOW?"TRIGGERED":"open");
          Serial.println(); sendOkFlag = false;
        }
        else if (g.cmd == "M220") if(!isnan(g.S)) speedFactor = g.S/100.0;
        else if (g.cmd == "M221") if(!isnan(g.S)) extrudeFactor = g.S/100.0;
        else if (g.cmd == "M302") { Serial.println(F("echo: Cold extrude allowed")); }
        else if (g.cmd == "M4000") { terminalUIEnabled = true; clearScreen(); drawDashboard(); sendOkFlag = false; }
        else if (g.cmd == "M4001") { terminalUIEnabled = false; Serial.println(F("Terminal UI disabled")); sendOkFlag = false; }
        else if (g.cmd == "M503") {
          Serial.println(F("echo: Settings"));
          Serial.print(F("M92 X")); Serial.print(STEPS_PER_MM_X); Serial.print(F(" Y")); Serial.print(STEPS_PER_MM_Y);
          Serial.print(F(" Z")); Serial.print(STEPS_PER_MM_Z); Serial.print(F(" E")); Serial.println(STEPS_PER_MM_E);
          sendOkFlag = true;
        }

        // Unknown
        else {
          Serial.print(F("echo: Unknown command "));
          Serial.println(g.cmd);
        }

        if (sendOkFlag) sendOk();
      }
    } else buffer += c;
  }

  updateHeaters();
  stepX.run(); stepY.run(); stepZ.run(); stepE.run();

  if (terminalUIEnabled && millis() - lastUIUpdate > UI_INTERVAL) {
    drawDashboard();
    lastUIUpdate = millis();
  }
}
