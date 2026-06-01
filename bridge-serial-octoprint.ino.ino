#include <AccelStepper.h>

/* ===========================
   HARDWARE PINS (Real Components)
   =========================== */
const float STEPS_PER_MM = 80.0;

// Stepper pins (STEP + DIR)
AccelStepper stepX(AccelStepper::DRIVER, 2, 5);
AccelStepper stepY(AccelStepper::DRIVER, 3, 6);
AccelStepper stepZ(AccelStepper::DRIVER, 4, 7);
AccelStepper stepE(AccelStepper::DRIVER, 12, 13);

// Common enable pin (for A4988/DRV8825 shields)
const int STEPPER_ENABLE_PIN = 8;

// Endstops
const int X_ENDSTOP = 9;
const int Y_ENDSTOP = 10;
const int Z_ENDSTOP = 11;

// Fan (e.g., hotend cooling or part cooling)
const int FAN_PIN = A0;  // PWM capable pin

// State
float xPos = 0.0, yPos = 0.0, zPos = 0.0, ePos = 0.0;
bool absolutePositioning = true;
bool absoluteExtrusion = true;

float feedrate = 1500.0;      // mm/min
float speedFactor = 1.0;      // M220 %
float extrudeFactor = 1.0;    // M221 %

float hotendTemp = 25.0, bedTemp = 25.0;
float targetHotend = 0.0, targetBed = 0.0;

const float HEAT_RATE = 2.2;
const float COOL_RATE = 0.35;

/* ===========================
   G-CODE PARSER
   =========================== */
struct GCodeCommand {
  String cmd;
  float X=NAN, Y=NAN, Z=NAN, E=NAN, F=NAN, S=NAN, P=NAN, R=NAN;
  float I=NAN, J=NAN, K=NAN;
};

GCodeCommand parseGCode(String line) {
  GCodeCommand g;
  line.trim();

  // Remove line number and checksum
  if (line.startsWith("N")) {
    int sp = line.indexOf(' ');
    if (sp > 0) line = line.substring(sp + 1);
  }
  int star = line.indexOf('*');
  if (star > 0) line = line.substring(0, star);

  // Remove comments
  int comment = line.indexOf(';');
  if (comment >= 0) line = line.substring(0, comment);

  if (line.length() < 2) return g;

  g.cmd = line.substring(0, 3);
  g.cmd.toUpperCase();

  for (int i = 3; i < line.length(); i++) {
    char c = toupper(line[i]);
    if (strchr("XYZEFSPR IJK", c)) {  // space intentional
      float val = line.substring(i + 1).toFloat();
      switch (c) {
        case 'X': g.X = val; break;
        case 'Y': g.Y = val; break;
        case 'Z': g.Z = val; break;
        case 'E': g.E = val; break;
        case 'F': g.F = val; break;
        case 'S': g.S = val; break;
        case 'P': g.P = val; break;
        case 'R': g.R = val; break;
        case 'I': g.I = val; break;
        case 'J': g.J = val; break;
        case 'K': g.K = val; break;
      }
    }
  }
  return g;
}

/* ===========================
   UTILITIES
   =========================== */
void updateHeaters() {
  if (hotendTemp < targetHotend) hotendTemp = min(hotendTemp + HEAT_RATE, targetHotend);
  else if (hotendTemp > targetHotend) hotendTemp = max(hotendTemp - COOL_RATE, targetHotend);

  if (bedTemp < targetBed) bedTemp = min(bedTemp + HEAT_RATE * 0.6, targetBed);
  else if (bedTemp > targetBed) bedTemp = max(bedTemp - COOL_RATE, targetBed);
}

void reportPosition() {
  Serial.print("X:"); Serial.print(xPos, 3);
  Serial.print(" Y:"); Serial.print(yPos, 3);
  Serial.print(" Z:"); Serial.print(zPos, 3);
  Serial.print(" E:"); Serial.println(ePos, 3);
}

void reportTemperatures() {
  Serial.print("ok T:");
  Serial.print(hotendTemp, 1); Serial.print(" /"); Serial.print(targetHotend, 1);
  Serial.print(" B:"); Serial.print(bedTemp, 1); Serial.print(" /"); Serial.println(targetBed, 1);
}

void setFan(int speed) {  // 0-255
  analogWrite(FAN_PIN, constrain(speed, 0, 255));
}

/* ===========================
   HOMING
   =========================== */
void homeAxis(AccelStepper &motor, int pin, float &pos) {
  motor.setMaxSpeed(2000);
  motor.setSpeed(-2000);
  while (digitalRead(pin) == HIGH) motor.runSpeed();

  motor.move(-5 * STEPS_PER_MM);
  while (motor.distanceToGo() != 0) motor.run();

  motor.setMaxSpeed(500);
  motor.setSpeed(-500);
  while (digitalRead(pin) == HIGH) motor.runSpeed();

  motor.setCurrentPosition(0);
  pos = 0.0;
}

/* ===========================
   MOVEMENT
   =========================== */
void executeLinearMove(GCodeCommand &g) {
  if (!isnan(g.F) && g.F > 0) feedrate = g.F;

  float newX = absolutePositioning ? (isnan(g.X) ? xPos : g.X) : xPos + (isnan(g.X) ? 0 : g.X);
  float newY = absolutePositioning ? (isnan(g.Y) ? yPos : g.Y) : yPos + (isnan(g.Y) ? 0 : g.Y);
  float newZ = absolutePositioning ? (isnan(g.Z) ? zPos : g.Z) : zPos + (isnan(g.Z) ? 0 : g.Z);
  float newE = absoluteExtrusion ? (isnan(g.E) ? ePos : g.E) : ePos + (isnan(g.E) ? 0 : g.E);

  if (!isnan(g.X)) xPos = newX;
  if (!isnan(g.Y)) yPos = newY;
  if (!isnan(g.Z)) zPos = newZ;
  if (!isnan(g.E)) ePos = newE * extrudeFactor;

  stepX.moveTo(xPos * STEPS_PER_MM);
  stepY.moveTo(yPos * STEPS_PER_MM);
  stepZ.moveTo(zPos * STEPS_PER_MM);
  stepE.moveTo(ePos * STEPS_PER_MM);

  float mmPerSec = (feedrate * speedFactor) / 60.0;
  float stepsPerSec = mmPerSec * STEPS_PER_MM;
  stepX.setMaxSpeed(stepsPerSec);
  stepY.setMaxSpeed(stepsPerSec);
  stepZ.setMaxSpeed(stepsPerSec);
  stepE.setMaxSpeed(stepsPerSec * 1.8);
}

// Very basic G2/G3 approximation (linear segments - limited accuracy)
void executeArc(GCodeCommand &g, bool clockwise) {
  // For real use: replace with proper arc interpolation library if needed
  Serial.println("ok"); // Placeholder - arcs approximated as straight moves
  executeLinearMove(g); // Fallback
}

/* ===========================
   SETUP
   =========================== */
void setup() {
  Serial.begin(115200);

  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(STEPPER_ENABLE_PIN, LOW); // Enable steppers

  pinMode(X_ENDSTOP, INPUT_PULLUP);
  pinMode(Y_ENDSTOP, INPUT_PULLUP);
  pinMode(Z_ENDSTOP, INPUT_PULLUP);
  pinMode(FAN_PIN, OUTPUT);

  stepX.setAcceleration(2500);
  stepY.setAcceleration(2500);
  stepZ.setAcceleration(1200);
  stepE.setAcceleration(4000);

  Serial.println("Marlin-compatible Arduino Simulator v2 Ready");
  Serial.println("ok");
}

/* ===========================
   MAIN LOOP
   =========================== */
void loop() {
  static String buffer = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buffer.length() > 0) {
        GCodeCommand g = parseGCode(buffer);
        buffer = "";

        // ==================== G CODES ====================
        if (g.cmd.startsWith("G0") || g.cmd.startsWith("G1")) {
          executeLinearMove(g);
        }
        else if (g.cmd == "G2" || g.cmd == "G02") executeArc(g, true);
        else if (g.cmd == "G3" || g.cmd == "G03") executeArc(g, false);
        else if (g.cmd == "G4" || g.cmd == "G04") {
          unsigned long ms = isnan(g.P) ? (isnan(g.S) ? 0 : (unsigned long)(g.S*1000)) : (unsigned long)g.P;
          unsigned long start = millis();
          while (millis() - start < ms) {
            updateHeaters();
            stepX.run(); stepY.run(); stepZ.run(); stepE.run();
          }
        }
        else if (g.cmd == "G28") {
          if (isnan(g.X) && isnan(g.Y) && isnan(g.Z)) {
            homeAxis(stepX, X_ENDSTOP, xPos);
            homeAxis(stepY, Y_ENDSTOP, yPos);
            homeAxis(stepZ, Z_ENDSTOP, zPos);
          } else {
            if (!isnan(g.X)) homeAxis(stepX, X_ENDSTOP, xPos);
            if (!isnan(g.Y)) homeAxis(stepY, Y_ENDSTOP, yPos);
            if (!isnan(g.Z)) homeAxis(stepZ, Z_ENDSTOP, zPos);
          }
        }
        else if (g.cmd == "G90") absolutePositioning = true;
        else if (g.cmd == "G91") absolutePositioning = false;
        else if (g.cmd == "G92") {
          if (!isnan(g.X)) { xPos = g.X; stepX.setCurrentPosition(xPos*STEPS_PER_MM); }
          if (!isnan(g.Y)) { yPos = g.Y; stepY.setCurrentPosition(yPos*STEPS_PER_MM); }
          if (!isnan(g.Z)) { zPos = g.Z; stepZ.setCurrentPosition(zPos*STEPS_PER_MM); }
          if (!isnan(g.E)) { ePos = g.E; stepE.setCurrentPosition(ePos*STEPS_PER_MM); }
        }
        else if (g.cmd == "G21") {} // mm (default)
        else if (g.cmd == "G20") {} // inches - not fully supported

        // ==================== M CODES ====================
        else if (g.cmd == "M104") if (!isnan(g.S)) targetHotend = g.S;
        else if (g.cmd == "M109") {
          if (!isnan(g.S)) targetHotend = g.S;
          while (hotendTemp < targetHotend - 0.5) { updateHeaters(); delay(100); }
        }
        else if (g.cmd == "M140") if (!isnan(g.S)) targetBed = g.S;
        else if (g.cmd == "M190") {
          if (!isnan(g.S)) targetBed = g.S;
          while (bedTemp < targetBed - 0.5) { updateHeaters(); delay(200); }
        }
        else if (g.cmd == "M105") reportTemperatures();
        else if (g.cmd == "M114") reportPosition();
        else if (g.cmd == "M106") setFan(isnan(g.S) ? 255 : (int)g.S);  // Fan on
        else if (g.cmd == "M107") setFan(0);                           // Fan off
        else if (g.cmd == "M82") absoluteExtrusion = true;
        else if (g.cmd == "M83") absoluteExtrusion = false;
        else if (g.cmd == "M84") {
          digitalWrite(STEPPER_ENABLE_PIN, HIGH); // Disable
          stepX.disableOutputs(); stepY.disableOutputs();
          stepZ.disableOutputs(); stepE.disableOutputs();
        }
        else if (g.cmd == "M220") if (!isnan(g.S)) speedFactor = g.S / 100.0;
        else if (g.cmd == "M221") if (!isnan(g.S)) extrudeFactor = g.S / 100.0;
        else if (g.cmd == "M115") Serial.println("FIRMWARE_NAME:Arduino_Simulator");
        else if (g.cmd == "M117") Serial.println("ok"); // LCD message (ignored)
        else if (g.cmd == "M300") {} // Beep (ignored)
        else if (g.cmd == "M999") {} // Reset

        Serial.println("ok");
      }
    } else {
      buffer += c;
    }
  }

  // Background
  updateHeaters();
  stepX.run();
  stepY.run();
  stepZ.run();
  stepE.run();
}
