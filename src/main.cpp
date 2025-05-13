#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

#define SERVOMIN  90
#define SERVOMAX  500
#define SERVO_MID ((SERVOMIN + SERVOMAX) / 2)

#define GRIPPER_CHANNEL 4
int gripperPos;
const int GRIPPER_OPEN = SERVOMAX;
const int GRIPPER_CLOSE = SERVOMIN;

PS2X ps2x;
int error = 0;
byte vibrate = 0;
byte type = 0;

// PID Parameters
float kp = 0.3, ki = 0.02, kd = 0.15;
float alpha = 0.4; // default: mode biasa
bool modeHalus = false;

// EEPROM address
#define EEPROM_MODE_ADDR      0
#define EEPROM_FREEZE_ADDR    1
#define EEPROM_POS_START_ADDR 10 // Posisi currentPWM[0..4]

// PID state
float errorPID[4] = {0}, prevError[4] = {0}, integral[4] = {0};
float currentPWM[5] = {SERVO_MID, SERVO_MID, SERVO_MID, SERVO_MID, SERVO_MID};
bool freeze[4] = {false, false, false, false};

// Tracking changes
int lastGripperPos = -1; // Used to store the last gripper position to avoid redundant writes
unsigned long lastSaveTime = 0; // Timestamp for the last save (to implement debounce)

void saveToEEPROM() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastSaveTime < 2000) {  // debounce: 2-second interval between saves
    return;  // Don't save if interval is too short
  }
  
  // Save modeHalus and freeze states
  EEPROM.update(EEPROM_MODE_ADDR, modeHalus ? 1 : 0);
  for (int i = 0; i < 4; i++) {
    EEPROM.update(EEPROM_FREEZE_ADDR + i, freeze[i] ? 1 : 0);
  }
  
  // Save gripper position only if it changed
  if (gripperPos != lastGripperPos) {
    EEPROM.update(EEPROM_POS_START_ADDR + GRIPPER_CHANNEL * sizeof(float), gripperPos);
    lastGripperPos = gripperPos;
  }
  
  // Save other servo positions if necessary
  for (int i = 0; i < 5; i++) {
    if (i != GRIPPER_CHANNEL && currentPWM[i] != EEPROM.read(EEPROM_POS_START_ADDR + i * sizeof(float))) {
      EEPROM.put(EEPROM_POS_START_ADDR + i * sizeof(float), currentPWM[i]);
    }
  }
  
  lastSaveTime = currentMillis; // Update the last save time
}

void loadFromEEPROM() {
  modeHalus = EEPROM.read(EEPROM_MODE_ADDR) == 1;
  alpha = modeHalus ? 0.2 : 0.4;
  for (int i = 0; i < 4; i++) {
    freeze[i] = EEPROM.read(EEPROM_FREEZE_ADDR + i) == 1;
  }
  for (int i = 0; i < 5; i++) {
    EEPROM.get(EEPROM_POS_START_ADDR + i * sizeof(float), currentPWM[i]);
  }
  gripperPos = currentPWM[GRIPPER_CHANNEL];
}

void setup() {
  Serial.begin(57600);
  pca.begin();
  pca.setPWMFreq(50);
  delay(1000);

  error = ps2x.config_gamepad(13, 23, 24, 25, true, true);
  if (error == 0) Serial.println("Controller terdeteksi dan dikonfigurasi.");
  else Serial.println("Gagal mendeteksi controller. Cek wiring.");

  type = ps2x.readType();
  if (type == 1) Serial.println("DualShock Controller terdeteksi");

  loadFromEEPROM();

  for (int i = 0; i < 5; i++) {
    pca.setPWM(i, 0, currentPWM[i]);
  }
}

void loop() {
  if (error == 1) return;
  ps2x.read_gamepad(false, vibrate);

  // Toggle mode halus/biasa
  if (ps2x.ButtonPressed(PSB_SELECT)) {
    modeHalus = !modeHalus;
    alpha = modeHalus ? 0.2 : 0.4;
    Serial.println(modeHalus ? "Mode: Halus" : "Mode: Biasa");
    saveToEEPROM();
  }

  if (ps2x.ButtonPressed(PSB_L2)) { freeze[2] = !freeze[2]; saveToEEPROM(); } // LX
  if (ps2x.ButtonPressed(PSB_L1)) { freeze[3] = !freeze[3]; saveToEEPROM(); } // LY
  if (ps2x.ButtonPressed(PSB_R2)) { freeze[0] = !freeze[0]; saveToEEPROM(); } // RX
  if (ps2x.ButtonPressed(PSB_R1)) { freeze[1] = !freeze[1]; saveToEEPROM(); } // RY


  if (ps2x.ButtonPressed(PSB_CROSS)) {
    gripperPos = GRIPPER_OPEN;
    currentPWM[GRIPPER_CHANNEL] = gripperPos;
    Serial.println("Gripper buka (X)");
    saveToEEPROM();
  } else if (ps2x.ButtonPressed(PSB_CIRCLE)) {
    gripperPos = GRIPPER_CLOSE;
    currentPWM[GRIPPER_CHANNEL] = gripperPos;
    Serial.println("Gripper tutup (O)");
    saveToEEPROM();
  }
  pca.setPWM(GRIPPER_CHANNEL, 0, gripperPos);

  int joy[4];
  joy[0] = ps2x.Analog(PSS_RX); // 0
  joy[1] = ps2x.Analog(PSS_RY); // 1
  joy[2] = ps2x.Analog(PSS_LX); // 2
  joy[3] = ps2x.Analog(PSS_LY); // 3

  for (int i = 0; i < 4; i++) {
    if (freeze[i]) continue;

    int targetPWM = map(joy[i], 0, 255, SERVOMIN, SERVOMAX);
    errorPID[i] = targetPWM - currentPWM[i];
    integral[i] += errorPID[i];
    float derivative = errorPID[i] - prevError[i];

    float output = kp * errorPID[i] + ki * integral[i] + kd * derivative;
    prevError[i] = errorPID[i];

    currentPWM[i] = (1 - alpha) * currentPWM[i] + alpha * (currentPWM[i] + output);
    if (currentPWM[i] < SERVOMIN) currentPWM[i] = SERVOMIN;
    if (currentPWM[i] > SERVOMAX) currentPWM[i] = SERVOMAX;

    pca.setPWM(i, 0, currentPWM[i]);
  }

  Serial.print("RX: "); Serial.print(joy[0]);
  Serial.print("  RY: "); Serial.print(joy[1]);
  Serial.print("  LX: "); Serial.print(joy[2]);
  Serial.print("  LY: "); Serial.println(joy[3]);
  delay(50);
}

