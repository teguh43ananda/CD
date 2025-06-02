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
float alpha = 0.2;
bool modeHalus = false;

// EEPROM address
#define EEPROM_MODE_ADDR      0
#define EEPROM_FREEZE_ADDR    1
#define EEPROM_POS_START_ADDR 10 // posisi currentPWM[0..4] mulai dari address 10

// PID state
float errorPID[4] = {0}, prevError[4] = {0}, integral[4] = {0};
float currentPWM[5] = {SERVO_MID, SERVO_MID, SERVO_MID, SERVO_MID, SERVO_MID};
bool freeze[4] = {false, false, false, false};

// Tracking changes
float lastGripperPos = -1;
unsigned long lastSaveTime = 0;

void saveToEEPROM() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastSaveTime < 2000) return;

  EEPROM.update(EEPROM_MODE_ADDR, modeHalus ? 1 : 0);
  for (int i = 0; i < 4; i++) {
    EEPROM.update(EEPROM_FREEZE_ADDR + i, freeze[i] ? 1 : 0);
  }

  if (gripperPos != lastGripperPos) {
    EEPROM.put(EEPROM_POS_START_ADDR + GRIPPER_CHANNEL * sizeof(float), (float)gripperPos);
    lastGripperPos = gripperPos;
  }

  for (int i = 0; i < 5; i++) {
    if (i != GRIPPER_CHANNEL) {
      float valInEEPROM;
      EEPROM.get(EEPROM_POS_START_ADDR + i * sizeof(float), valInEEPROM);
      if (currentPWM[i] != valInEEPROM) {
        EEPROM.put(EEPROM_POS_START_ADDR + i * sizeof(float), currentPWM[i]);
      }
    }
  }

  lastSaveTime = currentMillis;
}

void loadFromEEPROM() {
  modeHalus = EEPROM.read(EEPROM_MODE_ADDR) == 1;
  alpha = modeHalus ? 0.05 : 0.2;
  for (int i = 0; i < 4; i++) {
    freeze[i] = EEPROM.read(EEPROM_FREEZE_ADDR + i) == 1;
  }
  for (int i = 0; i < 5; i++) {
    EEPROM.get(EEPROM_POS_START_ADDR + i * sizeof(float), currentPWM[i]);
  }
  gripperPos = currentPWM[GRIPPER_CHANNEL];

  if (gripperPos < GRIPPER_CLOSE || gripperPos > GRIPPER_OPEN) {
    gripperPos = GRIPPER_CLOSE;
    currentPWM[GRIPPER_CHANNEL] = gripperPos;
  }
}

void setup() {
  Serial.begin(57600);
  pca.begin();
  pca.setPWMFreq(50);
  delay(1000);

  error = ps2x.config_gamepad(13, 14, 24, 25, true, true);
  if (error == 0) Serial.println("Controller terdeteksi dan dikonfigurasi.");
  else Serial.println("Gagal mendeteksi controller. Cek wiring.");

  type = ps2x.readType();
  if (type == 1) Serial.println("DualShock Controller terdeteksi");

  loadFromEEPROM();

  for (int i = 0; i < 5; i++) {
    pca.setPWM(i, 0, currentPWM[i]);
  }

  pca.setPWM(GRIPPER_CHANNEL, 0, gripperPos);
  delay(500);
}

void loop() {
  if (error == 1) return;
  ps2x.read_gamepad(false, vibrate);

  if (ps2x.ButtonPressed(PSB_R3)) {
    Serial.println("Menghapus EEPROM...");
    
    // Reset mode dan freeze
    EEPROM.update(EEPROM_MODE_ADDR, 0);
    for (int i = 0; i < 4; i++) {
      EEPROM.update(EEPROM_FREEZE_ADDR + i, 0);
    }
  
    // Reset posisi servo
    for (int i = 0; i < 5; i++) {
      float defaultPWM = (i == GRIPPER_CHANNEL) ? GRIPPER_CLOSE : SERVO_MID;
      EEPROM.put(EEPROM_POS_START_ADDR + i * sizeof(float), defaultPWM);
      currentPWM[i] = defaultPWM;
      pca.setPWM(i, 0, defaultPWM);
    }
  
    gripperPos = GRIPPER_CLOSE;
    lastGripperPos = -1; // supaya bisa disimpan lagi kalau berubah nanti
  
    Serial.println("EEPROM berhasil direset ke default.");
  }
  
  if (ps2x.ButtonPressed(PSB_SELECT)) {
    modeHalus = !modeHalus;
    alpha = modeHalus ? 0.05 : 0.2;
    Serial.println(modeHalus ? "Mode: Halus" : "Mode: Biasa");
    saveToEEPROM();
  }

  if (ps2x.ButtonPressed(PSB_L2)) { freeze[2] = !freeze[2]; saveToEEPROM(); }
  if (ps2x.ButtonPressed(PSB_L1)) { freeze[3] = !freeze[3]; saveToEEPROM(); }
  if (ps2x.ButtonPressed(PSB_R2)) { freeze[0] = !freeze[0]; saveToEEPROM(); }
  if (ps2x.ButtonPressed(PSB_R1)) { freeze[1] = !freeze[1]; saveToEEPROM(); }

  if (ps2x.ButtonPressed(PSB_CROSS)) {
    Serial.println("Wrist kiri (X)");
    currentPWM[3] -= 10;
    if (currentPWM[3] < SERVOMIN) currentPWM[3] = SERVOMIN;
    pca.setPWM(3, 0, currentPWM[3]);
    saveToEEPROM();
  } else if (ps2x.ButtonPressed(PSB_CIRCLE)) {
    Serial.println("Wrist kanan (O)");
    currentPWM[3] += 10;
    if (currentPWM[3] > SERVOMAX) currentPWM[3] = SERVOMAX;
    pca.setPWM(3, 0, currentPWM[3]);
    saveToEEPROM();
  }

  int joy[4];
  joy[0] = ps2x.Analog(PSS_RX);
  joy[1] = ps2x.Analog(PSS_RY);
  joy[2] = ps2x.Analog(PSS_LX);
  joy[3] = ps2x.Analog(PSS_LY);

  for (int i = 0; i < 4; i++) {
    if (i == 3 || freeze[i]) continue;

    int targetPWM = map(joy[i], 0, 255, SERVOMIN, SERVOMAX);
    errorPID[i] = targetPWM - currentPWM[i];
    integral[i] += errorPID[i];
    float derivative = errorPID[i] - prevError[i];

    float output = kp * errorPID[i] + ki * integral[i] + kd * derivative;
    prevError[i] = errorPID[i];

    currentPWM[i] = (1 - alpha) * currentPWM[i] + alpha * (currentPWM[i] + output);
    currentPWM[i] = constrain(currentPWM[i], SERVOMIN, SERVOMAX);

    pca.setPWM(i, 0, currentPWM[i]);
  }

  if (!freeze[3]) {
    int targetGripper = map(joy[3], 0, 255, GRIPPER_CLOSE, GRIPPER_OPEN);
    gripperPos = (1 - alpha) * gripperPos + alpha * targetGripper;
    gripperPos = constrain(gripperPos, GRIPPER_CLOSE, GRIPPER_OPEN);
    currentPWM[GRIPPER_CHANNEL] = gripperPos;
    pca.setPWM(GRIPPER_CHANNEL, 0, gripperPos);
  }

  Serial.print("RX: "); Serial.print(joy[0]);
  Serial.print("  RY: "); Serial.print(joy[1]);
  Serial.print("  LX: "); Serial.print(joy[2]);
  Serial.print("  LY: "); Serial.println(joy[3]);

  delay(50);
}