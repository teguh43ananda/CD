#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>
#include <EEPROM.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

#define SERVOMIN  90
#define SERVOMAX  500
#define SERVO_MID ((SERVOMIN + SERVOMAX) / 2)
#define GRIPPER_CHANNEL 4

const int GRIPPER_OPEN = SERVOMAX;
const int GRIPPER_CLOSE = SERVOMIN;

// EEPROM addresses
#define ADDR_MODE_HALUS 0
#define ADDR_FREEZE 1
#define ADDR_GRIPPER_POS 2

int gripperPos = SERVO_MID;
bool modeHalus = true;
float alpha = 0.2;

PS2X ps2x;
int error = 0;
byte vibrate = 0;

// PID parameters
float kp = 0.3;
float ki = 0.02;
float kd = 0.15;

// PID & control variables
float errorPID[4] = {0};
float prevError[4] = {0};
float integral[4] = {0};
float currentPWM[4] = {SERVO_MID, SERVO_MID, SERVO_MID, SERVO_MID};

// Freeze control
bool freezeAxis[4] = {false, false, false, false}; // RX, RY, LX, LY

void saveConfigToEEPROM() {
  EEPROM.write(ADDR_MODE_HALUS, modeHalus);
  EEPROM.write(ADDR_FREEZE, 
    (freezeAxis[0] << 3) | (freezeAxis[1] << 2) | (freezeAxis[2] << 1) | (freezeAxis[3] << 0));
  EEPROM.write(ADDR_GRIPPER_POS, gripperPos / 4); // Save 0–255, assuming gripper PWM within 0–1020
}

void loadConfigFromEEPROM() {
  modeHalus = EEPROM.read(ADDR_MODE_HALUS);
  byte freezeByte = EEPROM.read(ADDR_FREEZE);
  freezeAxis[0] = freezeByte & 0b1000;
  freezeAxis[1] = freezeByte & 0b0100;
  freezeAxis[2] = freezeByte & 0b0010;
  freezeAxis[3] = freezeByte & 0b0001;
  gripperPos = EEPROM.read(ADDR_GRIPPER_POS) * 4;
  alpha = modeHalus ? 0.2 : 0.4;
}

void setup() {
  Serial.begin(57600);
  Serial.println("Inisialisasi PCA9685");
  pca.begin();
  pca.setPWMFreq(50);
  delay(1000);

  error = ps2x.config_gamepad(13, 23, 24, 25, true, true);
  if (error == 0) Serial.println("Controller terdeteksi dan dikonfigurasi.");
  else Serial.println("Gagal mendeteksi controller. Cek wiring.");

  loadConfigFromEEPROM();

  for (int i = 0; i < 5; i++) {
    pca.setPWM(i, 0, (i == GRIPPER_CHANNEL) ? gripperPos : SERVO_MID);
  }
}

void loop() {
  if (error == 1) return;

  ps2x.read_gamepad(false, vibrate);

  // Toggle mode halus dengan SELECT
  if (ps2x.ButtonPressed(PSB_SELECT)) {
    modeHalus = !modeHalus;
    alpha = modeHalus ? 0.2 : 0.4;
    Serial.println(modeHalus ? "Mode: Halus" : "Mode: Biasa");
    saveConfigToEEPROM();
  }

  // Toggle freeze per-axis
  if (ps2x.ButtonPressed(PSB_L2)) { freezeAxis[2] = !freezeAxis[2]; saveConfigToEEPROM(); } // LX
  if (ps2x.ButtonPressed(PSB_L1)) { freezeAxis[3] = !freezeAxis[3]; saveConfigToEEPROM(); } // LY
  if (ps2x.ButtonPressed(PSB_R2)) { freezeAxis[0] = !freezeAxis[0]; saveConfigToEEPROM(); } // RX
  if (ps2x.ButtonPressed(PSB_R1)) { freezeAxis[1] = !freezeAxis[1]; saveConfigToEEPROM(); } // RY

  // Baca joystick
  int joy[4];
  joy[0] = ps2x.Analog(PSS_RX);
  joy[1] = ps2x.Analog(PSS_RY);
  joy[2] = ps2x.Analog(PSS_LX);
  joy[3] = ps2x.Analog(PSS_LY);

  for (int i = 0; i < 4; i++) {
    if (freezeAxis[i]) continue;

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

  // Gripper kontrol dengan tombol X dan O
  if (ps2x.ButtonPressed(PSB_CROSS)) {
    gripperPos = GRIPPER_CLOSE;
    Serial.println("Gripper tertutup (X)");
    saveConfigToEEPROM();
  } else if (ps2x.ButtonPressed(PSB_CIRCLE)) {
    gripperPos = GRIPPER_OPEN;
    Serial.println("Gripper terbuka (O)");
    saveConfigToEEPROM();
  }
  pca.setPWM(GRIPPER_CHANNEL, 0, gripperPos);

  // Debug info
  Serial.print("RX: "); Serial.print(joy[0]);
  Serial.print("  RY: "); Serial.print(joy[1]);
  Serial.print("  LX: "); Serial.print(joy[2]);
  Serial.print("  LY: "); Serial.println(joy[3]);

  delay(50);
}
