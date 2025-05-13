#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

#define SERVOMIN  90
#define SERVOMAX  500
#define SERVO_MID ((SERVOMIN + SERVOMAX) / 2)

#define GRIPPER_CHANNEL 4
int gripperPos = SERVO_MID;
const int GRIPPER_OPEN = SERVOMAX;
const int GRIPPER_CLOSE = SERVOMIN;

PS2X ps2x;
int error = 0;
byte type = 0;
byte vibrate = 0;

float kp = 0.3;
float ki = 0.02;
float kd = 0.15;

float alpha = 0.4; 
bool modeHalus = true; 

float errorPID[4] = {0, 0, 0, 0};
float prevError[4] = {0, 0, 0, 0};
float integral[4] = {0, 0, 0, 0};
float currentPWM[4] = {SERVO_MID, SERVO_MID, SERVO_MID, SERVO_MID};

bool freezeAxis[4] = {false, false, false, false}; // RX, RY, LX, LY

void setup() {
  Serial.begin(57600);
  Serial.println("Inisialisasi PCA9685");

  pca.begin();
  pca.setPWMFreq(50);
  delay(1000);

  error = ps2x.config_gamepad(13, 23, 24, 25, true, true);
  if (error == 0) Serial.println("Controller terdeteksi dan dikonfigurasi.");
  else Serial.println("Gagal mendeteksi controller. Cek wiring.");

  type = ps2x.readType();
  if (type == 1) Serial.println("DualShock Controller terdeteksi");

  for (int i = 0; i < 5; i++) {
    pca.setPWM(i, 0, SERVO_MID);
  }
}

void loop() {
  if (error == 1) return;

  ps2x.read_gamepad(false, vibrate);


  int joy[4];
  joy[0] = ps2x.Analog(PSS_RX);
  joy[1] = ps2x.Analog(PSS_RY);
  joy[2] = ps2x.Analog(PSS_LX);
  joy[3] = ps2x.Analog(PSS_LY);

  // Toggle mode halus/biasa 
  if (ps2x.ButtonPressed(PSB_START)) {
    modeHalus = !modeHalus;
    alpha = modeHalus ? 0.3 : 0.8;
    Serial.println(modeHalus ? "Mode: HALUS" : "Mode: BIASA");
  }

  // Freeze axis toggle
  if (ps2x.ButtonPressed(PSB_R2)) {
    freezeAxis[0] = !freezeAxis[0]; // RX
    Serial.println(freezeAxis[0] ? "RX dibekukan" : "RX aktif");
  }
  if (ps2x.ButtonPressed(PSB_R1)) {
    freezeAxis[1] = !freezeAxis[1]; // RY
    Serial.println(freezeAxis[1] ? "RY dibekukan" : "RY aktif");
  }
  if (ps2x.ButtonPressed(PSB_L2)) {
    freezeAxis[2] = !freezeAxis[2]; // LX
    Serial.println(freezeAxis[2] ? "LX dibekukan" : "LX aktif");
  }
  if (ps2x.ButtonPressed(PSB_L1)) {
    freezeAxis[3] = !freezeAxis[3]; // LY
    Serial.println(freezeAxis[3] ? "LY dibekukan" : "LY aktif");
  }

  // Update servo movement 
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

  
  if (ps2x.ButtonPressed(PSB_CROSS)) {
    gripperPos = GRIPPER_OPEN;
    Serial.println("Gripper terbuka (X)");
  }
  if (ps2x.ButtonPressed(PSB_CIRCLE)) {
    gripperPos = GRIPPER_CLOSE;
    Serial.println("Gripper tertutup (O)");
  }
  pca.setPWM(GRIPPER_CHANNEL, 0, gripperPos);

  
  Serial.print("RX: "); Serial.print(joy[0]);
  Serial.print("  RY: "); Serial.print(joy[1]);
  Serial.print("  LX: "); Serial.print(joy[2]);
  Serial.print("  LY: "); Serial.println(joy[3]);

  delay(50);
}
