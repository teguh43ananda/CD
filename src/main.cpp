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

// PID
float kp = 0.3;
float ki = 0.02;
float kd = 0.15;

// Default alpha dan mode
float alpha = 0.3;
bool modeHalus = true; // true = halus, false = biasa

// PID state
float errorPID[4] = {0, 0, 0, 0};
float prevError[4] = {0, 0, 0, 0};
float integral[4] = {0, 0, 0, 0};
float currentPWM[4] = {SERVO_MID, SERVO_MID, SERVO_MID, SERVO_MID};

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

  // Set awal ke mode halus
  alpha = 0.2;
  Serial.println("Mode: HALUS");
}

void loop() {
  if (error == 1) return;

  ps2x.read_gamepad(false, vibrate);

  // Toggle mode dengan tombol SELECT
  if (ps2x.ButtonPressed(PSB_SELECT)) {
    modeHalus = !modeHalus; // toggle
    alpha = modeHalus ? 0.2 : 0.4;
    Serial.print("Mode berubah ke: ");
    Serial.println(modeHalus ? "HALUS" : "BIASA");
  }

  // Joystick input
  int joy[4];
  joy[0] = ps2x.Analog(PSS_RX);
  joy[1] = ps2x.Analog(PSS_RY);
  joy[2] = ps2x.Analog(PSS_LX);
  joy[3] = ps2x.Analog(PSS_LY);

  for (int i = 0; i < 4; i++) {
    int targetPWM = map(joy[i], 0, 255, SERVOMIN, SERVOMAX);
    errorPID[i] = targetPWM - currentPWM[i];
    integral[i] += errorPID[i];
    float derivative = errorPID[i] - prevError[i];

    float output = kp * errorPID[i] + ki * integral[i] + kd * derivative;
    prevError[i] = errorPID[i];

    // Low-pass filter
    currentPWM[i] = (1 - alpha) * currentPWM[i] + alpha * (currentPWM[i] + output);

    // Clamp
    if (currentPWM[i] < SERVOMIN) currentPWM[i] = SERVOMIN;
    if (currentPWM[i] > SERVOMAX) currentPWM[i] = SERVOMAX;

    // Kirim ke servo
    pca.setPWM(i, 0, currentPWM[i]);
  }

  // Gripper
  if (ps2x.ButtonPressed(PSB_R1)) {
    gripperPos = GRIPPER_OPEN;
    Serial.println("Gripper terbuka (R1)");
  } else if (ps2x.ButtonPressed(PSB_L1)) {
    gripperPos = GRIPPER_CLOSE;
    Serial.println("Gripper tertutup (L1)");
  }
  pca.setPWM(GRIPPER_CHANNEL, 0, gripperPos);

  // Debug joystick
  Serial.print("RX: "); Serial.print(joy[0]);
  Serial.print("  RY: "); Serial.print(joy[1]);
  Serial.print("  LX: "); Serial.print(joy[2]);
  Serial.print("  LY: "); Serial.println(joy[3]);

  delay(50);
}
