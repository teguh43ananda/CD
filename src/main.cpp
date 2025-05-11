#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

#define SERVOMIN  90     // pulse untuk 0 derajat
#define SERVOMAX  500    // pulse untuk 180 derajat
#define SERVO_MID ((SERVOMIN + SERVOMAX) / 2)

#define GRIPPER_CHANNEL 4  // Channel servo gripper
int gripperPos = SERVO_MID;  // Posisi awal gripper
const int GRIPPER_OPEN = SERVOMAX;
const int GRIPPER_CLOSE = SERVOMIN;

PS2X ps2x;
int error = 0;
byte type = 0;
byte vibrate = 0;

// PID Variabel
float kp = 1.0, ki = 0.0, kd = 0.0;
float errorPID[4] = {0};
float previous_error[4] = {0};
float integral[4] = {0};
float currentPWM[4] = {SERVO_MID, SERVO_MID, SERVO_MID, SERVO_MID};

void setup() {
  Serial.begin(57600);
  Serial.println("Inisialisasi PCA9685");

  pca.begin();
  pca.setPWMFreq(50);
  delay(1000);

  error = ps2x.config_gamepad(13, 23, 24, 25, true, true);

  if (error == 0) {
    Serial.println("Controller terdeteksi dan dikonfigurasi.");
  } else {
    Serial.println("Gagal mendeteksi controller. Cek wiring.");
  }

  type = ps2x.readType();
  if (type == 1) Serial.println("DualShock Controller terdeteksi");

  for (int i = 0; i < 5; i++) {
    pca.setPWM(i, 0, SERVO_MID);
  }
}

void loop() {
  if (error == 1) return;

  ps2x.read_gamepad(false, vibrate);

  int joyVal[4] = {
    ps2x.Analog(PSS_RX),
    ps2x.Analog(PSS_RY),
    ps2x.Analog(PSS_LX),
    ps2x.Analog(PSS_LY)
  };

  int targetPWM[4];
  for (int i = 0; i < 4; i++) {
    targetPWM[i] = map(joyVal[i], 0, 255, SERVOMIN, SERVOMAX);
  }

  // PID loop untuk keempat servo
  for (int i = 0; i < 4; i++) {
    errorPID[i] = targetPWM[i] - currentPWM[i];
    integral[i] += errorPID[i];
    float derivative = errorPID[i] - previous_error[i];
    float output = kp * errorPID[i] + ki * integral[i] + kd * derivative;

    // Update posisi saat ini (simulasi gerakan servo)
    currentPWM[i] += output;

    // Batasi agar tidak keluar dari range
    currentPWM[i] = constrain(currentPWM[i], SERVOMIN, SERVOMAX);

    pca.setPWM(i, 0, currentPWM[i]);
    previous_error[i] = errorPID[i];
  }

  // Gripper control
  if (ps2x.ButtonPressed(PSB_R1)) {
    gripperPos = GRIPPER_OPEN;
    Serial.println("Gripper terbuka (R1)");
  } else if (ps2x.ButtonPressed(PSB_L1)) {
    gripperPos = GRIPPER_CLOSE;
    Serial.println("Gripper tertutup (L1)");
  }
  pca.setPWM(GRIPPER_CHANNEL, 0, gripperPos);

  // Tombol START
  if (ps2x.ButtonPressed(PSB_START)) {
    Serial.println("Tombol START ditekan - Servo reset ke 90 derajat");
    for (int i = 0; i < 5; i++) {
      pca.setPWM(i, 0, SERVO_MID);
    }
    for (int i = 0; i < 4; i++) {
      currentPWM[i] = SERVO_MID;
      integral[i] = 0;
      previous_error[i] = 0;
    }
    gripperPos = SERVO_MID;
  }

  Serial.print("RX: "); Serial.print(joyVal[0]);
  Serial.print("  RY: "); Serial.print(joyVal[1]);
  Serial.print("  LX: "); Serial.print(joyVal[2]);
  Serial.print("  LY: "); Serial.println(joyVal[3]);

  delay(50);
}
