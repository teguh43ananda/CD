// #include <Arduino.h>
// #include <Wire.h>
// #include <Adafruit_PWMServoDriver.h>
// #include <PS2X_lib.h>

// Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

// #define SERVOMIN  90     // pulse untuk 0 derajat
// #define SERVOMAX  500    // pulse untuk 180 derajat
// #define SERVO_MID ((SERVOMIN + SERVOMAX) / 2)

// PS2X ps2x;
// int error = 0;
// byte type = 0;
// byte vibrate = 0;

// void setup() {
//   Serial.begin(57600);
//   Serial.println("Inisialisasi PCA9685");

//   pca.begin();
//   pca.setPWMFreq(50);  // 50Hz PWM untuk servo

//   delay(1000);

//   // clock, command, attention, data
//   error = ps2x.config_gamepad(13, 23, 24, 25, true, true);

//   if (error == 0) {
//     Serial.println("Controller terdeteksi dan dikonfigurasi.");
//   } else {
//     Serial.println("Gagal mendeteksi controller. Cek wiring.");
//   }

//   type = ps2x.readType();
//   if (type == 1) Serial.println("DualShock Controller terdeteksi");

//   // Set posisi awal servo ke 90 derajat
//   for (int i = 0; i < 4; i++) {
//     pca.setPWM(i, 0, SERVO_MID);
//   }
// }

// void loop() {
//   if (error == 1) return; // controller tidak terhubung

//   ps2x.read_gamepad(false, vibrate);

//   // Baca nilai analog stick
//   int joyRX = ps2x.Analog(PSS_RX);
//   int joyRY = ps2x.Analog(PSS_RY);
//   int joyLX = ps2x.Analog(PSS_LX);
//   int joyLY = ps2x.Analog(PSS_LY);

//   // Mapping nilai analog (0–255) ke pulse servo (SERVOMIN–SERVOMAX)
//   int pwmRX = map(joyRX, 0, 255, SERVOMIN, SERVOMAX);  // Servo 0
//   int pwmRY = map(joyRY, 0, 255, SERVOMIN, SERVOMAX);  // Servo 1
//   int pwmLX = map(joyLX, 0, 255, SERVOMIN, SERVOMAX);  // Servo 2
//   int pwmLY = map(joyLY, 0, 255, SERVOMIN, SERVOMAX);  // Servo 3

//   // Kirim posisi servo dari joystick
//   pca.setPWM(0, 0, pwmRX);
//   pca.setPWM(1, 0, pwmRY);
//   pca.setPWM(2, 0, pwmLX);
//   pca.setPWM(3, 0, pwmLY);

//   // Tampilkan nilai analog ke Serial Monitor (Data Stream)
//   Serial.print("RX: ");
//   Serial.print(joyRX);
//   Serial.print("  RY: ");
//   Serial.print(joyRY);
//   Serial.print("  LX: ");
//   Serial.print(joyLX);
//   Serial.print("  LY: ");
//   Serial.println(joyLY);

//   // Tombol START → reset semua servo ke posisi tengah (90 derajat)
//   if (ps2x.ButtonPressed(PSB_START)) {
//     Serial.println("Tombol START ditekan - Servo reset ke 90 derajat");
//     for (int i = 0; i < 5; i++) {
//       pca.setPWM(i, 0, SERVO_MID);
//     }
//   }

//   delay(50); // delay kecil agar output stabil di Serial Monitor
// }

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
const int GRIPPER_OPEN = SERVOMAX;  // nilai gripper terbuka
const int GRIPPER_CLOSE = SERVOMIN; // nilai gripper tertutup

PS2X ps2x;
int error = 0;
byte type = 0;
byte vibrate = 0;

void setup() {
  Serial.begin(57600);
  Serial.println("Inisialisasi PCA9685");

  pca.begin();
  pca.setPWMFreq(50);  // 50Hz PWM untuk servo
  delay(1000);

  // clock, command, attention, data
  error = ps2x.config_gamepad(13, 23, 24, 25, true, true);

  if (error == 0) {
    Serial.println("Controller terdeteksi dan dikonfigurasi.");
  } else {
    Serial.println("Gagal mendeteksi controller. Cek wiring.");
  }

  type = ps2x.readType();
  if (type == 1) Serial.println("DualShock Controller terdeteksi");

  // Set semua servo ke posisi tengah

  
  for (int i = 0; i < 5; i++) {
    pca.setPWM(i, 0, SERVO_MID);
  }
}

void loop() {
  if (error == 1) return; // controller tidak terhubung

  ps2x.read_gamepad(false, vibrate);

  // Baca nilai analog stick
  int joyRX = ps2x.Analog(PSS_RX);
  int joyRY = ps2x.Analog(PSS_RY);
  int joyLX = ps2x.Analog(PSS_LX);
  int joyLY = ps2x.Analog(PSS_LY);

  // Mapping nilai analog ke PWM servo
  int pwmRX = map(joyRX, 0, 255, SERVOMIN, SERVOMAX);  // Servo 0
  int pwmRY = map(joyRY, 0, 255, SERVOMIN, SERVOMAX);  // Servo 1
  int pwmLX = map(joyLX, 0, 255, SERVOMIN, SERVOMAX);  // Servo 2
  int pwmLY = map(joyLY, 0, 255, SERVOMIN, SERVOMAX);  // Servo 3

  // Kirim posisi servo
  pca.setPWM(0, 0, pwmRX);
  pca.setPWM(1, 0, pwmRY);
  pca.setPWM(2, 0, pwmLX);
  pca.setPWM(3, 0, pwmLY);

  // iuyyfghjgf

  // Kontrol gripper
  if (ps2x.ButtonPressed(PSB_R1)) {
    gripperPos = GRIPPER_OPEN;
    Serial.println("Gripper terbuka (R1)");
  } else if (ps2x.ButtonPressed(PSB_L1)) {
    gripperPos = GRIPPER_CLOSE;
    Serial.println("Gripper tertutup (L1)");
  }
  pca.setPWM(GRIPPER_CHANNEL, 0, gripperPos);

  // Tombol START → reset semua servo ke posisi tengah
  if (ps2x.ButtonPressed(PSB_START)) {
    Serial.println("Tombol START ditekan - Servo reset ke 90 derajat");
    for (int i = 0; i < 5; i++) {
      pca.setPWM(i, 0, SERVO_MID);
    }
    gripperPos = SERVO_MID;  // reset posisi gripper juga
  }

  // Tampilkan data joystick
  Serial.print("RX: ");
  Serial.print(joyRX);
  Serial.print("  RY: ");
  Serial.print(joyRY);
  Serial.print("  LX: ");
  Serial.print(joyLX);
  Serial.print("  LY: ");
  Serial.println(joyLY);

  delay(50);
}
