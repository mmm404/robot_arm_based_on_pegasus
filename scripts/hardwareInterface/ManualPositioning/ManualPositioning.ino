#include <SCServo.h>

SMS_STS sms_sts;
#define SERVO_NUM 6
uint8_t IDs[SERVO_NUM] = {0, 1, 2, 3, 4, 5};

// HOME pose
uint16_t homePos[SERVO_NUM] = {
  0,     // Base 0°
  1843,  // Shoulder1 -15°
  1843,  // Shoulder2 -15°
  1433,  // Elbow -45°
  1051,  // Wrist -73°
  0      // Gripper closed
};

// EXTENDED pose
uint16_t extendedPos[SERVO_NUM] = {
  3276,  // Base 90° ii
  2457,   // Shoulder1 30°
  2457,   // Shoulder2 30°
  2867,   // Elbow 60°
  1638,  // Wrist -30°kb
  341    // Gripper open
};

void moveToPose(const uint16_t pose[SERVO_NUM], unsigned long moveMs) {
  Serial.print("Commanding pose (time ");
  Serial.print(moveMs);
  Serial.println(" ms) ...");

  // WritePosEx: ID, position, time(ms), speed
  for (uint8_t i = 0; i < SERVO_NUM; ++i) {
    sms_sts.WritePosEx(IDs[i], pose[i], moveMs, 50);
    delay(6);
  }

  unsigned long waitUntil = millis() + moveMs + 120;
  while (millis() < waitUntil) {
    delay(10);
  }
  Serial.println("Pose command issued.");
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, 18, 19); // RX=18, TX=19
  sms_sts.pSerial = &Serial1;

  delay(2000);
  Serial.println("Starting SCServo control...");

  // Start in HOME pose
  moveToPose(homePos, 1200);
}

void loop() {
  // Alternate every 10 seconds
  moveToPose(extendedPos, 1200);
  Serial.println("Extended command.");

  delay(10000);

  moveToPose(homePos, 1200);
  Serial.println("Home command.");
  delay(10000);
}
