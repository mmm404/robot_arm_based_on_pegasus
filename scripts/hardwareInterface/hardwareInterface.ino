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

// === Move function ===
void moveToPose(const uint16_t pose[SERVO_NUM], unsigned long moveMs) {
  Serial.printf("Commanding pose (time %lu ms) ...\n", moveMs);

  for (uint8_t i = 0; i < SERVO_NUM; ++i) {
    sms_sts.WritePosEx(IDs[i], pose[i], moveMs, 200); // use higher speed
    delay(5);
  }

  unsigned long waitUntil = millis() + moveMs + 100;
  while (millis() < waitUntil) {
    delay(5);
  }
  Serial.println("Pose command issued.");
}

// === SERIAL READER with echo ===
String readSerialLine() {
  static String line = "";
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      String result = line;
      line = "";

      // ✅ Echo back for confirmation
      Serial.print("ECHO: ");
      Serial.println(result);

      return result;
    } else if (c != '\r') {
      line += c;
    }
  }
  return "";
}

// === Parse "T1,1000,1800,1500,1200,800,400" ===
void parseAndExecute(String line) {
  if (line.length() == 0) return;
  if (line.charAt(0) != 'T') {
    Serial.println("⚠️ Ignoring line (no 'T' prefix)");
    return;
  }

  int firstComma = line.indexOf(',');
  if (firstComma == -1) {
    Serial.println("⚠️ Invalid format (no commas)");
    return;
  }

  int traj_id = line.substring(1, firstComma).toInt();
  Serial.printf("📡 Trajectory ID = %d\n", traj_id);

  uint16_t target[SERVO_NUM];
  int lastIndex = firstComma;
  for (int i = 0; i < SERVO_NUM; i++) {
    int nextIndex = line.indexOf(',', lastIndex + 1);
    String val = (nextIndex == -1) ? line.substring(lastIndex + 1)
                                   : line.substring(lastIndex + 1, nextIndex);

    target[i] = val.toInt();
    Serial.printf("  -> Joint %d target = %d\n", i, target[i]);
    lastIndex = nextIndex;
    if (nextIndex == -1) break;
  }

  // Execute trajectory
  moveToPose(target, 1500);

  // ✅ Send ACK back
  Serial.printf("ACK T%d,done\n", traj_id);
}

void setup() {
  Serial.begin(115200);   // USB Serial0 (bridge will talk here)
  Serial1.begin(1000000, SERIAL_8N1, 18, 19); // Servo bus RX=18, TX=19
  sms_sts.pSerial = &Serial1;

  delay(2000);
  Serial.println("✅ SCServo controller ready.");
  Serial.println("Moving to HOME pose...");
  moveToPose(homePos, 1500);
  Serial.println("✅ At HOME. Waiting for trajectory input on Serial0...");
}

void loop() {
  String line = readSerialLine();
  if (line.length() > 0) {
    parseAndExecute(line);
  }
  delay(5);
}
