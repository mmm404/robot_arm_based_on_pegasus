#include <SCServo.h>

SMS_STS sms_sts;

// Configuration
#define SERVO_NUM 6
// Servo IDs: Base, Shoulder1, Shoulder2, Elbow, Wrist, Gripper/Aux
uint8_t IDs[SERVO_NUM] = {0, 1, 2, 3, 4, 5};

// Kinematics / Calibration
// Assuming 4096 steps = 360 degrees = 2*PI radians
// Center (0 radians) is at step 2048
const int CENTER_OFFSET = 2048;
const float STEPS_PER_MRAD = 0.6519; // (4096 / (2 * PI)) / 1000

// Serial Communication
// Serial0 (USB) for commands from PC
// Serial1 (Pins 18, 19) for Servo Bus
#define BAUDRATE_PC 115200
#define BAUDRATE_SERVOS 1000000

void setup() {
  Serial.begin(BAUDRATE_PC);
  Serial1.begin(BAUDRATE_SERVOS, SERIAL_8N1, 18, 19);
  sms_sts.pSerial = &Serial1;
  
  delay(1000);
  
  // Optional: Soft start or initial position check could go here
  // For now, we just signal ready
  // Serial.println("READY"); // Python doesn't explicitly wait for this, but good for debug
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      processCommand(line);
    }
  }
}

void processCommand(String line) {
  // 1. LIMITS Query
  if (line.startsWith("LIMITS")) {
    // Python sends this to check connection. 
    // We can just reply OK or ignore, but sending OK is safer.
    Serial.println("OK"); 
    return;
  }

  // 2. Trajectory Start: "T{id}"
  if (line.startsWith("T")) {
    // Python expects "OK" to confirm start
    // We can reset any internal state if needed
    Serial.println("OK");
    return;
  }

  // 3. Point Data: "P{index},{val0},{val1},{val2},{val3},{val4},{val5}"
  if (line.startsWith("P")) {
    handlePoint(line);
    return;
  }

  // 4. Position Query: "READ_POS"
  if (line.startsWith("READ_POS")) {
    sendPositions();
    return;
  }
}

void handlePoint(String line) {
  // Format: P0,1000,2000,3000,4000,5000,0
  // Remove 'P' and parse
  
  int firstComma = line.indexOf(',');
  if (firstComma == -1) {
    Serial.println("ERROR:Format");
    return;
  }

  // We can ignore the index (P0, P1...) for execution if we just execute immediately
  // But we need to parse the values after the first comma
  
  String data = line.substring(firstComma + 1); // "1000,2000,..."
  
  int target_mrad[SERVO_NUM];
  int parsed_count = 0;
  int lastIndex = -1;
  
  for (int i = 0; i < SERVO_NUM; i++) {
    int nextIndex = data.indexOf(',', lastIndex + 1);
    
    String valStr;
    if (nextIndex == -1) {
      valStr = data.substring(lastIndex + 1);
    } else {
      valStr = data.substring(lastIndex + 1, nextIndex);
    }
    
    target_mrad[i] = valStr.toInt();
    lastIndex = nextIndex;
    parsed_count++;
    
    if (nextIndex == -1) break;
  }

  if (parsed_count < SERVO_NUM) {
    Serial.println("ERROR:MissingData");
    return;
  }

  // Move Servos
  // Python sends milli-radians. Convert to steps.
  // We use a fixed time for now, or we could try to calculate based on distance.
  // Python sends points fairly quickly (controlled by its own timer).
  // Let's use a small time for smooth motion, e.g., 200ms or match the send rate.
  // Python loop has sleeps, but sends points in a burst with small delays?
  // Actually python sends: "sleep_time = max(0, next_time - elapsed)"
  // So it streams them in real-time. We should move immediately.
  
  for (int i = 0; i < SERVO_NUM; i++) {
    int steps = mradToSteps(target_mrad[i]);
    // Safety clamp (0-4095)
    if (steps < 0) steps = 0;
    if (steps > 4095) steps = 4095;
    
    // WritePosEx(ID, Position, Speed, Acceleration)
    // Speed is steps/sec? Or time? 
    // Library usually: WritePosEx(id, pos, speed, acc) where speed is steps/sec
    // OR WritePos(id, pos, time, speed) depending on library version.
    // Looking at hardwareInterface.ino: sms_sts.WritePosEx(IDs[i], pose[i], moveMs, 200);
    // So 3rd arg is Time (ms)? Or Speed?
    // "moveMs" implies Time.
    // Let's assume 100ms for quick updates.
    sms_sts.WritePosEx(IDs[i], steps, 100, 50); 
  }
  
  Serial.println("OK");
}

void sendPositions() {
  // Format: POS:0=1000,1=2000,2=3000,3=4000,4=5000,5=6000,GRIP=0
  // Note: Python expects milli-radians.
  
  String response = "POS:";
  
  for (int i = 0; i < SERVO_NUM; i++) {
    int pos = sms_sts.ReadPos(IDs[i]);
    if (pos == -1) {
      // Error reading, assume 2048 or last known?
      pos = 2048; 
    }
    
    int mrad = stepsToMrad(pos);
    
    response += String(i) + "=" + String(mrad) + ",";
  }
  
  // Append GRIP (dummy or real if ID 5 is grip)
  // Python parser drops the last element, so we add a dummy ender
  response += "GRIP=0";
  
  Serial.println(response);
}

int mradToSteps(int mrad) {
  // 0 mrad -> 2048 steps
  // +val -> +steps
  return CENTER_OFFSET + (int)(mrad * STEPS_PER_MRAD);
}

int stepsToMrad(int steps) {
  return (int)((steps - CENTER_OFFSET) / STEPS_PER_MRAD);
}
