// H_interface.ino - Simplified & Fixed Version
// Based on "Working System" architecture

#include <SCServo.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_now.h>

SMS_STS sms_sts;
WebServer server(80);

#define SERVO_NUM 6
uint8_t IDs[SERVO_NUM] = {0, 1, 5, 3, 4, 2};

// Home positions (0-4095)
int homePos[SERVO_NUM] = {0, 2007, 900, 2987, 3001, 900};

// Current positions (for synchronization)
int currentPos[SERVO_NUM];

// Gripper address
uint8_t gripperAddress[] = {0xE8, 0x6B, 0xEA, 0xF6, 0x73, 0xF8};

// ESP-NOW message structure
typedef struct struct_message {
  char command[32];
  int value;
} struct_message;

struct_message outgoingMsg;

// Joint limits (Optional/Simple)
// Min/Max in RADIANS (for web interface)
// Min/Max in RADIANS (for web interface)
float min_rad[SERVO_NUM] = {-3.14, -0.61, -0.61, -1.75, -1.31, -6.28};
float max_rad[SERVO_NUM] = {3.14, 0.61, 0.61, 1.75, 1.31, 6.28};

// Helper: Convert Radians to Position (0-4095)
int radToPos(int id, float rad) {
  // Simple linear mapping based on STS3215 specs (0-4095 = 0-360 deg)
  // Calibrated Centers (Steps at 0 rad)
  // Derived from Home Position: {0, 2007, 900, 2987, 3001, 900}
  // ID 1 (Shoulder L) @ -0.26 rad = 2007 -> Center = 2176
  // ID 5 (Shoulder R) @ -0.26 rad = 900  -> Center = 731
  // ID 3 (Elbow)      @ -0.785 rad = 2987 -> Center = 3498
  // ID 4 (Wrist 1)    @ -1.27 rad = 3001 -> Center = 3828
  // ID 2 (Wrist 2)    @ 0.0 rad   = 900  -> Center = 900
  // ID 0 (Base)       @ 0.0 rad   = 0    -> Center = 0
  
  // Normalize base joint (ID 0) to [-PI, PI]
  if (id == 0) {
    while (rad > 3.14159) rad -= 6.28318;
    while (rad < -3.14159) rad += 6.28318;
  }

  int steps = (int)(rad * 651.73);
  
  if (id == 0) return 0 + steps; // Center 0
  if (id == 1) return 2176 + steps;
  if (id == 5) return 731 - steps; // Mirrored (Counter-Directional)
  if (id == 3) return 3498 + steps;
  if (id == 4) return 3828 + steps;
  if (id == 2) return 900 + steps;
  return 2048 + steps;
}

// Helper: Move to Pose (Synchronized via Speed Calculation)
void moveToPose(const int pose[SERVO_NUM], int timeMs) {
  // Constraints (Min, Max)
  int limits[6][2] = {
    {-2100, 2100},   // ID 0 - Restricted to +/- 180 deg (approx)
    {1779, 2573},    // ID 1 - URDF: -0.61 to 0.61 rad
    {334, 1128},     // ID 5 - URDF: -0.61 to 0.61 rad (Mirrored)
    {2358, 4095},    // ID 3 - URDF: -1.75 to 1.75 rad (Clamped at 4095)
    {2975, 4095},    // ID 4 - URDF: -1.31 to 1.31 rad (Clamped at 4095)
    {0, 4095}        // ID 2 - URDF: -6.28 to 6.28 rad
  };

  if (timeMs < 1) timeMs = 1; 
  
  for (int i = 0; i < SERVO_NUM; i++) {
    // Clamp target to limits
    int target = pose[i];
    
    if (target < limits[i][0]) target = limits[i][0];
    if (target > limits[i][1]) target = limits[i][1];
    
    // Enable torque
    sms_sts.EnableTorque(IDs[i], 1);
    
    // SMART START POSITION LOGIC
    // To ensure smooth velocity transitions, we prefer using the *previous target* (currentPos)
    // as the start point for speed calculation. This preserves the velocity vector.
    // However, if the robot has drifted significantly (e.g. external force), we must use actualPos.
    
    int startPos = currentPos[i]; // Default: Use previous target (assumes perfect tracking)
    
    int actualPos = sms_sts.ReadPos(IDs[i]);
    if (actualPos != -1) {
      // Check deviation
      if (abs(actualPos - currentPos[i]) > 100) {
        // Significant drift/lag -> Use actual position to catch up
        startPos = actualPos;
      }
      // Else: Deviation is small, ignore it to maintain smooth velocity flow
    }
    
    // Calculate distance
    int dist = abs(target - startPos);
    
    // Calculate speed (Steps per Second)
    int speed = (dist * 1000) / timeMs;
    if (speed == 0 && dist > 0) speed = 1;
    
    // Send move command (Register Write - Wait for Action)
    // ACCELERATION = 0 (Max/Instant) -> Crucial for trajectory tracking!
    // We want the servo to hit the calculated speed immediately, not ramp up.
    sms_sts.RegWritePosEx(IDs[i], target, speed, 0); 
    
    // Update current position tracker (Optimistic update for next loop)
    currentPos[i] = target;
  }
  
  // TRIGGER SYNC MOVEMENT
  sms_sts.RegWriteAction();
}

// ESP-NOW Callback
// Note: Signature updated for ESP32 Arduino Core v3.0+
// If using older version (v2.x), change 'const wifi_tx_info_t *info' to 'const uint8_t *mac_addr'
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // Serial.print("Last Packet Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void sendGripperCommand(const char *cmd, int val) {
  strcpy(outgoingMsg.command, cmd);
  outgoingMsg.value = val;
  esp_now_send(gripperAddress, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));
}

// Web Handlers
void handleRoot() {
  server.send(200, "text/plain", "Pegasus Arm Controller (Simplified)");
}

void handleSetJoint() {
  // Parse j1=0.5&j2=0.3 format
  int targetPose[SERVO_NUM];
  bool changed = false;
  
  // Initialize with current positions (read from servos)
  for(int i=0; i<SERVO_NUM; i++) {
    int pos = sms_sts.ReadPos(IDs[i]);
    if(pos == -1) pos = homePos[i]; // Fallback
    targetPose[i] = pos;
  }

  for (int k = 1; k <= 6; k++) {
    String param = "j" + String(k);
    if (server.hasArg(param)) {
      float rad = server.arg(param).toFloat();
      
      // Apply limits
      if(rad < min_rad[k-1]) rad = min_rad[k-1];
      if(rad > max_rad[k-1]) rad = max_rad[k-1];
      
      // Convert to position
      // Note: ID mapping: k=1->ID0, k=2->ID1/5, k=3->ID3, k=4->ID4, k=5->ID2
      if (k == 1) targetPose[0] = radToPos(0, rad);
      else if (k == 2) {
        targetPose[1] = radToPos(1, rad); // ID 1
        targetPose[2] = radToPos(5, rad); // ID 5 (Shoulder R)
      }
      else if (k == 3) targetPose[3] = radToPos(3, rad);
      else if (k == 4) targetPose[4] = radToPos(4, rad);
      else if (k == 5) targetPose[5] = radToPos(2, rad); // ID 2 (Wrist 2)
      
      changed = true;
    }
  }
  
  if(changed) {
    moveToPose(targetPose, 1000);
    server.send(200, "text/plain", "Moving");
  } else {
    server.send(200, "text/plain", "No changes");
  }
}

void handleHome() {
  int pose[SERVO_NUM];
  for(int i=0; i<SERVO_NUM; i++) pose[i] = homePos[i];
  moveToPose(pose, 2000);
  server.send(200, "text/plain", "Homing");
}

void handleGripper() {
  if (server.hasArg("pos")) {
    int pos = server.arg("pos").toInt();
    sendGripperCommand("SET", pos);
    server.send(200, "text/plain", "Gripper set");
  }
}

// Serial Parser
String readSerialLine() {
  String line = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') break;
    line += c;
  }
  return line;
}

void parseAndExecute(String line) {
  line.trim();
  if (line.length() == 0) return;
  
  // Format: T1,duration,val1,val2,val3,val4,val5,val6
  if (line.startsWith("T")) {
    Serial.print("ECHO: "); Serial.println(line);
    
    int firstComma = line.indexOf(',');
    if (firstComma == -1) return;
    
    // Extract Duration (2nd field)
    int secondComma = line.indexOf(',', firstComma + 1);
    if (secondComma == -1) return;
    
    String durationStr = line.substring(firstComma + 1, secondComma);
    int duration = durationStr.toInt();
    if (duration < 10) duration = 10; // Min safety
    
    // Extract Values
    String values = line.substring(secondComma + 1);
    int pose[SERVO_NUM];
    int idx = 0;
    int lastComma = -1;
    
    for (int i = 0; i <= values.length(); i++) {
      if (i == values.length() || values.charAt(i) == ',') {
        if (idx < SERVO_NUM) {
          String valStr = values.substring(lastComma + 1, i);
          pose[idx] = valStr.toInt();
          idx++;
        }
        lastComma = i;
      }
    }
    
    if (idx >= 5) { // Allow 5 or 6
      // Serial.println("Commanding pose...");
      moveToPose(pose, duration); // Synchronized move with specified duration
      Serial.println("ACK T,done");
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, 18, 19);
  sms_sts.pSerial = &Serial1;
  
  Serial.println("Pegasus Arm - Simplified Firmware");
  
  // Init Servos & Read Current Positions
  for(int i=0; i<SERVO_NUM; i++) {
    sms_sts.EnableTorque(IDs[i], 1);
    
    int pos = sms_sts.ReadPos(IDs[i]);
    if (pos == -1) pos = homePos[i]; // Fallback if read fails
    currentPos[i] = pos;
    
    delay(2);
  }
  
  // Move to Home Position on Startup (Synchronized)
  Serial.println("Moving to Home Position...");
  moveToPose(homePos, 2000); // Enabled: Moves to home on boot
  
  // WiFi
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("ESP32_DEV", "12345678");
  Serial.print("WiFi AP IP: "); Serial.println(WiFi.softAPIP());
  
  // ESP-NOW
  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(OnDataSent);
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, gripperAddress, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }
  
  // Web Server
  server.on("/", handleRoot);
  server.on("/set_joint", handleSetJoint);
  server.on("/home", handleHome);
  server.on("/gripper", handleGripper);
  server.begin();
}

void loop() {
  server.handleClient();
  
  if (Serial.available()) {
    String line = readSerialLine();
    parseAndExecute(line);
  }
  
  delay(1);
}
