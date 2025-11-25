// H_interface.ino - WITH JOINT LIMITS SYSTEM
#include <SCServo.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>

SMS_STS sms_sts;
WebServer server(80);

#define SERVO_NUM 6
uint8_t IDs[SERVO_NUM] = {0, 1, 2, 3, 4, 5};

// Gripper ESP32 MAC address
uint8_t gripperAddress[] = {0xE8, 0x6B, 0xEA, 0xF6, 0x73, 0xF8};

const float SERVO_MIN_POS = 0;
const float SERVO_MAX_POS = 4095;

float min_rad[SERVO_NUM] = {
    -6.28f,  // Base: ±360 degrees
    -0.61f,  // Shoulder: -35 degrees
    -0.61f,  // Shoulder (mirrored)
    -1.75f,  // Elbow: -100 degreesclck
    -1.31f,  // Wrist: -75 degrees
    -6.28f   // Wrist rotation: ±360 degrees
};

float max_rad[SERVO_NUM] = {
    6.28f,   // Base: ±360 degrees
    0.61f,   // Shoulder: +35 degrees
    0.61f,   // Shoulder (mirrored)
    1.75f,   // Elbow: +100 degrees
    1.31f,   // Wrist: +75 degrees
    6.28f    // Wrist rotation: ±360 degrees
};

float home_rad[SERVO_NUM] = {0.10f, 0.10f, 0.10f, 0.52f, 1.70f, -0.16f};

// ========================================
// CONFIGURABLE JOINT LIMITS (CHANGE THESE!)
// ========================================
// Set your operational limits here (in radians)
// These will override the hardware limits above
float joint_min_limit[SERVO_NUM] = {
  -2.5f,   // Joint 1: Base rotation min
  0.05f,  // Joint 2: Shoulder min
  0.05f,  // Joint 3: Shoulder min (mirrored)
  -0.2f,   // Joint 4: Elbow min
  1.1f,    // Joint 5: Wrist min
  -2.5f    // Joint 6: Wrist rotation min
};

float joint_max_limit[SERVO_NUM] = {
  2.5f,    // Joint 1: Base rotation max
  0.15f,   // Joint 2: Shoulder max
  0.15f,   // Joint 3: Shoulder max (mirrored)
  0.65f,    // Joint 4: Elbow max
  1.75f,    // Joint 5: Wrist max
  2.5f     // Joint 6: Wrist rotation max
};

// Enable/disable limits (set to false to disable limit checking)
bool limits_enabled = true;

// ========================================

// Gripper control
int currentGripperPos = 90;
bool gripperConnected = false;
unsigned long lastGripperPing = 0;

// ESP-NOW message structure
typedef struct struct_message {
  char command[32];
  int value;
} struct_message;

struct_message outgoingMsg;
struct_message incomingMsg;

// Trajectory buffer
struct TrajectoryPoint {
  int positions[SERVO_NUM];
  int trajectoryID;
  bool valid;
};

#define BUFFER_SIZE 100
TrajectoryPoint trajBuffer[BUFFER_SIZE];
int bufferHead = 0;
int bufferTail = 0;
int bufferCount = 0;

int currentTrajectoryID = -1;
int incomingTrajectoryID = -1;
bool executingTrajectory = false;

int currentPos[SERVO_NUM] = {0};
int targetPos[SERVO_NUM] = {0};

const int MOVEMENT_SPEED = 2000;
const int MOVEMENT_ACCEL = 50;

unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 50;

String inputBuffer = "";
bool isMoving = false;
int pointsReceived = 0;
int pointsExecuted = 0;

// ========================================
// JOINT LIMITS FUNCTIONS
// ========================================

// Constrain a radian value to joint limits
float constrainToJointLimits(int joint_index, float rad_value) {
  if (!limits_enabled) {
    // If limits disabled, only use hardware limits
    return constrain(rad_value, min_rad[joint_index], max_rad[joint_index]);
  }
  
  // Apply both hardware and joint limits
  float hw_min = min_rad[joint_index];
  float hw_max = max_rad[joint_index];
  float jl_min = joint_min_limit[joint_index];
  float jl_max = joint_max_limit[joint_index];
  
  // Use the most restrictive limits
  float effective_min = max(hw_min, jl_min);
  float effective_max = min(hw_max, jl_max);
  
  return constrain(rad_value, effective_min, effective_max);
}

// Check if a radian value is within joint limits
bool isWithinJointLimits(int joint_index, float rad_value) {
  if (!limits_enabled) return true;
  
  float effective_min = max(min_rad[joint_index], joint_min_limit[joint_index]);
  float effective_max = min(max_rad[joint_index], joint_max_limit[joint_index]);
  
  return (rad_value >= effective_min && rad_value <= effective_max);
}

// Get effective min limit for a joint
float getEffectiveMinLimit(int joint_index) {
  if (!limits_enabled) return min_rad[joint_index];
  return max(min_rad[joint_index], joint_min_limit[joint_index]);
}

// Get effective max limit for a joint
float getEffectiveMaxLimit(int joint_index) {
  if (!limits_enabled) return max_rad[joint_index];
  return min(max_rad[joint_index], joint_max_limit[joint_index]);
}

// Print joint limits to serial
void printJointLimits() {
  Serial.println("\n╔═══════════════════════════════════════════════════╗");
  Serial.println("║            JOINT LIMITS CONFIGURATION              ║");
  Serial.println("╚═══════════════════════════════════════════════════╝");
  Serial.print("  Limits Enabled: ");
  Serial.println(limits_enabled ? "YES" : "NO");
  Serial.println();
  
  for (int i = 0; i < SERVO_NUM; i++) {
    Serial.print("  Joint ");
    Serial.print(i + 1);
    Serial.print(": ");
    
    float eff_min = getEffectiveMinLimit(i);
    float eff_max = getEffectiveMaxLimit(i);
    
    Serial.print(eff_min, 3);
    Serial.print(" rad to ");
    Serial.print(eff_max, 3);
    Serial.print(" rad  (");
    Serial.print(eff_min * 180.0 / PI, 1);
    Serial.print("° to ");
    Serial.print(eff_max * 180.0 / PI, 1);
    Serial.println("°)");
  }
  Serial.println();
}

// Set joint limits via serial command
// Format: LIMIT_SET,joint_num,min,max
// Example: LIMIT_SET,1,-2.5,2.5
void setJointLimit(int joint_index, float new_min, float new_max) {
  if (joint_index < 0 || joint_index >= SERVO_NUM) {
    Serial.println("❌ Invalid joint index");
    return;
  }
  
  // Ensure min < max
  if (new_min >= new_max) {
    Serial.println("❌ Min must be less than max");
    return;
  }
  
  // Ensure within hardware limits
  if (new_min < min_rad[joint_index] || new_max > max_rad[joint_index]) {
    Serial.println("⚠️ Warning: Limits exceed hardware limits, clamping...");
    new_min = max(new_min, min_rad[joint_index]);
    new_max = min(new_max, max_rad[joint_index]);
  }
  
  joint_min_limit[joint_index] = new_min;
  joint_max_limit[joint_index] = new_max;
  
  Serial.print("✅ Joint ");
  Serial.print(joint_index + 1);
  Serial.print(" limits set: ");
  Serial.print(new_min, 3);
  Serial.print(" to ");
  Serial.println(new_max, 3);
}

// ========================================

// ESP-NOW callbacks
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  gripperConnected = (status == ESP_NOW_SEND_SUCCESS);
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&incomingMsg, incomingData, sizeof(incomingMsg));

  if (strcmp(incomingMsg.command, "POS") == 0) {
    currentGripperPos = incomingMsg.value;
  } else if (strcmp(incomingMsg.command, "PONG") == 0) {
    gripperConnected = true;
    lastGripperPing = millis();
  }
}

int radToPos(int id, float rad) {
  // Apply joint limits first
  rad = constrainToJointLimits(id, rad);
  
  float total_range = max_rad[id] - min_rad[id];
  if (total_range <= 0.0f) return 2048;
  float normalized = (rad - min_rad[id]) / total_range;
  int pos = (int)round(SERVO_MIN_POS + normalized * (SERVO_MAX_POS - SERVO_MIN_POS));
  return constrain(pos, SERVO_MIN_POS, SERVO_MAX_POS);
}

float posToRad(int id, int pos) {
  pos = constrain(pos, SERVO_MIN_POS, SERVO_MAX_POS);
  float total_range = max_rad[id] - min_rad[id];
  if (total_range <= 0.0f) return home_rad[id];
  float normalized = (pos - SERVO_MIN_POS) / (float)(SERVO_MAX_POS - SERVO_MIN_POS);
  return min_rad[id] + normalized * total_range;
}

void sendGripperCommand(const char* cmd, int val) {
  strcpy(outgoingMsg.command, cmd);
  outgoingMsg.value = val;
  esp_now_send(gripperAddress, (uint8_t *) &outgoingMsg, sizeof(outgoingMsg));
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, 18, 19);
  sms_sts.pSerial = &Serial1;
  
  delay(1000);
  Serial.println("\n╔═══════════════════════════════════╗");
  Serial.println("║   🤖 PEGASUS ARM CONTROLLER v5.2  ║");
  Serial.println("║   WiFi Gripper + Joint Limits     ║");
  Serial.println("╚═══════════════════════════════════╝\n");
  
  // Print joint limits
  printJointLimits();
  
  // Initialize servos
  Serial.println("📡 Initializing servos...");
  delay(100);
  
  bool allServosOK = true;
  for (int i = 0; i < SERVO_NUM; i++) {
    Serial.print("Checking servo ID ");
    Serial.print(IDs[i]);
    Serial.print("... ");
    int pos = sms_sts.ReadPos(IDs[i]);
    if (pos != -1) {
      currentPos[i] = pos;
      targetPos[i] = pos;
      
      // Check if current position is within limits
      float current_rad = posToRad(i, pos);
      if (!isWithinJointLimits(i, current_rad)) {
        Serial.print("  ⚠️ Servo ");
        Serial.print(IDs[i]);
        Serial.print(": ");
        Serial.print(pos);
        Serial.println(" (OUT OF LIMITS!)");
      } else {
        Serial.print("  ✓ Servo ");
        Serial.print(IDs[i]);
        Serial.print(": ");
        Serial.println(pos);
      }
    } else {
      Serial.print("  ✗ Servo ");
      Serial.print(IDs[i]);
      Serial.println(": NO RESPONSE");
      allServosOK = false;
    }
    delay(10);
  }
  
  Serial.println();
  if (allServosOK) {
    Serial.println("✅ All servos connected!");
  } else {
    Serial.println("⚠️ Warning: Some servos not responding");
  }
  
  // Move to home
  Serial.println("🏠 Moving to home position...");
  int homePos[SERVO_NUM];
  for (int i = 0; i < SERVO_NUM; i++) {
    homePos[i] = radToPos(i, home_rad[i]);
    sms_sts.WritePosEx(IDs[i], homePos[i], MOVEMENT_SPEED, MOVEMENT_ACCEL);
    targetPos[i] = homePos[i];
    delay(50);
  }
  delay(1000);
  Serial.println("✅ Homing complete.");
  
  initWiFiAndESPNow();
  delay(1000);
  
  Serial.println("\n┌────────────────────────────────────┐");
  Serial.println("│ WiFi: ESP32_DEV (pass: 12345678)   │");
  Serial.println("│ IP: 192.168.4.1                    │");
  Serial.println("│ Gripper: ESP-NOW Communication     │");
  Serial.println("│ Type 'LIMITS' to view joint limits │");
  Serial.println("└────────────────────────────────────┘\n");
  
  // Ping gripper
  sendGripperCommand("PING", 0);
}

void initWiFiAndESPNow() {
  WiFi.mode(WIFI_AP_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  
  // Initialize ESP-NOW first
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed!");
    return;
  }
  
  // Then start WiFi AP
  WiFi.softAP("ESP32_DEV", "12345678", 1, 1);  // Channel 1 explicitly
  delay(100);
  
  Serial.print("🌐 WiFi Hotspot: ESP32_DEV | IP: ");
  Serial.println(WiFi.softAPIP());
  
  Serial.print("📍 Main MAC Address: ");
  Serial.println(WiFi.softAPmacAddress());
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed!");
    return;
  }
  
  Serial.println("✅ ESP-NOW initialized");
  
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, gripperAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("⚠️ Failed to add gripper peer");
  } else {
    Serial.println("✅ Gripper peer registered");
  }
  
  // Web server routes
  server.on("/", handleRoot);
  server.on("/read_pos", handleReadPos);
  server.on("/set_joint", handleSetJoint);
  server.on("/set_pos", handleSetPos);
  server.on("/scan_ids", handleScanIDs);
  server.on("/home", handleHome);
  server.on("/gripper", handleGripper);
  server.on("/gripper_status", handleGripperStatus);
  server.on("/limits", handleGetLimits);
  
  server.begin();
  Serial.println("📱 Web server started!");
}

void loop() {
  server.handleClient();
  readSerialData();
  
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    lastUpdateTime = currentTime;
    updateMotion();
  }
  
  if (executingTrajectory && isNearTarget()) {
    if (bufferCount > 0) {
      loadNextTrajectoryPoint();
    } else {
      executingTrajectory = false;
    }
  }
  
  if (!executingTrajectory && bufferCount > 0) {
    currentTrajectoryID = incomingTrajectoryID;
    executingTrajectory = true;
    loadNextTrajectoryPoint();
  }
  
  if (currentTime - lastGripperPing > 5000) {
    sendGripperCommand("PING", 0);
    lastGripperPing = currentTime;
  }
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head><title>Pegasus Arm Control</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body { 
  font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; 
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  min-height: 100vh;
  padding: 20px;
  color: #fff;
}
.container { max-width: 700px; margin: auto; }
.header { 
  text-align: center; 
  padding: 30px 20px;
  background: rgba(255,255,255,0.1);
  border-radius: 20px;
  backdrop-filter: blur(10px);
  margin-bottom: 20px;
  box-shadow: 0 8px 32px rgba(0,0,0,0.3);
}
h1 { font-size: 2em; margin-bottom: 10px; text-shadow: 2px 2px 4px rgba(0,0,0,0.3); }
.subtitle { opacity: 0.9; font-size: 0.95em; }
.card { 
  background: rgba(255,255,255,0.95);
  border-radius: 15px;
  padding: 25px;
  margin-bottom: 20px;
  box-shadow: 0 8px 32px rgba(0,0,0,0.2);
  color: #333;
}
.card h2 { 
  color: #667eea; 
  margin-bottom: 20px;
  font-size: 1.4em;
  border-bottom: 2px solid #667eea;
  padding-bottom: 10px;
}
.joint { 
  margin-bottom: 20px;
  padding: 15px;
  background: #f8f9fa;
  border-radius: 10px;
  border-left: 4px solid #667eea;
}
label { 
  display: block;
  font-weight: 600;
  margin-bottom: 8px;
  color: #555;
  font-size: 0.95em;
}
.limit-info {
  font-size: 0.85em;
  color: #888;
  margin-top: 4px;
}
input[type=range] { 
  width: 100%;
  height: 8px;
  border-radius: 5px;
  background: linear-gradient(to right, #667eea, #764ba2);
  outline: none;
  -webkit-appearance: none;
  cursor: pointer;
}
input[type=range]::-webkit-slider-thumb { 
  -webkit-appearance: none;
  width: 24px;
  height: 24px;
  border-radius: 50%;
  background: #fff;
  border: 3px solid #667eea;
  cursor: pointer;
  box-shadow: 0 2px 8px rgba(0,0,0,0.2);
}
.value { 
  display: inline-block;
  min-width: 60px;
  font-size: 1.1em;
  color: #667eea;
  font-weight: bold;
  margin-left: 10px;
  text-align: right;
}
.gripper-control {
  display: flex;
  gap: 15px;
  align-items: center;
  flex-wrap: wrap;
}
.gripper-slider {
  flex: 1;
  min-width: 200px;
}
.status-badge {
  display: inline-block;
  padding: 6px 12px;
  border-radius: 20px;
  font-size: 0.85em;
  font-weight: 600;
  margin-left: 10px;
}
.status-connected { background: #4caf50; color: white; }
.status-disconnected { background: #f44336; color: white; }
.btn-group { 
  display: flex;
  gap: 10px;
  flex-wrap: wrap;
  justify-content: center;
  margin-top: 20px;
}
button { 
  background: linear-gradient(135deg, #667eea, #764ba2);
  color: white;
  border: none;
  padding: 14px 28px;
  border-radius: 25px;
  cursor: pointer;
  font-size: 16px;
  font-weight: 600;
  transition: transform 0.2s, box-shadow 0.2s;
  box-shadow: 0 4px 15px rgba(102, 126, 234, 0.4);
}
button:hover { 
  transform: translateY(-2px);
  box-shadow: 0 6px 20px rgba(102, 126, 234, 0.6);
}
button:active {
  transform: translateY(0);
}
.btn-danger {
  background: linear-gradient(135deg, #f44336, #e91e63);
}
#status { 
  text-align: center;
  padding: 15px;
  background: rgba(255,255,255,0.1);
  border-radius: 10px;
  font-weight: 600;
  margin-top: 20px;
  backdrop-filter: blur(10px);
}
.icon { margin-right: 8px; }
</style>
</head>
<body>
<div class="container">
<div class="header">
<h1>🤖 Pegasus Arm Control</h1>
<p class="subtitle">Advanced 6-DOF Robotic Arm with Joint Limits</p>
</div>

<div class="card">
<h2>🦾 Joint Control</h2>
<div class="joint">
<label for="j1"><span class="icon">🔄</span>Joint 1: Base Rotation</label>
<input type="range" id="j1" min="-3.14" max="3.14" step="0.01" value="0.10" oninput="updateValue('j1', this.value)" onchange="setJoint(1, this.value)">
<span class="value" id="val_j1">0.10</span> rad
<div class="limit-info" id="limit_j1">Loading limits...</div>
</div>
<div class="joint">
<label for="j2"><span class="icon">💪</span>Joint 2: Shoulder (Dual)</label>
<input type="range" id="j2" min="-0.19" max="0.42" step="0.01" value="0.10" oninput="updateValue('j2', this.value)" onchange="setJoint(2, this.value)">
<span class="value" id="val_j2">0.10</span> rad
<div class="limit-info" id="limit_j2">Loading limits...</div>
</div>
<div class="joint">
<label for="j4"><span class="icon">💪</span>Joint 4: Elbow</label>
<input type="range" id="j4" min="-0.67" max="1.08" step="0.01" value="0.64" oninput="updateValue('j4', this.value)" onchange="setJoint(4, this.value)">
<span class="value" id="val_j4">0.64</span> rad
<div class="limit-info" id="limit_j4">Loading limits...</div>
</div>
<div class="joint">
<label for="j5"><span class="icon">🔧</span>Joint 5: Wrist</label>
<input type="range" id="j5" min="0.77" max="2.07" step="0.01" value="1.42" oninput="updateValue('j5', this.value)" onchange="setJoint(5, this.value)">
<span class="value" id="val_j5">1.42</span> rad
<div class="limit-info" id="limit_j5">Loading limits...</div>
</div>
<div class="joint">
<label for="j6"><span class="icon">🔧</span>Joint 6: Wrist Rotation</label>
<input type="range" id="j6" min="-3.14" max="3.14" step="0.01" value="-0.16" oninput="updateValue('j6', this.value)" onchange="setJoint(6, this.value)">
<span class="value" id="val_j6">-0.16</span> rad
<div class="limit-info" id="limit_j6">Loading limits...</div>
</div>
</div>

<div class="card">
<h2>✋ Gripper Control<span id="gripper_status" class="status-badge status-disconnected">Disconnected</span></h2>
<div class="gripper-control">
<div class="gripper-slider">
<label for="gripper"><span class="icon">🤏</span>Position: <span class="value" id="val_gripper">90</span>°</label>
<input type="range" id="gripper" min="0" max="180" step="5" value="90" oninput="updateValue('gripper', this.value)" onchange="setGripper(this.value)">
</div>
<div>
<button onclick="setGripper(0)" class="btn-danger"><span class="icon">🔒</span>Close</button>
<button onclick="setGripper(180)"><span class="icon">🔓</span>Open</button>
</div>
</div>
</div>

<div class="card">
<div class="btn-group">
<button onclick="goHome()"><span class="icon">🏠</span>Home</button>
<button onclick="emergencyStop()" class="btn-danger"><span class="icon">⛔</span>E-Stop</button>
<button onclick="fetchPositions()"><span class="icon">🔄</span>Refresh</button>
<button onclick="scanServos()"><span class="icon">🔍</span>Scan</button>
</div>
</div>

<div id="status">🟢 Ready</div>
</div>

<script>
let isUpdating = false;

function updateValue(id, val) {
  document.getElementById('val_' + id).innerText = parseFloat(val).toFixed(2);
}

function setJoint(jnum, val) {
  if (isUpdating) return;
  fetch(`/set_joint?j${jnum}=${val}`).then(r => r.text()).then(data => {
    updateStatus(data);
  });
}

function setGripper(pos) {
  document.getElementById('gripper').value = pos;
  document.getElementById('val_gripper').innerText = pos;
  fetch(`/gripper?pos=${pos}`).then(r => r.text()).then(data => {
    updateStatus(data);
  });
}

function fetchLimits() {
  fetch('/limits').then(r => r.text()).then(data => {
    let lines = data.split('\n');
    lines.forEach(line => {
      let parts = line.split(':');
      if (parts.length === 2) {
        let jointNum = parseInt(parts[0].replace('j', ''));
        let limits = parts[1].split(',');
        if (limits.length === 2) {
          let min = parseFloat(limits[0]);
          let max = parseFloat(limits[1]);
          
          // Update slider ranges
          let slider = document.getElementById('j' + (jointNum === 2 ? '2' : jointNum));
          if (slider && jointNum !== 3) {
            slider.min = min.toFixed(2);
            slider.max = max.toFixed(2);
          }
          
          // Update limit info display
          let limitInfo = document.getElementById('limit_j' + (jointNum === 2 ? '2' : jointNum));
          if (limitInfo && jointNum !== 3) {
            limitInfo.innerText = `Limits: ${min.toFixed(2)} to ${max.toFixed(2)} rad (${(min*180/Math.PI).toFixed(1)}° to ${(max*180/Math.PI).toFixed(1)}°)`;
          }
        }
      }
    });
  });
}

function fetchPositions() {
  isUpdating = true;
  fetch('/read_pos').then(r => r.text()).then(data => {
    let params = new URLSearchParams(data);
    for (let k = 1; k <= 6; k++) {
      if (k === 3) continue;
      let v = params.get('j' + k);
      if (v !== null) {
        let sliderId = (k === 2) ? 'j2' : 'j' + k;
        let valId = (k === 2) ? 'val_j2' : 'val_j' + k;
        document.getElementById(sliderId).value = v;
        document.getElementById(valId).innerText = parseFloat(v).toFixed(2);
      }
    }
    let moving = params.get('moving') == '1';
    let queue = params.get('queue') || 0;
    let traj = params.get('traj') || '--';
    let statusText = moving ? '🟢 Moving' : '🔵 Idle';
    if (parseInt(queue) > 0) statusText += ` | Queue: ${queue}`;
    if (traj !== '--') statusText += ` | Traj: T${traj}`;
    updateStatus(statusText);
    isUpdating = false;
  });
}

function fetchGripperStatus() {
  fetch('/gripper_status').then(r => r.text()).then(data => {
    let params = new URLSearchParams(data);
    let connected = params.get('connected') === '1';
    let pos = params.get('pos') || 90;
    
    let badge = document.getElementById('gripper_status');
    badge.className = 'status-badge ' + (connected ? 'status-connected' : 'status-disconnected');
    badge.innerText = connected ? 'Connected' : 'Disconnected';
    
    document.getElementById('gripper').value = pos;
    document.getElementById('val_gripper').innerText = pos;
  });
}

function goHome() {
  fetch('/home').then(r => r.text()).then(data => {
    updateStatus(data);
    setTimeout(fetchPositions, 2000);
  });
}

function emergencyStop() {
  if (confirm('⚠️ Emergency Stop: Halt all motion?')) {
    updateStatus('🛑 EMERGENCY STOP ACTIVATED');
    location.reload();
  }
}

function scanServos() {
  fetch('/scan_ids').then(r => r.text()).then(data => {
    alert('📡 Servo Scan Results:\n\n' + data);
  });
}

function updateStatus(text) {
  document.getElementById('status').innerText = text;
}

// Auto-refresh
fetchLimits();
fetchPositions();
fetchGripperStatus();
setInterval(fetchPositions, 1500);
setInterval(fetchGripperStatus, 2000);
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

void handleReadPos() {
  updateMotion();
  String angleStr = "";
  for (int i = 0; i < SERVO_NUM; i++) {
    float rad = posToRad(i, currentPos[i]);
    angleStr += "j" + String(i + 1) + "=" + String(rad, 3) + "&";
  }
  angleStr += "moving=" + String(isMoving ? 1 : 0) + "&";
  angleStr += "queue=" + String(bufferCount) + "&";
  angleStr += "traj=" + String(currentTrajectoryID >= 0 ? currentTrajectoryID : -1);
  server.send(200, "text/plain", angleStr);
}

void handleGetLimits() {
  String limitsStr = "";
  for (int i = 0; i < SERVO_NUM; i++) {
    float eff_min = getEffectiveMinLimit(i);
    float eff_max = getEffectiveMaxLimit(i);
    limitsStr += "j" + String(i + 1) + ":" + String(eff_min, 3) + "," + String(eff_max, 3) + "\n";
  }
  server.send(200, "text/plain", limitsStr);
}

void handleGripper() {
  if (server.hasArg("pos")) {
    int pos = constrain(server.arg("pos").toInt(), 0, 180);
    sendGripperCommand("SET", pos);
    currentGripperPos = pos;
    server.send(200, "text/plain", "Gripper moving to " + String(pos) + "°");
  } else {
    server.send(400, "text/plain", "Missing pos parameter");
  }
}

void handleGripperStatus() {
  String status = "connected=" + String(gripperConnected ? 1 : 0);
  status += "&pos=" + String(currentGripperPos);
  server.send(200, "text/plain", status);
}

void handleSetJoint() {
  bool changed = false;
  for (int k = 1; k <= 6; k++) {
    String param = "j" + String(k);
    if (server.hasArg(param)) {
      float rad = server.arg(param).toFloat();
      
      // Apply joint limits
      rad = constrainToJointLimits(k - 1, rad);
      
      if (k == 2 || k == 3) {
        int pos = radToPos(1, rad);
        targetPos[1] = pos;
        targetPos[2] = pos;
        changed = true;
      } else {
        int i = k - 1;
        int pos = radToPos(i, rad);
        targetPos[i] = pos;
        changed = true;
      }
    }
  }
  if (changed) {
    sendMotionCommands();
    server.send(200, "text/plain", "Joint(s) updated");
  } else {
    server.send(400, "text/plain", "No parameters");
  }
}

void handleSetPos() {
  if (server.hasArg("id") && server.hasArg("pos")) {
    int id = server.arg("id").toInt();
    int pos = constrain(server.arg("pos").toInt(), SERVO_MIN_POS, SERVO_MAX_POS);
    sms_sts.WritePosEx(id, pos, MOVEMENT_SPEED, MOVEMENT_ACCEL);
    server.send(200, "text/plain", "Position set");
  } else {
    server.send(400, "text/plain", "Missing parameters");
  }
}

void handleScanIDs() {
  String scanResult = "Detected IDs: ";
  for (int i = 0; i <= 20; i++) {
    int pos = sms_sts.ReadPos(i);
    if (pos != -1) {
      scanResult += String(i) + " ";
    }
  }
  server.send(200, "text/plain", scanResult);
}

void handleHome() {
  int homePos[SERVO_NUM];
  for (int i = 0; i < SERVO_NUM; i++) {
    homePos[i] = radToPos(i, home_rad[i]);
    sms_sts.WritePosEx(IDs[i], homePos[i], MOVEMENT_SPEED, MOVEMENT_ACCEL);
    targetPos[i] = homePos[i];
  }
  server.send(200, "text/plain", "🏠 Moving to home");
}

void parseJointCommand(String line) {
  line.trim();
  if (!line.startsWith("J")) return;
  
  // Always start from home position
  for (int i = 0; i < SERVO_NUM; i++) {
    currentPos[i] = radToPos(i, home_rad[i]);
  }
  
  String valuesStr = line.substring(1);
  int valueIndex = 0;
  int lastComma = -1;
  
  for (int i = 0; i <= valuesStr.length() && valueIndex < SERVO_NUM; i++) {
    if (i == valuesStr.length() || valuesStr[i] == ',') {
      String valueStr = valuesStr.substring(lastComma + 1, i);
      float radValue = valueStr.toFloat();
      radValue /= 1000.0f;  // Scale from milli-radians
      
      // Apply joint limits
      radValue = constrainToJointLimits(valueIndex, radValue);
      
      targetPos[valueIndex] = radToPos(valueIndex, radValue);
      valueIndex++;
      lastComma = i;
    }
  }
  
  if (valueIndex > 1) targetPos[2] = targetPos[1];
  if (valueIndex == SERVO_NUM) sendMotionCommands();
}

void readSerialData() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        inputBuffer.trim();
        String command = inputBuffer;
        
        if (command == "READ_POS") {
          sendCurrentPositions();
        } else if (command == "LIMITS") {
          printJointLimits();
        } else if (command.startsWith("LIMIT_SET")) {
          // Format: LIMIT_SET,joint_num,min,max
          // Example: LIMIT_SET,1,-2.5,2.5
          int comma1 = command.indexOf(',');
          int comma2 = command.indexOf(',', comma1 + 1);
          int comma3 = command.indexOf(',', comma2 + 1);
          
          if (comma1 > 0 && comma2 > 0 && comma3 > 0) {
            int joint_num = command.substring(comma1 + 1, comma2).toInt();
            float new_min = command.substring(comma2 + 1, comma3).toFloat();
            float new_max = command.substring(comma3 + 1).toFloat();
            setJointLimit(joint_num - 1, new_min, new_max);
          } else {
            Serial.println("❌ Format: LIMIT_SET,joint_num,min,max");
          }
        } else if (command == "LIMITS_ENABLE") {
          limits_enabled = true;
          Serial.println("✅ Joint limits ENABLED");
        } else if (command == "LIMITS_DISABLE") {
          limits_enabled = false;
          Serial.println("⚠️ Joint limits DISABLED");
        } else if (command.startsWith("T")) {
          parseTrajectoryLine(command);
        } else if (command.startsWith("J")) {
          parseJointCommand(command);
        } else if (command.startsWith("G")) {
          // Gripper command from serial: G90 = set to 90 degrees
          int pos = command.substring(1).toInt();
          pos = constrain(pos, 0, 180);
          sendGripperCommand("SET", pos);
        }
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
      if (inputBuffer.length() > 250) inputBuffer = "";
    }
  }
}

void sendCurrentPositions() {
  updateMotion();
  Serial.print("POS:");
  for (int i = 0; i < SERVO_NUM; i++) {
    Serial.print(IDs[i]);
    Serial.print("=");
    Serial.print(currentPos[i]);
    if (i < SERVO_NUM - 1) Serial.print(",");
  }
  Serial.print(",GRIP=");
  Serial.println(currentGripperPos);
}

void parseTrajectoryLine(String line) {
  line.trim();
  if (!line.startsWith("T")) return;
  
  int firstComma = line.indexOf(',');
  if (firstComma == -1) return;
  
  int trajID = line.substring(1, firstComma).toInt();
  incomingTrajectoryID = trajID;
  Serial.print("📥 Received trajectory point for T");
  Serial.println(trajID);
  
  String valuesStr = line.substring(firstComma + 1);
  TrajectoryPoint point;
  point.trajectoryID = trajID;
  point.valid = true;
  
  int valueIndex = 0;
  int lastComma = -1;
  
  for (int i = 0; i <= valuesStr.length() && valueIndex < SERVO_NUM; i++) {
    if (i == valuesStr.length() || valuesStr[i] == ',') {
      String valueStr = valuesStr.substring(lastComma + 1, i);
      float radValue = valueStr.toFloat();
      radValue /= 1000.0f;  // Scale from milli-radians
      
      // Apply joint limits to trajectory points
      radValue = constrainToJointLimits(valueIndex, radValue);
      
      point.positions[valueIndex] = radToPos(valueIndex, radValue);
      valueIndex++;
      lastComma = i;
    }
  }
  
  if (valueIndex > 1) point.positions[2] = point.positions[1];
  if (valueIndex != SERVO_NUM) return;
  
  if (bufferCount < BUFFER_SIZE) {
    trajBuffer[bufferHead] = point;
    bufferHead = (bufferHead + 1) % BUFFER_SIZE;
    bufferCount++;
    pointsReceived++;
  }
}

void loadNextTrajectoryPoint() {
  if (bufferCount == 0) {
    executingTrajectory = false;
    Serial.println("✅ Trajectory complete");
    return;
  }
  
  TrajectoryPoint point = trajBuffer[bufferTail];
  bufferTail = (bufferTail + 1) % BUFFER_SIZE;
  bufferCount--;
  
  Serial.print("🔄 Executing point ");
  Serial.print(pointsExecuted);
  Serial.print(" of trajectory T");
  Serial.println(point.trajectoryID);
  
  if (currentTrajectoryID == -1 || point.trajectoryID != currentTrajectoryID) {
    currentTrajectoryID = point.trajectoryID;
  }
  
  pointsExecuted++;
  
  for (int i = 0; i < SERVO_NUM; i++) {
    targetPos[i] = point.positions[i];
  }
  
  sendMotionCommands();
}

void sendMotionCommands() {
  for (int i = 0; i < SERVO_NUM; i++) {
    if (sms_sts.WritePosEx(IDs[i], targetPos[i], MOVEMENT_SPEED, MOVEMENT_ACCEL) != 1) {
      Serial.print("⚠️ Failed to move servo ");
      Serial.println(IDs[i]);
    }
    // Add small delay between commands to prevent bus congestion
    delay(5);
  }
}

void updateMotion() {
  isMoving = false;
  
  for (int i = 0; i < SERVO_NUM; i++) {
    int pos = sms_sts.ReadPos(IDs[i]);
    if (pos != -1) {
      currentPos[i] = pos;
    }
    
    if (abs(targetPos[i] - currentPos[i]) > 10) {
      isMoving = true;
    }
  }
}

bool isNearTarget() {
  const int threshold = 10;
  
  for (int i = 0; i < SERVO_NUM; i++) {
    if (abs(targetPos[i] - currentPos[i]) > threshold) {
      return false;
    }
  }
  return true;
}