// GRIP.ino
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

Servo gripperServo;

// Robot controller MAC address
uint8_t robot_addr[] = {0x30, 0xC9, 0x22, 0xB1, 0x34, 0x09};

// Gripper settings
#define SERVO_PIN 15        // GPIO15 for servo signal
#define MIN_ANGLE 0         // Fully closed
#define MAX_ANGLE 180       // Fully open
#define HOME_ANGLE 90       // Default/home position

int currentPosition = HOME_ANGLE;
int targetPosition = HOME_ANGLE;

// ESP-NOW message structure (must match main controller)
typedef struct struct_message {
  char command[32];
  int value;
} struct_message;

struct_message incomingMsg;
struct_message outgoingMsg;

// Timing
unsigned long lastPositionUpdate = 0;
const unsigned long UPDATE_INTERVAL = 20;  // 20ms for smooth servo control
unsigned long lastPongSent = 0;

// LED for status indication (built-in LED)
#define LED_PIN 2
bool ledState = false;

// Callback when data is received (ESP32 Arduino Core v3.x)
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&incomingMsg, incomingData, sizeof(incomingMsg));
  
  if (strcmp(incomingMsg.command, "SET") == 0) {
    // Set gripper position
    targetPosition = constrain(incomingMsg.value, MIN_ANGLE, MAX_ANGLE);
    
    Serial.print("📥 Received SET command: ");
    Serial.print(targetPosition);
    Serial.println("°");
    
    // Blink LED to show command received
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    
  } else if (strcmp(incomingMsg.command, "PING") == 0) {
    // Respond to ping with PONG
    sendResponse("PONG", currentPosition);
    Serial.println("📡 PING received, sending PONG");
    
  } else if (strcmp(incomingMsg.command, "OPEN") == 0) {
    // Fully open gripper
    targetPosition = MAX_ANGLE;
    Serial.println("📥 OPEN command");
    
  } else if (strcmp(incomingMsg.command, "CLOSE") == 0) {
    // Fully close gripper
    targetPosition = MIN_ANGLE;
    Serial.println("📥 CLOSE command");
    
  } else if (strcmp(incomingMsg.command, "HOME") == 0) {
    // Return to home position
    targetPosition = HOME_ANGLE;
    Serial.println("📥 HOME command");
  }
}

// Callback when data is sent (ESP32 Arduino Core v3.x)
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("✅ Response sent successfully");
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  } else {
    Serial.println("❌ Response send failed");
  }
}

void sendResponse(const char* cmd, int val) {
  strcpy(outgoingMsg.command, cmd);
  outgoingMsg.value = val;
  esp_now_send(robot_addr, (uint8_t *) &outgoingMsg, sizeof(outgoingMsg));  // Send to robot controller
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔═══════════════════════════════════╗");
  Serial.println("║   ✋ GRIPPER CONTROLLER v2.0      ║");
  Serial.println("║   ESP-NOW WiFi Communication      ║");
  Serial.println("╚═══════════════════════════════════╝\n");
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Blink LED on startup
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
  // Initialize servo
  Serial.println("🔧 Initializing servo...");
  gripperServo.attach(SERVO_PIN);
  gripperServo.write(HOME_ANGLE);
  currentPosition = HOME_ANGLE;
  targetPosition = HOME_ANGLE;
  delay(500);
  Serial.println("✅ Servo initialized at home position (90°)");
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  
  // Initialize ESP-NOW first
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW initialization failed!");
    return;
  }
  
  // Connect briefly to get proper WiFi initialization
  WiFi.begin("ESP32_DEV", "12345678");
  delay(100);
  WiFi.disconnect();
  
  // Print MAC address for reference (optional, since broadcast)
  Serial.print("📍 ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW initialization failed!");
    return;
  }
  
  Serial.println("✅ ESP-NOW initialized");
  
  // Register callbacks
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  
  // Register broadcast peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, robot_addr, 6);
  peerInfo.channel = 1;  // Match explicit channel
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("⚠️  Failed to add peer");
  } else {
    Serial.println("✅ Broadcast peer registered");
  }
  
  Serial.println("\n┌────────────────────────────────────┐");
  Serial.println("│  🟢 Ready for commands!            │");
  Serial.println("│  Waiting for ESP-NOW messages...   │");
  Serial.println("└────────────────────────────────────┘\n");
  
  // Initial status LED blink
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Smooth servo movement
  if (currentTime - lastPositionUpdate >= UPDATE_INTERVAL) {
    lastPositionUpdate = currentTime;
    
    if (currentPosition != targetPosition) {
      // Move gradually towards target (smooth motion)
      int step = 2;  // Degrees per update (adjust for speed)
      
      if (currentPosition < targetPosition) {
        currentPosition += step;
        if (currentPosition > targetPosition) {
          currentPosition = targetPosition;
        }
      } else if (currentPosition > targetPosition) {
        currentPosition -= step;
        if (currentPosition < targetPosition) {
          currentPosition = targetPosition;
        }
      }
      
      gripperServo.write(currentPosition);
      
      // Send position update when target reached
      if (currentPosition == targetPosition) {
        sendResponse("POS", currentPosition);
        Serial.print("✅ Reached target position: ");
        Serial.print(currentPosition);
        Serial.println("°");
      }
    }
  }
  
  // Periodic heartbeat (send position every 3 seconds)
  if (currentTime - lastPongSent > 3000) {
    sendResponse("POS", currentPosition);
    lastPongSent = currentTime;
    
    // Heartbeat LED blink
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
  }
  
  // Check for serial commands (for testing/debugging)
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.startsWith("SET:")) {
      int pos = input.substring(4).toInt();
      targetPosition = constrain(pos, MIN_ANGLE, MAX_ANGLE);
      Serial.print("🖥️  Serial command: Moving to ");
      Serial.print(targetPosition);
      Serial.println("°");
      
    } else if (input == "OPEN") {
      targetPosition = MAX_ANGLE;
      Serial.println("🖥️  Serial command: OPEN");
      
    } else if (input == "CLOSE") {
      targetPosition = MIN_ANGLE;
      Serial.println("🖥️  Serial command: CLOSE");
      
    } else if (input == "HOME") {
      targetPosition = HOME_ANGLE;
      Serial.println("🖥️  Serial command: HOME");
      
    } else if (input == "STATUS") {
      Serial.print("📊 Current Position: ");
      Serial.print(currentPosition);
      Serial.print("° | Target: ");
      Serial.print(targetPosition);
      Serial.println("°");
      Serial.print("📍 MAC Address: ");
      Serial.println(WiFi.macAddress());
      
    } else if (input == "HELP") {
      Serial.println("\n📖 Available Serial Commands:");
      Serial.println("  SET:XXX  - Set position (0-180)");
      Serial.println("  OPEN     - Fully open (180°)");
      Serial.println("  CLOSE    - Fully close (0°)");
      Serial.println("  HOME     - Home position (90°)");
      Serial.println("  STATUS   - Show current status");
      Serial.println("  HELP     - Show this help\n");
    }
  }
}