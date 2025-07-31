/*
 * Pegasus Arm 6-DOF Robot Controller for ST3215 Servos
 * Compatible with Virtual Arduino Simulator and Python ROS2 Controller
 * 
 * Servo Configuration:
 * - Servo 0 (Base): 360° rotation - Full base rotation
 * - Servo 1 (Shoulder1): 35° range - Upper shoulder joint
 * - Servo 2 (Shoulder2): 35° range - Lower shoulder joint  
 * - Servo 3 (Elbow): 100° range - Elbow joint
 * - Servo 4 (Wrist): 75° range - Wrist pitch/roll
 * - Servo 5 (Gripper): 360° rotation - End effector rotation
 * 
 * Hardware Requirements:
 * - Arduino Uno/Mega/ESP32
 * - 6x ST3215 Servos
 * - External power supply for servos (6-7.4V recommended)
 * 
 * Wiring:
 * - Servo 0 (Base): Pin 3
 * - Servo 1 (Shoulder1): Pin 5  
 * - Servo 2 (Shoulder2): Pin 6
 * - Servo 3 (Elbow): Pin 9
 * - Servo 4 (Wrist): Pin 10
 * - Servo 5 (Gripper): Pin 11
 */

#include <Servo.h>

// ===== CONFIGURATION =====
const int SERVO_COUNT = 6;
const int SERVO_PINS[SERVO_COUNT] = {3, 5, 6, 9, 10, 11};  // PWM pins for servos
const int BAUD_RATE = 9600;
const int COMMAND_TIMEOUT = 10000;  // 10 seconds timeout
const int MOVEMENT_DELAY = 20;      // Delay between servo steps (ms)

// Joint names matching the Python controller
const String JOINT_NAMES[SERVO_COUNT] = {
  "joint1_base",      // Servo 0 - Base rotation
  "joint2_shoulder",  // Servo 1 - Shoulder joint 1
  "joint3_elbow",     // Servo 2 - Shoulder joint 2 (mapped from Python elbow)
  "joint4",           // Servo 3 - Elbow joint (mapped from Python joint4)
  "joint5_wrist",     // Servo 4 - Wrist joint
  "joint6_gripper"    // Servo 5 - Gripper rotation (if exists in Python)
};

// ===== SERVO CONFIGURATION =====
struct ServoConfig {
  int minAngle;         // Minimum servo angle (degrees)
  int maxAngle;         // Maximum servo angle (degrees)
  int homeAngle;        // Home/center position (degrees)
  int currentAngle;     // Current servo position
  float maxRadians;     // Maximum radians from center
  bool isFullRotation;  // True for 360° servos
};

ServoConfig servoConfigs[SERVO_COUNT] = {
  // Servo 0 - Base: 360° rotation (0-360°)
  {0, 359, 180, 180, 3.14159, true},
  
  // Servo 1 - Shoulder1: 35° range (centered at 90°, ±17.5°)
  {73, 108, 90, 90, 0.3054, false},  // 35°/2 = 17.5° = 0.3054 rad
  
  // Servo 2 - Shoulder2: 35° range (centered at 90°, ±17.5°)
  {73, 108, 90, 90, 0.3054, false},
  
  // Servo 3 - Elbow: 100° range (centered at 90°, ±50°)
  {40, 140, 90, 90, 0.8727, false},  // 100°/2 = 50° = 0.8727 rad
  
  // Servo 4 - Wrist: 75° range (centered at 90°, ±37.5°)
  {53, 128, 90, 90, 0.6545, false},  // 75°/2 = 37.5° = 0.6545 rad
  
  // Servo 5 - Gripper: 360° rotation (0-360°)
  {0, 359, 180, 180, 3.14159, true}
};

// ===== GLOBAL VARIABLES =====
Servo servos[SERVO_COUNT];
bool servosInitialized = false;
unsigned long lastCommandTime = 0;
String inputBuffer = "";
bool emergencyStop = false;
bool debugMode = true;

// ===== SETUP =====
void setup() {
  Serial.begin(BAUD_RATE);
  
  // Initialize serial communication
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("=== Pegasus Arm 6-DOF Controller ===");
  Serial.println("Initializing 6-servo configuration...");
  
  // Initialize servos
  initializeServos();
  
  // Move to home position
  moveToHome();
  
  Serial.println("Pegasus Arm Controller Ready");
  Serial.println("Listening for Python ROS2 commands...");
  printServoRanges();
  
  lastCommandTime = millis();
}

// ===== MAIN LOOP =====
void loop() {
  // Check for serial commands
  if (Serial.available()) {
    readSerialCommand();
  }
  
  // Safety timeout check
  checkCommandTimeout();
  
  // Small delay
  delay(5);
}

// ===== SERVO INITIALIZATION =====
void initializeServos() {
  Serial.println("Initializing 6 servos...");
  
  for (int i = 0; i < SERVO_COUNT; i++) {
    servos[i].attach(SERVO_PINS[i]);
    
    // Set to home position initially
    servos[i].write(servoConfigs[i].homeAngle);
    servoConfigs[i].currentAngle = servoConfigs[i].homeAngle;
    
    delay(200);  // Allow servo to reach position
    
    Serial.print("Servo ");
    Serial.print(i);
    Serial.print(" (");
    Serial.print(JOINT_NAMES[i]);
    Serial.print(") on pin ");
    Serial.print(SERVO_PINS[i]);
    Serial.print(" - Range: ");
    Serial.print(servoConfigs[i].minAngle);
    Serial.print("°-");
    Serial.print(servoConfigs[i].maxAngle);
    Serial.print("° Home: ");
    Serial.print(servoConfigs[i].homeAngle);
    Serial.println("°");
  }
  
  servosInitialized = true;
  Serial.println("All 6 servos initialized successfully!");
}

void printServoRanges() {
  Serial.println("\n=== Servo Specifications ===");
  Serial.println("Servo 0 (Base): 360° full rotation");
  Serial.println("Servo 1 (Shoulder1): 35° range");
  Serial.println("Servo 2 (Shoulder2): 35° range");
  Serial.println("Servo 3 (Elbow): 100° range");
  Serial.println("Servo 4 (Wrist): 75° range");
  Serial.println("Servo 5 (Gripper): 360° full rotation");
  Serial.println("=============================\n");
}

// ===== SERIAL COMMAND PROCESSING =====
void readSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else if (c != '\0') {  // Ignore null characters
      inputBuffer += c;
    }
  }
}

void processCommand(String command) {
  command.trim();
  lastCommandTime = millis();
  
  if (debugMode) {
    Serial.print("RX: [");
    Serial.print(command);
    Serial.println("]");
  }
  
  if (command == "PING") {
    Serial.println("PONG");
    return;
  }
  
  if (command == "HOME") {
    moveToHome();
    Serial.println("ACK: Moved to home position");
    return;
  }
  
  if (command == "STOP" || command == "EMERGENCY_STOP") {
    emergencyStop = true;
    Serial.println("ACK: Emergency stop activated");
    return;
  }
  
  if (command == "RESET") {
    emergencyStop = false;
    Serial.println("ACK: Emergency stop reset");
    return;
  }
  
  if (command == "STATUS") {
    printStatus();
    return;
  }
  
  if (command == "DEBUG_ON") {
    debugMode = true;
    Serial.println("ACK: Debug mode enabled");
    return;
  }
  
  if (command == "DEBUG_OFF") {
    debugMode = false;
    Serial.println("ACK: Debug mode disabled");
    return;
  }
  
  // Process joint commands
  if (command.indexOf(':') > 0) {
    parseAndExecuteJointCommand(command);
  } else {
    Serial.println("ERROR: Unknown command");
  }
}

// ===== JOINT COMMAND PARSING =====
void parseAndExecuteJointCommand(String command) {
  if (emergencyStop) {
    Serial.println("ERROR: Emergency stop active");
    return;
  }
  
  if (!servosInitialized) {
    Serial.println("ERROR: Servos not initialized");
    return;
  }
  
  if (debugMode) {
    Serial.println("Processing joints...");
  }
  
  // Parse joint values from command string
  float jointRadians[SERVO_COUNT];
  bool jointUpdated[SERVO_COUNT] = {false};
  int jointsFound = 0;
  
  // Initialize with current positions converted to radians
  for (int i = 0; i < SERVO_COUNT; i++) {
    jointRadians[i] = angleToRadians(servoConfigs[i].currentAngle, i);
  }
  
  // Split command by commas and parse each joint
  int startPos = 0;
  int commaPos = 0;
  
  do {
    commaPos = command.indexOf(',', startPos);
    String jointPair;
    
    if (commaPos == -1) {
      jointPair = command.substring(startPos);
    } else {
      jointPair = command.substring(startPos, commaPos);
    }
    
    jointPair.trim();
    
    // Parse joint name and value
    int colonPos = jointPair.indexOf(':');
    if (colonPos > 0) {
      String jointName = jointPair.substring(0, colonPos);
      String valueStr = jointPair.substring(colonPos + 1);
      
      jointName.trim();
      valueStr.trim();
      
      // Map Python joints to servo indices
      int servoIndex = mapJointNameToServo(jointName);
      
      if (servoIndex >= 0) {
        float radianValue = valueStr.toFloat();
        jointRadians[servoIndex] = radianValue;
        jointUpdated[servoIndex] = true;
        jointsFound++;
        
        if (debugMode) {
          Serial.print("  ");
          Serial.print(jointName);
          Serial.print(" -> Servo");
          Serial.print(servoIndex);
          Serial.print(" = ");
          Serial.print(radianValue, 4);
          Serial.println(" rad");
        }
      } else {
        if (debugMode) {
          Serial.print("  Unknown joint: ");
          Serial.println(jointName);
        }
      }
    }
    
    startPos = commaPos + 1;
  } while (commaPos != -1 && startPos < command.length());
  
  if (jointsFound > 0) {
    // Execute movement
    executeJointMovement(jointRadians, jointUpdated);
    Serial.println("ACK: Joints received");
  } else {
    Serial.println("ERROR: No valid joints found");
  }
}

// ===== JOINT NAME MAPPING =====
int mapJointNameToServo(String jointName) {
  // Map Python ROS2 joint names to servo indices
  if (jointName == "joint1_base") return 0;           // Base rotation
  if (jointName == "joint2_shoulder") return 1;       // Shoulder joint 1
  if (jointName == "joint3_elbow") return 2;          // Shoulder joint 2 (Python calls it elbow)
  if (jointName == "joint4") return 3;                // Actual elbow joint
  if (jointName == "joint5_wrist") return 4;          // Wrist joint
  if (jointName == "joint6_gripper") return 5;        // Gripper rotation (if exists)
  
  // Alternative names
  if (jointName == "base") return 0;
  if (jointName == "shoulder1") return 1;
  if (jointName == "shoulder2") return 2;
  if (jointName == "elbow") return 3;
  if (jointName == "wrist") return 4;
  if (jointName == "gripper") return 5;
  
  return -1; // Unknown joint
}

// ===== MOVEMENT EXECUTION =====
void executeJointMovement(float jointRadians[], bool jointUpdated[]) {
  // Convert radians to servo angles and validate
  int targetAngles[SERVO_COUNT];
  bool hasMovement = false;
  
  for (int i = 0; i < SERVO_COUNT; i++) {
    if (jointUpdated[i]) {
      targetAngles[i] = radiansToAngle(jointRadians[i], i);
      
      // Clamp to servo limits
      targetAngles[i] = constrain(targetAngles[i], 
                                  servoConfigs[i].minAngle, 
                                  servoConfigs[i].maxAngle);
      
      if (targetAngles[i] != servoConfigs[i].currentAngle) {
        hasMovement = true;
      }
    } else {
      targetAngles[i] = servoConfigs[i].currentAngle; // Keep current position
    }
  }
  
  if (hasMovement) {
    // Execute smooth movement
    smoothMoveToAngles(targetAngles);
    
    if (debugMode) {
      printCurrentPositions();
    }
  }
}

// ===== COORDINATE CONVERSION =====
int radiansToAngle(float radians, int servoIndex) {
  ServoConfig& config = servoConfigs[servoIndex];
  
  if (config.isFullRotation) {
    // For 360° servos, map -π to π -> 0° to 360°
    float normalizedRadians = radians;
    while (normalizedRadians < -PI) normalizedRadians += 2 * PI;
    while (normalizedRadians > PI) normalizedRadians -= 2 * PI;
    
    int angle = (int)((normalizedRadians + PI) * 180.0 / PI);
    return constrain(angle, config.minAngle, config.maxAngle);
  } else {
    // For limited range servos, map radians to degrees around home position
    float degrees = radians * 180.0 / PI;
    int angle = config.homeAngle + (int)degrees;
    return constrain(angle, config.minAngle, config.maxAngle);
  }
}

float angleToRadians(int angle, int servoIndex) {
  ServoConfig& config = servoConfigs[servoIndex];
  
  if (config.isFullRotation) {
    // For 360° servos, map 0° to 360° -> -π to π
    return ((float)angle * PI / 180.0) - PI;
  } else {
    // For limited range servos, convert offset from home to radians
    return (float)(angle - config.homeAngle) * PI / 180.0;
  }
}

// ===== SMOOTH MOVEMENT =====
void smoothMoveToAngles(int targetAngles[]) {
  const int MAX_STEPS = 30;
  int steps = 1;
  
  // Calculate maximum angle difference to determine steps
  int maxDiff = 0;
  for (int i = 0; i < SERVO_COUNT; i++) {
    int diff = abs(targetAngles[i] - servoConfigs[i].currentAngle);
    if (diff > maxDiff) {
      maxDiff = diff;
    }
  }
  
  // Calculate steps (more steps for larger movements)
  if (maxDiff > 90) {
    steps = MAX_STEPS;
  } else if (maxDiff > 45) {
    steps = 20;
  } else if (maxDiff > 10) {
    steps = 10;
  } else {
    steps = 5;
  }
  
  if (debugMode && maxDiff > 2) {
    Serial.print("Smooth movement: ");
    Serial.print(maxDiff);
    Serial.print("° max diff, ");
    Serial.print(steps);
    Serial.println(" steps");
  }
  
  // Perform smooth interpolated movement
  for (int step = 1; step <= steps; step++) {
    for (int i = 0; i < SERVO_COUNT; i++) {
      int startAngle = servoConfigs[i].currentAngle;
      int endAngle = targetAngles[i];
      
      // Linear interpolation
      int currentAngle = startAngle + ((endAngle - startAngle) * step) / steps;
      
      servos[i].write(currentAngle);
      servoConfigs[i].currentAngle = currentAngle;
    }
    
    // Check for emergency stop
    if (emergencyStop) {
      Serial.println("Movement stopped by emergency stop");
      return;
    }
    
    delay(MOVEMENT_DELAY);
  }
  
  // Ensure final positions are exact
  for (int i = 0; i < SERVO_COUNT; i++) {
    servos[i].write(targetAngles[i]);
    servoConfigs[i].currentAngle = targetAngles[i];
  }
}

// ===== HOME POSITION =====
void moveToHome() {
  Serial.println("Moving to home position...");
  
  int homeAngles[SERVO_COUNT];
  for (int i = 0; i < SERVO_COUNT; i++) {
    homeAngles[i] = servoConfigs[i].homeAngle;
  }
  
  smoothMoveToAngles(homeAngles);
  Serial.println("Home position reached");
}

// ===== UTILITY FUNCTIONS =====
void checkCommandTimeout() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastCommandTime > COMMAND_TIMEOUT) {
    // Optional: implement timeout behavior
    // For now, just reset the timer
    lastCommandTime = currentTime;
  }
}

void printCurrentPositions() {
  Serial.println("Current positions:");
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.print("  Servo");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(servoConfigs[i].currentAngle);
    Serial.print("° (");
    Serial.print(angleToRadians(servoConfigs[i].currentAngle, i), 3);
    Serial.println(" rad)");
  }
}

void printStatus() {
  Serial.println("=== Pegasus Arm Status ===");
  Serial.print("Emergency Stop: ");
  Serial.println(emergencyStop ? "ACTIVE" : "INACTIVE");
  Serial.print("Servos Initialized: ");
  Serial.println(servosInitialized ? "YES" : "NO");
  Serial.print("Debug Mode: ");
  Serial.println(debugMode ? "ON" : "OFF");
  
  printCurrentPositions();
  Serial.println("=========================");
}