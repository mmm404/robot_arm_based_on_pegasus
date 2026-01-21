#include <SCServo.h>

SMS_STS sms_sts;
uint8_t IDs[] = {0, 1, 5, 3, 4, 2};
int servo_num = 6;

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, 18, 19);
  sms_sts.pSerial = &Serial1;
  
  Serial.println("Pegasus Calibration Tool");
  Serial.println("Torque DISABLED - Move robot manually to desired Home position.");
  
  // Disable torque so user can move the robot
  for(int i=0; i<servo_num; i++){
    sms_sts.EnableTorque(IDs[i], 0); 
  }
}

void loop() {
  String output = "HOME_VALUES: {";
  for(int i=0; i<servo_num; i++){
    int pos = sms_sts.ReadPos(IDs[i]);
    output += String(pos);
    if(i < servo_num - 1) output += ", ";
  }
  output += "}";
  
  Serial.println(output);
  Serial.println("Format: {ID0, ID1, ID5, ID3, ID4, ID2}");
  Serial.println("----------------------------------------");
  delay(1000);
}
