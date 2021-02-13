#include<Servo.h>

// Orient the rocket such that the symbol faces you
Servo servo_Yaw_N;
Servo servo_Yaw_S;
Servo servo_Pitch_E;
Servo servo_Pitch_W;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, LOW); // Off
  pinMode(PA10, OUTPUT);
  pinMode(PA9, OUTPUT);
  pinMode(PA8, OUTPUT);
  pinMode(PA7, OUTPUT);
  
  servo_Yaw_N.attach(PA7);
  servo_Yaw_S.attach(PA8);
  servo_Pitch_E.attach(PA9);
  servo_Pitch_W.attach(PA10);
  
  servo_Yaw_N.write(77);
  servo_Yaw_S.write(77);
  servo_Pitch_E.write(77);
  servo_Pitch_W.write(77);
  delay(8000);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(PC13, LOW); //On
  
  servo_Yaw_N.write(77);
  servo_Yaw_S.write(77);
  servo_Pitch_E.write(77);
  servo_Pitch_W.write(77);
  delay(200);
  servo_Yaw_N.write(140);
  servo_Yaw_S.write(140);
  servo_Pitch_E.write(140);
  servo_Pitch_W.write(140);
  delay(200);
  servo_Yaw_N.write(77);
  servo_Yaw_S.write(77);
  servo_Pitch_E.write(77);
  servo_Pitch_W.write(77);
  delay(200);
  servo_Yaw_N.write(14);
  servo_Yaw_S.write(14);
  servo_Pitch_E.write(14);
  servo_Pitch_W.write(14);
  delay(200);
  
  
}
