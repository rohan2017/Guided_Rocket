#include <Wire.h>

//Declaring some global variables
int16_t gyro_x, gyro_y, gyro_z;
int16_t acc_x, acc_y, acc_z, acc_total_vector;
int16_t temperature;
int32_t gyro_x_cal, gyro_y_cal, gyro_z_cal;
int32_t acc_x_cal, acc_y_cal, acc_z_cal;
uint32_t loop_timer;
int16_t lcd_loop_counter;
float angle_pitch, angle_roll;
int16_t angle_pitch_buffer, angle_roll_buffer;
uint8_t set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

void setup() {
  
  Wire.begin();
  Serial.begin(115200);  //Use only for debugging
  pinMode(PC13, OUTPUT);
  setup_mpu_6050_registers();
  digitalWrite(PC13, LOW);
  
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){
    if(cal_int % 125 == 0)Serial.print(".");
    read_mpu_6050_data(); 
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;

    acc_x_cal += acc_x;
    acc_y_cal += acc_y;
    acc_z_cal += acc_z;
    delay(3);
    
  }
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
  
  acc_x_cal /= 2000;
  acc_y_cal /= 2000;
  acc_z_cal /= 2000;

  Serial.println(acc_x_cal);
  Serial.println(acc_y_cal);
  Serial.println(acc_z_cal);

  // Start end of calibration LED sequence
  digitalWrite(PC13, HIGH);
  delay(500);
  digitalWrite(PC13, LOW);
  delay(500);
  digitalWrite(PC13, HIGH);
  delay(500);
  digitalWrite(PC13, LOW);

  // Initialize loop timer
  loop_timer = micros();
  
}

void loop(){

  read_mpu_6050_data();  //Read the raw acc and gyro data from the MPU-6050

  //Subtract the calibration offsets from the raw readings
  //gyro_x -= gyro_x_cal;
  //gyro_y -= gyro_y_cal;
  //gyro_z -= gyro_z_cal;
  //acc_x = acc_x - acc_x_cal;
  //acc_y = acc_y - acc_y_cal;
  //acc_z = acc_z - acc_z_cal;
  
  Serial.print(acc_x);
  Serial.print(",");
  Serial.print(acc_y);
  Serial.print(",");
  Serial.print(acc_z);
  Serial.print(",");
  Serial.print(gyro_x);
  Serial.print(",");
  Serial.print(gyro_y);
  Serial.print(",");
  Serial.println(gyro_z);
  
  //Serial.print(angle_roll_output);
  //Serial.print(",");
  //Serial.println(angle_pitch_output);
  
  while(micros() - loop_timer < 2000);  //Wait until the loop_timer reaches 2000us (500Hz) before starting the next loop
  loop_timer = micros();  //Reset the loop timer
  
}

void read_mpu_6050_data(){  //Subroutine for reading the raw gyro and accelerometer data
  
  Wire.beginTransmission(0x68);  //Start communicating with the MPU-6050
  Wire.write(0x3B);  //Send the requested starting register
  Wire.endTransmission();  //End the transmission
  Wire.requestFrom(0x68,14);  //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);  //Wait until all the bytes are received
  //Add the low and high byte to the acc_x variable
  acc_x = Wire.read()<<8|Wire.read();
  //Add the low and high byte to the acc_y variable
  acc_y = Wire.read()<<8|Wire.read();
  //Add the low and high byte to the acc_z variable
  acc_z = Wire.read()<<8|Wire.read();
  //Add the low and high byte to the temperature variable
  temperature = Wire.read()<<8|Wire.read();  
  //Add the low and high byte to the gyro_x variable
  gyro_x = Wire.read()<<8|Wire.read();
  //Add the low and high byte to the gyro_y variable
  gyro_y = Wire.read()<<8|Wire.read();
  //Add the low and high byte to the gyro_z variable
  gyro_z = Wire.read()<<8|Wire.read();

  //2,048,000 = 4096*500
  //acc_x /= 4096;
  //acc_y /= 4096;
  //acc_z /= 4096;
  //32,750 = 65.5*500
  //gyro_x /= 65.5;
  //gyro_y /= 65.5;
  //gyro_z /= 65.5;
  
}

void setup_mpu_6050_registers(){
  
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);  //Start communicating with the MPU-6050
  Wire.write(0x6B);  //Send the requested starting register
  Wire.write(0x00);  //Set the requested starting register
  Wire.endTransmission();  //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);  //Start communicating with the MPU-6050
  Wire.write(0x1C);  //Send the requested starting register
  Wire.write(0x10);  //Set the requested starting register
  Wire.endTransmission();  //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);  //Start communicating with the MPU-6050
  Wire.write(0x1B);  //Send the requested starting register
  Wire.write(0x08);  //Set the requested starting register
  Wire.endTransmission();  //End the transmission
  
}
