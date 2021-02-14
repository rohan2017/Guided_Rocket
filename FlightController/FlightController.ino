#include <BMP388_DEV.h>  // Include the custom BMP388_DEV.h library
#include <Wire.h>  // Include I2C library
#include <Servo.h>  // Include servo library
#include <Kalman.h>  // Include Kalman filter library

// Declaring Altimeter variables
float temperature, pressure, altitude;  
BMP388_DEV bmp388;  // Declare a BMP388_DEV object and set-up for I2C operation (address 0x77

//Declaring IMU variables
int16_t gyro_x, gyro_y, gyro_z; // Raw Gyro readings
int16_t acc_x, acc_y, acc_z; // Raw Acc readings
int16_t temperature_imu; // Raw temperature (USELESS)
int32_t gyro_x_cal, gyro_y_cal, gyro_z_cal; // Calibration offsets for gyro
float gyro_x_rate, gyro_y_rate, gyro_z_rate; // Filtered gyro rates in deg/s
float angle_pitch = 0; // Final Euler angle orientation
float angle_yaw = 0;
float angle_roll = 0;

// Create the Kalman instances
int16_t kalX_sum = 0; // Useless variables for the Kalman filters
int16_t kalY_sum = 0;
int16_t kalZ_sum = 0;
Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;

//Declaring Servo Objects
// Orient the rocket such that the symbol faces you
Servo servo_Yaw_N;
Servo servo_Yaw_S;
Servo servo_Pitch_E;
Servo servo_Pitch_W;

// Feedback Loop Variables
float yaw_Error, pitch_Error, roll_Error;
float yaw_Correction = 0;
float pitch_Correction = 0;
float roll_Correction = 0;
float yaw_LastError = 0;
float pitch_LastError = 0;
float roll_LastError = 0;
float yaw_P, yaw_I, yaw_D;
float pitch_P, pitch_I, pitch_D;
float roll_P, roll_I, roll_D;
float max_I = 8;
float yaw_setpoint = 0;
float pitch_setpoint = 0;
float roll_setpoint = 0;

// Loop variables
uint32_t loop_timer;
int16_t loop_counter;
int8_t flying = false; // Flag if the rocket is flying or not
int8_t standby = false; // Standby flag
//int8_t error_code; // Create error flag variable

//Logging
int16_t log_gyro_x[2000];
int16_t log_gyro_y[2000];
int16_t log_gyro_z[2000];
int16_t log_accel_x[2000];
int16_t log_accel_y[2000];
int16_t log_accel_z[2000];
int16_t log_alt[2000];
int8_t logging = false; // Logging flag
int8_t reporting = false; // Reporting flag
int16_t log_flag = 1;
int32_t log_counter;
int buttonState = 0;


void setup() {
  //Serial.begin(115200); // ONLY FOR DEBUGGING
  
  // Initialize onboard LED
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, LOW); // Light onboard status LED
  pinMode(PA1, INPUT); // Initialize Button
  pinMode(PA0, OUTPUT); // Initialize status LED

  // Initialize the BMP388
  bmp388.begin(SLEEP_MODE, OVERSAMPLING_SKIP, OVERSAMPLING_SKIP, IIR_FILTER_OFF, TIME_STANDBY_5MS); // Initialisation, place the BMP388 into Normal_MODE 
  delay(10);
  bmp388.startNormalConversion();  // Start BMP388 continuous conversion in NORMAL_MODE
  delay(10);

  // Initialize the MPU6050
  setup_mpu_6050_registers(); // Setup IMU I2C registers
  // Calibrate the gyro
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){
    //if(cal_int % 125 == 0)Serial.print(".");
    read_mpu_6050_data(); 
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    delay(3);
  }
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;

  // Initialize Kalman Filters
  kalmanX.setAngle(kalX_sum);
  kalmanY.setAngle(kalY_sum);
  kalmanZ.setAngle(kalZ_sum);

  setup_servos();

  // Signal that setup is over
  digitalWrite(PC13, HIGH);
  delay(500);
  digitalWrite(PC13, LOW);
  delay(500);
  digitalWrite(PC13, HIGH);
  delay(500);
  digitalWrite(PC13, LOW);
  delay(500);
  digitalWrite(PC13, HIGH);// Flash onboard LED
  
  loop_timer = micros(); // Start the loop timer
  log_counter = 0;
  standby = true;
  //logging = true; // ONLY FOR DEBUGGING This would start logging right after setup
}


void loop() {
  
  // Get readings from the Altimeter. This only happens once every few loops due to lower frequency
  if (bmp388.getMeasurements(temperature, pressure, altitude)) { // Check if the measurement is complete
  }  // TODO: add else that triggers error

  // Angle Calculations ----------------------------------------------------------------------------------
  // Get readings from the IMU
  read_mpu_6050_data();
  
  // Subtract gyro calibration offsets
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
  
  //Gyro angle calculations
  //Convert to degrees per second by dividing by 65.5
  gyro_x_rate = (float)gyro_x * 0.015267; // Pitch
  gyro_y_rate = (float)gyro_y * 0.015267; // Roll
  gyro_z_rate = (float)gyro_z * 0.015267; // Yaw

  /*
  kalmanX.update(kalX_sum, gyro_x_rate, 0.002); // sum, rate, dt
  kalmanY.update(kalY_sum, gyro_y_rate, 0.002); // dt = 1/500hz = 0.002
  kalmanZ.update(kalZ_sum, gyro_z_rate, 0.002); 
  kalX_sum = kalmanX.getAngle();
  kalY_sum = kalmanY.getAngle();
  kalZ_sum = kalmanZ.getAngle();
  
  gyro_x_rate = kalmanX.getRate();
  gyro_y_rate = kalmanY.getRate();
  gyro_z_rate = kalmanZ.getRate();*/

  gyro_x_rate = (float)gyro_x_rate * 0.002; // Pitch
  gyro_y_rate = (float)gyro_y_rate * 0.002; // Roll
  gyro_z_rate = (float)gyro_z_rate * 0.002; // Yaw

  angle_pitch += gyro_x_rate;
  angle_yaw += gyro_z_rate;
  angle_roll += gyro_y_rate;
  //If the IMU has rolled transfer the yaw angle to the pitch angle
  //angle_yaw += angle_pitch * sin((float)gyro_y_rate * 0.17453); // Convert to radians, pi/180 = 0.17453 then multiply by 0.002 = 0.0000349
  //angle_pitch -= angle_yaw * sin((float)gyro_y_rate * 0.17453); // BROKEN

  //do_PIDs();
  write_servos();
  
  // Logging Functions ------------------------------------------------------------------------------------
  // Detect a massive acceleration and trigger logging
  if(abs(acc_y) > 15000 && standby){
    flying = true;
    standby = false;
    logging = true;
    digitalWrite(PC13, LOW); // Signal that logging has begun
  }

  // Log data
  if(log_flag == 5){
    log_flag = 0;
    if(log_counter < 2000){
      if(logging){
        log_gyro_x[log_counter] = gyro_x;
        log_gyro_y[log_counter] = gyro_y;
        log_gyro_z[log_counter] = gyro_z;
        log_accel_x[log_counter] = acc_x;
        log_accel_y[log_counter] = acc_y;
        log_accel_z[log_counter] = acc_z;
        log_alt[log_counter] = (int16_t)altitude;
        log_counter++;
      }
    }
  }

  // Check for reporting
  buttonState = digitalRead(PA1);
  if (buttonState == HIGH) {
    if(flying){ // Ensure that this only runs once
      digitalWrite(PA0, HIGH); // turn LED on
      logging = false;
      reporting = true;
      flying = false;
      log_counter = 0;
      Serial.begin(115200);  // Begin Serial telemetry (Only for debugging)
      delay(1000);
      digitalWrite(PA0, LOW); // turn LED off
    }
  } else {
    digitalWrite(PA0, LOW); // turn LED off
  }

  // Report if triggered
  if(reporting && log_counter < 2000){ // If the button has been pressed
    Serial.print(log_gyro_x[log_counter]);
    Serial.print(",");
    Serial.print(log_gyro_y[log_counter]);
    Serial.print(",");
    Serial.println(log_gyro_z[log_counter]);
    Serial.print(log_accel_x[log_counter]);
    Serial.print(",");
    Serial.print(log_accel_y[log_counter]);
    Serial.print(",");
    Serial.println(log_accel_z[log_counter]);
    Serial.println(log_alt[log_counter]);
    Serial.println();
    log_counter++;
  }

  //serial_debug(); //ONLY FOR DEBUGGING

  // Manage loop variables
  while(micros() - loop_timer < 2000);
  loop_timer = micros(); //Reset the loop timer
  log_flag++;
  loop_counter++;
}


// IMU Functions ------------------------------------------------------------------------------------------------
void read_mpu_6050_data(){ //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Send the requested starting register
  Wire.endTransmission(); //End the transmission
  Wire.requestFrom(0x68,14); //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14); //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read(); //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read(); //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read(); //Add the low and high byte to the acc_z variable
  temperature_imu = Wire.read()<<8|Wire.read(); //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read(); //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read(); //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read(); //Add the low and high byte to the gyro_z variable

}

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68); //Start communicating with the MPU-6050
  Wire.write(0x6B); //Send the requested starting register
  Wire.write(0x00); //Set the requested starting register
  Wire.endTransmission(); //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68); //Start communicating with the MPU-6050
  Wire.write(0x1C); //Send the requested starting register
  Wire.write(0x10); //Set the requested starting register
  Wire.endTransmission(); //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68); //Start communicating with the MPU-6050
  Wire.write(0x1B); //Send the requested starting register
  Wire.write(0x08); //Set the requested starting register
  Wire.endTransmission(); //End the transmission
}

// Feedback Loops --------------------------------------------------------------------------------------

void do_PIDs(){

  yaw_Error = yaw_setpoint - angle_yaw;
  yaw_P = yaw_Error * 0.8; // P GAIN
  yaw_I += yaw_Error * 0.02; // I GAIN
  if(yaw_I > max_I){
    yaw_I = max_I;
  }else if(yaw_I < -max_I){
    yaw_I = -max_I;
  }
  yaw_D = (yaw_Error - yaw_LastError) * 0.2; // D GAIN
  yaw_LastError = yaw_Error;

  yaw_Correction = yaw_P + yaw_I + yaw_D;
  
  pitch_Error = pitch_setpoint - angle_pitch;
  pitch_P = pitch_Error * 1.2; // P GAIN
  pitch_I += pitch_Error * 0.02; // I GAIN
  if(pitch_I > max_I){
    pitch_I = max_I;
  }else if(pitch_I < -max_I){
    pitch_I = -max_I;
  }
  pitch_D = (pitch_Error - pitch_LastError) * 0.2; // D GAIN
  pitch_LastError = pitch_Error;

  pitch_Correction = -(pitch_P + pitch_I + pitch_D);

  roll_Error = roll_setpoint - angle_roll;
  roll_P = roll_Error * 1.2; // P GAIN
  roll_I += roll_Error * 0.02; // I GAIN
  if(roll_I > max_I){
    roll_I = max_I;
  }else if(roll_I < -max_I){
    roll_I = -max_I;
  }
  roll_D = (roll_Error - roll_LastError) * 0.2; // D GAIN
  roll_LastError = roll_Error;

  roll_Correction = roll_P + roll_I + roll_D;
  
  roll_Correction = roll_Correction * -0.8;
}

// Servo Functions --------------------------------------------------------------------------------------
void setup_servos(){
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
  delay(1000);
}

void write_servos(){

  if(pitch_Correction > 55){
    pitch_Correction = 55;
  }else if(pitch_Correction < -55){
    pitch_Correction = -55;
  }

  if(yaw_Correction > 55){
    yaw_Correction = 55;
  }else if(yaw_Correction < -55){
    yaw_Correction = -55;
  }

  if(roll_Correction > 12){
    roll_Correction = 12;
  }else if(roll_Correction < -12){
    roll_Correction = -12;
  }
  
  servo_Yaw_N.write(77+pitch_Correction + roll_Correction);
  servo_Yaw_S.write(77-pitch_Correction + roll_Correction);
  servo_Pitch_E.write(77+yaw_Correction + roll_Correction);
  servo_Pitch_W.write(77-yaw_Correction + roll_Correction);
}

// Serial Debugging -----------------------------------------------------------------------------
void serial_debug(){
  // Send altimeter readings
  /*
  Serial.print(temperature);   
  Serial.print(F("*C   "));
  Serial.print(pressure);
  Serial.print(F("hPa   "));
  Serial.print(altitude);
  Serial.println(F("m"));*/
  // Send orientation
  Serial.print(gyro_x_rate);
  Serial.print(",");
  Serial.print(gyro_y_rate);
  Serial.print(",");
  Serial.println(gyro_z_rate);
}
