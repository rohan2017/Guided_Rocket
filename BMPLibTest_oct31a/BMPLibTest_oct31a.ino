#include <BMP388_DEV.h>  // Include the BMP388_DEV.h library

uint32_t loop_timer;
float temperature, pressure, altitude;  // Create the temperature, pressure and altitude variables
BMP388_DEV bmp388;  // Instantiate (create) a BMP388_DEV object and set-up for I2C operation (address 0x77)

void setup()
{
  Serial.begin(115200);  // Initialise the serial port
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, LOW); // Light onboard status LED
  
  bmp388.begin(SLEEP_MODE, OVERSAMPLING_SKIP, OVERSAMPLING_SKIP, IIR_FILTER_OFF, TIME_STANDBY_5MS); // Initialisation, place the BMP388 into Normal_MODE 
  delay(10);
  bmp388.startNormalConversion();  // Start BMP388 continuous conversion in NORMAL_MODE
  delay(10);
  loop_timer = micros(); // Start the loop timer
}

void loop()
{
  if (bmp388.getMeasurements(temperature, pressure, altitude))  // Check if the measurement is complete
  {
    Serial.print(temperature);  // Display the results    
    Serial.print(F("*C   "));
    Serial.print(pressure);    
    Serial.print(F("hPa   "));
    Serial.print(altitude);
    Serial.println(F("m"));  
  }

  while(micros() - loop_timer < 5000);
  loop_timer = micros(); //Reset the loop timer
}
