//////////////////////////////////////////////////////////////////////////////////////////
// BMP388_DEV - I2C Communications, Default Configuration, Normal Conversion, Interrupt
//////////////////////////////////////////////////////////////////////////////////////////

#include <BMP388_DEV.h>                           // Include the BMP388_DEV.h library

volatile boolean dataReady = false;
float temperature, pressure, altitude;

BMP388_DEV bmp388;                                // Instantiate (create) a BMP388_DEV object and set-up for I2C operation (address 0x77)

void setup() 
{
  Serial.begin(115200);                           // Initialise the serial port
  bmp388.begin();                                 // Default initialisation, place the BMP388 into SLEEP_MODE 
  bmp388.enableInterrupt();                       // Enable the BMP388's interrupt (INT) pin
  attachInterrupt(digitalPinToInterrupt(2), interruptHandler, RISING);   // Set interrupt to call interruptHandler function on D2
  bmp388.setTimeStandby(TIME_STANDBY_1280MS);     // Set the standby time to 1.3 seconds
  bmp388.startNormalConversion();                 // Start BMP388 continuous conversion in NORMAL_MODE  
}

void loop() 
{
  if (dataReady)
  {  
    bmp388.getMeasurements(temperature, pressure, altitude);      // Read the measurements
    Serial.print(temperature);                    // Display the results    
    Serial.print(F("*C   "));
    Serial.print(pressure);    
    Serial.print(F("hPa   "));
    Serial.print(altitude);
    Serial.println(F("m")); 
    dataReady = false;                            // Clear the dataReady flag
  }   
}

void interruptHandler()                           // Interrupt handler function
{
  dataReady = true;                               // Set the dataReady flag
}
