/*
   AutoPID BasicTempControl Example Sketch

   This program reads a dallas temperature probe as input, potentiometer as setpoint, drives an analog output.
   It lights an LED when the temperature has reached the setpoint.
*/
#include <AutoPID.h>
#include <MAX31855.h> // Include MAX31855 Sensor library

//pins
#define LED_PIN 13

#define TEMP_READ_DELAY 800 //can only read digital temp sensor every ~750ms
#define T_preheat 40  // 60
unsigned long sendRate = 500; // Send data to app every 0.5s

//pid settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 1
#define KP 48
#define KI 0.012
#define KD 0

/*******************************************************************************************************************
** Declare all program constants                                                                                  **
*******************************************************************************************************************/
const uint32_t SERIAL_SPEED     = 38400; ///< Set the baud rate for Serial I/O
const uint8_t  SPI_CHIP_SELECT  =    SS; ///< D8 < IO5
const uint8_t  SPI_MISO         =  MISO; ///< D6  < IO19
const uint8_t  SPI_SYSTEM_CLOCK =   SCK; ///< D5  < IO18

unsigned long previousMillis = 0;
double temperature, setPoint, outputVal;

MAX31855_Class MAX31855; ///< Create an instance of MAX31855

//input/output variables passed by reference, so they are updated automatically
AutoPID myPID(&temperature, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

unsigned long lastTempUpdate; //tracks clock time of last temp update

//call repeatedly in loop, only updates after a certain time interval
//returns true if update happened

bool updateTemperature() {
  if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
    int32_t ambientTemperature = MAX31855.readAmbient(); // retrieve MAX31855 die ambient temperature
    int32_t probeTemperature   = MAX31855.readProbe();   // retrieve thermocouple probe temp
    uint8_t faultCode          = MAX31855.fault();       // retrieve any error codes
    if ( faultCode )                                     // Display error code if present
    {
      temperature = sqrt(-1);
      Serial.print("Probe: fault code ");
      Serial.print(faultCode);
      Serial.println(" returned.");
    }
    else
    {
      temperature = (double)probeTemperature / 1000.0;
    }
    lastTempUpdate = millis();
    
    return true;
  }
  
  return false;
}//void updateTemperature

void setup() {
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN,0);

  //Initialize Serial port
  Serial.begin(38400);
  Serial.print("Build: ");
  while (!MAX31855.begin(SPI_CHIP_SELECT))    // Hardware SPI for MAX31855
  {
    Serial.println(F("Unable to start MAX31855. Waiting 3 seconds."));
    delay(3000);
  }

  //if temperature is more than 4 degrees below or above setpoint, OUTPUT will be set to min or max respectively
  myPID.setBangBang(4,4);
  //set PID update interval to 4000ms
  myPID.setTimeStep(4000);

}//void setup


void loop() {
  
  updateTemperature();
  setPoint = T_preheat;
  myPID.run(); //call every loop, updates automatically at certain time interval
  digitalWrite(LED_PIN, myPID.atSetPoint(1)); //light up LED when we're at setpoint + -1 degree
 Serial.println(outputVal); 
  if (millis() - previousMillis > sendRate) {
    previousMillis = millis();
    Serial.print("measured value = ");
    Serial.print(temperature);
    Serial.print(" / ");
    Serial.print("  out ");
    //Serial.println(outputVal);  
  }

}//void loop
