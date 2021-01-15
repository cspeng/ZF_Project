/*
    AD7606 SPI Demo Sketch
    Connect the SPI Master device to the VSPI on the esp32:
    SCLK = 18, MISO = 19, MOSI = 23, SS = 5
    
    GPIO     ESPMCU   Name  |   AD7606
   ===================================
     5        D5      SS    |   ADSS
     23       D23     MOSI  |   ADRST
     19       D19     MISO  |   DB7
     18       D18     SCK   |   RD
      2       D2      CVA   |   CVA
      4       D4      CVB   |   CVB
     15       D15     BUSY  |   ADBUSY
      0       D0      D0    |   LED_BUILTIN
              GND     GND   |   RAGE
              3.3V    3.3V  |   VIO
              3.3V    3.3V  |   OS2
              3.3V    3.3V  |   OS1
              0.0v    0.0v  |   OS0

*/
#include <SPI.h>

static const int spiClk = 1000000; // 1.0 MHz

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;

uint8_t ADSS = 5;
uint8_t ADRST = 23;
uint8_t ADBUSY = 15;
uint8_t CVA = 2;
uint8_t CVB = 4;

//setup loop variables
uint16_t loopCtn = 0;    // counter states for loop
int32_t ADC_val[8];

void AD7606_StartConv(){
  /* Conv in rising edge，at least 25ns  */
  /*digitalWrite(ADRST, HIGH);
  delayMicroseconds(10);
  digitalWrite(ADRST, LOW);
  delayMicroseconds(10);*/

  digitalWrite(ADSS, HIGH);   
  delayMicroseconds(10);
  digitalWrite(CVA, LOW);
  digitalWrite(CVB, LOW);
  delayMicroseconds(10);
  digitalWrite(CVA, HIGH);
  digitalWrite(CVB, HIGH);
}

void pulse_SS() {
  digitalWrite(ADSS, HIGH);
  delayMicroseconds(10);
  digitalWrite(ADSS, LOW);
  delayMicroseconds(10);
}

//setup ADC
void AD7606_Reset(){
  //set range(RAGE = GND) to +/-5V
  //set no OverSample(OS2 OS1 OS0 = 000)
  digitalWrite(CVA, HIGH);
  digitalWrite(CVB, HIGH);
  digitalWrite(ADSS, HIGH);
  
  // AD7606 is high level reset，at least 50ns
  digitalWrite(ADRST, HIGH);
  for (uint8_t i = 0; i < 8; i++)  {ADC_val[i] = 0;}
  delayMicroseconds(10);
  digitalWrite(ADRST, LOW);
}


void setup() {
  Serial.begin(38400);

  //set AD7606 pins
  pinMode(CVA,OUTPUT);
  pinMode(CVB,OUTPUT);
  pinMode(ADBUSY,INPUT_PULLUP);
  pinMode(ADSS, OUTPUT);
  pinMode(ADRST, OUTPUT);
  
  //set VSPI pins: SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  vspi = new SPIClass(VSPI);
  vspi->begin();

  AD7606_Reset();
  delay(400);

  Serial.print("\nMaster: online ");
  Serial.print(ADSS);
  Serial.print("\n");
}

void loop() {
  loopCtn = (loopCtn+1) % 3200;

  Serial.print("VSPI poll:");
  AD7606_StartConv();
  Serial.print(digitalRead(ADBUSY));
  while(digitalRead(ADBUSY) != 0);
  vspiCommand();
  Serial.print("\n");
  delayMicroseconds(250);
}

void vspiCommand() {
  //byte data = 0b01010101; // junk data to illustrate usage
  //vspi->transfer(data); 

  //use it as you would the regular arduino SPI API
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE2));
  pulse_SS();
  for (uint8_t i = 0; i < 2; i++) {
    int16_t data = vspi->transfer16(0);
    ADC_val[i] = ADC_val[i]*85/100 + data*15;
    Serial.print(" ");
    Serial.print(ADC_val[i]/100);
  }
  pulse_SS();
  vspi->endTransaction();
}
