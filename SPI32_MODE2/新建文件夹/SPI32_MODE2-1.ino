/*
    AD7606 SPI Demo Sketch
    Connect the SPI Master device to the VSPI on the esp32:
    SCLK = 18, MISO = 19, MOSI = 23, SS = 5
    Connect the TTL to USB serial port with
    com2: RX2 = D16, TX2 = D17.
    set Range +/-10V  when set RANG(D27) to 3.3V
    
    GPIO     ESPMCU   Name  |   AD7606
   ===================================
     5         D5       SS      |   ADSS
     23       D23     MOSI  |   ADRST
     19       D19     MISO  |   DB7
     18       D18     SCK    |   RD
      2        D2       CVA    |  CVA
      4        D2       CVB    |  CVB
     15       D15     BUSY  |   ADBUSY
      0        D0       D0     |   LED_BUILTIN
                GND    GND   |   RANGE
                3.3V    3.3V   |   VIO
                D14    3.3V   |   OS2
                D13    3.3V   |   OS1
                D12    0.0V   |   OS0
                D27    0.0V   |   RANG      

*/
#include <SPI.h>

static const int spiClk = 3700000; // min 1.6 MHz

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;

uint8_t ADSS = 5;
uint8_t ADRST = 23;
uint8_t ADBUSY = 15;
uint8_t CVA = 2;
uint8_t CVB = 2;
uint8_t OS0 = 12;
uint8_t OS1 = 13;
uint8_t OS2 = 14;
uint8_t LEDMODE = 0;
uint8_t RANG = 27;

//setup loop variables
uint32_t loopCtn = 0;    // counter states for loop
int32_t ADC_val[4];
int32_t avg1_val;
int32_t peak1_val;

int32_t DATAFACTOR=3600; // data factor 3000
unsigned long sendRate = 160;    // Send data to pi3 every 0.2s
unsigned long previousMillis = 0;
uint8_t sendState = 0;

void AD7606_StartConv(){
  /* Conv in rising edge，at least 25ns  */
  /*digitalWrite(ADRST, HIGH);
  delayMicroseconds(10);
  digitalWrite(ADRST, LOW);
  delayMicroseconds(10);*/

  digitalWrite(ADSS, HIGH);   
  delayMicroseconds(1);
  digitalWrite(CVA, LOW);
  digitalWrite(CVB, LOW);
  delayMicroseconds(1);
  digitalWrite(CVA, HIGH);
  digitalWrite(CVB, HIGH);
}

void pulse_SS() {
  digitalWrite(ADSS, HIGH);
  delayMicroseconds(1);
  digitalWrite(ADSS, LOW);
  delayMicroseconds(1);
}

//setup ADC
void AD7606_Reset(){
  //set range(RANG = GND) to +/-5V
  //set no OverSample(OS2 OS1 OS0 = 000)
  digitalWrite(RANG, LOW);
  digitalWrite(CVA, HIGH);
  digitalWrite(CVB, HIGH);
  digitalWrite(ADSS, HIGH);
  digitalWrite(OS2, HIGH);
  digitalWrite(OS1, LOW);
  digitalWrite(OS0, LOW);
  
  // AD7606 is high level reset，at least 50ns
  digitalWrite(ADRST, HIGH);
  for (uint8_t i = 0; i < 4; i++)  {ADC_val[i] = 0;}
  avg1_val = 0;
  peak1_val = 0;
  delayMicroseconds(10);
  digitalWrite(ADRST, LOW);
}


void setup() {
  Serial2.begin(38400);

  //set AD7606 pins
  pinMode(CVA,OUTPUT);
  pinMode(CVB,OUTPUT);
  pinMode(ADBUSY,INPUT_PULLUP);
  pinMode(ADSS, OUTPUT);
  pinMode(ADRST, OUTPUT);
  pinMode(RANG,OUTPUT);
  pinMode(OS2,OUTPUT);
  pinMode(OS1,OUTPUT);
  pinMode(OS0,OUTPUT);
  pinMode(LEDMODE, OUTPUT);
  digitalWrite(LEDMODE, HIGH);
  
  //set VSPI pins: SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  vspi = new SPIClass(VSPI);
  vspi->begin();

  AD7606_Reset();
  delay(1200);
  AD7606_StartConv();

  digitalWrite(LEDMODE, LOW);
  Serial.print("\nMaster online, SPI Select:");
  Serial.print(ADSS);
  Serial.print("\n");

}

void loop() {
  while(digitalRead(ADBUSY) != 0);
  vspiCommand();
  AD7606_StartConv();

  // Update average and peak value(ratio 86% to 14%) and status
  avg1_val = avg1_val*96/100 + ADC_val[1]*4/100; 
  if (peak1_val < ADC_val[1])  { peak1_val = ADC_val[1]; }
  if ((peak1_val > ADC_val[1]*2)&(peak1_val>5000))  { digitalWrite(LEDMODE, HIGH); }  
  if (ADC_val[1]<-5000)  { digitalWrite(LEDMODE, LOW); }

  // Send data to pi3 periodically
  loopCtn = (loopCtn+1) % 10000;
  if (millis() - previousMillis > sendRate) {
    if (Serial2.available()) {
        char inChar = (char)Serial2.read();
        if (inChar=='Z'|inChar=='z')  { peak1_val = 0;   digitalWrite(LEDMODE, LOW); }
    } else {   
      if (sendState == 0) {
        Serial2.print("VSPI item-");
        if (loopCtn < 1000) { Serial2.print(" ");}
        Serial2.print(loopCtn/100);
        sendState = 'P';
      } 
      else if (sendState == 'P') {
        Serial2.print("00  peak: ");
        if (peak1_val < 1000) { Serial2.print(" ");}  
        if (peak1_val < 10000) { Serial2.print(" ");}  
        Serial2.print(peak1_val/100);
        Serial2.print("  ");  
        sendState = 'D';
      }
      else if (sendState == 'D') {
        previousMillis = millis();
        Serial2.print(ADC_val[0]/100);
        Serial2.print(", ");
        Serial2.print(avg1_val/100);
        Serial2.print("\n");
        sendState = 0;
      } else  {
        sendState = 0; 
      }
    }
  } else  {
    delayMicroseconds(1); 
    sendState = 0;
  }
}

void vspiCommand() {
  //vspi->transfer(data); // junk data to illustrate usage
  //use it as you would the regular arduino SPI API
  
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE2));
  pulse_SS();
  for (uint8_t i = 0; i < 3; i++) {
    int16_t data_16 = vspi->transfer16(0);
    int32_t data_32 = (int32_t)data_16 * DATAFACTOR/100;  //multipler:3/10 * 100
    ADC_val[i] = data_32;    
  }
  pulse_SS();
  vspi->endTransaction();
}

