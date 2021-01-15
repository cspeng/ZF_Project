/*
  Modbus RTU Server POT

  This sketch creates a Modbus RTU Server with a simulated coil.
  The value of the simulated coil is set on the LED

  Circuit:
   - micro board
   - ISO 485 shield
   - Serail1 to 485 TTL input

  created Nov 2018
  by Clarence Li
*/

#include <EEPROM.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include "U8glib.h"
U8GLIB_SH1106_128X64 u8g(12, 11, 9, 10);  // SW SPI Com: SCK = 12, MOSI = 11, CS = 9, DC = 10

const int ledPin = LED_BUILTIN;
int ee_address = 0;
union {
    byte asBytes[4];
    long asLong;
} reg_value;

// 子程序
void button_states(void);                      // get button interface
String count_down(void);                      // get button interface
void u8g_init(void);
int mb_potid();

int potID = 2 ;                                      // modbus address(default=2)
int update_Time = 0;                            // download value of Timer
byte ID_valid = 0;                                 // enable Timer when ID is valid
String ID_Name = "";                            // Name for Pot ID

// button Variables to trigger:
boolean I8 = false;                               // indicator D7 on/off state for btn D8
boolean I8_timeup = false;                   // start timeup beep
boolean btnPress = false;                     // init button press-state
unsigned long debounceDelay = 50;      // the debounce time; increase if the output flickers
unsigned long lastDebounceTime = 0;   // the last time the output pin was toggled
unsigned long lastCountTime = 0;        // the last time the output pin was toggled
int pressHoldCount = 0;                       // counter for holding time of press
int reading = 0;                                   // current reading of the button
int buttonState;                                   // the current reading from the input pin
int lastButtonState = LOW;                   // the previous reading from the input pin


void setup() {
  // configure the LED
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // config Modbus Address with ID 2,3,4,P,Q  thro input A0-A4
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);  
  pinMode(A4, INPUT_PULLUP);  

  //u8g.setFont(u8g_font_gdr25r);
  u8g.setFont(u8g_font_gdr20r);
  u8g_init();

  // start the Modbus RTU server, with (slave) id defalut = 2
  potID = mb_potid();
  if (!ModbusRTUServer.begin(potID, 9600)) {
    Serial.println("Failed to start ModbusPOT on ID:"+String(potID));
    while (1);
  }

  Serial.begin(9600);
  Serial.println("Modbus POT ID:" + String(potID));

  // config output D7, D6, D6 and input D8
  pinMode(8, INPUT_PULLUP);    // button switch
  pinMode(7, OUTPUT);             // button indicator
  pinMode(6, OUTPUT);             // buzzer horn
  pinMode(5, OUTPUT);             // not-used as if +V for horn
  digitalWrite(6, HIGH);             // horn-off
  digitalWrite(5, HIGH);             // +5V

  // configure coils at address 0x00
  ModbusRTUServer.configureCoils(0x00, 7);

  // configure holding registers at address 0x00
  ModbusRTUServer.configureHoldingRegisters(0x00, 7);

  // configure input registers at address 0x00
  ModbusRTUServer.configureInputRegisters(0x00, 7);

  // simple test on ee_address
  byte value = EEPROM.read(0x00);
  ModbusRTUServer.coilWrite(0x00, value);

  // get stored ID_Value
  for (int i = 0; i < 4; i++)  {reg_value.asBytes[i] = EEPROM.read(0x08+i);}
  if (reg_value.asBytes[3] !=0)  {reg_value.asLong = 0;}
  ModbusRTUServer.holdingRegisterWrite(0, reg_value.asLong);

  // get stored update_Time
  for (int i = 0; i < 4; i++)  {reg_value.asBytes[i] = EEPROM.read(0x10+i);}
  if (reg_value.asBytes[3] !=0)  {reg_value.asLong = 40;}
  ModbusRTUServer.holdingRegisterWrite(potID, reg_value.asLong);
}

void loop() {
  u8g.firstPage();

  // test bit on ee_address
  int coilValue = ModbusRTUServer.coilRead(0x00);
  EEPROM.update(ee_address, coilValue);

  // update pot memory
  ID_Name = "";
  ID_valid = ModbusRTUServer.coilRead(0x01);  
  ModbusRTUServer.coilWrite(potID, I8); 

  reg_value.asLong = ModbusRTUServer.holdingRegisterRead(0);
  for (int i = 0; i < 4; i++)  {EEPROM.update(0x08+i, reg_value.asBytes[i]);}
  for (int i = 1; i >= 0; i--)  {ID_Name += char(reg_value.asBytes[i]);}

  reg_value.asLong = ModbusRTUServer.holdingRegisterRead(potID);
  for (int i = 0; i < 4; i++)  {EEPROM.update(0x10+i, reg_value.asBytes[i]);}
  update_Time = reg_value.asLong;
   
  do {
    button_states();

    char heading_str[9];
    String head = "NC ";
    head = ID_Name;
    if (ID_valid==1)  {head += "  pot" + String(potID);}
    head.toCharArray(heading_str, 9);
    
    char remain_str[9];
    String remain_sec = count_down() + " sec";
    remain_sec.toCharArray(remain_str, 9);
    
    u8g.drawStr( 0, 25, heading_str); 
    u8g.drawStr( 10, 55, remain_str); 
    
    // poll for Modbus RTU requests  
    ModbusRTUServer.poll();
  
    if (coilValue) {
      // coil value set, turn LED on
      digitalWrite(ledPin, HIGH);
    } else {
      // coild value clear, turn LED off
      digitalWrite(ledPin, LOW);
    }
  
    // map the holiding register values to the input register values
    for (int i = 0; i < 7; i++) {
      long holdingRegisterValue = ModbusRTUServer.holdingRegisterRead(i);
      ModbusRTUServer.inputRegisterWrite(i, holdingRegisterValue);
    }
  } while( u8g.nextPage() );
  
}

// get pot ID for modbus address
int mb_potid(void) {
    int id = potID;
    if (digitalRead(A0)==LOW) {id=2;}
    else  if (digitalRead(A1)==LOW) {id=3;} 
    else  if (digitalRead(A2)==LOW) {id=4;} 
    else  if (digitalRead(A3)==LOW) {id=5;} 
    else  if (digitalRead(A4)==LOW) {id=6;} 
    return  id;
}

// assign default color value on u8g
void u8g_init(void) {
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
}

// Timer count down steps
String count_down(void) {
      String value;
      char value_t[9];      // default "040"
      sprintf(value_t, "%03d", update_Time);

      int remain_t = (millis() - lastCountTime);    
      if(I8) {
        // timeup after Press
        if (remain_t > update_Time*100) {
          I8 = 0;                 // indicator off when timeup
          I8_timeup = 1;     // start timeup beep
        }
        // start beep during press
        if (remain_t < 150) {digitalWrite(6, LOW); }  // horn-on
        else {digitalWrite(6, HIGH);}                        //horn-off
        sprintf(value_t, "%03d",  remain_t/100);
      }
      else {
        if (I8_timeup) {
          // end beep after timeup
          if  (remain_t < update_Time*100+300) {
            digitalWrite(6, LOW);    // horn-on
          }
          else {
            digitalWrite(6, HIGH);    // horn-off
            I8_timeup = 0;             // end timeup beep
          }
        }
      }
      value = value_t[1];
      value += ".";
      value += value_t[2];
      return value;
}

// Timer start/stop Button D8 and indicator D7
void button_states(void) {
      reading = digitalRead(8);
      if (reading != lastButtonState) {
        if (reading == HIGH)  {pressHoldCount=1;}
        // reset the debouncing timer
        lastDebounceTime = millis();
        if (reading == LOW)  {lastCountTime=lastDebounceTime;}
      }
      if (reading == LOW)  {pressHoldCount++;}
      // reserved button for api test
      if ((millis() - lastDebounceTime) > debounceDelay) {
          if (reading==HIGH) {
              if (I8)  {digitalWrite(7, LOW);} 
              else  {digitalWrite(7, HIGH);}                  
          }
          if (reading != buttonState) {
            if (reading == LOW) {
                btnPress = true;
                digitalWrite(7, LOW);
                if (pressHoldCount>160) {
                  btnPress = false;
                  I8 = 0;
                  I8_timeup = 0;
                  digitalWrite(7, HIGH);
                  pressHoldCount = 0;
                  buttonState = reading;
                }
             }
             else  {buttonState=reading;}
          }
          if (reading==LOW & btnPress) {
                //do something after button release
                if (!I8) {I8 = true;}
                btnPress = false;
           }
      }
      lastButtonState = reading;
}

