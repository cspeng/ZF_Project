/*

  >>> connected graphics-display with 8bit Graphics Library  
  >>> https://github.com/olikraus/u8glib/
  >>> rfdata indicator: flash LED_BUILTIN on command with valid heading(remoteStr)
  >>> rfdata cmd: I0000_, _togg_
  >>> rfdata message: WAKEff, WAKEon, WAKElb
  >>> message stop: termAA, termBB, stopAA, stopBB, stopB1, stopB2, stopB3, stopB4
  >>> message run: gotoAA, gotoBB, gotoB1, gotoB2, gotoB3, gotoB4, _goto_, _hold_

*/

#include "U8glib.h"
#include <EEPROM.h>
U8GLIB_SH1106_128X64 u8g(12, 11, 9, 10);  // SW SPI Com: SCK = 12, MOSI = 11, CS = 9, DC = 10

//loop states
String OLED_msg = "";             // receive incoming OLED msg
String inputString = "";          // a string to hold incoming data
boolean inputMark = false;        // usb buffer status
boolean stringComplete = false;   // whether the string is complete
int ee_started = 4;               // flag address to show valid of memory when set

// RF通信格式 0x17 channel transparent mode
String connStr = "Z.";          // init conn pattern
String remoteStr = "Z.17";      // incoming rf id from AGV
String hiStr = "R.17";          // outgoing rf id to AGV

// RF serial conn assignment
String comdata;                 // remote buffer
boolean mark = false;           // buffer status
byte   M1 = A5;  //模式
byte   M0 = A4;  //模式

// 子程序
void button_states(void);       // get button interface
void get_rfdata(void);          // get remote channel data
boolean set_usrN(void);         // set multi stationN
void set_Lamp(char);            // set running signal
void bell_onAA(char);           // bell when AA arrival
void bell_lbat(char);           // tick sound when low battery

boolean verifyID(String Str) {return (Str==remoteStr);}

// joystick Variables store triggers:
int reading = 0;                // current reading of the button
int buttonState;                // the current reading from the input pin
int lastButtonState = LOW;      // the previous reading from the input pin
int buttonState1;               // the current reading from the input pin
int lastButtonState1 = LOW;     // the previous reading from the input pin
int buttonState2;               // the current reading from the input pin
int lastButtonState2 = LOW;     // the previous reading from the input pin
int buttonState3;               // the current reading from the input pin
int lastButtonState3 = LOW;     // the previous reading from the input pin

boolean I8 = false;                 // indicator on/off state on btn A0
boolean I7 = false;                 // indicator on/off state on btn A1
boolean I6 = false;                 // indicator on/off state on btn A2
boolean I5 = false;                 // indicator on/off state on btn A3

boolean M8 = I8;                    // memory on btn A0
boolean M7 = I7;                    // memory on btn A1
boolean M6 = I6;                    // memory on btn A2
boolean M5 = I5;                    // memory on btn A3

unsigned long debounceDelay = 50;   // the debounce time; increase if the output flickers
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long lastDebounceTime1 = 0; // the last time the output pin was toggled
unsigned long lastDebounceTime2 = 0; // the last time the output pin was toggled
unsigned long lastDebounceTime3 = 0; // the last time the output pin was toggled

boolean A0Press = false;             // init A0 hold func
boolean A1Press = false;             // init A1 hold func
boolean A2Press = false;             // init A2 hold func
boolean A3Press = false;             // init A3 hold func
int A0HoldCounter = 0;               // counter for A0 Hold
int A1HoldCounter = 0;               // counter for A1 Hold
int A2HoldCounter = 0;               // counter for A2 Hold
int A3HoldCounter = 0;               // counter for A3 Hold
int disp_margin = 0;                 // display margin from left

// lamp and buzzer logical sequence 
boolean BeepOn = false;             // beep on/off state to AA arrival
boolean LbatOn = false;             // beep-tick on/off state to low battery
boolean LampOn = false;             // lamp on/off state to run mode with normal flash
char Buzzer_N = 2;                  // buzzer on/off state to load data into buzzer_sequence
unsigned int lamp_sequence = 0;     // right-shift logic to on/off lamp with seq counter
unsigned int buzzer_sequence = 0;   // right-shift logic to on/off buzzer with seq counter
const int AlertPin = 2;             // the number of the red pin
const int LampPin =  3;             // the number of the yellow pin
const int BuzzPin =  4;             // the number of the buzzer pin
const int naPin = 13;               // LED_BUILTIN as na pin

int seq = 0;                //sequence timer
int seq2 = 0;               //lamp and buzzer timer
int seq_flash = 1;          //start seq to flash on switch indicators
boolean gotoHold = true;    //not running
boolean ontheWAY = false;   //on the way after goto until stop
boolean stationAA = false;  // stop at AA

boolean artLoad = false;    // loaded carrier
boolean artRemove = true;   // empty carrier
char stationB = 0;          // where at station B 
char *strN[6]={"A ", "1", "2", "3", "4", "B"};

void draw(int seq, char *head) {

      // show running animation
      if (stationB==0 & stationAA==0)  { u8g.drawStr(disp_margin, 40, " mitgo "); }
      else if (LbatOn) 
      {
          u8g.drawStr(0, 40, " ");
          u8g.drawStr(disp_margin, 40, "low BAT ");
      }
      
      else if (ontheWAY) 
      {
        u8g.drawStr(0, 40, " ");
        if (!gotoHold) {
          if (stationAA==1)  { u8g.drawStr(disp_margin, 40, "to pick "); }
          else{
            u8g.drawStr(disp_margin, 40, " to   B "); 
            u8g.drawStr(disp_margin+96, 40, head);
          }
        } 
        else {
          if (stationAA==1) {u8g.drawStr(disp_margin, 40, "-       ");}
          else {u8g.drawStr(disp_margin, 40, "B       ");}
          u8g.drawStr(disp_margin+20, 40, head);
          if (seq%3<2) {u8g.drawStr(disp_margin+52, 40, "hold"); } 
        }
      
      } else {
        u8g.drawStr(0, 40, " ");
        if (stationAA==1)  {
          if (seq < 9) {u8g.drawStr(disp_margin, 40, "-A pick");}
          else {u8g.drawStr(disp_margin, 40, "-A     ");}          
        }
        else { 
          if (seq<9) {u8g.drawStr(disp_margin, 40, " B   out");}
          else  {u8g.drawStr(disp_margin, 40, " B      ");}
          u8g.drawStr(disp_margin+30, 40, head);
        }           
      }
 
      seq2++;
      if(seq2>79) {seq2 = 0;}
      if (seq2%5==0) {set_Lamp(seq2);}
      if (seq2%16==0) {
        if (LbatOn){bell_lbat(seq2);}
        else {bell_onAA(seq2);}
      }
}

// get rf command from mitgo
void get_rfdata(void)
{
    while(Serial1.available())
   {    
      comdata += char(Serial1.read());
      mark = true;  //标记已接收  
      if (comdata.charAt(0)!='Z') {mark = false;} 
      if (comdata.length()>1)
      {
        if (comdata.substring(0,2)!=connStr) {mark = false;}          
      }
    }
    if(mark)
    {
        if (verifyID(comdata.substring(0,4))) digitalWrite(LED_BUILTIN, HIGH);
        if(comdata.length()>9) 
        {
            mark = false;
            delay(20);
            if (verifyID(comdata.substring(0,4)))
            {
              String readcmd = comdata.substring(4,10);
              if(readcmd=="_PING_") {tele_cmd(hiStr, "I");} 
              digitalWrite(LED_BUILTIN, LOW);
              
              // term states
              if(readcmd=="termAA") {stationAA=1; stationB=0; I5=M5; I6=M6; I7=M7; I8=M8;} 
              else if (readcmd=="termBB") {stationAA=0; stationB=5; I5=M5; I6=M6; I7=M7; I8=M8;} 
              else if (readcmd=="stopAA") {stationB= 0; stationAA=1; set_usrN(); BeepOn=true;}
              else if (readcmd=="stopBB") {stationB= 5; stationAA=0; set_usrN(); BeepOn=true;}
              else if (readcmd=="stopB1") {stationB= 1; stationAA=0;}
              else if (readcmd=="stopB2") {stationB= 2; stationAA=0;}
              else if (readcmd=="stopB3") {stationB= 3; stationAA=0;}
              else if (readcmd=="stopB4") {stationB= 4; stationAA=0;}
 
              // goto states
              else if (readcmd=="gotoAA") {stationAA=1; stationB=0;} 
              else if (readcmd=="gotoBB") {stationAA=0; stationB=5;} 
              else if (readcmd=="gotoB1") {stationAA=0; stationB=1; I8=1;} 
              else if (readcmd=="gotoB2") {stationAA=0; stationB=2; I7=1;} 
              else if (readcmd=="gotoB3") {stationAA=0; stationB=3; I6=1;} 
              else if (readcmd=="gotoB4") {stationAA=0; stationB=4; I5=1;}               
              else if (readcmd=="WAKEon") {LampOn=true; LbatOn=false;}
              else if (readcmd=="WAKEff") {LampOn=false; LbatOn=false;}
              else if (readcmd=="WAKElb") {LampOn=false; LbatOn=true;}

              if (readcmd.substring(0,4)=="term") {gotoHold=1; LampOn=false; ontheWAY=0;}
              else if (readcmd.substring(0,4)=="stop") {gotoHold=1; LampOn=false; seq=0; seq2=0; ontheWAY=0;} 
              else if (readcmd.substring(0,4)=="goto") {gotoHold=0; LampOn=true; ontheWAY=1;}
              else if (readcmd=="_hold_") {gotoHold=1; LampOn=false;}
              else if (readcmd=="_goto_") {gotoHold=0; LampOn=true;}
            }
        }
    }
    else {comdata="";}
}

void set_Lamp(char seq2) {
  if (LampOn) {
    if ((seq2 % 4)==1){digitalWrite(LampPin, LOW);} 
    else {digitalWrite(LampPin, HIGH);}
  }
  else {digitalWrite(LampPin, HIGH);}
}

// bell when AA arrival
void bell_onAA(char seq2){
  //first bell 
  if (BeepOn) {
    if ((seq2 % 4)==1) {digitalWrite(BuzzPin, HIGH);}
    else {digitalWrite(BuzzPin, LOW);}
    if ((seq2 % 8)==7) {BeepOn=false;}
  }
  else {digitalWrite(BuzzPin, LOW);}
}

// bell when low battery
void bell_lbat(char seq2){
  if (LbatOn) {
    if ((seq2 % 80)==1){digitalWrite(BuzzPin, HIGH);} 
    else {digitalWrite(BuzzPin, LOW);}
  }
  else {digitalWrite(BuzzPin, LOW);}
}

/*
void set_Buzzer(char seq2) {
  if (buzzer_sequence!=0){
    if (buzzer_sequence&1) {digitalWrite(BuzzPin, LOW);}
    else {digitalWrite(BuzzPin, HIGH);}
    buzzer_sequence= (unsigned int)buzzer_sequence >> 1;
  } else {
    digitalWrite(BuzzPin, HIGH);
    buzzer_sequence = 0;
    if (Buzzer_N==1) {buzzer_sequence = 0x004c;}
    else if (Buzzer_N==2) {buzzer_sequence = 0x014c;}    
    else if (Buzzer_N==3) {buzzer_sequence = 0x054c;}
    else if (Buzzer_N==4) {buzzer_sequence = 0x294c;}
    //Buzzer_N++;
    //if (Buzzer_N>0x60) {Buzzer_N=0;}
    Buzzer_N=0;
  }
}
*/

void tele_cmd(String hiStr, String dataStr) {
    // send out teleop cmd to RF Serial1
    if (dataStr=="I") {
      dataStr += String(int(I8));
      dataStr += String(int(I7));
      dataStr += String(int(I6));
      dataStr += String(int(I5));
      dataStr += "_";
    }
    String ss1 = hiStr;
    ss1 += dataStr;
    Serial.print(ss1);
    Serial1.print(ss1);                 //head
    // delay on next transmit  
    delay(5);  
}

void get_msg() {
    while (Serial.available()) {
      inputString += char(Serial.read());
      inputMark = true;  //mark on USB input ready
      if (inputString.charAt(0)!=hiStr.charAt(0)) {inputMark = false;} 
      else if (inputString.length()>1) {
        if (inputString.substring(0,2)!=hiStr.substring(0,2)) {inputMark = false;}          
      }
    }
    if(inputMark) {
      if(inputString.length()>9) {
        inputMark = false;
        delay(5);
        if (inputString.substring(0,4)==hiStr) {
          OLED_msg = inputString.substring(4,10);
          stringComplete = true;
        }
      }
    }
    else {inputString="";}  
}


void setup(void) {
  u8g.setFont(u8g_font_gdr25r);
  Serial.begin(9600);

  //配置RFIO
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  digitalWrite(M0, LOW);    //模式0 
  digitalWrite(M1, LOW); 
  Serial1.begin(9600);
  comdata = "";

  if (EEPROM.read(ee_started)==1)
  {
    M5 = EEPROM.read(8);
    M6 = EEPROM.read(7);
    M7 = EEPROM.read(6);
    M8 = EEPROM.read(5);
    I8 = M8;
    I7 = M7;
    I6 = M6;
    I5 = M5;
  }

  // reserve 50 bytes for the inputString:
  inputString.reserve(50);

  // assign default color value
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

  pinMode(AlertPin, OUTPUT);
  pinMode(LampPin, OUTPUT);
  pinMode(BuzzPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);  
  digitalWrite(AlertPin, LOW);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);  
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);  
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);   

  u8g.firstPage(); 
  digitalWrite(8, HIGH);
  digitalWrite(7, HIGH);
  digitalWrite(6, HIGH);
  digitalWrite(5, HIGH);
  while( u8g.nextPage() );
  delay(200);
  digitalWrite(8, LOW);
  digitalWrite(7, LOW);
  digitalWrite(6, LOW);
  digitalWrite(5, LOW);
  delay(400);

  tele_cmd(hiStr, "I");
}

void loop(void) {
  String dataStr = "000000";

  seq++;
  if(seq>9) {seq = 0;}
  u8g.firstPage();  

  do {
      // indicate I8 when pos at B and indicate I5 when pos at A
      // compare the A0 button at P to A3 button at A to its previous state
      button_states();
      get_rfdata();
     
      // show result on display
      draw(seq, strN[stationB]);

      // change display states when new OLED cmd arrives:
      if (stringComplete) {
        // get msg from usb port
        String readcmd = OLED_msg.substring(0,6);

        // u8g.drawStr( 0, 40, "         ");
        if (OLED_msg=="") {}
        else if (readcmd=="_CALL_") {tele_cmd(hiStr, "_CALL_");}
        /*
        else if (readcmd=="WAKEn1") {Buzzer_N=1;}
        else if (readcmd=="WAKEn2") {Buzzer_N=2;}
        else if (readcmd=="WAKEn3") {Buzzer_N=3;}        
        else if (readcmd=="WAKEn4") {Buzzer_N=4;}
        else if (readcmd=="ART_on") {artLoad=true; artRemove=false;}
        else if (readcmd=="ART_ff") {artLoad=false; artRemove=true;}
        else if (readcmd=="ART_ex") {artLoad=false; artRemove=false;}
        */

        // clear the string after parse
        inputString = "";
        stringComplete = false;
      } else get_msg();

    // rebuild the picture after some delay 
    delay(30);
  } while( u8g.nextPage() );

}

//set usrN memory M8,M7,M6,M5
boolean set_usrN(void)
{
  boolean setN =(stationAA==1);
  if (setN) {M5=I5; M6=I6; M7=I7; M8=I8;} 
  if (setN) {
    EEPROM.update(ee_started, 1);
    EEPROM.update(5, (byte)M5);
    EEPROM.update(6, (byte)M6);
    EEPROM.update(7, (byte)M7);
    EEPROM.update(8, (byte)M8);
  }
  
  return (EEPROM.read(ee_started)==1);
}


// station panel to select states of M8/M7/M6/M5 
void button_states(void) {
      boolean park = (stationAA==0 & !ontheWAY);
      boolean picked = (stationAA==1 & !ontheWAY & seq==0);
      
      // I8/M8/A0 button states
      reading = digitalRead(A0);
      if (reading != lastButtonState) {
        if (reading == HIGH)  {A0HoldCounter=1;}
        // reset the debouncing timer
        lastDebounceTime = millis();
      }
      if (reading == LOW)  {A0HoldCounter++;}
      // reserved button for api test
      if ((millis() - lastDebounceTime) > debounceDelay) {
          if (reading==HIGH) {
              boolean parked = (stationB==1 & park);
              if ((I8 & !parked & !picked) |  (seq<seq_flash & parked))  {digitalWrite(8, HIGH);} 
              else  {digitalWrite(8, LOW);}                  
          }
          if (reading != buttonState) {
            if (reading == LOW) {
                A0Press = true;
                digitalWrite(8, HIGH);
                if (A0HoldCounter>26) {
                  A0Press = false;
                  I8 = 0;
                  digitalWrite(8, LOW);
                  tele_cmd(hiStr, "I");
                               
                  A0HoldCounter = 0;
                  buttonState = reading;
                }
             }
             else  {buttonState=reading;}
          }
          if (reading==HIGH & A0Press) {
                //do something after button release
                if (!I8 & !ontheWAY) {I8 = true; tele_cmd(hiStr, "I");}
                else {tele_cmd(hiStr, "_togg_");}    // echo user selection to sensors.py
                A0Press = false;
           }
      }
      lastButtonState = reading;

      // I5/M5/A3 button states
      reading = digitalRead(A3);
      if (reading != lastButtonState3) {
        if (reading == HIGH)  {A3HoldCounter=1;}
        // reset the debouncing timer
        lastDebounceTime3 = millis();
      }
      if (reading == LOW)  {A3HoldCounter++;}
      // reserved button for api test
      if ((millis() - lastDebounceTime3) > debounceDelay) {
          boolean parked = (stationB==4 & park);
          if (reading==HIGH) {
              if ((I5 & !parked & !picked) |  (seq<seq_flash & parked))  {digitalWrite(5, HIGH);} 
              else  {digitalWrite(5, LOW);}                  
          }
          if (reading != buttonState3) {
            if (reading == LOW) {
                A3Press = true;
                digitalWrite(5, HIGH);
                if (A3HoldCounter>26) {
                  A3Press = false;
                  I5 = 0;
                  digitalWrite(5, LOW);
                  tele_cmd(hiStr, "I"); 
                    
                  A3HoldCounter = 0;
                  buttonState3 = reading;
                }
             }
             else  {buttonState3=reading;}
          }
          if (reading==HIGH & A3Press) {
                //do something after button release
                if (!I5 & !ontheWAY) {I5 = true; tele_cmd(hiStr, "I");}
                else {tele_cmd(hiStr, "_togg_");}    // echo user selection to sensors.py
                A3Press = false;
           }
      }
      lastButtonState3 = reading;

      // I7/M7/A1 button states
      reading = digitalRead(A1);
      if (reading != lastButtonState1) {
        if (reading == HIGH)  {A1HoldCounter=1;}
        // reset the debouncing timer
        lastDebounceTime1 = millis();
      }
      if (reading == LOW)  {A1HoldCounter++;}
      // reserved button for api test
      if ((millis() - lastDebounceTime1) > debounceDelay) {
          boolean parked = (stationB==2 & park);
          if (reading==HIGH) {
              if ((I7 & !parked & !picked) |  (seq<seq_flash & parked)) {digitalWrite(7, HIGH);} 
              else  {digitalWrite(7, LOW);}                  
          }
          if (reading != buttonState1) {
            if (reading == LOW) {
                A1Press = true;
                digitalWrite(7, HIGH);
                
                if (A1HoldCounter>26) {
                  A1Press = false;
                  I7 = 0;
                  digitalWrite(7, LOW);
                  tele_cmd(hiStr, "I");   

                  A1HoldCounter = 0;
                  buttonState1 = reading;
                }
             }
             else  {buttonState1=reading;}
          }
          if (reading==HIGH & A1Press) {
                //do something after button release
                if (!I7 & !ontheWAY) {I7 = true; tele_cmd(hiStr, "I");}
                else {tele_cmd(hiStr, "_togg_");}    // echo user selection to sensors.py
                A1Press = false;
           }
      }
      lastButtonState1 = reading;

      // I6/M6/A2 button states
      reading = digitalRead(A2);
      if (reading != lastButtonState2) {
        if (reading == HIGH)  {A2HoldCounter=1;}
        // reset the debouncing timer
        lastDebounceTime2 = millis();
      }
      if (reading == LOW)  {A2HoldCounter++;}
      // reserved button for api test
      if ((millis() - lastDebounceTime2) > debounceDelay) {
          boolean parked = (stationB==3 & park);
          if (reading==HIGH) {
              if ((I6 & !parked & !picked) |  (seq<seq_flash & parked)) {digitalWrite(6, HIGH);} 
              else  {digitalWrite(6, LOW);}                  
          }
          if (reading != buttonState2) {
            if (reading == LOW) {
                A2Press = true;
                digitalWrite(6, HIGH);
                if (A2HoldCounter>26) {
                  A2Press = false;
                  I6 = 0;
                  digitalWrite(6, LOW);
                  tele_cmd(hiStr, "I"); 

                  A2HoldCounter = 0;
                  buttonState2 = reading;
                }
             }
             else  {buttonState2=reading;}
          }
          if (reading==HIGH & A2Press) {
                //do something after button release
                if (!I6 & !ontheWAY) {I6 = true; tele_cmd(hiStr, "I");}
                else {tele_cmd(hiStr, "_togg_");}    // echo user selection to sensors.py
                A2Press = false;
          }
      }
      lastButtonState2 = reading;
      
}
