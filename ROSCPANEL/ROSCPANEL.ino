/*

  >>> connected graphics display (see below).
  Universal 8bit Graphics Library, https://github.com/olikraus/u8glib/
  >>> serial message: _togg_, _stAA_, _stBB_, _CALL_, _usr1_, _usr2_, _usr3_, usr4

*/

#include "U8glib.h"
U8GLIB_SH1106_128X64 u8g(12, 11, 9, 10);  // SW SPI Com: SCK = 12, MOSI = 11, CS = 9, DC = 10

String hiStr = "F5A0";            // floor code
String OLED_msg = "";             // receive incoming OLED msg
String inputString = "";          // a string to hold incoming data
boolean stringComplete = false;   // whether the string is complete

// RF通信格式 target mode // master channel == 75.00
// IP:139 75(117):25(37)
byte CH = 128;                // remote channel = 25.01
byte ADDR = 00;              // master id
String connStr = "Z.";       // init conn pattern
String remoteStr = "Z.01";  // incoming remote id
String masterStr = "Z.00";  // master broadcast id
// 子程序
void button_states(void);       // get button interface
void get_rfdata(void);          // get remote channel data
void rfop_cmd(String dataStr);  // ping station thro rf
void setI_proper(void);         // set proper logic for button indicators 
boolean setN_usr(void);         // set multi stationN
boolean verifyID(String Str) {return (Str==remoteStr) | (Str==masterStr);}
String comdata;                 // remote buffer
boolean mark = false;           // buffer status

// RF serial conn assignment
byte   M1 = A5;  //模式
byte   M0 = A4;  //模式

// joystick Variables store triggers:
int reading = 0;            // current reading of the button
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;  // the previous reading from the input pin
int buttonState1;             // the current reading from the input pin
int lastButtonState1 = LOW;  // the previous reading from the input pin
int buttonState2;             // the current reading from the input pin
int lastButtonState2 = LOW;  // the previous reading from the input pin
int buttonState3;             // the current reading from the input pin
int lastButtonState3 = LOW;  // the previous reading from the input pin

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
int disp_margin = 25;                // display margin from left

// lamp and buzzer logical sequence 
boolean LbatOn = false;             // lamp on/off state to low battery with slow flash
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
boolean gotoHold = true;    //not running
boolean showTry = true;     //initial search
boolean dirBA = true;       //direction from B to A

boolean stationAA = false;  // stop at AA
char stationB = 0;          // where is station B 
char *strN[6]={"P ", "1", "2", "3", "4", "A"};

void draw(char seq, char *head) {
      int score_pose = (int)(stationB) * 27;
      char seq1= seq / 2;

      //show config row
      if(showTry) {
        u8g.drawStr(0, 40, head);        
        u8g.drawStr(disp_margin, 40, "        ");
        if (gotoHold) {draw_site();}  
        else  {draw_arrow(seq1);} 
      }
      else if(gotoHold) {
        u8g.drawStr(0, 40, head);
        if (!showTry) {u8g.drawStr(disp_margin, 40, "        A");}
        else {u8g.drawStr(disp_margin, 40, "          ");}
        draw_site();
      }
      else {
        // show running animation
        if(seq1==5 & !dirBA) {u8g.drawStr(0, 40, " ");}
        else {u8g.drawStr(0, 40, head);}
        
        if(seq1==0){
              if(dirBA) u8g.drawStr(disp_margin, 40, " >     A");
              else u8g.drawStr(disp_margin, 40, "     < A");
        }
        else if(seq1==1){
              if(dirBA) u8g.drawStr(disp_margin, 40, "  >    A");
              else u8g.drawStr(disp_margin, 40, "    <  A");
        }
        else if(seq1==2){
              if(dirBA) u8g.drawStr(disp_margin, 40, "   >   A");
              else u8g.drawStr(disp_margin, 40, "   <   A");
        }
        else if(seq1==3){
              if(dirBA) u8g.drawStr(disp_margin, 40, "    >  A");
              else u8g.drawStr(disp_margin, 40, "  <    A");                
        }
        else if(seq1==4){
              if(dirBA) u8g.drawStr(disp_margin, 40, "     > A");
              else u8g.drawStr(disp_margin, 40, " <     A");
        }
        else if(seq1==5){
              if(dirBA) {u8g.drawStr(disp_margin, 40, "        ");} 
              else {u8g.drawStr(disp_margin, 40, "        A");}     
        }    
     }
     
     //show misc icons
     if (stationAA) {score_pose = 98;} else {score_pose = 0;} 
     
    seq2++;
    if(seq2>79) {seq2 = 0;}
    if (seq2%5==0) {
      set_Lamp(seq2);
      set_Buzzer(seq2);
    }
    if (LbatOn) {
      if ((seq2 % 80)==1){digitalWrite(AlertPin, LOW);} 
      else {digitalWrite(AlertPin, HIGH);}      
    }
    else {digitalWrite(AlertPin, HIGH);} 
}

void set_Lamp(char seq2) {
  if (LampOn) {
    if ((seq2 % 4)==1){digitalWrite(LampPin, LOW);} 
    else {digitalWrite(LampPin, HIGH);}
  }
  else {digitalWrite(LampPin, HIGH);}
}

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

void draw_arrow(char seq1) {
  if ((seq1 % 3)==0) {
    if(dirBA) {u8g.drawStr(30, 40,">>");}
    else {u8g.drawStr(60, 40,"<<");}
  } else {
    if(dirBA) {u8g.drawStr(50, 40,">>");}
    else {u8g.drawStr(40, 40,"<<");}     
  }
}

void draw_site() {
  if(dirBA) {u8g.drawStr( 50, 40,">.");}
  else {u8g.drawStr( 50, 40,".<");}
}

void tele_cmd(String hiStr, String dataStr) {
    // send out teleop cmd toggle;
    Serial.write(2);                    //sof
    Serial.print(hiStr);                //head
    Serial.print(dataStr);              //data
    Serial.write(3);                    //eof 
    // delay on next transmit  
    delay(5);       
}

void get_msg() {
    while (Serial.available()) {
      // get the new byte:
      char inChar = (char)Serial.read();
      if (inChar == 2) {
        inputString = "";
      }
      else {
        // add it to the inputString:
        inputString += inChar;
        // if the incoming character is a eof, set a flag
        if (inChar == 3) {
          if (inputString.substring(0,4)==hiStr) {
            OLED_msg = inputString.substring(4,10);
            stringComplete = true;
          }
        delay(5);  
        }
      }
    }  
}


void setup(void) {
  u8g.setFont(u8g_font_gdr25r);
  Serial.begin(9600);

  //配置RFIO口
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  digitalWrite(M0, LOW);    //模式0 
  digitalWrite(M1, LOW); 
  Serial1.begin(9600);
  comdata = "";

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
  digitalWrite(LED_BUILTIN, HIGH); 
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);  
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);  
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);   
}

void loop(void) {
  String dataStr = "000000";

  seq++;
  if(seq>10) {seq = 0;}
  u8g.firstPage();  

  do {
      // indicate I8 when pos at B and indicate I5 when pos at A
      // compare the A0 button at P to A3 button at A to its previous state
      setI_proper();
      button_states();
      get_rfdata();
     
      // show result on display
      draw(seq, strN[stationB]);

      // change display states when new OLED cmd arrives:
      if (stringComplete) {
        // echo cmd to users for verification
        u8g.drawStr( 0, 40, "         ");
        tele_cmd(hiStr, OLED_msg);

        //change states with respect to cmd
        String readcmd = OLED_msg.substring(0,6);
        if (OLED_msg=="") {}
        // goto states
        else if (readcmd=="gotoA1") {dirBA=0; stationB= 1; I5=1;}
        else if (readcmd=="gotoA2") {dirBA=0; stationB= 2; I6=1;}
        else if (readcmd=="gotoA3") {dirBA=0; stationB= 3; I7=1;}
        else if (readcmd=="gotoA4") {dirBA=0; stationB= 4; I8=1;}
        else if (readcmd=="goto1A") {dirBA=1; stationB= 1;}
        else if (readcmd=="goto2A") {dirBA=1; stationB= 2;}
        else if (readcmd=="goto3A") {dirBA=1; stationB= 3;}        
        else if (readcmd=="goto4A") {dirBA=1; stationB= 4;}
        else if (readcmd=="gotoA0") {dirBA=0; stationB= 0;}
        else if (readcmd=="goto0_") {dirBA=1; stationB= 0; showTry=1;}
        else if (readcmd=="goto_0") {dirBA=0; stationB= 0; showTry=1;} 

        // term states
        else if (readcmd=="stop00") {dirBA=1; stationB= 0; stationAA=0; showTry=1;}
        else if (readcmd=="stop11") {stationB= 1; stationAA=0; showTry=0;}
        else if (readcmd=="stop22") {stationB= 2; stationAA=0; showTry=0;}
        else if (readcmd=="stop33") {stationB= 3; stationAA=0; showTry=0;}
        else if (readcmd=="stop44") {stationB= 4; stationAA=0; showTry=0;}
        else if (readcmd=="stopAA") {stationAA= 1;showTry=0; I5=M5; I6=M6; I7=M7; I8=M8;}
        else if (readcmd=="hold0_") {dirBA=1; stationB=0; showTry=1;}
        else if (readcmd=="hold_0") {dirBA=0; stationB=0; showTry=1;}        
        else if (readcmd=="holdA_") {dirBA=0; showTry=0;}
        else if (readcmd=="hold_A") {dirBA=1; showTry=0;}
        else if (readcmd=="hold__") {}
        else if (readcmd=="termAA") {dirBA=0; stationB=1; stationAA=1; I5=1; I6=0; I7=0; I8=0;showTry=0;}      
        else if (readcmd=="term11") {dirBA=0; stationB=0; I5=0; I6=0; I7=0; I8=0; showTry=0;}
        else if (readcmd=="WAKEon") {LampOn=true; LbatOn=false;}
        else if (readcmd=="WAKEff") {LampOn=false; LbatOn=false;}
        else if (readcmd=="WAKElb") {LampOn=false; LbatOn=true;}
        else if (readcmd=="WAKEn1") {Buzzer_N=1;}
        else if (readcmd=="WAKEn2") {Buzzer_N=2;}
        else if (readcmd=="WAKEn3") {Buzzer_N=3;}        
        else if (readcmd=="WAKEn4") {Buzzer_N=4;}

        if (readcmd.substring(0,4)=="term") {gotoHold=0; LampOn=false;}
        else if (readcmd.substring(0,4)=="goto") {gotoHold=0; seq=0; LampOn=true;}
        else if (readcmd.substring(0,4)=="stop") {gotoHold=1; LampOn=false;} 
        else if (readcmd.substring(0,4)=="hold") {gotoHold=1; LampOn=false;}
        if (readcmd.substring(0,4)=="goto")
        {
          if (readcmd.substring(4,5)=="A") {showTry= 0;}
          if (readcmd.substring(5,6)=="A") {showTry= 0;}  
        }
        //misc logic stopped at station
        if (readcmd=="stopAA") {rfop_cmd("_PING_");setN_usr();}
        if (readcmd=="stop22") {setN_usr();}
        if (readcmd=="stop33") {setN_usr();}
        if (readcmd=="stop44") {setN_usr();}

        // clear the string:
        inputString = "";
        stringComplete = false;
      } else get_msg();

    // rebuild the picture after some delay 
    delay(30);
  } while( u8g.nextPage() );

}

// setI
void setI_proper(void)
{
  char stationN = stationB;
  if (stationAA) {stationN = 5;}
  if (stationB==0) {stationN = 0;}
  if (stationN<4) {I8 = false;}
  if (stationN<3) {I7 = false;}
  if (stationN<2) {I6 = false;}
  if (stationN==0) {I5 = false;}
}

//set usrN
boolean setN_usr(void)
{
  setI_proper();
  boolean toggle = 0;
  char stationN = stationB;
  if (stationAA) {stationN = 5;}
  if (stationB!=0 and gotoHold)
  {
    if (stationN!=4 & I8) {tele_cmd(hiStr, "_usr4_"); toggle=1;}
    else if (stationN!=3 & I7) {tele_cmd(hiStr, "_usr3_"); toggle=1;}
    else if (stationN!=2 & I6) {tele_cmd(hiStr, "_usr2_"); toggle=1;}          
    else if (stationN!=1 & I5) {tele_cmd(hiStr, "_usr1_"); toggle=1;} 
    if (toggle) {delay(15);}
    if (toggle & stationAA) {M5=I5; M6=I6; M7=I7; M8=I8;} 
  }
  return toggle;
}

//key usrN
void keyN_usr(void)
{
  setN_usr();
  //echo _togg_ to sensors.py
  tele_cmd(hiStr, "_togg_");
  delay(5); 
}

// get rf data
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
              if(readcmd=="_CALL_") {digitalWrite(LED_BUILTIN, LOW);}
              if(readcmd=="_CALL_") {tele_cmd(hiStr, "_CALL_");} 
              //if(readcmd=="_CALL_") {stationB += 1;}  //debug
              //if(stationB>4) {stationB = 0;}          //debug
             }
        }
    }
    else {comdata="";}
}

// send out rfop cmd toggle;
void rfop_cmd(String dataStr) {
    String ss1 = "";    // Modbus Comm String
    ss1 += char(255);   //broadcast string
    ss1 += char(255);
    ss1 += char(CH);    //信道 
    ss1 += masterStr;
    ss1 += dataStr;
    Serial1.print(ss1);                //head
    // delay on next transmit  
    delay(5);  
}

// compare the A0 to A3 button to its previous state
void button_states(void) {
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
              if (I8)  {digitalWrite(8, HIGH);} 
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
                  if (gotoHold)  {tele_cmd(hiStr, "_stAA_");}  //echo term11 from sensors.py  
                  else {tele_cmd(hiStr, "_togg_");}              
                  A0HoldCounter = 0;
                  buttonState = reading;
                }
             }
             else  {buttonState=reading;}
          }
          if (reading==HIGH & A0Press) {
                //do something after button release
                if (stationAA==0 & !I8) {tele_cmd(hiStr, "_togg_");}     
                if (!I8) {I8 = true;}
                else {keyN_usr();}    // echo user selection to sensors.py
                A0Press = false;
           }
      }
      lastButtonState = reading;

      reading = digitalRead(A1);
      if (reading != lastButtonState1) {
        if (reading == HIGH)  {A1HoldCounter=1;}
        // reset the debouncing timer
        lastDebounceTime1 = millis();
      }
      if (reading == LOW)  {A1HoldCounter++;}
      // reserved button for api test
      if ((millis() - lastDebounceTime1) > debounceDelay) {
          if (reading==HIGH) {
              if (I7)  {digitalWrite(7, HIGH);} 
              else  {digitalWrite(7, LOW);}                  
          }
          if (reading != buttonState1) {
            if (reading == LOW) {
                A1Press = true;
                digitalWrite(7, HIGH);
                if (A1HoldCounter>26) {
                  A1Press = false;
                  digitalWrite(7, LOW);
                  // clean all I
                  I8 = 0;
                  I7 = 0;
                  I6 = 0;
                  I5 = 0;
                  A1HoldCounter = 0;
                  buttonState1 = reading;
                }
             }
             else  {buttonState1=reading;}
          }
          if (reading==HIGH & A1Press) {
                //do something after button release
                if (!I7) {I7 = true;}
                else {keyN_usr();}    // echo user selection to sensors.py
                A1Press = false;
           }
      }
      lastButtonState1 = reading;

      reading = digitalRead(A2);
      if (reading != lastButtonState2) {
        if (reading == HIGH)  {A2HoldCounter=1;}
        // reset the debouncing timer
        lastDebounceTime2 = millis();
      }
      if (reading == LOW)  {A2HoldCounter++;}
      // reserved button for api test
      if ((millis() - lastDebounceTime2) > debounceDelay) {
          if (reading==HIGH) {
              if (I6)  {digitalWrite(6, HIGH);} 
              else  {digitalWrite(6, LOW);}                  
          }
          if (reading != buttonState2) {
            if (reading == LOW) {
                A2Press = true;
                digitalWrite(6, HIGH);
                if (A2HoldCounter>26) {
                  A2Press = false;
                  digitalWrite(6, LOW);
                  // set all I
                  I8 = 1;
                  I7 = 1;
                  I6 = 1;
                  I5 = 1;
                  A2HoldCounter = 0;
                  buttonState2 = reading;
                }
             }
             else  {buttonState2=reading;}
          }
          if (reading==HIGH & A2Press) {
                //do something after button release
                if (!I6) {I6 = true;}
                else {keyN_usr();}    // echo user selection to sensors.py
                A2Press = false;
          }
      }
      lastButtonState2 = reading;

      reading = digitalRead(A3);
      if (reading != lastButtonState3) {
        if (reading == HIGH)  {A3HoldCounter=1;}
        // reset the debouncing timer
        lastDebounceTime3 = millis();
      }
      if (reading == LOW)  {A3HoldCounter++;}
      // reserved button for api test
      if ((millis() - lastDebounceTime3) > debounceDelay) {
          if (reading==HIGH) {
              if (I5)  {digitalWrite(5, HIGH);} 
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
                  if (gotoHold)  {tele_cmd(hiStr, "_stBB_");}  //echo term11 from sensors.py 
                  else {tele_cmd(hiStr, "_togg_");}      
                  A3HoldCounter = 0;
                  buttonState3 = reading;
                }
             }
             else  {buttonState3=reading;}
          }
          if (reading==HIGH & A3Press) {
                //do something after button release
                if (stationB==0) {tele_cmd(hiStr, "_togg_");}               
                if (!I5) {I5 = true;}
                else {keyN_usr();}    // echo user selection to sensors.py
                A3Press = false;
           }
      }
      lastButtonState3 = reading;
}
