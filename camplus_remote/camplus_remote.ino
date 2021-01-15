//
// Name:  camplus_remote
// created by:  Clarence / PEMS@2020
// command: _PING_  _NORM_  _HALT_
// usage:   send command from cam_plus to 220V-socket to control conveyor-motor
//

#include <EEPROM.h>

String hiStr = "CAM+";  //start id code
boolean mark = false;
int ee_started = 1;     // flag address to show valid of memory when set

byte IN1 = 4;  //relay in1 signal pulse to turn-off remote 220V-socket(low-trigger)
byte IN2 = 3;  //relay in2 signal pulse to turn-on remote 220V-socket(low-rigger)

byte HOME = 2;    //switch key not connected
byte  AUX = 6;    //wifi status not used
byte BEEP = 9;    //buzzer not used
byte  LED = 13;   //indicator not used

int reading = 0;                     // current reading of the button
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

boolean pulse_GEN = false;           // generate pulse when set
boolean HALT_ON = false;             // on/off state of remote-socket 220V
int seq2 = 0;                        // pulse timer

// push-key triggering
int homeState;                           // the current reading from the input pin
int lastHomeState = LOW;                 // the previous reading from the input pin
int homeTwice = LOW;
unsigned long lastHomeDebounceTime = 0;  // the last time the output pin was toggled

// variables
String comdata;

// 子程序
void get_usbdata();
void home_sensor();
boolean verifyID(String Str);
void cam_cmd(String dataStr);

String dec2String(unsigned int decValue) {
  String decString = String(decValue);
  while (decString.length() < 3) decString = "0" + decString;
  return decString;
}

// switch on-button and hold button for half sec if HALT_ON.
void pulse_ON(char seq2) {
  if (pulse_GEN) {
    if(seq2<25) {digitalWrite(IN2, LOW); digitalWrite(LED, HIGH);} 
    else {digitalWrite(IN2, HIGH); pulse_GEN = false; digitalWrite(LED, LOW);}
  }
  else {digitalWrite(IN2, HIGH);}
}

// switch off-button and hold button for half sec if not HALT_ON.
void pulse_OFF(char seq2) {
  if (pulse_GEN) {
    if(seq2<25) {digitalWrite(IN1, LOW); digitalWrite(LED, HIGH);} 
    else {digitalWrite(IN1, HIGH); pulse_GEN = false; digitalWrite(LED, LOW);}
  }
  else {digitalWrite(IN1, HIGH);}
}

void set_ee(void) {
    EEPROM.update(ee_started, 1);
    EEPROM.update(2, (byte)HALT_ON);
}

void setup() //初始化程序
{  
    //配置IO口
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(HOME, INPUT_PULLUP);   
    pinMode(AUX, INPUT_PULLUP);
    pinMode(BEEP, OUTPUT);
    pinMode(LED, OUTPUT);
    
    //初始化IO
    digitalWrite(IN1,HIGH);  //pulse IN1 OFF
    digitalWrite(IN2, HIGH);  //pulse IN2 OFF

    Serial.begin(9600);
    delay(600);
    while (! digitalRead(AUX));
    pulse_GEN = true; 
    comdata = "";

  if (EEPROM.read(ee_started)==1)
  {
    byte M2 = EEPROM.read(2);
    HALT_ON = (M2!=0);
  }
}

void loop() {
    //get serial data and keys
    get_usbdata();
    
    seq2++;
    if(seq2>79) {seq2 = 0;}
    if (seq2%5==0) {
      if (HALT_ON) {pulse_OFF(seq2);}
      else {pulse_ON(seq2);}
    }

    //home_sensor();
    delay(30);
}

// get usb data and do cmd action
void get_usbdata()
{
    while(Serial.available())
    {    
         comdata += char(Serial.read());
         mark = true;  //标记已接收  
         if (comdata.charAt(0)!=hiStr.charAt(0)) {mark = false;} 
     }
     if(mark)
     {
         if (verifyID(comdata.substring(0,4))) digitalWrite(LED, LOW);
         if(comdata.length()>9) 
         {
             mark = false;
             delay(20);
             digitalWrite(LED, HIGH);
             String readcmd = comdata.substring(4,10);
             if(readcmd=="_PING_") {
               seq2=0; pulse_GEN = true; 
               if (HALT_ON) {cam_cmd("_halt_");}
               else {cam_cmd("_norm_");} 
               digitalWrite(LED, LOW);               
             }
             else if (readcmd=="_NORM_") {
               HALT_ON = false; pulse_GEN = true; cam_cmd("_NORM_"); 
               digitalWrite(LED, LOW); seq2=0; set_ee();
             } 
             else if (readcmd=="_HALT_") {
               HALT_ON = true; pulse_GEN = true; cam_cmd("_HALT_");
               digitalWrite(LED, LOW); seq2=0; set_ee();
             }              
        }
    }
    else {comdata="";}
}

void home_sensor()
{
    // check HOME if signal from HIGH to LOW
    int readmark = digitalRead(LED);
    if (readmark==LOW)
    {
      reading = digitalRead(HOME);
    }

    if (reading != lastHomeState) {
      // reset the debouncing timer
      lastHomeDebounceTime = millis();
    }

    if ((millis() - lastHomeDebounceTime) > 300)  {homeTwice = LOW;}

    if ((millis() - lastHomeDebounceTime) > debounceDelay) 
    {
      // longer than the debounce delay, take reading as the actual current state:
        if (reading != homeState) 
        {
          homeState = reading;

        if (homeState == LOW) {
            //set home position
            digitalWrite(LED, HIGH);
            delay(100);    
            digitalWrite(LED, LOW);

            if (homeTwice == HIGH)
            {
                cam_cmd("_PING_");  
                seq2=0;
                pulse_GEN = true; 
                homeTwice = LOW;
            }
            else {homeTwice = HIGH;}
        }
      }
    }
    lastHomeState = reading;
}

// verify string id
boolean verifyID(String Str)
{
  return (Str==hiStr);
}

// send out cam cmd status;
void cam_cmd(String dataStr) {
  // print out readings to serial;
  Serial.write(2);                //sof
  Serial.print(hiStr);            //head
  Serial.print(dataStr);          //data
  Serial.write(3);                //eof
  // delay on next transmit  
  delay(5);     
}
