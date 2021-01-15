//
// created by Clarence / PEMS@2020
// send command from MITGO Pi4 to relay board
// realay board command:  _PING_ RESUME WAKEn1 WAKEn2 WAKEn3 WAKEn4 
// belt conveyor command: _OUTL_  _OUTR_  _INTL_  _INTR_

String hiStr = "MIT+";  //start id code
boolean mark = false;

byte IN1 = 4;     //relay in1 turn on buzzer sound in lamp post(low-trigger)
byte IN2 = 3;     //relay in2 switch on resume button(low-trigger)

byte HOME = 2;    //switch key not connected
byte  AUX = 6;    //AUX status not used
byte BEEP = 9;    //beep sound not used
byte LED = 13;    //indicator on board

int reading = 0;                    // current reading of the button
unsigned long debounceDelay = 50;   // the debounce time; increase if the output flickers

boolean RESUME_ON = false;          // resume button on off state
int seq2 = 0;                       // buzzer timer
char Buzzer_N = 2;                  // buzzer on/off state to load data into buzzer_sequence
unsigned int buzzer_sequence = 0;   // right-shift logic to on/off buzzer with seq counter

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
void relay_cmd(String dataStr);


String dec2String(unsigned int decValue) {
  String decString = String(decValue);
  while (decString.length() < 3) decString = "0" + decString;
  return decString;
}

// switch on resume button and hold button for half sec if RESUME_ON.
void set_Resume(char seq2) {
  if (RESUME_ON) {
    if(seq2<25) {digitalWrite(IN2, LOW);} 
    else {digitalWrite(IN2, HIGH); RESUME_ON = false;}
  }
  else {digitalWrite(IN2, HIGH);}
}

// switch on buzzer with beep sound in n times provided that Buzzer_N not equal zero
void set_Buzzer(char seq2) {
  if (buzzer_sequence!=0){
    if (buzzer_sequence&1) {digitalWrite(IN1, LOW); digitalWrite(LED, HIGH);}
    else {digitalWrite(IN1, HIGH); digitalWrite(LED, LOW);}
    buzzer_sequence= (unsigned int)buzzer_sequence >> 1;
    if (buzzer_sequence==0) {digitalWrite(LED, LOW);}
  } else {
    digitalWrite(IN1, HIGH);
    buzzer_sequence = 0;
    if (Buzzer_N==1) {buzzer_sequence = 0x004c;}
    else if (Buzzer_N==2) {buzzer_sequence = 0x014c;}    
    else if (Buzzer_N==3) {buzzer_sequence = 0x054c;}
    else if (Buzzer_N==4) {buzzer_sequence = 0x294c;}
    Buzzer_N=0;
  }
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
    digitalWrite(IN1, HIGH);  //buzzer IN1 OFF
    digitalWrite(IN2, HIGH);  //relay IN2 OFF

    Serial.begin(9600);
    delay(3);
    while (! digitalRead(AUX));
    comdata = "";
}

void loop() {
    //get serial data and keys
    get_usbdata();
    seq2++;
    if(seq2>79) {seq2 = 0;}
    if (seq2%5==0) {
      set_Resume(seq2);
      set_Buzzer(seq2);
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
             if(readcmd=="_PING_") {RESUME_ON = false; relay_cmd("_PING_"); digitalWrite(LED, LOW);}
             else if (readcmd=="RESUME") {RESUME_ON = true; relay_cmd("RESUME"); digitalWrite(LED, LOW); seq2=0;}  
             else if (readcmd=="WAKEn1") {Buzzer_N=1; relay_cmd("WAKEn1"); digitalWrite(LED, LOW);}
             else if (readcmd=="WAKEn2") {Buzzer_N=2; relay_cmd("WAKEn2"); digitalWrite(LED, LOW);}
             else if (readcmd=="WAKEn3") {Buzzer_N=3; relay_cmd("WAKEn3"); digitalWrite(LED, LOW);}       
             else if (readcmd=="WAKEn4") {Buzzer_N=4; relay_cmd("WAKEn4"); digitalWrite(LED, LOW);}
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
                relay_cmd("RESUME");  
                seq2=0;
                RESUME_ON = true;
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
void relay_cmd(String dataStr) {
  // print out readings to serial;
  Serial.write(2);                //sof
  Serial.print(hiStr);            //head
  Serial.print(dataStr);        //data
  Serial.write(3);                //eof
  // delay on next transmit  
  delay(5);     
}
