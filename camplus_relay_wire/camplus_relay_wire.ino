//
// created by Clarence / PEMS@2018
// send command from cam_plus to arrow relay
// _NORM_  _HALT_

String hiStr = "CAM+";  //start id code
boolean mark = false;

byte IN1 = 4;  //relay in1 not used
byte IN2 = 3;  //relay in2 low-trigger to turn-on belt motor

byte HOME = 2;  //switch key not connected
byte  AUX = 6;    //wifi status not used
byte BEEP = 9;    //buzzer not used
byte  LED = 13;  //indicator not used

int reading = 0;  // current reading of the button
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

int homeTwice = LOW;

// push-key triggering
int homeState;                  // the current reading from the input pin
int lastHomeState = LOW;  // the previous reading from the input pin
unsigned long lastHomeDebounceTime = 0;  // the last time the output pin was toggled

// variables
String comdata;
String ss1;  

// 子程序
void get_rfdata();
void home_sensor();
boolean verifyID(String Str);
void cam_cmd(String dataStr);
void all_off();
void b2b();

String dec2String(unsigned int decValue) {
  String decString = String(decValue);
  while (decString.length() < 3) decString = "0" + decString;
  return decString;
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
    digitalWrite(IN2, HIGH);  //relay IN2 OFF
    digitalWrite(IN1, HIGH);  //relay IN1 OFF
    Serial.begin(9600);
    delay(3);
    while (! digitalRead(AUX));
    b2b();
    comdata = "";
}

void loop() {
    //get serial data and keys
    get_rfdata();
    home_sensor();
}

// get rf data
void get_rfdata()
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
             if(readcmd=="_NORM_") {
                 cam_cmd("_NORM_");
                 digitalWrite(IN2, LOW);     
                 digitalWrite(LED, LOW);
             }
             else if(readcmd=="_HALT_")
             {
                 cam_cmd("_HALT_");
                 digitalWrite(IN2, LOW); 
                 digitalWrite(LED, LOW);   
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
                digitalWrite(IN2, LOW);     
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
  Serial.print(dataStr);        //data
  Serial.write(3);                //eof
  // delay on next transmit  
  delay(5);     
}

void all_off()
{
    digitalWrite(BEEP, LOW);
    digitalWrite(LED, LOW);  
}

//init bell
void b2b(){
    all_off();
    delay(800);
    digitalWrite(BEEP, HIGH);
    digitalWrite(LED, HIGH);
    delay(800);
    all_off();
    delay(1000);
    digitalWrite(BEEP, HIGH);
    digitalWrite(LED, HIGH);
    delay(800); 
    all_off();
}  
