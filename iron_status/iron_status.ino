//
// created by Clarence / PEMS@2018
// send IRON status
// _HOME_    _P001_  ...  _P999_    _SERV_    _NORM_
byte  AUX = 6;  //wifi status not used
byte BEEP = 9;   //buzzer
byte  LED = 11;  //maintenance indicator

byte KEY = 10;    //SERV/NORM key on maintenance indication
byte HOME = 3;  //home position sensor low effective
byte METAL = 2; //iron strike sensor low effective

int reading = 0;  // current reading of the button
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

int isMaintenance = LOW;
int homePosition = LOW;
int strikePosition = 0;

int homeState;
int lastHomeState = LOW;
unsigned long lastHomeDebounceTime = 0;

int strikeState;
int lastStrikeState = LOW;
unsigned long lastStrikeDebounceTime = 0;

// push-key Variables store triggers:
int maintenanceState;                  // the current reading from the input pin
int lastmaintenanceState = LOW;  // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled

// variables
String comdata;
String ss1;  

String hiStr = "IRON";  //start id code
boolean mark = false;

// 子程序
void get_rfdata();
void home_sensor();
void strike_sensor();
void debounce_key();
boolean verifyID(String Str);
void iron_cmd(String dataStr);
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
    pinMode(AUX, INPUT_PULLUP);  
    pinMode(METAL, INPUT_PULLUP);  
    pinMode(HOME, INPUT_PULLUP);  
    pinMode(KEY, INPUT_PULLUP); 

    pinMode(LED, OUTPUT);
    pinMode(BEEP, OUTPUT);
    
    //初始化IO
    digitalWrite(LED, HIGH);  //LED OFF
    
    Serial.begin(9600);
    delay(3);
    while (! digitalRead(AUX));
    b2b();
    comdata = "";
}

void loop() {
    //get key state and rf data
    get_rfdata();
    debounce_key();
    strike_sensor();
    home_sensor();
}

// get rf data
void get_rfdata()
{
    while(Serial.available())
    {    
         comdata += char(Serial.read());
         mark = true;  //标记已接收  
         if (comdata.charAt(0)!='I') {mark = false;} 
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
                 isMaintenance = LOW; 
                         
                 if (homePosition==HIGH) {
                     delay(200);
                     digitalWrite(BEEP, HIGH);
                     delay(400);
                     digitalWrite(BEEP, LOW);
                     //sync master and slave
                     strikePosition = 0;
                     iron_cmd("_HOME_");                        
                 }
                 else {
                     iron_cmd("_PING_");
                 }
             }
        }
    }
    else {comdata="";}
}

void home_sensor()
{
    // check HOME if signal from HIGH to LOW
    reading = digitalRead(HOME);

    if (reading != lastHomeState) {
      // reset the debouncing timer
      lastHomeDebounceTime = millis();
    }

    if ((millis() - lastHomeDebounceTime) > debounceDelay) 
    {
      // longer than the debounce delay, take reading as the actual current state:
        if (reading != homeState) 
        {
          homeState = reading;

        if (homeState == LOW) {
            //set home position
            homePosition = HIGH;
            digitalWrite(BEEP, HIGH);
            delay(400);    
            digitalWrite(BEEP, LOW);
            strikePosition = 0;
            iron_cmd("_HOME_");  
        }
        else {homePosition = LOW;}
      }
    }
    lastHomeState = reading;
}

void strike_sensor()
{
    // check strike when arm move down from top
    reading = digitalRead(METAL);

    if (reading != lastStrikeState) {
      // reset the debouncing timer
      lastStrikeDebounceTime = millis();
    }

    if ((millis() - lastStrikeDebounceTime) > debounceDelay) 
    {
        // strike counter e.g. "_P001_"
        String dataStr = "_P";
        if (reading != strikeState) 
        {
          strikeState = reading;

        if (strikeState == HIGH) {
            //check not in home position
            if (homePosition == LOW)
            {
                strikePosition += 1; 
                if (strikePosition>999)  {strikePosition=999;}
                dataStr += dec2String(strikePosition);
                dataStr += "_"; 
                iron_cmd(dataStr); 
            }
        }
      }
    }
    lastStrikeState = reading;
}

void debounce_key()
{
    // check service mode
    reading = digitalRead(KEY);

    if (reading != lastmaintenanceState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) 
    {
      // longer than the debounce delay, take reading as the actual current state:
        if (reading != maintenanceState) 
        {
          maintenanceState = reading;

        if (maintenanceState == LOW) {
            //do action maintenance button
            if (isMaintenance == LOW) {
              digitalWrite(LED, LOW); 
              iron_cmd("_SERV_");  
              isMaintenance = HIGH;
            }
            else {
              digitalWrite(LED, HIGH); 
              iron_cmd("_NORM_");  
              isMaintenance = LOW;              
            }
            digitalWrite(BEEP, LOW); 
        }
      }
    }

    lastmaintenanceState = reading;
}


// verify string id
boolean verifyID(String Str)
{
  return (Str==hiStr);
}

// send out iron cmd status;
void iron_cmd(String dataStr) {
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
    digitalWrite(LED, HIGH);  
}

//init bell
void b2b(){
    all_off();
    delay(800);
    digitalWrite(BEEP, HIGH);
    digitalWrite(LED, LOW);
    delay(800);
    all_off();
    delay(1000);
    digitalWrite(BEEP, HIGH);
    digitalWrite(LED, LOW);
    delay(800); 
    all_off();
}  

