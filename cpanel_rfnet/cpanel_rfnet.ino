//
// created by Clarence / PEMS@2017
// serial conn assignment
byte   M1 = A4;  //模式
byte   M0 = A5;  //模式
byte  AUX = 6;  //wifi 模块状态
byte BEEP = 9;  //蜂鸣器
byte  LED = 11;  //按键灯

// push-key Variables store triggers:
byte  KEY = 10;//按键CALL
int reading = 0;            // current reading of the button
int buttonState;            // the current reading from the input pin
int lastButtonState = LOW;  // the previous reading from the input pin

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

// 通信格式 4A // 无线开关 == 20.01（32）// BOX channel == 50.00（80）
// 通信格式 5A // 无线开关 == 30.01（48）/ BOX channel == 60.00（96）
// 通信格式 2A // 无线开关 == 35.01（53）/ BOX channel == 55.00（85）
// 通信格式 5A靠窗 // 无线开关 == 40.01（64）/ BOX channel == 70.00（112）
// IP:135 8A:80
// IP:133 4A:45
// IP:139 75(117):25(37)
// IP:131 1F(31):A1(161)
byte CH = 85;               // master channel = 1f.00
byte ADDR = 01;             // local id
String connStr = "Z.";      // init conn pattern
String hiStr = "Z.01";      // start id code
String bcStr = "Z.00";      // incoming broadcast id

// variables
String comdata;
String ss1;  
byte amode = 0;
boolean mark = false;

// 子程序
void get_rfdata();
void debounce_key();
boolean verifyID(String Str);
String modbusStr(); 
void rfop_cmd(String hiStr, String dataStr);
void all_off();
void bell();
void b2b();

void setup() //初始化程序
{  
    //配置IO口
    pinMode(AUX, INPUT);  
    pinMode(M0, OUTPUT);
    pinMode(M1, OUTPUT);
  
    pinMode(KEY, OUTPUT); 
    pinMode(LED, OUTPUT);
    pinMode(BEEP, OUTPUT);
    
    //初始化IO
    digitalWrite(KEY, HIGH);
    digitalWrite(LED, HIGH);  //LED OFF
    digitalWrite(M0, LOW);    //模式0 
    digitalWrite(M1, LOW); 
    
    Serial.begin(38400);delay(3);
    Serial1.begin(9600);
    while (! digitalRead(AUX));
    while (!Serial1);
    b2b();
    amode = 2;
    comdata = "";
}

void loop() {
    //get key state and rf data
    get_rfdata();
    debounce_key();
}

// get rf data
void get_rfdata()
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
        if (verifyID(comdata.substring(0,4))) digitalWrite(LED, LOW);
        if(comdata.length()>9) 
        {
            mark = false;
            delay(20);
            digitalWrite(LED, HIGH);
            if (verifyID(comdata.substring(0,4)))
            {
              String readcmd = comdata.substring(4,10);
              if(readcmd=="_PING_") bell();
              if(readcmd=="_PING_") rfop_cmd(hiStr, readcmd);
            }
        }
    }
    else {comdata="";}
}

void debounce_key()
{
    // compare the A1 and A2 button to its previous state
    reading = digitalRead(KEY);

    if (reading != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) 
    {
      // longer than the debounce delay, take reading as the actual current state:
        if (reading != buttonState) 
        {
          buttonState = reading;

        if (buttonState == LOW) {
            //do action pushing button
            rfop_cmd(hiStr, "_CALL_");  
            digitalWrite(LED, LOW); 
            digitalWrite(BEEP, LOW); 
        }
        else {digitalWrite(LED, HIGH);}
      }
    }

    if ((millis() - lastDebounceTime) > 400) {digitalWrite(LED, HIGH);}
    lastButtonState = reading;
}


// verify string id
boolean verifyID(String Str)
{
  return (Str==hiStr) | (Str==bcStr);
}

// Modbus Comm String
String modbusStr(){
    String str = "";
    str += char(255);  //broadcast string
    str += char(255);
    str += char(CH);  //信道 
    return str;
}

// send out rfop cmd toggle;
void rfop_cmd(String hiStr, String dataStr) {
    ss1 = modbusStr();
    ss1 += hiStr;
    ss1 += dataStr;
    Serial1.print(ss1);                //head
    Serial.println(ss1);               //verify with usb com
    // delay on next transmit  
    delay(5);  
}

void all_off()
{
    digitalWrite(BEEP, LOW);
    digitalWrite(LED, HIGH);  
}

void bell() {
    all_off();
    for (int i; i<10; i++)
    {
        delay(20);
        debounce_key();
    }
    for (int i; i<20; i++)
    {
        digitalWrite(BEEP, HIGH);
        digitalWrite(LED, LOW);
        delay(20);          
        debounce_key();
    }
    all_off();
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
