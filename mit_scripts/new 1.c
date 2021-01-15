String hiStr = "MIT+";
boolean mark = false;

byte IN1 =4;
byte IN2 = 3;

byte HOME = 2;
byte AUX = 6;
byte BEEP = 9;
byte LED = 13;

int reading = 0;
unsigned long debounceDelay = 50;

boolean RESUME_ON = false;
int seq2 = 0;
char Buzzer_N = 2;
unsigned int buzzer_sequence = 0;

int homeState;
int lastHomeState = LOW;
int homeTwice= LOW;
unsigned long lastHomeDebounceTime = 0;

String comdata;

void get_usbdata();
void home_sensor(String Str);
void relay_cmd(String dataStr);

String dec2String(unsigned int decValue){
	String decString = String(decValue);
	while (decString.length() < 3) decString = "0" + decString;
	return decString;
}

void set_Resume(char seq2){
	if(RESUME_ON){
		if(seq2 < 25){digitalWrite(IN2, LOW);}
		else(digitalWrite(IN2,HIGH); RESUME_ON = false;)
	}
	else digitalWrite(IN2,HIGH);
}

void set_Buzzer(char seq2){
	if(buzzer_sequence!=0){
		if(buzzer_sequence&1){digitalWrite(IN1,LOW); digitalWrite(LED,HIGH);}
		else{digitalWrite(IN1,HIGH);digitalWrite(LED,LOW);}
		buzzer_sequence = (unsigned int)buzzer_sequence>>1;
		if(buzzer_sequence==0){digitalWrite(LED,LOW);}
	}else{
		digitalWrite(IN1,HIGH);
		buzzer_sequence = 0;
		if(Buzzer_N = 1){buzzer_sequence = 0x004c;}
		else if(Buzzer_N == 2){buzzer_sequence = 0x014c;}
		else if(Buzzer_N == 3){buzzer_sequence = 0x054c;}
		else if(Buzzer_N == 4){buzzer_sequence = 0x0294c;}
		
	}
	
}

void setup()
{
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	
	pinMode(HOME, INPUT_PULLUP);
	pinMode(AUX, INPUT_PULLUP);
	pinMode(BEEP, OUTPUT);
	pinMode(LED, OUTPUT);
	
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, HIGH);
	
	Serial.begin(9600);
	delay(3);
	while(!digitalRead(AUX));
	comdata = '';
}

void loop(){
	get_usbdata();
	seq2++;
	if(seq2>79){seq2 = 0;}
	if(seq2%5==0){
		set_Resume(seq2);
		set_Buzzer(seq2);
	}
	//home_sensor();
	delay(30);
}

void get_usbdata(){
	while(Serial.available()){
		comdata += char(Serial.read());
		mark = true;
		if(comdata.charAt(0)!=hiStr.charAt(0)){mark = false;}
	}
	
	if(mark){
		if(verifyID(comdata.substring(0,4))) digitalWrite(LED, LOW);
		if(comdata.length() > 9){
			mark = false;
			delay(20);
			digitalWrite(LED, HIGH);
			String readcmd = comdata.substring(4,10);
			if(readcmd == "_PING_"){RESUME_ON = false; relay_cmd("_PING_");digitalWrite(LED,LOW);}
			else if(readcmd == "RESUME") {RESUME_ON =true;relay_cmd("RESUME");digitalWrite(LED,LOW);}
			else if(readcmd == "WAKEn1") {Buzzer_N = 1;relay_cmd("WAKEn1");digitalWrite(LED,LOW);}
			else if(readcmd == "WAKEn2") {Buzzer_N = 2;relay_cmd("WAKEn2");digitalWrite(LED,LOW);}
			else if(readcmd == "WAKEn3") {Buzzer_N = 3;relay_cmd("WAKEn3");digitalWrite(LED,LOW);}
			else if(readcmd == "WAKEn4") {Buzzer_N = 4;relay_cmd("WAKEn4");digitalWrite(LED,LOW);}
		}
	}
	else{comdata = "";}
}

boolean verifyID(String Str){
return (Str == hiStr);}

void relay_cmd(String dataStr){
	Serial.write(2);
	Serial.print(hiStr);
	Serial.print(dataStr);
	Serial.write(3);
	
	delay(5);
}

void home_sensor()
{
	int readmark = digitalRead(LED);
	if(readmark == LOW){
		reading = digitalRead(HOME);
	}
	if(reading != lastHomeState){
		lastHomeDebounceTime = millis();
	}
	
	if((millis() - lastHomeDebounceTime) > 300) {homeTwice = LOW;}
	
	if((millis() - lastHomeDebounceTime) > debounceDelay)
	{
		if(reading!=homeState){
			homeState = reading;
			if(homeState == LOW){
				digitalWrite(LED,HIGH);
				delay(100);
				digitalWrite(LED, LOW);
				if(homeTwice == HIGH)
				{
					relay_cmd("RESUME");
					seq2 = 0;
					RESUME_ON = true;
					homeTwice = LOW;
					
				}
				else{homeTwice = HIGH;}
			}
		}
	}
	lastHomeState = reading;
}

#include <EEPROM.h>
#include "U8glib.h"
U8GLIB_SH1106_128X64 u8g(12,11,9,10);

String OLED_MSG = "";
String inputString = "";
boolean inputMark = false;
boolean stringComplete = false;
int ee_started = 4;

String connStr = "Z.";
String remoteStr = "Z.17";
String hiStr = "R.17";

String comdata;
boolean mark = false;
byte M1 = A5;
byte M0 = A4;

void button_states(void);
void get_rfdata(void);
boolean set_usrN(void);
void set_Lamp(char);
void bell_onAA(char);
void bell_lbat(char);

boolean verifyID(String Str){return (Str == remoteStr);}

int reading = 0;
int buttonState;
int lastButtonState = LOW;
int buttonState1;
int lastButtonState1 = LOW;
int buttonState2;
int lastButtonState2 = LOW;
int buttonState3;
int lastButtonState3 = LOW;
boolean I8 = false;
boolean I7 = false;
boolean I6 = false;
boolean I5 = false;

boolean M8 = I8;
boolean M7 = I7;
boolean M6 = I6;
boolean M5 = I5;

unsigned long debounceDelay = 50;
unsigned long lastDebounceTime = 0;
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long lastDebounceTime3 = 0;

boolean A0Press = false;
boolean A1Press = false;
boolean A2Press = false;
boolean A3Press = false;
int A0HoldCounter = 0;
int A1HoldCounter = 0;
int A2HoldCounter = 0;
int A3HoldCounter = 0;
int disp_margin = 0;

boolean BeepOn = false;
boolean LbatOn = false;
boolean LampOn = false;
char Buzzer_N = 2;
unsigned int lamp_sequence = 0;
unsigned int buzzer_sequence = 0;
const int AlertPinn = 2;
const int LampPin = 3;
const int BuzzPin = 4;
const int naPin = 13;

int seq = 0;
int seq2 = 0;
int seq_flash = 1;
boolean gotoHold = true;
boolean ontheWAY = false;
boolean stationAA = false;

boolean artLoad = false;
boolean artRemove = true;
char stationB = 0;
char *strN[6] = {"A","1","2","3","4","B"} ;

void draw(int seq, char *head){
	if(stationB == 0& stationAA == 0){u8g.drawStr(disp_margin,40,"mitgo ");}
	else if(LbatOn)
	{
		u8g.drawStr(0,40,"");
		u8g.drawStr(disp_margin,40,"low BAT");
	}
	else if(ontheWAY)
	{
		u8g.drawStr(0,40,"");
		if(!gotoHold){
			if(stationAA == 1){u8g.drawStr(disp_margin,40, "to pick");}
			else{
				u8g.drawStr(disp_margin,40," to B ");
				u8g.drawStr(disp_margin+96,40,head);
			}
		}
		else{
			if(stationAA = 1){u8g.drawStr(disp_margin,40,"-    ");}
			else{u8g.drawStr(disp_margin,40,"B   ");}
			u8g.drawStr(disp_margin+20,40,head);
			if(seq%3<2){u8g.drawStr(disp_margin+52,40,"hold");}
		}
	}
	else{
		u8g.drawStr(0,40,"");
		if((stationAA == 1){
			if(seq<9){u8g.drawStr(disp_margin,40,"-A pick");}
			else {u8g.drawStr(disp_margin,40,"-A    ");}
		}
		else{
			if(seq < 9){u8g.drawStr(disp_margin,40,"B out");}
			else{u8g.drawStr(disp_margin,40,"B   ");}
			u8g.drawStr(disp_margin+3,40,head);
		}
	}
	
	seq2++;
	if(seq2>79){seq2=0;}
	if(seq2%5==0){set_Lamp(seq2);}
	if(seq%16==0){
		if(LbatOn){bell_lbat(seq2);}
		else(bell_onAA(seq2);)
	}
}

void get_rfdata(void)
{
	while(Serial.available())
	{
		comdata+=char(Serial.read());
		mark = true;
		if(comdata.charAt(0)!='Z'){mark = false;}
		if(comdata.length()>1)
		{
			if(comdata.substring(0,2)!=connStr){mark = false;}
		}
	}
	if(mark)
	{
		if(verifyID(comdata.substring(0,4))) digitalWrite(LED_BUILTIN,HIGH);
		if(comdata.length()>9)
		{
			mark = false;
			delay(20);
			if(verifyID(comdata.substring(0,4)))
			{
				String readcmd = comdata.substring(4,10);
				if(readcmd=="_PING_"){tele_cmd(hiStr,"I");}
				digitalWrite(LED_BUILTIN,LOW);
				if(readcmd == "termAA"){stationAA=1;stationB=0;I5=M5;I6=M6;I7=M7;I8=M8;}
				else if(readcmd == "termBB"){stationAA=0;stationB=5;I5=M5;I6=M6;I7=M7;I8=M8;}
				else if(readcmd == "stopAA"){stationB=0;stationAA=1;set_usrN();BeepOn=true;}
				else if(readcmd =="stopBB"){stationB=5;stationAA=0;set_usrN();BeepOn=true;}
				else if(readcmd == "stopB1"){stationB=1;stationAA=0;}
				else if(readcmd == "stopB2"){stationB=2;stationAA=0;}
				else if(readcmd == "stopB3"){stationB=3;stationAA=0;}
				else if(readcmd == "stopB4"){stationB=4;stationAA=0;}
				
				else if(readcmd =="gotoAA"){stationAA=1;stationB=0;}
				else if(readcmd =="gotoBB"){stationAA=0;stationB=5;}
				else if(readcmd =="gotoB1"){stationAA=0;stationB=1;I8=1;}
				else if(readcmd=="gotoB2"){stationAA=0;stationB=2;I7=1;}
				else if(readcmd=="gotoB3"){stationAA=0;stationB=3;I6=1;}
				else if(readcmd == "gotoB4"){stationAA=0;stationB=4;I5=1;}
				else if(readcmd=="WAKEon"){LampOn = true;LbatOn= false;}
				else if(readcmd == "WAKEff"){LampOn=false;LbatOn=false;}
				else if(readcmd=="WAKElb"){LampOn = false;LbatOn = true;}
				if(readcmd.substring(0,4)=="term"){gotoHold==1;LampOn=false;ontheWAY=0;}
				else if(readcmd.substring(0,4)=="stop"){gotoHold=1;LampOn=false;seq=0;seq2=0;ontheWAY=0;}
				else if(readcmd.substring(0,4)=="goto"){gotoHold=0;LampOn=true;ontheWAY=1;}
				else if(readcmd =="_hold_"){gotoHold = 1;LampOn = false;}
				else if(readcmd=="_goto_"){gotoHold = 0;LampOn=true;}
			}
		}
	}
	else {comdata="";}
}

void set_Lamp(char seq2){
	if(LampOn){
		if((seq2%4)==1){digitalWrit(LampPin,LOW);}
		else(digitalWrite(LampPin,HIGH);)
	}
	else{digitalWrite(LampPin,HIGH);}
}

void bell_onAA(char seq2){
	if(BeepOn){
		if(seq2%4)==1){digitalWrit(BuzzPin,HIGH);}
		else{digitalWrite(BuzzPin,LOW);}
		if((seq2%8)==7){BeepOn=false;}
	}
	else{digitalWrite(BuzzPin,LOW);}
}

void bell_lbat(char seq2){
	if(LbatOn){
		if((seq2%80)==1){digitalWrite(BuzzPin,HIGH);}
		else{digitalWrite(BuzzPin,LOW);}
	}
	else{digitalWrite(BuzzPin,LOW);}
}

void tele_cmd(String hiStr,String dataStr){
	if(dataStr=="I"){
		dataStr+=String(int(I8));
		dataStr+=String(int(I7));
		dataStr+=String(int(I6));
		dataStr+=String(int(I5));
		dataStr+="_";
	}
	
	String ss1 = hiStr;
	ss1 += dataStr;
	Serial.print(ss1);
	Serial1.print(ss1);
	delay(5);
}

void get_msg(){
	while(Serial.available()){
		inputString+=char(Serial.read());
		inputMark = true;
		if(inputString.charAt(0)!=hiStr.charAt(0)){inputMark=false;}
		else if(inputString.length()>1){
			if(inputString.substring(0,2)!=hiStr.substring(0,2)){inputMark = }
		}
	}
	
	if(inputMark){
		if(inputString.length()>9){
			inputMark = false;
			delay(5);
			if(inputString.substring(0,4)==hiStr){
				OLED_MSG = inputString.substring(4,10);
				stringComplete = true;
			}
		}
	}
}

void button_states(void){
	boolean park = (stationAA == 0 & !ontheWAY);
	boolean picked = (stationAA==1&!ontheWAY&seq==0);
	reading = digitalRead(A0);
	if(reading!=lastButtonState){
		if(reading==HIGH){A0HoldCounter=1;}
		lastDebounceTime = millis();
	}
	if(reading ==LOW){A0HoldCounter++;}
	if((millis()-lastDebounceTime)>debounceDelay){
		if(reading==HIGH){
			boolean parked = (stationB==1&park);
			if((I8&!parked&!picked)|(seq<seq_flash&parked)) {digitalWrite(8,HIGH);}
			else{digitalWrite(8,LOW);}
		}
		if(reading!=buttonState){
			if(reading==LOW){
				A0Press = true;
				digitalWrite(8,HIGH);
				if(A0HoldCounter>26){
					A0Press = false;
					I8=0;
					digitalWrite(8,LOW);
					tele_cmd(hiStr,"I");
					A0HoldCounter = 0;
					buttonState = reading;
				}
			}
			else  {buttonState = reading;}
		}
		if(reading == HIGH&A0Press){
			if(!I8&!ontheWAY){I8 = true;tele_cmd(hiStr,"I");}
			else{tele_cmd(hiStr,"_togg_");}
			A0Press = false;
		}
	}
	lastButtonState = reading;
	
	reading = digitalRead(A3);
	if(reading!=lastButtonState3){
		if(reading==HIGH){A3HoldCounter=1;}
		lastDebounceTime3 = millis();
	}
	if(reading==LOW){A3HoldCounter++;}
	if((millis()-lastDebounceTime3)>debounceDelay){
		boolean parked = (stationB ==4&park);
		if(reading==HIGH){
			if((I5&!parked&!picked)|(seq<seq_flash&parked)) {digitalWrite(5,HIGH);}
			else{digitalWrite(5,LOW);}
		}
		if(reading!=buttonState3){
			if(reading==LOW){
				A3Press = true;
				digitalWrite(5,HIGH);
				if(A3HoldCounter>26){
					A3Press = false;
					I5=0;
					digitalWrite(5,LOW);
					tele_cmd(hiStr,"I");
					A3HoldCounter = 0;
					buttonState3 = reading;
				}
			}
			else {buttonStae3 = reading;}
		}
		if(reading==HIGH&A3Press){
			if(!I5&!ontheWAY){I5 = true;tele_cmd(hiStr,"I");}
			else{tele_cmd(hiStr,"_togg_");}
			A3Press = false;
		}
	}
	lastButtonState3 = reading;
	
	reading = digitalRead(A1);
	if(reading!=lastButtonState1){
		lastDebounceTime1 = millis();
	}
	if(reading ==LOW){A1HoldCounter++;}
	if((millis() - lastDebounceTime)>debounceDelay){
		boolean parked = (stationB=2&park);
		if(reading==HIGH){
			if((I7&!parked&!picked)|(seq<seq_flash & parked)){digitalWrite(7,HIGH);}
			else{digitalWrite(7,LOW);}
		}
		if(reading!=buttonState1){
			if(reading==LOW){
				A1Press = true;
				digitalWrite(7,HIGH);
				if(A1HoldCounter>26){
					A1Press=false;
					I7=0;
					digitalWrite(7,LOW);
					tele_cmd(hiStr,"I");
					A1HoldCounter = 0;
					buttonState1 = reading;
				}
			}
			else{buttonState1 = reading;}
		}
		if(reading == HIGH&A1Press){
			if(!I7&!ontheWAY){
				if(!I7&!ontheWAY){I7 = true;tele_cmd(hiStr,"I");}
				else{tele_cmd(hiStr,"_togg_");}
				A1Press = false;
			}
		}
		lastButtonState1 = reading;
		
		reading = digitalRead(A2);
		if(reading!=lastButtonState2){
			if(reading ==HIGH){A2HoldCounter=1;}
			lastDebounceTime2 = millis();
		}
		if(reading==LOW){A2HoldCounter++;}
		if((millis()-lastDebounceTime2)>debounceDelay);
		boolean parked = (stationB==3&park;)
		if(reading == HIGH){
			if((I6&!parked&!picked)|(seq<seq_flash&parked)){digitalWrite(6,HIGH);}
			else{digitalWrite(6,LOW);}
		}
		if(reading!=buttonState2){
			if(reading==LOW){
				A2Press = true;
				digitalWrite(6,HIGH);
				if(A2HoldCounter>26){
					A2Press =false;
					I6=0;
					digitalWrite(6,LOW);
					tele_cmd(hiStr,"I");
					A2HoldCounter = 0;
					buttonState2 = reading;
				}
			}
			else{buttonState2 = reading;}
		}
		if(reading==HIGH&A2Press){
			if(!I6&!ontheWAY){I6 = true;tele_cmd(hiStr,"I");}
			else{tele_cmd(hiStr,"_togg_");}
			A2Press =false;
		}
	}
	lastButtonState2= reading;
}