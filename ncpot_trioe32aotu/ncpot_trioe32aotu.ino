// Title: NCPOT TRIOE32 Rev 1.0 Test
// Default Arduino includes
#include <Arduino.h>
#include "WiFi.h"
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <nvs.h>
#include <nvs_flash.h>

// Includes for JSON object handling
// Requires ArduinoJson library
// https://arduinojson.org
// https://github.com/bblanchon/ArduinoJson
#include <ArduinoJson.h>

// Includes for BLE
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEDevice.h>
#include <BLEAdvertising.h>
#include <BLE2902.h>
#include <Preferences.h>

#include <ModbusRTU.h>
//#include <modbus.h>
//#include <modbusDevice.h>
//#include <modbusRegBank.h>
//#include <modbusSlave.h>
//HardwareSerial& swSer=Serial2;  // modbusSlave RTU pins   RX2(16),TX2(17)

// Library for thermocouple and PID control
#include <MAX31855.h> // Include MAX31855 Sensor library
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library
#include <ESP32Servo.h>
#include <AutoPID.h>

// Define pid settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP 48
#define KI 0.006  //0.012
#define KD 0

// Define preheat temperature profile parameters (in *C)
#define T_const1 40  // PWM zone was 30*C on midPoint
#define T_const2 1  // PID overshoot was 10*C on setPoint
#define T_constC 10  // PWM shift with eqt: PWM = 1 - (temperature-midPoint+T_constC)/T_const1;
#define T_size_X 430
#define T_size_L 430
#define T_size_M 420
#define T_size_S 410
#define T_preheat 400  // 60
#define PID_sampleTime 1000 // 1s
#define Kp_reheat 180
#define Ki_reheat 0.04
#define Kd_reheat 50
#define Bt_NESS 0.13
#define B0_NESS 0.12

#define REGADD 0
#define SLAVE_ID 1 
bool isSend = 1;


struct para_mode
{
  String chProbemsg;
  String chSetpoint;
  String chT_offset;
  String chBrightness0;
  String chBrightness;
  String chunnamed_1;
  String chunnamed_2;
};
struct para_mode paraDataPak;
String douHao = ",";
String header = "$BLE,";
String firstHeader = "$BLE1,";
String secondHeader = "$BLE2,";
String firstParaData;
String secondParaData;
String paraData; 
String BlueName = "";
int varT_offset =0;
int varKp_reheat = Kp_reheat;
float varBt_NESS = Bt_NESS;
float varB0_NESS = B0_NESS;
float varBm = 0.6;
std::string PARAM_READ_MODE = "MODE_NORM";
int varTime_set1 = 30;
int varTime_set2 = 30;
int varCount_set1 = 0;
int varCount_set2 = 0;

double temperature, outPut, setPoint, midPoint; // Input, output, set point
int PID_out = LOW;
int BLE_out = HIGH;
int windowSize = 4000;
unsigned long previousMillis = 0;
unsigned long windowStartTime;
unsigned long sendRate = 500; // Send data to app every 0.5s
unsigned long lastTempUpdate = 0;
unsigned long trackRate = 250; // track temperature every 0.5s

//PID myPID(&temperature, &outPut, &setPoint, Kp_reheat, Ki_reheat, Kd_reheat, DIRECT);
AutoPID altPID(&temperature, &setPoint, &outPut, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

ESP32PWM pwm;
const int APin = 2;   // PWM switch
const int BPin = 4;   // full power switch
int AFreq = 10000;  // orginal 5KHz of 13 bits
float brightness = 0;         // brightness predicted by linguistic logic

/** Build time */
const char compileDate[] = __DATE__ " " __TIME__;

/** http server */
const int led = 13;
char probemsg[6];
WebServer server(80);

/** Unique device name */
char apName[] = "TRIOE32-xxxxxxxxxxxx";
/** Selected network 
    true = use primary network
    false = use secondary network
*/
/** Flag if BLE is used as AP config otherwise as UART connect */
bool useConfigAP = false;
bool deviceConnected = false;
bool usePrimAP = true;
/** Flag if stored AP credentials are available */
bool hasCredentials = false;
/** Connection status */
volatile bool isConnected = false;
/** Connection change status */
bool connStatusChanged = false;

/**
 * Create unique device name from MAC address
 **/
void createName() {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  // Write unique name into apName
  if(BlueName == "0")
  {
    sprintf(apName, "TRIOE32-%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    }
   else{
        sprintf(apName, "TRIO%s-%02X%02X%02X%02X%02X%02X",BlueName.c_str(), baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);  
    }
        
  
}

/*******************************************************************************************************************
// See the following for generating UUIDs: https://www.uuidgenerator.net/
*******************************************************************************************************************/
// service for ESP32WiFiBlueTooth nRF Connect
#define SERVICE_UUID  "0000aaaa-ead2-11e7-80c1-9a214cf093ae"
#define WIFI_UUID     "00005555-ead2-11e7-80c1-9a214cf093ae"
// UART service UUID
#define SERVICE_UUID_UART      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" 
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

/*******************************************************************************************************************
** Declare all program constants                                                                                  **
*******************************************************************************************************************/
const uint32_t SERIAL_SPEED     = 38400; ///< Set the baud rate for Serial I/O
const uint8_t  SPI_CHIP_SELECT  =    SS; ///< D8 < IO5
const uint8_t  SPI_MISO         =  MISO; ///< D6  < IO19
const uint8_t  SPI_SYSTEM_CLOCK =   SCK; ///< D5  < IO18

/*******************************************************************************************************************
** Declare global variables and instantiate classes                                                               **
*******************************************************************************************************************/
MAX31855_Class MAX31855; ///< Create an instance of MAX31855

/** SSIDs of local WiFi networks */
String ssidPrim;
String ssidSec;
/** Password for local WiFi network */
String pwPrim;
String pwSec;

/** base BLE format */
BLECharacteristic *pCharacteristicUart;
/** Characteristic for digital output */
BLECharacteristic *pCharacteristicWiFi;
/** BLE Advertiser */
BLEAdvertising* pAdvertising;
/** BLE Service */
BLEService *pService;
/** BLE Server */
BLEServer *pServer;

/** Buffer for JSON string */
// MAx size is 51 bytes for frame: 
// {"ssidPrim":"","pwPrim":"","ssidSec":"","pwSec":""}
// + 4 x 32 bytes for 2 SSID's and 2 passwords
StaticJsonBuffer<200> jsonBuffer;

/** modbus serial ESP32 */
//modbusDevice regBank;
//modbusSlave slave;
ModbusRTU mb;
//Callback function to read correspondingDI
uint16_t cbRead(TRegister* reg, uint16_t val){
  Serial.print("New ");
  Serial.println(val);
  setPoint = float(val);
  return val;
 }
/**
 * MyServerCallbacks
 * Callbacks for client connection and disconnection
 */
class MyServerCallbacks: public BLEServerCallbacks {
  // TODO this doesn't take into account several clients being connected
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("BLE client connected");
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("BLE client disconnected");
    pAdvertising->start();
    delay(600);
  }
};

/**
 * MyCallbackHandler
 * Callbacks for BLE client read/write requests
 */
class MyCallbackHandler: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() == 0) {
      return;
    }
    Serial.println("Received over BLE: " + String((char *)&value[0]));

    // Decode data
    int keyIndex = 0;
    for (int index = 0; index < value.length(); index ++) {
      value[index] = (char) value[index] ^ (char) apName[keyIndex];
      keyIndex++;
      if (keyIndex >= strlen(apName)) keyIndex = 0;
    }

    /** Json object for incoming data */
    JsonObject& jsonIn = jsonBuffer.parseObject((char *)&value[0]);
    if (jsonIn.success()) {
      if (jsonIn.containsKey("ssidPrim") &&
          jsonIn.containsKey("pwPrim") && 
          jsonIn.containsKey("ssidSec") &&
          jsonIn.containsKey("pwSec")) {
        ssidPrim = jsonIn["ssidPrim"].as<String>();
        pwPrim = jsonIn["pwPrim"].as<String>();
        ssidSec = jsonIn["ssidSec"].as<String>();
        pwSec = jsonIn["pwSec"].as<String>();

        Preferences preferences;
        preferences.begin("WiFiCred", false);
        preferences.putString("ssidPrim", ssidPrim);
        preferences.putString("ssidSec", ssidSec);
        preferences.putString("pwPrim", pwPrim);
        preferences.putString("pwSec", pwSec);
        preferences.putBool("valid", true);
        preferences.end();

        Serial.println("Received over bluetooth:");
        Serial.println("primary SSID: "+ssidPrim+" password: "+pwPrim);
        Serial.println("secondary SSID: "+ssidSec+" password: "+pwSec);
        connStatusChanged = true;
        hasCredentials = true;
      } else if (jsonIn.containsKey("erase")) {
        Serial.println("Received erase command");
        Preferences preferences;
        preferences.begin("WiFiCred", false);
        preferences.clear();
        preferences.end();
        connStatusChanged = true;
        hasCredentials = false;
        ssidPrim = "";
        pwPrim = "";
        ssidSec = "";
        pwSec = "";

        int err;
        err=nvs_flash_init();
        Serial.println("nvs_flash_init: " + err);
        err=nvs_flash_erase();
        Serial.println("nvs_flash_erase: " + err);
      } else if (jsonIn.containsKey("reset")) {
        WiFi.disconnect();
        esp_restart();
      }
    } else {
      Serial.println("Received invalid JSON");
    }
    jsonBuffer.clear();
  };

  void onRead(BLECharacteristic *pCharacteristic) {
    Serial.println("BLE onRead request");
    String wifiCredentials;

    /** Json object for outgoing data */
    JsonObject& jsonOut = jsonBuffer.createObject();
    jsonOut["ssidPrim"] = ssidPrim;
    jsonOut["pwPrim"] = pwPrim;
    jsonOut["ssidSec"] = ssidSec;
    jsonOut["pwSec"] = pwSec;
    // Convert JSON object into a string
    jsonOut.printTo(wifiCredentials);

    // encode the data
    int keyIndex = 0;
    Serial.println("Stored settings: " + wifiCredentials);
    for (int index = 0; index < wifiCredentials.length(); index ++) {
      wifiCredentials[index] = (char) wifiCredentials[index] ^ (char) apName[keyIndex];
      keyIndex++;
      if (keyIndex >= strlen(apName)) keyIndex = 0;
    }
    pCharacteristicWiFi->setValue((uint8_t*)&wifiCredentials[0],wifiCredentials.length());
    jsonBuffer.clear();
  }
};

class UsartCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue(); 
      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");

        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
        }

        Serial.println();

        // Do stuff based on the command received from the, app
        if (rxValue.find("BLE ") != -1) { 
          if (rxValue.substr(4,2)=="ON") {
            BLE_out = HIGH;
            Serial.print("Turning ON!");
          }
          else if (rxValue.substr(4,3)=="OFF") {
            BLE_out = LOW;
            Serial.print("Turning OFF!");
          }
          else {
          int rxNum = String(rxValue.substr(4,4).c_str()).toInt();
           if (rxNum == 1001) { BLE_out = HIGH; }
           if (rxNum == 1000) { BLE_out = LOW; }
          }
        }
        // set param
        else if (rxValue.find("SETTp ") != -1) { 
          int rxNum = String(rxValue.substr(6, 3).c_str()).toInt();          
          if (rxNum>0) {
            setPoint = rxNum +T_const2;
            midPoint = rxNum - T_const1;
            mb.Hreg(16, setPoint - T_const2);
            Serial.print("setpoint set to ");
            Serial.print(setPoint - T_const2);
          }
        }
        else if (rxValue.find("SETOs ") != -1) { 
          int rxNum = String(rxValue.substr(6, 3).c_str()).toInt();          
//          if (rxNum!=0) {
            varT_offset = rxNum;
            //setPoint = float(mb.Hreg(16)) + varT_offset;
            //mb.Hreg(16, setPoint);
            Preferences preferences;
            preferences.begin("T_offset",false);
            preferences.putString("value",String(varT_offset));
            preferences.putBool("valid", true);
            preferences.end();
           // mb.Hreg(17, varT_offset);
            Serial.print("T_offset set to ");
            Serial.print( varT_offset);
//          }
        }
        else if (rxValue.find("SETB0 ") != -1) { 
          float rxNum = String(rxValue.substr(6, 2).c_str()).toFloat();          
          if (rxNum>0) {
            varB0_NESS = rxNum / 100;
            Serial.print("B0_NESS set to ");
            Serial.print( varB0_NESS);
          }
        }
        else if (rxValue.find("SETBt ") != -1) { 
          float rxNum = String(rxValue.substr(6, 2).c_str()).toFloat();          
          if (rxNum>0) {
            varBt_NESS = rxNum / 100;
            Serial.print("Bt_NESS set to ");
            Serial.print( varBt_NESS);
          }
        }
        else if(rxValue.find("MODE_") != -1){
          if(rxValue.substr(5,4) == "NORM"){
            PARAM_READ_MODE = rxValue;  
            Serial.print("NORMAL_MODE!");     
          }
          else if(rxValue.substr(5,4) == "PARA"){
            PARAM_READ_MODE = rxValue;
            Serial.print("PARA_MODE!");
          }
        }
        else if(rxValue.find("SETP1 ") != -1){
          paraDataPak.chunnamed_1 = String(rxValue.substr(6,4).c_str());
          Serial.print("SET_PARA1 set to");
          Serial.print(paraDataPak.chunnamed_1);
        }
        else if(rxValue.find("SETP2 ") != -1){
          paraDataPak.chunnamed_2 = String(rxValue.substr(6,4).c_str());
          Serial.print("SET_PARA2 set to");
          Serial.print(paraDataPak.chunnamed_2);
          }
         else if(rxValue.find("SETBM ") != -1){
          float rxNum = String(rxValue.substr(6,2).c_str()).toFloat();
          varBm = rxNum / 10;
          Serial.print("varBm set to ");
          Serial.print( varBm);
         }
         else if(rxValue.find("ADD1") != -1){
          varCount_set1++;
          Serial.print("ADD1:");
         Serial.print( varCount_set1);
          }
        //end param
         else if(rxValue.find("SETNAME ") != -1){
         BlueName = rxValue[8];
         
          for(int i = 0; i < 2; i++)
          {
            BlueName += rxValue[9 + i];
            }
            Preferences preferences;
            preferences.begin("BLUENAME",false);
            preferences.putString("value",BlueName);
            preferences.putBool("valid", true);
            preferences.end();
        }
        else if (rxValue.find("X") != -1) {
          setPoint = T_size_X+T_const2;
          midPoint = T_size_X - T_const1;
          mb.Hreg(16, setPoint - T_const2);
          Serial.print("Target Temperature-X");
        }
        else if (rxValue.find("L") != -1) {
          setPoint = T_size_L+T_const2 ;
          midPoint = T_size_L - T_const1;
          mb.Hreg(16, setPoint - T_const2);
          Serial.print("Target Temperature-L");
        }
        else if (rxValue.find("M") != -1) {
          setPoint = T_size_M+T_const2 ;
          midPoint = T_size_M - T_const1;  
          mb.Hreg(16, setPoint - T_const2);       
          Serial.print("Target Temperature-M");
        }
        else if (rxValue.find("S") != -1) {
          setPoint = T_size_S+T_const2 ;
          midPoint = T_size_S - T_const1;    
          mb.Hreg(16, setPoint - T_const2);
          Serial.print("Target Temperature-S");
        }
        else if (rxValue.find("R") != -1) {
          Preferences preferences;
          preferences.begin("WiFiCred", false);
          preferences.putBool("resetPref", true);
          preferences.end();
          Serial.print("Reset credentials SSID on next startup!");
        }
        Serial.println();
        Serial.println("*********");
      }
    }
};

void data_Convert_char()
{
   paraData.remove(0);                            // Clear
   paraDataPak.chSetpoint = String((setPoint - T_const2),0);
   paraDataPak.chT_offset = String(varT_offset);
   paraDataPak.chBrightness0 = String((varB0_NESS * 100), 0);
   paraDataPak.chBrightness = String((varBt_NESS * 100),0);
   paraDataPak.chunnamed_1 = "0001";
   paraDataPak.chunnamed_2 = "0002";
   //paraData = header + paraDataPak.chProbemsg + douHao + paraDataPak.chSetpoint + douHao +   paraDataPak.chBrightness0 + douHao + paraDataPak.chBrightness 
            //  + douHao + paraDataPak.chT_offset + douHao +paraDataPak.chunnamed_1 + douHao + paraDataPak.chunnamed_2;
   firstParaData = firstHeader + paraDataPak.chProbemsg + douHao + paraDataPak.chSetpoint + douHao +   paraDataPak.chBrightness0 + douHao + paraDataPak.chBrightness;
   secondParaData = secondHeader + paraDataPak.chT_offset + douHao +paraDataPak.chunnamed_1 + douHao + paraDataPak.chunnamed_2;
}

void probe_access() 
{
  if ((millis() - lastTempUpdate) > trackRate) {
	  int32_t ambientTemperature = MAX31855.readAmbient(); // retrieve MAX31855 die ambient temperature
	  int32_t probeTemperature   = MAX31855.readProbe();   // retrieve thermocouple probe temp
	  uint8_t faultCode          = MAX31855.fault();       // retrieve any error codes
	  if ( faultCode )                                     // Display error code if present
	  {
		temperature = sqrt(-1);
		Serial.print("Probe: fault code ");
		Serial.print(faultCode);
		Serial.println(" returned.");
	  }
	  else
	  {
		 temperature = (double)probeTemperature / 1000.0;
     setPoint = float(mb.Hreg(16)) + T_const2 + varT_offset;
     if(setPoint > T_preheat + T_const2) {  midPoint = setPoint - T_const1; }

		//update RTU
//		regBank.set(40001,  probeTemperature/100 + varT_offset * 10); 
//		regBank.set(40002,  ambientTemperature/100); 
//		regBank.set(40003,  (setPoint-T_const2)*10); 
//		regBank.set(40004, brightness*100);
//		regBank.set(40005, outPut);
//    if (PID_out==HIGH) { regBank.set(40006, 1); }
//    else { regBank.set(40006, 0); }

      mb.Hreg(0,probeTemperature/100 - varT_offset * 10);
      //else if(varT_offset < 0) {mb.Hreg(0,probeTemperature/100 - varT_offset * 10);}
      //else if(varT_offset == 0){mb.Hreg(0,probeTemperature/100);}
      mb.Hreg(1,ambientTemperature/100);
      mb.Hreg(2,(setPoint-T_const2 - varT_offset) * 10);
      //else if(varT_offset < 0) {mb.Hreg(2,(setPoint-T_const2 - varT_offset) * 10);}
      //else if(varT_offset == 0){mb.Hreg(2,(setPoint-T_const2)*10);}
      mb.Hreg(3, brightness*100);
      mb.Hreg(4, outPut);
      if(PID_out == HIGH){mb.Hreg(5,1);}
      else {mb.Hreg(5,0);}
      mb.Hreg(10, apName[8]*256 + apName[9]);
      mb.Hreg(11, apName[10]*256 + apName[11]);
      mb.Hreg(12, apName[12]*256 + apName[13]);
      mb.Hreg(13, apName[14]*256 + apName[15]);
      mb.Hreg(14, apName[16]*256 + apName[17]);
      mb.Hreg(15, apName[18]*256 + apName[19]);
      delay(2);
	  	memset(probemsg, 0, 6);
     
      dtostrf((float)probeTemperature/1000, 5, 1, probemsg);
      //else if(varT_offset < 0) {dtostrf((float)probeTemperature/1000 - varT_offset, 5, 1, probemsg);}
      //else if(varT_offset == 0){dtostrf((float)probeTemperature/1000, 5, 1, probemsg);}
      paraDataPak.chProbemsg = String(probeTemperature/1000 - varT_offset);
		//Serial.print("Probe/Ambient(C): ");
		//Serial.print(probemsg);
		//Serial.print(" ");
		//Serial.print((float)ambientTemperature/1000,1);
		//Serial.println();
	  }
	  lastTempUpdate = millis();
  }
}

//void reg_access()
//{
//    int Mdelay = 5; // micro delay 
//    
//    Serial.print("[0] ");
//    Serial.print(regBank.get(40001)); 
//    delay(Mdelay);
//    Serial.print(" [1] ");
//    Serial.print(regBank.get(40002));
//    delay(Mdelay);
//    Serial.print(" [2] ");
//    Serial.print(regBank.get(40003));
//    delay(Mdelay);
//    Serial.print(" [3] ");
//    Serial.print(regBank.get(40004));
//    delay(Mdelay);
//    Serial.print(" [4] ");
//    Serial.println(regBank.get(40005));
//    delay(Mdelay);
//
//    regBank.set(40011,  random(1, 100));        
//    regBank.set(40012,  random(1, 100));        
//    regBank.set(40013,  random(1, 100));              
//    regBank.set(40014,  random(1, 100));              
//    regBank.set(40015,  random(1, 100)); 
//
//}

/** initial slave of modbus_RTU softwarre serial*/
void init_ModbusSlave()
{
  Serial2.begin(9600,SERIAL_8N1);
  Serial2.flush();
  
  mb.begin(&Serial2);
  mb.slave(SLAVE_ID);
  for(int index = 0; index < 17; index++){
    mb.addHreg(index, 0x0);
    }
      mb.Hreg(6, varTime_set1);
      mb.Hreg(7, varTime_set2);
      mb.Hreg(8, varCount_set1);
      mb.Hreg(9, varCount_set2);
      mb.Hreg(16, setPoint);
      //mb.Hreg(17, varT_offset);
      delay(1);
      
}
/** setup slave of modbus_RTU thro' software serial */ 
//void init_RTU()
//{   
////Assign the modbus device ID.  
//  regBank.setId(1);
//  
//// Holding registers
//  regBank.add(40001);  
//  regBank.add(40002);  
//  regBank.add(40003);  
//  regBank.add(40004);  
//  regBank.add(40005);  
//  regBank.add(40006);  
//  
//  regBank.add(40011);  
//  regBank.add(40012);  
//  regBank.add(40013);  
//  regBank.add(40014);  
//  regBank.add(40015);  
//  regBank.add(40016);  
//
//  slave._device = &regBank; 
//  delay(100);
//  slave.setBaud(9600);
//  delay(100);
//  
//  Serial.println("\nModbus RTU Slave connected and Online\n");
//  
//}

/** Start BLE server and service advertising **/
void connectBLE() {
  // Initialize BLE and set output power
  BLEDevice::init(apName);
  BLEDevice::setPower(ESP_PWR_LVL_N11);  //ESP_PWR_LVL_P7

  // Create BLE Server
  pServer = BLEDevice::createServer();

  // Set server callbacks
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE Service
  if (useConfigAP) {
    // Create BLE Characteristic for WiFi config
    pService = pServer->createService(BLEUUID(SERVICE_UUID),20);
    pCharacteristicWiFi = pService->createCharacteristic(
                            BLEUUID(WIFI_UUID),
                            BLECharacteristic::PROPERTY_READ |
                            BLECharacteristic::PROPERTY_WRITE
                          );
    pCharacteristicWiFi->setCallbacks(new MyCallbackHandler());
  } else {
    // Create BLE Characteristic for USART connect
    pService = pServer->createService(BLEUUID(SERVICE_UUID_UART));
    pCharacteristicUart = pService->createCharacteristic(
                        BLEUUID(CHARACTERISTIC_UUID_TX),
                        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
                      );
    pCharacteristicUart->addDescriptor(new BLE2902());
    
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                        BLEUUID(CHARACTERISTIC_UUID_RX),
                        BLECharacteristic::PROPERTY_WRITE
                      );
    pCharacteristic->setCallbacks(new UsartCallbacks());  
  }
  
  // Start advertising
  pService->start();
  pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

/** Callback for receiving IP address from AP */
void gotIP(system_event_id_t event) {
  isConnected = true;
  connStatusChanged = true;
}

/** Callback for connection loss */
void lostCon(system_event_id_t event) {
  isConnected = false;
  connStatusChanged = true;
}

/**
   scanWiFi
   Scans for available networks 
   and decides if a switch between
   allowed networks makes sense

   @return <code>bool</code>
          True if at least one allowed network was found
*/
bool scanWiFi() {
  /** RSSI for primary network */
  int8_t rssiPrim;
  /** RSSI for secondary network */
  int8_t rssiSec;
  /** Result of this function */
  bool result = false;

  Serial.println("Start scanning for networks");

  WiFi.disconnect(true);
  WiFi.enableSTA(true);
  WiFi.mode(WIFI_STA);

  // Scan for AP
  int apNum = WiFi.scanNetworks(false,true,false,1000);
  if (apNum == 0) {
    Serial.println("Found no networks?????");
    return false;
  }
  
  byte foundAP = 0;
  bool foundPrim = false;

  for (int index=0; index<apNum; index++) {
    String ssid = WiFi.SSID(index);
    Serial.println("Found AP: " + ssid + " RSSI: " + WiFi.RSSI(index));
    if (!strcmp((const char*) &ssid[0], (const char*) &ssidPrim[0])) {
      Serial.println("Found primary AP");
      foundAP++;
      foundPrim = true;
      rssiPrim = WiFi.RSSI(index);
    }
    if (!strcmp((const char*) &ssid[0], (const char*) &ssidSec[0])) {
      Serial.println("Found secondary AP");
      foundAP++;
      rssiSec = WiFi.RSSI(index);
    }
  }

  switch (foundAP) {
    case 0:
      result = false;
      break;
    case 1:
      if (foundPrim) {
        usePrimAP = true;
      } else {
        usePrimAP = false;
      }
      result = true;
      break;
    default:
      Serial.printf("RSSI Prim: %d Sec: %d\n", rssiPrim, rssiSec);
      if (rssiPrim > rssiSec) {
        usePrimAP = true; // RSSI of primary network is better
      } else {
        usePrimAP = false; // RSSI of secondary network is better
      }
      result = true;
      break;
  }
  return result;
}

/**
 * Start connection to AP
 */
void connectWiFi() {
  // Setup callback function for successful connection
  WiFi.onEvent(gotIP, SYSTEM_EVENT_STA_GOT_IP);
  // Setup callback function for lost connection
  WiFi.onEvent(lostCon, SYSTEM_EVENT_STA_DISCONNECTED);

  WiFi.disconnect(true);
  WiFi.enableSTA(true);
  WiFi.mode(WIFI_STA);

  Serial.println();
  Serial.print("Start connection to ");
  if (usePrimAP) {
    Serial.println(ssidPrim);
    WiFi.begin(ssidPrim.c_str(), pwPrim.c_str());
  } else {
    Serial.println(ssidSec);
    WiFi.begin(ssidSec.c_str(), pwSec.c_str());
  }
}

void handleRoot() {
  server.send(200, "text/plain", probemsg);
}

void handleNotFound() {
  digitalWrite(led, 1);
  String message = "Resource Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  digitalWrite(led, 0);
}

void setup() {
  
  pinMode(led, OUTPUT);
  digitalWrite(led, 0);

  // Initialize Serial port
  Serial.begin(38400);
  // Send some device info
  Serial.print("Build: ");
  Serial.println(compileDate);

  Preferences preferBlueName;
  preferBlueName.begin("BLUENAME", false);
  bool hasPrefbluename = preferBlueName.getBool("valid", false);
  if(hasPrefbluename){
     BlueName = preferBlueName.getString("value", "0");
  }else{
    preferBlueName.putBool("vaild", true);
    preferBlueName.putString("value", "0");
    BlueName = "0";
    Serial.println("Init BLUENAME preferences of value = 0");
  }
  preferBlueName.end();
  
  // Create unique device name
  createName();
  
  Preferences preferOffset;
  preferOffset.begin("T_offset", false);
  bool hasPrefoffset = preferOffset.getBool("valid", false);
  if(hasPrefoffset){
    String value_string = preferOffset.getString("value", "0");
    varT_offset = value_string.toInt();
    Serial.print("get T_offset preferences value = ");
    Serial.println(value_string);
  }else{
    preferOffset.putBool("vaild", true);
    preferOffset.putString("value", "0");
    varT_offset = 0;
    Serial.println("Init T_offset preferences of value = 0");
  }
  preferOffset.end();
  
  Preferences preferences;
  preferences.begin("WiFiCred", false);
  bool hasPref = preferences.getBool("valid", false);
  bool resetPref = preferences.getBool("resetPref", false);
  preferences.putBool("resetPref", false);
  if (hasPref) {
    ssidPrim = preferences.getString("ssidPrim","");
    ssidSec = preferences.getString("ssidSec","");
    pwPrim = preferences.getString("pwPrim","");
    pwSec = preferences.getString("pwSec","");

    if (ssidPrim.equals("") 
        || pwPrim.equals("")
        || ssidSec.equals("")
        || pwPrim.equals("")) {
      Serial.println("Found preferences but credentials are invalid");
    } else {
      Serial.println("Read from preferences:");
      Serial.println("primary SSID: "+ssidPrim+" password: "+pwPrim);
      Serial.println("secondary SSID: "+ssidSec+" password: "+pwSec);
      hasCredentials = true;
    }
  } else {
    Serial.println("Could not find preferences, need send data over BLE");
  }
  preferences.end();

  // start PWM on analog Pin 2
  pwm.attachPin(APin, AFreq, 10); // 10KHz 12 bit
  pinMode(BPin, OUTPUT);
  digitalWrite(BPin, 0);

  if (!hasCredentials || resetPref)  {
    Serial.println("\n\n****** Reset SSID and PassWord *****");
    Serial.println("Please use app .. ESP32-WiFi-BLE .. to change SSID\n\n");
    useConfigAP = true; 
  }
  
  // Start BLE server
  connectBLE();

  Serial.println(F("Starting SPI for MAX31855 and Initializing sensor"));
 
  /********************************************************************************************
  ** Uncomment out either the hardware or software SPI call, depending upon which is in use  **
  ********************************************************************************************/
  while (!MAX31855.begin(SPI_CHIP_SELECT))    // Hardware SPI for MAX31855
  {
    Serial.println(F("Unable to start MAX31855. Waiting 3 seconds."));
    delay(3000);
  }
  Serial.print("Hardware SPI of enable on GPIO ");
  Serial.println(SPI_CHIP_SELECT);
  Serial.println();

  //if temperature is more than 4 degrees below or above setpoint,
  //OUTPUT will be set to min or max respectively
  //set PID update interval to 4000ms
  altPID.setBangBang(10);
  altPID.setTimeStep(3000);

  //myPID.SetOutputLimits(0, windowSize);
  //myPID.SetSampleTime(PID_sampleTime);
  //myPID.SetMode(AUTOMATIC); // Turn on PID control
  setPoint = T_preheat;
  //setPoint = setPoint + varT_offset;
  midPoint = T_preheat/2;
  windowStartTime = millis();

 /*
  if (hasCredentials) {
    // Check for available AP's
    if (!scanWiFi) {
      Serial.println("Could not find any AP");
    } else {
      // If AP was found, start connection
      connectWiFi();
    }
  }
  if (hasCredentials) {
    server.on("/", handleRoot);
    server.on("/inline", []() {
      server.send(200, "text/plain", "this works as well");
    });
    server.onNotFound(handleNotFound);
    server.begin();
  }
 */
 
  //init_RTU();
  init_ModbusSlave();
}

void loop() {
  /*
  if (connStatusChanged) {
    if (isConnected) {
      Serial.print("Connected to AP: ");
      Serial.print(WiFi.SSID());
      Serial.print(" with IP: ");
      Serial.print(WiFi.localIP());
      Serial.print(" RSSI: ");
      Serial.println(WiFi.RSSI());
      Serial.println("HTTP server started");

      if (MDNS.begin("ncport")) {
        Serial.println("MDNS responder started");
      }
    } else {
      if (hasCredentials) {
        Serial.println("Lost WiFi connection");
        // Received WiFi credentials
        if (!scanWiFi) { // Check for available AP's
          Serial.println("Could not find any AP");
        } else { // If AP was found, start connection
          connectWiFi();
        }
      } 
    }
    connStatusChanged = false;
  }
 */

  //PID control with brightness
  if (PID_out==HIGH) {
    pwm.writeScaled(brightness);
  } else {
    digitalWrite(BPin, 0);
    pwm.writeScaled(0.0);
  }
  delay(2);
  
 
  probe_access();
  //control the firing phase with brightness on different temperature zone
  if (isnan(temperature))  {
    digitalWrite(BPin, 0); 
    brightness = 0;
  }  else {
    brightness = (temperature-midPoint+T_constC)/T_const1;
    brightness = 1.0 - brightness;
    if (brightness<varBt_NESS+varB0_NESS)  { brightness = varBt_NESS+varB0_NESS; }
    if (brightness>1.0)  { brightness = 1.0; }
    if (temperature > setPoint)  {brightness = 0; }
    if ((PID_out==HIGH)&&(temperature < midPoint))  { digitalWrite(BPin, 1); }
    else { digitalWrite(BPin, 0); }
  }
 

  //reg_access();
  //if (isConnected)  { server.handleClient();}
  //slave.run();
  mb.task();
  yield();
  
  altPID.run();  //call every loop, updates automatically at certain time interval
  //myPID.Compute();  // Compute PID output (from 0 to windowSize) and control relay accordingly
  if (outPut!=0) { 
    PID_out = HIGH;
    if(outPut<OUTPUT_MAX) {
      brightness = varBt_NESS + outPut/2550.0*varBm;
    }
  } else { PID_out = LOW; }
  /*
  if (millis() - windowStartTime >= windowSize)  windowStartTime += windowSize; 
  if ( outPut> millis() - windowStartTime) PID_out = HIGH;
  else PID_out = LOW;
  */
  
  // Send data to the app periodically
  if (millis() - previousMillis > sendRate) {
    previousMillis = millis();

    if (BLE_out==HIGH) {
      Serial.print("measured value = ");
      Serial.print(temperature);
      Serial.print(" / ");
      if (PID_out==HIGH)  Serial.print("ON ");
      else  Serial.print("OFF");
      Serial.print("  out ");
      Serial.print(outPut);
      Serial.print("  PWM ");
      Serial.println(brightness);
    }
    
    if (deviceConnected && !useConfigAP) {
      if (PARAM_READ_MODE=="MODE_NORM")
        { pCharacteristicUart->setValue(probemsg); }
      else if(PARAM_READ_MODE == "MODE_PARA")
        { 
           data_Convert_char();
           if(isSend){
             isSend = 0;
            pCharacteristicUart->setValue(firstParaData.c_str());
            }
            else{
              isSend = 1;
              pCharacteristicUart->setValue(secondParaData.c_str()); 
             }
        }
      pCharacteristicUart->notify(); // Send the value to the app!
    }
  }
}
