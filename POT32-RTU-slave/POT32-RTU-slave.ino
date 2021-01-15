/*
  ModbusRTU ESP8266/ESP32
  Simple slave example
  
  (c)2019 Alexander Emelianov (a.m.emelianov@gmail.com)
  https://github.com/emelianov/modbus-esp8266
*/

#include <ModbusRTU.h>

/**Configure Holding Register (offset 100) with initial value 0x100*/
#define REGADD 0
#define SLAVE_ID 1

/**HardwareSerial& swSer=Serial2;  // modbusSlave RTU pins   RX2(16),TX2(17)*/
ModbusRTU mb;

/** Build time */
const char compileDate[] = __DATE__ " " __TIME__;


void setup() {
  // Initialize Serial port
  Serial.begin(38400);
  Serial2.begin(9600, SERIAL_8N1);
  Serial2.flush();
  
  mb.begin(&Serial2);
  mb.slave(SLAVE_ID);
  for (int index=0; index<8; index++) {
    mb.addHreg(REGADD+index, 0x100);
  }

  // Send some device info
  Serial.print("Build: ");
  Serial.println(compileDate);
}

void loop() {
  //uint16_t res = 0;
  //mb.readHreg(1, 1, &res); 
  mb.Hreg(2, 0x200);
  mb.Hreg(3, 0x300);
  mb.task();
  yield();
  delay(10);

}

