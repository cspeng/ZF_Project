/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/
// Pin setup
int dPins[] = {
  5, 6, 4, 7, 3, 8, 2, 9,
  A3, 10, A2, 11, A1, 12, A0, 13
};

// Converting from Decimal to Hex:

// NOTE: This function can handle a positive decimal value from 0 - 65535, and it will pad it
//       with 0's (on the left) if it is less than the desired string length.
//       For larger/longer values, change "unsigned int" to "long" for the decValue parameter.

String dec2Hex(unsigned int decValue) {
  String hexString = String(decValue, HEX);
  while (hexString.length() < 4) hexString = "0" + hexString;
  return hexString;
}

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  //Serial.begin(9600);
  Serial1.begin(9600);
  for (int i = 0; i < 16; i++) {
    pinMode(dPins[i], INPUT_PULLUP);      
  }
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  String hiStr = "V4";
  String dataStr = "01";
  int sensorVal= 0;
  for (int i = 0; i < 16; i++) {
    sensorVal+= (digitalRead(dPins[i])==LOW)*(1<<i);      
  }
  dataStr+= dec2Hex(sensorVal);
  // print out readings to serial;
  Serial1.write(2);                    //sof
  Serial1.print(hiStr);                //head
  Serial1.print(dataStr);              //data
  Serial1.write((uint8_t)(0));         //eod
  Serial1.write(3);                    //eof
  // delay on next transmit  
  delay(16);             
}

