

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(7,INPUT_PULLUP);
}

void loop() {
  if(digitalRead(7) == HIGH){
    Serial.println("1");
  }
  else if(digitalRead(7)==LOW)
  {
    Serial.println("0");
    }
  // put your main code here, to run repeatedly:

}
