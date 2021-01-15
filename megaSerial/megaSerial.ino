/*
  Multple Serial ROS
 Receives from the main serial ports, sends to the ROS.
 port 1 data from RFID as Header msgs
 port 2 and port 3 datas from AGVS sensors at front and back.
 * Serial monitor open on Serial port 0:
 * first release on 20 May 2017 by Clarence
 * reversed on 30 Oct 2017
 */
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <rosserial_arduino/Adc.h>
ros::NodeHandle  nh;

std_msgs::String ch3_msg;
std_msgs::Header ch1_msg;
rosserial_arduino::Adc adc_msg;
ros::Publisher chatter("chatter", &ch3_msg);
ros::Publisher locater("locater", &ch1_msg);
ros::Publisher batmon("batmon", &adc_msg);

char hello1[12] = "";
char hello2[12] = ""; 
char hello3[12] = ""; 
int i = 0;
int j = 0;
int k = 0;
int t = 0;

int batteryPin = A0;    // select the input pin for the potentiometer 0.25 times actual voltage
int Values[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // battery voltage sort in order

void setup() {
  // initialize both serial ports:
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(locater);
  nh.advertise(batmon);
  ch1_msg.frame_id =  "";
  ch1_msg.stamp = nh.now();
  //Serial.begin(19200);
  Serial3.begin(9600);
  Serial2.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  
  // k loop
  
  if (Serial1.available()) {
    int inByte = Serial1.read(); 
    //Serial.write(inByte);
    if (inByte==02) {
      k = 0;
    }
    else {  
      if (inByte==03) {
        if (k>10) k = 13;
        else k = 0;
      } else {
        hello1[k] = inByte;
        k += 1;
      }
    }
 
    if (k > 11) {
      k = 0;
      hello1[10]= 0;
      ch1_msg.frame_id =  hello1;
      //Serial.print(ch1_msg.frame_id);
      //Serial.print(Serial1.available());
      ch1_msg.seq += 1;
      //ch1_msg.stamp = nh.now();
      locater.publish( &ch1_msg );
      nh.spinOnce();
      delay(2);      
    }
  }

  // j loop
  if (Serial2.available()) {
    int inByte = Serial2.read();
    if (inByte==02) {
      j = 0;
    }
    else {  
      if (inByte==03) {
        if (j>8) j = 11;
        else j = 0;
      } else {
        hello2[j] = inByte;
        j += 1;
      }
    }
 
    if (j > 9) {
      j = 0;
      hello2[8]= 0;
      hello2[2]= 'B';
      ch3_msg.data = hello2;
      chatter.publish( &ch3_msg );
      nh.spinOnce();
      delay(4);
    }
  }

  // i loop
  //Serial.print(Serial1.available());
  if (Serial3.available()) {
    int inByte = Serial3.read();
    if (inByte==02) {
      
      i = 0;
    }
    else {  
      if (inByte==03) {
        
        if (i>8) i = 11;
        else i = 0;
      } else {
        hello3[i] = inByte;
        i += 1;
        
      }
    }
 
    if (i > 9) {
      i = 0;
      // update t for reading on battery volt
      t += 1;
      hello3[8]= 0;
      hello3[2]= 'A';
      ch3_msg.data = hello3;
      chatter.publish( &ch3_msg );
      nh.spinOnce();
      delay(4);
    }
  }

  // t loop
  int r = t % 30;
  if (r==0) {
    
    // read the value from the potentimeter and sort by bubble
    int o = t / 30;
    t += 1;
    if (o < 8) {
      if (o!=7) {
        // volt = v * 500 / 1024
        int v = 0;
        for(int x=0; x<4; x++) v+= analogRead(batteryPin);
        long v32 = (long)(v) * 500;
        v = (int)(v32>>10);
        if(v > Values[o+1]) {
            Values[o] = Values[o+1];
            Values[o+1] = v;
        }
        else {
          Values[o] = v;
        }
      }
      else {
        if (Values[0]!=0) {
          int v = 0;
          for(int x=0; x<8; x++) v+= Values[x];
          Values[7] = v / 8;
        }
      }
    }
    else {
      
      t = 0;
      adc_msg.adc0 = Values[0];
      adc_msg.adc1 = Values[1];
      adc_msg.adc2 = Values[2];
      adc_msg.adc3 = Values[3];
      adc_msg.adc4 = Values[4];
      adc_msg.adc5 = Values[7];
      batmon.publish( &adc_msg );
      nh.spinOnce();
      delay(2);
    }
  }
  
  // end of loop
}
