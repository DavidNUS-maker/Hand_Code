#include<iostream>    
#include<array> 
#include <FlexCAN_T4.h>
using namespace std;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

CAN_message_t message;

int ledPin = 13;
int count = 0;

//! Setup the CAN communication between the Teensy and the PC104. The refresh rate is 250kHz.
void setup() 
{
  digitalWrite(ledPin,HIGH);
  can1.begin();
  can1.setBaudRate(250000);
  pinMode(ledPin, OUTPUT);
}

//! Send array of sensor values over CAN. The ID corresponds to the array number so it is possible to know which values belong to a specific array
void send_array_data(int16_t sensor_data_array[],int array_size)
{
  int msg_id = 6;// Set the CAN ID to 6 since the angles occupy the IDs from 0 to 5 (6 values since there are 6 motors)
  message.len = 8;

  for(int k=0;k<array_size;k++)
  {
    message.buf[0] = k;// The first element of the CAN message encodes the sensor's value position in the array
    message.buf[1] = 0;
    message.buf[2] = 0;
    message.buf[3] = 0;
    message.buf[4] = 0;
    message.buf[5] = 0;
    message.buf[6] = sensor_data_array[k]&0xff;
    message.buf[7] = (sensor_data_array[k]>>8)&0xff;
    Serial.print("Message id: ");Serial.print(message.id);Serial.print(" value: ");Serial.print(sensor_data_array[k]);Serial.print("\n");
    message.id = msg_id;
    msg_id++;
    can1.write(message);
  }
}

//! Infinite loop. 
void loop() {
  int16_t sensor_data_array[] = {-1,-2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,-30,-31,-32,-33,-34};
  digitalWrite(ledPin,HIGH);
  send_array_data(sensor_data_array,sizeof(sensor_data_array)/sizeof(sensor_data_array[0]));
  //delay(1000);
  
  /*
  message.buf[0] = 255;
  
  can1.write(message);
  */
  //can3.write(message);

  /*
  count = count + 1;
  int16_t value = begin(sensor_data_array);
  if (count%2){
    digitalWrite(ledPin,HIGH);
  }else{
    digitalWrite(ledPin,LOW);
  }
  delay(1000);
  */
}
