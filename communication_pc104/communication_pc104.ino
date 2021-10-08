#include <FlexCAN_T4.h>
#include <iostream>
#include <vector>
#include<array> 

using namespace std;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

struct AngleMsg
{
  int16_t angle_value;
  uint8_t can_id;
};

CAN_message_t send_msg,receiv_msg;

int ledPin = 13;
int count = 0;

std::vector<int16_t> motor_angles;
int motors_ids_length = 6;
int send_array_length = 34;

//! Structure to save the motor angles sent by the PC104
AngleMsg unpack_read_angles()
{
  int16_t angle = (receiv_msg.buf[7]<<8)|(receiv_msg.buf[6]);
  uint8_t can_id = receiv_msg.id;
  AngleMsg angle_msg = {angle, can_id};
  return angle_msg;
}

//! Print a C++ vector
void print_vector(std::vector<int16_t> vector_to_print)
{
  Serial.print("< ");
  for (auto it = vector_to_print.begin(); it != vector_to_print.end(); ++it)
  {
    Serial.print(*it);Serial.print(" ");
  }
  Serial.print(">");
}

//! Send sensory data to the PC104
void send_array_data(int16_t sensor_data_array[],int array_size)
{
  send_msg.len = 8;

  for(int k=0;k<array_size;k++)
  {
    send_msg.buf[0] = 0;// The first element of the CAN message encodes the sensor's value position in the array
    send_msg.buf[1] = 0;
    send_msg.buf[2] = 0;
    send_msg.buf[3] = 0;
    send_msg.buf[4] = 0;
    send_msg.buf[5] = 0;
    send_msg.buf[6] = sensor_data_array[k]&0xff;
    send_msg.buf[7] = (sensor_data_array[k]>>8)&0xff;
    send_msg.id = k+motors_ids_length;// The motor ids are already used as can_ids to send the data to the motors
    Serial.print("Message id: ");Serial.print(send_msg.id);Serial.print(" value: ");Serial.print(sensor_data_array[k]);Serial.print("\n");
    can1.write(send_msg);
  }
}

//! Setup the Arduino parameters
void setup() 
{
  digitalWrite(ledPin,HIGH);
  can1.begin();
  can1.setBaudRate(1000000);//1000000Hz bitrate for the CAN
  pinMode(ledPin, OUTPUT);
}

//! Infinite loop
void loop() 
{
  // Read the CAN messages coming from the PC104
  if(can1.read(receiv_msg)) 
  {
    AngleMsg angle_msg = unpack_read_angles();
    Serial.print(" The angle is: ");Serial.print(angle_msg.angle_value);
    Serial.print(" The can id is: ");Serial.print(angle_msg.can_id);Serial.print("\n");
    delay(5000);
  }

  // Send sensor readings to the PC104
  int16_t sensor_data_array[] = {-1,-2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,-30,-31,-32,-33,-34};

  // The array length might not be 34 all time. Therefore, add extra check
  if(sizeof(sensor_data_array)/sizeof(sensor_data_array[0])==send_array_length){
    send_array_data(sensor_data_array,sizeof(sensor_data_array)/sizeof(sensor_data_array[0]));
  }else{
    Serial.print(" The array's length is not 34.");
  }
  
  /*
  if(motor_angles.size() == motors_ids_length)
  {
    print_vector(motor_angles);
    //Serial.print("The size of motor angles is: ");Serial.print(motor_angles.size());Serial.print("\n");
    motor_angles.clear();
  }
  */

}
