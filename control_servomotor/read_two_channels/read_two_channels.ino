#include <FlexCAN_T4.h>
#include <iostream>
#include <vector>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
std::vector<int16_t> motor_angles(6,0);
int motors_ids_length = 6;
uint8_t max_can_id = 6;
CAN_message_t msg;

// Read the CAN message that contains all the servomotors angle values
int receive_can_msg(){
  int i = 0;
  uint8_t current_array_id = -1;
  uint8_t can_id = 0;
  std::vector<uint8_t> current_array_ids_vector;
  
  current_array_id = msg.buf[0];
  can_id = msg.id;
  current_array_ids_vector.push_back(current_array_id);
  Serial.print("can_id is: ");Serial.println(can_id);
  unpack_read_angles();
  return 1; 
}

// Printing the vector
void print_vector(std::vector<int16_t> vector_to_print)
{
  Serial.print("< ");
  for (auto it = vector_to_print.begin(); it != vector_to_print.end(); ++it)
  {
    Serial.print(*it);Serial.print(" ");
  }
  Serial.print(">");
}

void setup(void){
  can1.begin();
  can1.setBaudRate(1000000); //1000000Hz
}

std::vector<int16_t> populate_angles_array(uint8_t can_id, uint8_t array_id, int16_t motor_angle){
  Serial.println("In populate");
  motor_angles[can_id] = motor_angle;  
  return motor_angles;
}

int unpack_read_angles()
{
  int16_t angle = (msg.buf[7]<<8)|(msg.buf[6]);
  uint8_t can_id = msg.id;
  uint8_t array_id = msg.buf[0];
  Serial.print("angle is: ");Serial.print(angle);Serial.print("can_id is: ");Serial.print(can_id);Serial.print("array_id is: ");Serial.println(array_id);
  std::vector<int16_t> angles = populate_angles_array(can_id, array_id, angle);
  print_vector(angles);
  return 1;
}

void loop() {
  if ( can1.read(msg) ) {
    Serial.print("CAN1 \n"); 
    receive_can_msg();
  }
}
