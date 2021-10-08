#include <FlexCAN_T4.h>
#include <iostream>
#include <vector>
#include <Servo.h>

// name init-final 20210515
/*#define Servo_1 6 //thumb abduction 
#define Servo_2 7 //thumb flexion 
#define Servo_3 8 //midlle 
#define Servo_4 9 //index 
#define Servo_5 10 //little 
#define Servo_6 12 //ring*/

#define Servo_1 6 //thumb abduction 
#define Servo_2 7 //thumb flexion 
#define Servo_3 8 //midlle 
#define Servo_4 9 //index 
#define Servo_5 10 //little 
#define Servo_6 12 //ring

////////****************** SERVOMOTORS INITIALIZATION **********************///////////
Servo servo_PWM1,servo_PWM2, servo_PWM3, servo_PWM6, servo_PWM5, servo_PWM4;  // create servo object to control a servo 
int val_PWM1,val_PWM2, val_PWM3, val_PWM6, val_PWM5, val_PWM4;    // variable to read the value from the analog pin 

////////********************* CAN INITIALIZATION **********************///////////////
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
std::vector<int16_t> motor_angles(6,0);
uint8_t max_can_id = 5;
CAN_message_t msg;

////////////********************* MAIN CODE **********************///////////////////
// Set the motor's positions
int set_motor_position(std::vector<int16_t> motor_angles){
  
    val_PWM1 = motor_angles[0];
    val_PWM2 = motor_angles[1];
    val_PWM3 = motor_angles[2];
    val_PWM4 = motor_angles[3];
    val_PWM5 = motor_angles[4];
    val_PWM6 = motor_angles[5];
    
    servo_PWM1.write(val_PWM1); // sets the servo position according to the scaled value. It ranges from 20 to 170. Rest position is 20. Max position is 170.
    servo_PWM2.write(val_PWM2); // sets the servo position according to the scaled value. It ranges from 20 to 160. Rest position is 20. Max position is 160.
    servo_PWM3.write(val_PWM3); // sets the servo position according to the scaled value. It ranges from 20 to 180. Rest position is 20. Max position is 180.
    servo_PWM4.write(val_PWM4); // sets the servo position according to the scaled value. It ranges from 160 to 0. Rest position is 160. Max position is 0.
    servo_PWM5.write(val_PWM5); // sets the servo position according to the scaled value. It ranges from 160 to 10. Rest position is 160. Max position is 0.
    servo_PWM6.write(val_PWM6); // sets the servo position according to the scaled value. It ranges from 140 to 30. Rest position is 140. Max position is 30.
    //delay(2000);
}

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

  if(can_id == max_can_id){
    Serial.println("I am in maximum");
  }
  
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

  // Servomotors
  servo_PWM1.attach(Servo_3);  // attach the servo connected to PCB's PWM1 to Teensy's pin2. This is motor 1. It is the middle finger.
  servo_PWM2.attach(Servo_6);  // attach the servo connected to PCB's PWM2 to Teensy's pin3. This is motor 2. It is the ring finger.
  servo_PWM3.attach(Servo_5);  // attach the servo connected to PCB's PWM3 to Teensy's pin4. This is motor 3. It is the little finger.
  servo_PWM4.attach(Servo_4);  // attach the servo connected to PCB's PWM4 to Teensy's pin5. This is motor 4. It is the index finger.
  servo_PWM5.attach(Servo_2);  // attach the servo connected to PCB's PWM6 to Teensy's pin5. This is motor 5. It is the thumb flexion.
  servo_PWM6.attach(Servo_1);  // attach the servo connected to PCB's PWM6 to Teensy's pin6. This is motor 6. It is the thumb abduction.

  servo_PWM1.write(20);
  servo_PWM2.write(20); 
  servo_PWM3.write(20); 
  servo_PWM4.write(160); 
  servo_PWM5.write(20); 
  servo_PWM6.write(140); 
}

// Add the motors' angles to the array
std::vector<int16_t> populate_angles_array(uint8_t can_id, uint8_t array_id, int16_t motor_angle){
  Serial.println("In populate");
  motor_angles[can_id] = motor_angle;  
  return motor_angles;
}

// Unpack the angles from the CAN message
int unpack_read_angles()
{
  int16_t angle = (msg.buf[7]<<8)|(msg.buf[6]);
  uint8_t can_id = msg.id;
  uint8_t array_id = msg.buf[0];
  Serial.print("angle is: ");Serial.print(angle);Serial.print("can_id is: ");Serial.print(can_id);Serial.print("array_id is: ");Serial.println(array_id);
  std::vector<int16_t> angles = populate_angles_array(can_id, array_id, angle);
  
  if(can_id==max_can_id){
    set_motor_position(angles);
  }
  return 1;
}

void loop() {
  if (can1.read(msg)) {
    Serial.print("CAN1 \n"); 
    receive_can_msg();
  }
}
