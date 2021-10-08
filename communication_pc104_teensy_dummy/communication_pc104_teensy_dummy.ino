//sending data by CAN BUS
//#include <Servo.h> 
//int servoPin = 9; 
//Servo Servo1;
//////////////////******** CAN BUS LIBRARIES **********//////////////////
#include <FlexCAN_T4.h>
#include <iostream>
#include <vector>
#include <array> 
using namespace std;
//////////////////*************************************//////////////////

//////////////////******** THREAD LIBRARIES **********///////////////////
#include <TeensyThreads.h>
//////////////////*************************************//////////////////

////******** VARIABLES INITIALIZATION FOR THREADS **********/////////////

//////////////////*************************************//////////////////

////////******** VARIABLES INITIALIZATION FOR CAN BUS **********/////////
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_32> can1;
CAN_message_t send_msg,receiv_msg;

struct AngleMsg
{
  int16_t angle_value;
  uint8_t can_id;
};

int ledPin = 13;
int count = 0;
std::vector<int16_t> motor_angles;
int motors_ids_length = 6;
int send_array_length = 28;
uint8_t array_id = 0;// Each CAN array is identified by an ID
//////////////////*************************************//////////////////

#include <CD74HC4067.h>

CD74HC4067 my_mux(0, 1, 2, 3);
#define S3 3
#define S2 2
#define S1 1
#define S0 0

int x = 0;

#include <Wire.h>
#include <MLX90393.h> //From https://github.com/tedyapo/arduino-MLX90393 by Theodore Yapo

MLX90393 mlx;
MLX90393::txyz data; //Create a structure, called data, of four floats (t, x, y, and z)

MLX90393 mlx_2;
MLX90393::txyz data_2; //Create a structure, called data, of four floats (t, x, y, and z)

MLX90393 mlx_3;
MLX90393::txyz data_3; //Create a structure, called data, of four floats (t, x, y, and z)

#define TCAADDR 0x70
#define bus1 0  
#define bus2 1


#define delay_n (uint8_t)1

#define A_0 4
#define B_0 5

int8_t n = 0; //starting from 0 can make the equavalent number for each column reading
uint8_t col;
uint8_t col_ind;
int16_t mag_x;
int16_t mag_y;
int16_t mag_z;
int8_t temp;
int16_t mag_x_2;
int16_t mag_y_2;
int16_t mag_z_2;
int8_t temp_2;
int16_t mag_x_3;
int16_t mag_y_3;
int16_t mag_z_3;
int16_t temp_3;

int16_t data_to_send[28];

////////****************** CODE FOR CAN **********************///////////
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

//! Send sensory data to the PC104 using the CAN bus
void send_array_data(uint8_t array_id,int16_t sensor_data_array[],int array_size)
{
  send_msg.len = 8;
  Serial.print("Array size is: ");Serial.println(array_size);
  for(int k=0;k<array_size;k++)
  {
    send_msg.buf[0] = array_id;// The first element in the CAN encodes the array unique identifier. This identifier is uint8_t and ranges between 0 and 255.
    send_msg.buf[1] = 0;
    send_msg.buf[2] = 0;
    send_msg.buf[3] = 0;
    send_msg.buf[4] = 0;
    send_msg.buf[5] = 0;
    send_msg.buf[6] = sensor_data_array[k]&0xff;
    send_msg.buf[7] = (sensor_data_array[k]>>8)&0xff;
    send_msg.id = k + motors_ids_length;// The motor ids are already used as can_ids to send the data to the motors
    //Serial.print("Message id: ");Serial.print(send_msg.id);Serial.print(" value: ");Serial.print(sensor_data_array[k]);Serial.print("\n");
    can1.write(send_msg);
    delayMicroseconds(100);
  }
}
////////////////////////////////////////////////////////*************************************//////////////////////////////////////////////

void setup() {
  can1.begin();
  can1.setBaudRate(1000000);//1000000Hz bitrate for the CAN
  // put your setup code here, to run once:
  Serial.begin(115200);
//  Serial.setTimeout(50);
  delay(100);
  // set up mlx90393
  Wire.begin();
  //  //Connect to sensor with I2C address jumpers set: A1 = 1, A0 = 0
  //  //Use DRDY pin connected to A3
  //  //Returns byte containing status bytes
  delay(500);
  activate_finger(bus1);
  activate_finger(bus2);

  delay(100);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  pinMode(S0, OUTPUT);  // sets the pin as output
  pinMode(S1, OUTPUT);  // sets the pin as output
  pinMode(S2, OUTPUT);  // sets the pin as output
  pinMode(S3, OUTPUT);  // sets the pin as output

//  Servo1.attach(servoPin);
  delay(100);
}

//////////////////*************************************//////////////////

void loop() {
 
//  x++;
//  Servo1.write(x / 90);
  // put your main code here, to run repeatedly:
  //  unsigned long currentMillis = millis();
  unsigned long start = micros();

//Matrix data
  col_ind = n / delay_n;
  switch(col_ind) {
    case 0:
      digital_write_COL1();
      n++;
      col=0;
      break;
    case 1:
      digital_write_COL2();
      n++;
      col=1;
      break;
    case 2:
      digital_write_COL3();
      n++;
      col=2;
      break;
    case 3:
      digital_write_COL4();
      n++;
      col=3;
      break;
    case 4:
      digital_write_COL5();
      n++;
      col=4;
      break;
    case 5:
      digital_write_COL6();
      n++;
      col=5;
      break;
    case 6:
      digital_write_COL7();
      n++;
      col=6;
      break;
    case 7:
      digital_write_COL8();
      n++;
      col=7;
      break;
    case 8:
      digital_write_COL9();
      n++;
      col=8;
      break;
    case 9:
      digital_write_COL10();
      n++;
      col=9;
      break;
    case 10:
      digital_write_COL11();
      n++;
      col=10;
      break;
    case 11:
      digital_write_COL12();
      n++;
      col=11;
      break;
    default:
      n = 0;
      return;
    }
  
//I2C data
  read_finger(bus1,&data_to_send[0],&data_to_send[1],&data_to_send[2],&data_to_send[3],&data_to_send[4],&data_to_send[5],&data_to_send[6],&data_to_send[7],&data_to_send[8]);
//  print_finger();
  read_finger(bus2,&data_to_send[9],&data_to_send[10],&data_to_send[11],&data_to_send[12],&data_to_send[13],&data_to_send[14],&data_to_send[15],&data_to_send[16],&data_to_send[17]);
//  print_finger();
  data_to_send[18] = col;
  send_data(&data_to_send[19],&data_to_send[20],&data_to_send[21],&data_to_send[22],&data_to_send[23],&data_to_send[24],&data_to_send[25],&data_to_send[26]);
  
  unsigned long end = micros();
  unsigned long delta = end - start;
  data_to_send[27] = 1000000/delta;

  for (byte i = 0; i < 28; i = i + 1) {
    Serial.print(data_to_send[i]);
    Serial.print(' ');
  }
  Serial.println(1000000/delta);

  // Read the CAN messages coming from the PC104
  if(can1.read(receiv_msg)) 
  {
    AngleMsg angle_msg = unpack_read_angles();
    Serial.print(" The angle is: ");Serial.print(angle_msg.angle_value);
    Serial.print(" The can id is: ");Serial.print(angle_msg.can_id);Serial.print("\n");
    delayMicroseconds(100);
  }
  
  // Send sensor readings to the PC104
  int16_t sensor_data_array[] = {-1,-2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28}; 

  // The array length might not be 28 all time. Therefore, add extra check
  //Serial.print("Array length is: ");Serial.print(sizeof(data_to_send)/sizeof(data_to_send[0]));
  if(sizeof(data_to_send)/sizeof(data_to_send[0])==send_array_length){
    send_array_data(array_id,data_to_send,sizeof(data_to_send)/sizeof(data_to_send[0]));
    //threads.addThread(send_array_data_thread, array_id,data_to_send,sizeof(data_to_send)/sizeof(data_to_send[0]));
    //send_array_data_thread(array_id,data_to_send,sizeof(data_to_send)/sizeof(data_to_send[0]));
  }else{
    Serial.print(" The array's length is not 28.");
  }

  // When the array_id overflows 255 (maximum capacity of uint8_t), get back to 0.
  if(array_id > 255){
    array_id = 0;
  }else{
    array_id = array_id + 1;
    Serial.println(array_id);
  }
  
  //Serial.println(1000000/delta);
  //Serial.flush();
  //Serial.println(" Hz");
  
}// Ending the void loop() function

//void switch_mux()

void switch_SP4T(byte B_bool, byte A_bool){
  digitalWrite(A_0,A_bool);
  digitalWrite(B_0,B_bool);
  }

void send_data(int16_t* mat1,int16_t* mat2,int16_t* mat3,int16_t* mat4,int16_t* mat5,int16_t* mat6,int16_t* mat7,int16_t* mat8){
  
  switch_SP4T(LOW,LOW);
  *mat1 = analogRead(A0);
  *mat2 = analogRead(A1);

  switch_SP4T(LOW,HIGH);
  *mat3 = analogRead(A0);
  *mat4 = analogRead(A1);

  switch_SP4T(HIGH,LOW);
  *mat5 = analogRead(A0);
  *mat6 = analogRead(A1);

  switch_SP4T(HIGH,HIGH);
  *mat7 = analogRead(A0);
  *mat8 = analogRead(A1);
  
//  int ROW_1 = analogRead(A0);  // read the input pin 0-1023
//  int ROW_2 = analogRead(A1);  // read the input pin
//  int ROW_3 = analogRead(A2);  // read the input pin
//  int ROW_4 = analogRead(A3);  // read the input pin

//  int read_digital1 = analogRead(A6);
//  int read_digital2 = analogRead(A7);
//  int read_digital3 = analogRead(A8);
//  int read_digital4 = analogRead(A9);

//  Serial.print(col);      // debug value
//  Serial.print(" ");
//  Serial.print(ROW_1_0);          // debug value
//  Serial.print(" ");
//  Serial.print(ROW_2_0);          // debug value
//  Serial.print(" ");
//  Serial.print(ROW_3_0);          // debug value
//  Serial.print(" ");
//  Serial.print(ROW_4_0);           // debug value
//  Serial.print(" ");
//  Serial.print(ROW_1_1);          // debug value
//  Serial.print(" ");
//  Serial.print(ROW_2_1);          // debug value
//  Serial.print(" ");
//  Serial.print(ROW_3_1);          // debug value
//  Serial.print(" ");
//  Serial.print(ROW_4_1);           // debug value
//  Serial.print(" ");
  return;
  }

void tcaselect(byte i){
  if (i>7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
  }

void activate_finger(byte bus){
  tcaselect(bus);
  byte status_1 = mlx.begin(0,0);
  delay(200);
  byte status_2 = mlx_2.begin(0,1);
  delay(200);
  byte status_3 = mlx_3.begin(1,1);
  delay(200);
  mlx.setGainSel(7);
  mlx.setResolution(0, 0, 0); //x, y, z
  mlx.setOverSampling(0);
  mlx.setDigitalFiltering(0);

  mlx_2.setGainSel(7);
  mlx_2.setResolution(0, 0, 0); //x, y, z
  mlx_2.setOverSampling(0);
  mlx_2.setDigitalFiltering(0);

  mlx_3.setGainSel(7);
  mlx_3.setResolution(0, 0, 0); //x, y, z
  mlx_3.setOverSampling(0);
  mlx_3.setDigitalFiltering(0);
  delay(100);
  }

void read_finger(byte bus,int16_t* mag_x,int16_t* mag_y,int16_t* mag_z,int16_t* mag_x_2,int16_t* mag_y_2,int16_t* mag_z_2,int16_t* mag_x_3,int16_t* mag_y_3,int16_t* mag_z_3){
  tcaselect(bus);
  mlx.readData(data); //Read the values from the MXL90393
  *mag_x = data.x;
  *mag_y = data.y;
  *mag_z = data.z;
  temp = data.t;

  mlx_2.readData(data_2); //Read the values from the MXL90393
  *mag_x_2 = data_2.x;
  *mag_y_2 = data_2.y;
  *mag_z_2 = data_2.z;
  temp_2 = data_2.t;
  
  mlx_3.readData(data_3); //Read the values from the MXL90393
  *mag_x_3 = data_3.x;
  *mag_y_3 = data_3.y;
  *mag_z_3 = data_3.z;
  temp_3 = data_3.t;

  return;
  }
void print_finger(){
//  Serial.println("device1:");
  Serial.print(mag_x);
  Serial.print(" ");
  Serial.print(mag_y);
  Serial.print(" ");
  Serial.print(mag_z);
  Serial.print(" ");
//  Serial.print(temp);
//  Serial.println("device2:");
//  Serial.print(" ");
  Serial.print(mag_x_2);
  Serial.print(" ");
  Serial.print(mag_y_2);
  Serial.print(" ");
  Serial.print(mag_z_2);
  Serial.print(" ");
//  Serial.print(temp_2);
//  Serial.println("device3:");
//  Serial.print(" ");
  Serial.print(mag_x_3);
  Serial.print(" ");
  Serial.print(mag_y_3);
  Serial.print(" ");
  Serial.print(mag_z_3);
//  Serial.print(" ");
//  Serial.print(temp_3);
  Serial.print(" ");

  }

void digital_write_COL1() {
//    digitalWrite(IN1, HIGH);
//    digitalWrite(IN2, LOW);
//    digitalWrite(IN3, LOW);
//    digitalWrite(IN4, LOW);
//    digitalWrite(IN5, LOW);
//    digitalWrite(IN6, LOW);
//    digitalWrite(IN7, LOW);
//    digitalWrite(IN8, LOW);
//    digitalWrite(IN9, LOW);
//    digitalWrite(IN10, LOW);
//    digitalWrite(IN11, LOW);
//    digitalWrite(IN12, LOW);
my_mux.channel(11);
  }
  
void digital_write_COL2() {
//    digitalWrite(IN1, LOW);
//    digitalWrite(IN2, HIGH);
//    digitalWrite(IN3, LOW);
//    digitalWrite(IN4, LOW);
//    digitalWrite(IN5, LOW);
//    digitalWrite(IN6, LOW);
//    digitalWrite(IN7, LOW);
//    digitalWrite(IN8, LOW);
//    digitalWrite(IN9, LOW);
//    digitalWrite(IN10, LOW);
//    digitalWrite(IN11, LOW);
//    digitalWrite(IN12, LOW);
my_mux.channel(10);
  }
  
void digital_write_COL3() {
//    digitalWrite(IN1, LOW);
//    digitalWrite(IN2, LOW);
//    digitalWrite(IN3, HIGH);
//    digitalWrite(IN4, LOW);
//    digitalWrite(IN5, LOW);
//    digitalWrite(IN6, LOW);
//    digitalWrite(IN7, LOW);
//    digitalWrite(IN8, LOW);
//    digitalWrite(IN9, LOW);
//    digitalWrite(IN10, LOW);
//    digitalWrite(IN11, LOW);
//    digitalWrite(IN12, LOW);
my_mux.channel(9);
  }
  
void digital_write_COL4() {
//    digitalWrite(IN1, LOW);
//    digitalWrite(IN2, LOW);
//    digitalWrite(IN3, LOW);
//    digitalWrite(IN4, HIGH);
//    digitalWrite(IN5, LOW);
//    digitalWrite(IN6, LOW);
//    digitalWrite(IN7, LOW);
//    digitalWrite(IN8, LOW);
//    digitalWrite(IN9, LOW);
//    digitalWrite(IN10, LOW);
//    digitalWrite(IN11, LOW);
//    digitalWrite(IN12, LOW);
my_mux.channel(8);
  }
void digital_write_COL5() {
//    digitalWrite(IN1, LOW);
//    digitalWrite(IN2, LOW);
//    digitalWrite(IN3, LOW);
//    digitalWrite(IN4, LOW);
//    digitalWrite(IN5, HIGH);
//    digitalWrite(IN6, LOW);
//    digitalWrite(IN7, LOW);
//    digitalWrite(IN8, LOW);
//    digitalWrite(IN9, LOW);
//    digitalWrite(IN10, LOW);
//    digitalWrite(IN11, LOW);
//    digitalWrite(IN12, LOW);
my_mux.channel(7);
  }
void digital_write_COL6() {
//    digitalWrite(IN1, LOW);
//    digitalWrite(IN2, LOW);
//    digitalWrite(IN3, LOW);
//    digitalWrite(IN4, LOW);
//    digitalWrite(IN5, LOW);
//    digitalWrite(IN6, HIGH);
//    digitalWrite(IN7, LOW);
//    digitalWrite(IN8, LOW);
//    digitalWrite(IN9, LOW);
//    digitalWrite(IN10, LOW);
//    digitalWrite(IN11, LOW);
//    digitalWrite(IN12, LOW);
my_mux.channel(6);
  }
void digital_write_COL7() {
//    digitalWrite(IN1, LOW);
//    digitalWrite(IN2, LOW);
//    digitalWrite(IN3, LOW);
//    digitalWrite(IN4, LOW);
//    digitalWrite(IN5, LOW);
//    digitalWrite(IN6, LOW);
//    digitalWrite(IN7, HIGH);
//    digitalWrite(IN8, LOW);
//    digitalWrite(IN9, LOW);
//    digitalWrite(IN10, LOW);
//    digitalWrite(IN11, LOW);
//    digitalWrite(IN12, LOW);
my_mux.channel(5);
  }
void digital_write_COL8() {
//    digitalWrite(IN1, LOW);
//    digitalWrite(IN2, LOW);
//    digitalWrite(IN3, LOW);
//    digitalWrite(IN4, LOW);
//    digitalWrite(IN5, LOW);
//    digitalWrite(IN6, LOW);
//    digitalWrite(IN7, LOW);
//    digitalWrite(IN8, HIGH);
//    digitalWrite(IN9, LOW);
//    digitalWrite(IN10, LOW);
//    digitalWrite(IN11, LOW);
//    digitalWrite(IN12, LOW);
my_mux.channel(4);
  }
void digital_write_COL9() {
//    digitalWrite(IN1, LOW);
//    digitalWrite(IN2, LOW);
//    digitalWrite(IN3, LOW);
//    digitalWrite(IN4, LOW);
//    digitalWrite(IN5, LOW);
//    digitalWrite(IN6, LOW);
//    digitalWrite(IN7, LOW);
//    digitalWrite(IN8, LOW);
//    digitalWrite(IN9, HIGH);
//    digitalWrite(IN10, LOW);
//    digitalWrite(IN11, LOW);
//    digitalWrite(IN12, LOW);
my_mux.channel(3);
  }
void digital_write_COL10() {
//    digitalWrite(IN1, LOW);
//    digitalWrite(IN2, LOW);
//    digitalWrite(IN3, LOW);
//    digitalWrite(IN4, LOW);
//    digitalWrite(IN5, LOW);
//    digitalWrite(IN6, LOW);
//    digitalWrite(IN7, LOW);
//    digitalWrite(IN8, LOW);
//    digitalWrite(IN9, LOW);
//    digitalWrite(IN10, HIGH);
//    digitalWrite(IN11, LOW);
//    digitalWrite(IN12, LOW);
my_mux.channel(2);
  }
void digital_write_COL11() {
//    digitalWrite(IN1, LOW);
//    digitalWrite(IN2, LOW);
//    digitalWrite(IN3, LOW);
//    digitalWrite(IN4, LOW);
//    digitalWrite(IN5, LOW);
//    digitalWrite(IN6, LOW);
//    digitalWrite(IN7, LOW);
//    digitalWrite(IN8, LOW);
//    digitalWrite(IN9, LOW);
//    digitalWrite(IN10, LOW);
//    digitalWrite(IN11, HIGH);
//    digitalWrite(IN12, LOW);
my_mux.channel(1);
  }
void digital_write_COL12() {
//    digitalWrite(IN1, LOW);
//    digitalWrite(IN2, LOW);
//    digitalWrite(IN3, LOW);
//    digitalWrite(IN4, LOW);
//    digitalWrite(IN5, LOW);
//    digitalWrite(IN6, LOW);
//    digitalWrite(IN7, LOW);
//    digitalWrite(IN8, LOW);
//    digitalWrite(IN9, LOW);
//    digitalWrite(IN10, LOW);
//    digitalWrite(IN11, LOW);
//    digitalWrite(IN12, HIGH);
my_mux.channel(0);
  }
