#include <Servo.h> 

// name init-final 20210515
#define Servo_1 6 //thumb abduction 
#define Servo_2 7 //thumb flexion 
#define Servo_3 8 //midlle 
#define Servo_4 9 //index 
#define Servo_5 10 //little 
#define Servo_6 12 //ring 

Servo thumb_abduction,thumb_flexion,  servo_PWM3, servo_PWM6, servo_PWM5, servo_PWM4, servo_PWM7;  // create servo object to control a servo 
 
int val_PWM1,val_PWM2, val_PWM3, val_PWM6, val_PWM5, val_PWM4, val_PWM7;    // variable to read the value from the analog pin 
 
void setup() 
{ 
  // Servomotors
  servo_PWM1.attach(Servo_3);  // attach the servo connected to PCB's PWM1 to Teensy's pin2. This is motor 1. It is the thumb abduction.
  servo_PWM2.attach(Servo_6);  // attach the servo connected to PCB's PWM2 to Teensy's pin3. This is motor 2. It is the thumb flexion.
  servo_PWM3.attach(Servo_5);  // attach the servo connected to PCB's PWM3 to Teensy's pin4. This is motor 3. It is the middle.
  servo_PWM4.attach(Servo_4);  // attach the servo connected to PCB's PWM4 to Teensy's pin5. This is motor 4. It is the index.
  servo_PWM5.attach(Servo_2);  // attach the servo connected to PCB's PWM6 to Teensy's pin5. This is motor 5. It is the little.
  servo_PWM6.attach(Servo_1);  // attach the servo connected to PCB's PWM6 to Teensy's pin6. This is motor 6. It is the ring.
} 
 
void loop() 
{ 
  int values[5][6] = {{0,0,0,270,270,270},{160,0,0,270,270,270},{0,160,0,270,270,270},{0,0,160,270,270,270},{0,0,0,40,40,40}};
  //int values[6][2] = {{0,160},{0,270},{0,270},{270,0},{270,0},{270,40}};
  
  for(int k=0;k<5;k++){
    
    val_PWM1 = values[k][0];
    val_PWM2 = values[k][1];
    val_PWM3 = values[k][2];
    val_PWM4 = values[k][3];
    val_PWM5 = values[k][4];
    val_PWM6 = values[k][5];
    Serial.print("val_PWM1: ");Serial.print(val_PWM1);Serial.print("\n");
    Serial.print("val_PWM2: ");Serial.print(val_PWM2);Serial.print("\n");
    Serial.print("val_PWM3: ");Serial.print(val_PWM3);Serial.print("\n");
    Serial.print("val_PWM4: ");Serial.print(val_PWM4);Serial.print("\n");
    Serial.print("val_PWM5: ");Serial.print(val_PWM5);Serial.print("\n");
    Serial.print("val_PWM6: ");Serial.print(val_PWM6);Serial.print("\n");
    Serial.print("\n");
    
    //val_PWM1 = 20;//[min=20;max=170]
    servo_PWM1.write(val_PWM1); // sets the servo position according to the scaled value. It ranges from 0 to 160. Rest position is 0. Max position is 160.
  
    //val_PWM2 = 20;//[min=20;max=160]
    servo_PWM2.write(val_PWM2); // sets the servo position according to the scaled value. It ranges from 0 to 270. Rest position is 0. Max position is 270.
    
    //val_PWM3 = 20;//[min=20;max=180]
    servo_PWM3.write(val_PWM3); // sets the servo position according to the scaled value. It ranges from 0 to 130. Rest position is 0. Max position is 130.
  
    //val_PWM6 = 160;//[min=160;max=0]
    servo_PWM4.write(val_PWM4); // sets the servo position according to the scaled value. It ranges from 270 to 0. Rest position is 270. Max position is 0.
  
    //val_PWM5 = 160;// [min=160;max=10]
    servo_PWM5.write(val_PWM5); // sets the servo position according to the scaled value. It ranges from 270 to 0. Rest position is 270. Max position is 0.
  
    //val_PWM7 = 140;// [min=140;max=30]
    servo_PWM6.write(val_PWM6); // sets the servo position according to the scaled value. It ranges from 270 to 40. Rest position is 270. Max position is 40.
    delay(2000);
  }
} 
