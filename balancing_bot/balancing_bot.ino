/* ============================================
  balancing_bot arduino code is placed under the MIT license
  Copyright (c) 2020 Manhyeon

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/


#include "mpu.h"

int A_motor_L = 7;                 //set number of pins to motor driver's control pin
int A_motor_R = 6;                 //4,5,6,7 pins are for direction control      
int B_motor_L = 4;                 // 3,9 pins ar for speed control using PWM         
int B_motor_R = 5;    
                     
int A_motor_S = 9;                         
int B_motor_S = 3;                          

int P_control, I_control, D_control;
int PID_control;

float Time = 0.0034;      //set Time value. It is based on the time 
                          //that the program use for one loop

float ref_ang;        // ref_ang : the angle that the robot should be keeping
float cur_ang;        // cur_ang : current angle of the robot 
float pre_ang;        // pre_ang : previous angle of the robot

float Kp = 80;       // P,I,D gain for PID control
float Ki = 0.5;         // Please set these value based on experimental test
float Kd = 1;         // or simulation.

float pre_error;      // Previous error

unsigned long cur_time;
unsigned long pre_time = 0;

void doMotor(bool dir, int vel) {    // set the funtion that control the motor
  digitalWrite(A_motor_R, dir);
  digitalWrite(A_motor_L, not dir);
  analogWrite(A_motor_S, vel);

  digitalWrite(B_motor_L, dir);
  digitalWrite(B_motor_R, not dir);
  analogWrite(B_motor_S, vel);
}


void pidcontrol(float desired_angle) {  // set the funtion that control the robot based on PID control
  
  float error = desired_angle - cur_ang;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
  P_control = Kp * error;                           //get P control value
  I_control += Ki * error * Time;                   //get I control value
  D_control = Kd * (error - pre_error) / Time;      //get D control value

  PID_control = P_control + I_control + D_control;  // get PID control value 

 // If you want t use PI control or just P control,remove the remark
 
  //PID_control = P_control + I_control; 
  //PID_control = P_control;

  doMotor( (PID_control >= 0) ? HIGH : LOW, min(abs(PID_control), 255));
  pre_error = error;
}

// Please set the robot stable position and keep it at start of program 
// it sets ref_ang as first stable angel

void setup() {
  
  mpu_setup(); // this function is defined at mpu.h and mpu.cpp

  while (1)
  {
    cur_ang = mpu_get_pitch();  //get the current angle. The function is defined at mpu.h and mpu.cpp
    cur_time = millis(); // get the current time

//As the program run, the program takes some time get a stable value.
//I choosed the method that comparing current angle with previous angle 
//at sepecific intervals to know it is stable

    if (cur_time - pre_time >= 1000) 
    {
      pre_time = cur_time;
      if (abs(pre_ang - cur_ang) <= 0.1)  // If the difference of cunrrent angel and previous angle
      {                                   // is less than 0.1 degree, I regard it is stable
        ref_ang = cur_ang;           //set the ref angle
        Serial.print("pre_ang : ");
        Serial.println(pre_ang);
        break;
      }
      pre_ang = cur_ang;
    }
  }
  Serial.print("ref _ang : ");  
  Serial.println(ref_ang);

}

void loop() {

// main loop function are here2
  cur_ang = mpu_get_pitch();
  Serial.print("ang : ");
  Serial.println(cur_ang);
  Serial.print("pid value : ");
  Serial.println(PID_control);
  pidcontrol(ref_ang);

}
