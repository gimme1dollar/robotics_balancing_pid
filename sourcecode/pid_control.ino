// modified from https://github.com/kimkimsh/Educational_Balancing_Bot
#include "mpu.h"

int P_control, I_control, D_control;
float dt = 0.0034; // time for one loop

float Kp = 80;  
float Ki = 0.5;
float Kd = 1;  

float ref_ang;
float cur_ang; 
float pre_ang; 

float cur_error;
float pre_error;

unsigned long cur_time;
unsigned long pre_time = 0;

int A_motor_PWM = 10;
int A_motor_L   = 9; 
int A_motor_R   = 6; 
int B_motor_PWM = 11;
int B_motor_L   = 5; 
int B_motor_R   = 3;

void motor_setup() {
  pinMode(A_motor_PWM, OUTPUT);
  pinMode(A_motor_L, OUTPUT);
  pinMode(A_motor_R, OUTPUT);
  pinMode(B_motor_PWM, OUTPUT);
  pinMode(B_motor_L, OUTPUT);
  pinMode(B_motor_R, OUTPUT);
}

void motor_control(bool dir, int vel)
{ 
  analogWrite(A_motor_S, vel);
  digitalWrite(A_motor_R, dir);
  digitalWrite(A_motor_L, not dir);

  analogWrite(B_motor_S, vel);
  digitalWrite(B_motor_L, dir);
  digitalWrite(B_motor_R, not dir);
}

void pid_control(float desired_angle) { 
  cur_error = desired_angle - cur_ang;
  pre_error = cur_error;

  P_control = Kp * cur_error; 
  I_control += Ki * cur_error * dt;
  D_control = Kd * (cur_error - pre_error) / dt;

  return P_control + I_control + D_control;
}

void setup()
{
  // setup modules
  mpu_setup();
  motor_setup();

  // setup ref_ang
  while (1) {
    cur_ang = mpu_get_pitch(); 
    cur_time = millis(); 

    if (cur_time - pre_time >= 1000)
    {
      pre_time = cur_time;
      if (abs(pre_ang - cur_ang) <= 0.1) 
      {                                  
        ref_ang = cur_ang;
        break;
      }
      pre_ang = cur_ang;
    }
  }
  Serial.print("ref _ang : ");
  Serial.println(ref_ang);
}

void loop()
{
  cur_ang = mpu_get_pitch();
  output = pid_control(ref_ang);
  motor_control((output >= 0) ? HIGH : LOW, min(abs(output), 255));
  
  Serial.print("current state : ");
  Serial.print(cur_ang);
  Serial.print(" ");
  Serial.print("pid output : ");
  Serial.println(output);
}
