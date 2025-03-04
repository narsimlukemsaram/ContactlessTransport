// Include the Mona_ESP library
// This script finds the dead zone of each wheel
// Right: 85.71, Left: 85.71 RPM
// AcoustoBot_3
#include <Wire.h>
#include "Mona_ESP_lib.h"
#include <ESP32Encoder.h> // https://github.com/madhephaestus/ESP32Encoder.git
 
ESP32Encoder right_encoder;
ESP32Encoder left_encoder;
 
 
int right_new_pos, right_old_pos, left_new_pos, left_old_pos;
// float x_d = 300, x_new, x_old = 100, y_d = 500, y_new, y_old = 500, theta_new, theta_old = 0, theta_d =0, theta_f =0, last_theta = 1;// AcoustoBot_1
float x_old = 600, y_old = 200, x_d = 500, y_d = 500,  x_new, y_new, theta_new, theta_old = 0, theta_d = 0, theta_f = 0, last_theta = 1;// AcoustoBot_2
//float x_d = 400, x_new, x_old = 700, y_d = 600, y_new, y_old = 800, theta_new, theta_old = 0, theta_d =0, theta_f =-1.570795, last_theta = 1;// AcoustoBot_3
float right_vel, left_vel, right_ref_vel, left_ref_vel, d_theta, d_x, d_y;
 
bool trans_done = false;
bool orient_done = false;
 
// set sample time
float Ts = 0.01;
float start_time, current_time, elapsed_time;
 
// Wheel radius is 15mm, every revolute is 3500 pulse
const float robot_radius = 40;
const float wheel_radius = 15;
const float scale_encoder = 0.0043;    // 15/3500
const float pi = 3.14159;
 
 
// initialise variables
float err = 0;
float control_theta=0, control_lin=0;
float control_R=0, control_L=0, control = 0;
float intError = 0;
int segment;
 
void setup(){
  Mona_ESP_init();
  Serial.begin(115200);
  right_encoder.attachHalfQuad ( Mot_right_feedback, Mot_right_feedback_2 );
  left_encoder.attachHalfQuad( Mot_left_feedback_2, Mot_left_feedback );
    
  // clear starting values
  right_encoder.clearCount();
  left_encoder.clearCount();
  
  // set the lastToggle
  current_time = millis();
  // set the initial positions
  right_old_pos = 0;
  left_old_pos = 0;
 
  // initial segment
  segment = 0;
  delay(5000);
}
void disp(){
  Serial.print("x :");
  Serial.print(x_new);
  Serial.print("\t y :");
  Serial.print(y_new);
  Serial.print("\t theta :");
  Serial.print(theta_new);
  Serial.print("\t theta_d :");
  Serial.print(theta_d);
  Serial.print("\t Control Theta :");
  Serial.print(control_theta);
  Serial.print("\t Control XY :");
  Serial.print(control_lin);
  Serial.print("\t Right_ref :");
  Serial.print(right_ref_vel);
  Serial.print("\t Left_ref :");
  Serial.print(left_ref_vel);
  Serial.println();
}
void position(){
  elapsed_time = (millis()-current_time)/1000;    // elapsed time in seconds
  right_new_pos = right_encoder.getCount();
  left_new_pos = left_encoder.getCount();
  right_vel = 2*pi*((right_new_pos-right_old_pos)/(3500*elapsed_time));
  left_vel = 2*pi*((left_new_pos-left_old_pos)/(3500*elapsed_time));
  d_theta = (0.5*(wheel_radius/robot_radius)*right_vel)-(0.5*(wheel_radius/robot_radius)*left_vel);
  d_x = ((wheel_radius*cos(theta_old)/2)*right_vel)+((wheel_radius*cos(theta_old)/2)*left_vel);
  d_y = ((wheel_radius*sin(theta_old)/2)*right_vel)+((wheel_radius*sin(theta_old)/2)*left_vel);
  x_new = x_old + (d_x*elapsed_time);
  y_new = y_old + (d_y*elapsed_time);
  if((theta_old + d_theta*elapsed_time) > 2*pi){
    theta_new = theta_old + d_theta*elapsed_time -(2*pi);
  }
  else if((theta_old + d_theta*elapsed_time) < -2*pi){
    theta_new = theta_old + d_theta*elapsed_time +(2*pi);
  }
  else{
    theta_new = theta_old + d_theta*elapsed_time;
  }
  theta_d = atan2(y_d-y_new,x_d-x_new);
  right_old_pos = right_new_pos;
  left_old_pos = left_new_pos;
  x_old = x_new;
  y_old = y_new;
  theta_old = theta_new;
}
 
void Propotional(float ref, float act, float Kp){
  err = (ref-act);
  control = Kp*err;
  if (control<-64){
    control = -64;
  }
}
void Propotional_theta(float ref, float act, float Kp, float&control_angle){
  err = (ref-act);
  control_angle = Kp*err;
}
void Propotional_XY(float ref_x, float ref_y, float act_x, float act_y,float Kp){
  float a = (ref_x-act_x)*(ref_x-act_x);
  float b = (ref_y-act_y)*(ref_y-act_y);
  err = sqrt(a+b);
  control_lin = Kp*err;
}
//void PI(float ref, float act, float Kp, float Ki){
//  err = (ref-act);
//  intError += err * Ts;
//  intError = bound(intError, -128, 128);
//  control = Kp*err+Ki*intError;
//}
float bound(float x, float x_min, float x_max){
  if(x < x_min){x = x_min;}
  if(x > x_max){x = x_max;}
  return x;
}
 
void dif_inverse(float Vb, float dot_theta, float&w_r, float&w_l){
  w_r = 0.0667*Vb+2.6667*dot_theta;
  w_l = 0.0667*Vb-2.6667*dot_theta;
}
 
void loop(){
  position();
  current_time = millis();
  if(trans_done==false){
    Propotional_theta(theta_d,theta_new, 1, control_theta);
    Propotional_XY(x_d,y_d,x_new,y_new,0.1);
    dif_inverse(control_lin,control_theta, right_ref_vel, left_ref_vel);
  }
    else{
      Propotional_theta(theta_f,theta_new, 1, last_theta);
      dif_inverse(control_lin,last_theta, right_ref_vel, left_ref_vel);
    }
  if(abs(control_lin)<0.01 && abs(control_theta<0.01)){
    trans_done = true;
  }
  if(trans_done && abs(last_theta)<0.01){
    orient_done = true;
  }
  if (trans_done==false || orient_done==false){
    if (right_ref_vel<0){
        Propotional(-right_ref_vel, -right_vel,-255/right_ref_vel);
        control_R = control+64;
        Right_mot_backward(control_R);
      }
      else{
        Propotional(right_ref_vel, right_vel,255/right_ref_vel);
        control_R = control+64;
        Right_mot_forward(control_R);
      }
      if (left_ref_vel<0){
        Propotional(-left_ref_vel,-left_vel,-255/(left_ref_vel));
        control_L = control+70;
        Left_mot_backward(control_L);
      }
      else{
        Propotional(left_ref_vel,left_vel,255/(left_ref_vel));
        control_L = control+70;
        Left_mot_forward(control_L);
      }
  }
  else{
    Motors_stop();
    delay(2000);
  }
  disp();
  delay(Ts*1000);
}
