// platform.cpp
// Implementation file for platform.h
//
// JACK MYERS 2 JUNE 2019

#define _USE_MATH_DEFINES
#include <cmath>
#include <tuple>
#include <iostream>
#include "platform.h"

std::tuple<float,float,float> platform::estimate(float time, float steering_angle, int encoder_ticks, float angular_velocity){
  
  float delta_time = time - this->time;
  int delta_ticks = encoder_ticks - this->encoder_ticks;
  
  float wheel_speed = est_wheel_speed(delta_time, delta_ticks);
  float dx, dy, dtheta;
  // if the wheel speed is 0, the platform doesn't move
  if (wheel_speed == 0){
    dx = 0;
    dy = 0;
    dtheta = 0;
  }
  // if the angular velocity or steering angle is 0 platform travels straight
  // "or" is used over "and" to account for noise in the sensors
  else if (angular_velocity == 0 || steering_angle==0){
    dx = wheel_speed * delta_time * std::cos(theta);
    dy = wheel_speed * delta_time * std::sin(theta);
    dtheta = 0;
  }
  // the platform is turning otherwise
  else{
    dtheta = delta_time*(angular_velocity);
    dx = r_axel / std::tan(steering_angle) * 
      (std::sin(theta + dtheta) - std::sin(theta));
    dy =  (r_axel / std::tan(steering_angle) * 
      (-std::cos(theta + dtheta)+std::cos(theta)));
  }
  x += dx;
  y += dy;
  theta += dtheta;
  this->time = time;
  this->encoder_ticks = encoder_ticks;

  return std::make_tuple(x, y, theta);
}

float platform::est_wheel_speed(float delta_time, int delta_ticks){
  float wheel_speed = (2*M_PI*R/gamma) * delta_ticks / delta_time;
  return wheel_speed;
}


