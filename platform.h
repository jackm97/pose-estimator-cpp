// platform.h
//
// Definition of the platform class used for pose estimation. 
// The platform class is used for the pose estimation of
// the three-wheeled platform described in "pose_estimation.html".
//
// Once a platform object is created the estimate() method is called
// to estimate the pose.
//
// Methods:
//  std::tuple<float,float,float> estimate(float time,float steering_angle,int encoder_ticks,float angular_velocity)
//      Parameters:
//          time - time reading of the input data in seconds (should be zero upon initialization of platform object)
//          steering_angle - steering wheel angle in radians
//          encoder_ticks - number of ticks from the traction motor encoder 
//          angular_velocity - reading from a gyroscope measuring the rotation velocity around the Z axis
//      Returns:
//          std::tuple<float,float,float> representing pose estimation (x,y,heading) with units
//          (meters,meters,rad). The heading is initialized to (0,0,0).
//
//      See "pose_estimation.pdf" for deeper explanation of the parameters
//
// JACK MYERS, 2 June 2019

#define _USE_MATH_DEFINES
#include <cmath>
#include <tuple>

class platform{
  public:
    // Estimate current pose using data from encoders and gyro
    std::tuple<float,float,float> estimate(float time, float steering_angle, int encoder_ticks, float angular_velcoity);

  private:
    float time = 0.0;
    int encoder_ticks = 0;
    float x = 0.0;
    float y = 0.0;
    float theta = 0.0;
    float R = .2; // wheel radius (m)
    int gamma = 512; // encoder ticks per revolution
    float r_axel = 1.0; // distance from front wheel to back axel (m)
    
    // Estimate the speed of the front wheel
    float est_wheel_speed(float delta_time, int delta_ticks);
};
