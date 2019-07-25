// tests.cpp
// The test functions contained in this file test the pose estimation algorithm
// defined in platform.h. The result of each test is saved in its own .csv file
// with the first column representing x and the second representing y. 
// For an in depth explanation and the result of each testing routine, refer to 
// 'pose_estimation.html', or for the Jupyter Notebook 'pose_estimation.ipynb',
// in this directory.
//
// JACK MYERS, 2 JUNE 2019
//
// References:
// [1] Wean Hall Dataset
//    @inproceedings{Alismail_2011_6990,
//    author = {Hatem Alismail and Brett Browning and M Bernardine Dias},
//    title = {Evaluating Pose Estimation Methods for Stereo Visual Odometry on Robots},
//    booktitle = {the 11th International Conference on Intelligen Autonomous Systems (IAS-11)},
//    year = {2011},
//    url = {https://www.ri.cmu.edu/publication_view.html?pub_id=6990}}

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>
#include <string>
#include "platform.h"
#include <fstream>

// test: steering angle of 0deg at a constant speed
// expected result: straight line
void lineTest();

// test: steering angle of 30deg at a constant speed
// expected result: one full circle
void circleTest();

// test: steering angle of 30deg for one full circle at a constant speed, then
//       steering angle of -30deg for one full circle at a constant speed
// expected result: a figure 8 path
void fig8Test();

// test: straight line path to half circle path then back to straight line and
//       finishing with another half circle path at the same steering angle
// expected result: a path similar to a racetrack
void racetrackTest();

// test: simulated driving data using the wheel encoder and gyro data
//       from the Wean Hall Dataset
//       NOTE: The timestamps from the gyro and encoder data don't match up
//       so the result of my pose estimation isn't expected to match up with
//       the pose data given in the Wean Hall Dataset
// expected result: smooth, connected path with no sudden changes in direction
void variableDataTest();


// Helper function to read data from Wean Hall Dataset
// Preconditions: The string objects encoder_file and gyro_file are a valid path
//                to the encoder and gyro data respectively
// Postconditions: Returns a tuple object containing a three vectors of type
//                 (double, int, double) representing in order from left to 
//                 right: timestamps starting from 0s, encoder ticks starting
//                 from 0 ticks, and angular velcoity in rad/s
std::tuple<std::vector<double>,std::vector<int>,std::vector<double>> getSensorData(std::string encoder_file, std::string gyro_file);



int main(){
  circleTest();
  fig8Test();
  racetrackTest();
  lineTest();
  variableDataTest();
  return 0;
}

void lineTest(){
  // simulate driving data
  float steering_angle = 0.0;
  float wheel_speed = 5.0; // m/s
  float angular_velocity = 0.0; // rad/s
  float delta_time = .001; // s
  int delta_ticks = (int)(wheel_speed * 1/(2*M_PI*.2) * 512 * delta_time);
  platform test_platform;

  std::ofstream outfile;
  outfile.open("lineTest.csv");

  float time, x, y;
  int encoder_ticks;
  for (int i=1; i<=1000; i++){
    time = i*delta_time;
    encoder_ticks = i*delta_ticks;
    auto estimate_pose = test_platform.estimate(time, steering_angle, encoder_ticks, angular_velocity);
    x = std::get<0>(estimate_pose);
    y = std::get<1>(estimate_pose);
    outfile << x << "," << y << std::endl;
  }
  outfile.close();
}

void circleTest(){
  // simulate driving data
  float steering_angle = 30*M_PI/180; //rad
  float wheel_speed = 5.0; // m/s
  float angular_velocity = wheel_speed * std::sin(steering_angle); // rad/s
  float delta_time = 2*M_PI/angular_velocity/1000; // s
  int delta_ticks = (int)(wheel_speed * 1/(2*M_PI*.2) * 512 * delta_time);
  platform test_platform; // set up platform object

  std::ofstream outfile;
  outfile.open("circleTest.csv");

  float time, x, y;
  int encoder_ticks;
  for (int i=1; i<=1000; i++){
    time = i*delta_time;
    encoder_ticks = i*delta_ticks;
    auto estimate_pose = test_platform.estimate(time, steering_angle, encoder_ticks, angular_velocity);
    x = std::get<0>(estimate_pose);
    y = std::get<1>(estimate_pose);
    outfile << x << "," << y << std::endl;
  }
  outfile.close();
}

void fig8Test(){
  // simulate driving data
  float steering_angle = 30*M_PI/180;
  float wheel_speed = 5.0; // m/s
  float angular_velocity = wheel_speed * std::sin(steering_angle); // rad/s
  float delta_time = 2*M_PI/angular_velocity/1000; // s
  int delta_ticks = (int)(wheel_speed * 1/(2*M_PI*.2) * 512 * delta_time);
  platform test_platform; // set up platform object

  std::ofstream outfile;
  outfile.open("fig8Test.csv");

  float time, x, y;
  int encoder_ticks;
  for (int i=1; i<=2000; i++){
    time = i*delta_time;
    encoder_ticks = i*delta_ticks;
    // left turn
    if (i<=1000){
      auto estimate_pose = test_platform.estimate(time, steering_angle, encoder_ticks, angular_velocity);
      x = std::get<0>(estimate_pose);
      y = std::get<1>(estimate_pose);
    }
    // right turn
    else{
      auto estimate_pose = test_platform.estimate(time, -1*steering_angle, encoder_ticks, -1*angular_velocity);
      x = std::get<0>(estimate_pose);
      y = std::get<1>(estimate_pose);
    }
    outfile << x << "," << y << std::endl;
  }
  outfile.close();
}

void racetrackTest(){
  // simulate driving data for the turns
  float steering_angle = 45*M_PI/180;
  float wheel_speed = 7.0; // m/s
  float angular_velocity = wheel_speed * std::sin(steering_angle); // rad/s
  float delta_time = 2*M_PI/angular_velocity/100; // s
  int delta_ticks = (int)(wheel_speed * 1/(2*M_PI*.2) * 512 * delta_time);
  int turn_iters = (int)(M_PI/angular_velocity/delta_time); // iterations for the turn
  int straight_iters = (int)(.5/delta_time);
  platform test_platform; // set up platform object

  std::ofstream outfile;
  outfile.open("racetrackTest.csv");

  float time, x, y;
  int encoder_ticks;
  for (int i=1; i<=(2*(straight_iters+turn_iters)); i++){
    // first straight away
    if (i <= straight_iters){ 
      steering_angle = 0;
      angular_velocity = 0;
    }
    // first turn
    else if (i <= (straight_iters + turn_iters)){
      steering_angle = 45*M_PI/180;      
      angular_velocity = wheel_speed * std::sin(steering_angle); // rad/s
    }
    // second straight away
    else if (i <= (2*straight_iters + turn_iters)){
      steering_angle = 0;      
      angular_velocity = 0; // rad/s
    }
    // second turn
    else if (i <= (2*(straight_iters+turn_iters))){
      steering_angle = 45*M_PI/180;      
      angular_velocity = wheel_speed * std::sin(steering_angle); // rad/s
    }
    time = delta_time*i;
    encoder_ticks = delta_ticks*i;
    auto estimate_pose = test_platform.estimate(time, steering_angle, encoder_ticks, angular_velocity);
    x = std::get<0>(estimate_pose);
    y = std::get<1>(estimate_pose);
    outfile << x << "," << y << std::endl;
  }
  outfile.close();
}

void variableDataTest(){
  // simulate driving data using wean dataset
  auto data = getSensorData("encoder.txt","gyro.txt");
  std::vector<double> time = std::get<0>(data);
  std::vector<int> encoder_ticks = std::get<1>(data);
  std::vector<double> angular_velocity = std::get<2>(data);
  float wheel_speed, steering_angle;
  platform test_platform;
  
  std::ofstream outfile;
  outfile.open("variableDataTest.csv");
  
  float x,y;
  for (unsigned int i=1; i<(time.size()-1); i++){
    wheel_speed = 2*M_PI*.2/512*(encoder_ticks[i] - encoder_ticks[i-1])/(time[i] - time[i-1]);
    if ((encoder_ticks[i]-encoder_ticks[i-1]) > 0){
      steering_angle = std::asin(angular_velocity[i]/wheel_speed);
      auto estimate_pose = test_platform.estimate((float)time[i], 
                                                  (float)steering_angle, 
                                                  (int)encoder_ticks[i], 
                                                  (float)angular_velocity[i]);
      x = std::get<0>(estimate_pose);
      y = std::get<1>(estimate_pose);
      outfile << x << "," << y << std::endl;
    }
    else{
      steering_angle = 0;
      auto estimate_pose = test_platform.estimate((float)time[i], 
                                                  (float)steering_angle, 
                                                  (int)encoder_ticks[i], 
                                                  (float)angular_velocity[i]);
      x = std::get<0>(estimate_pose);
      y = std::get<1>(estimate_pose);
      outfile << x << "," << y << std::endl;
    }
  }
  outfile.close();
}

std::tuple<std::vector<double>,std::vector<int>,std::vector<double>> getSensorData(std::string encoder_file, std::string gyro_file){
  std::ifstream encoder_data, gyro_data;
  encoder_data.open(encoder_file);
  gyro_data.open(gyro_file);
  
  std::vector<double> time, angular_velocity;
  std::vector<int> encoder_ticks;
  std::string data;

  // read to the to the middle of the encoder data 
  for (int i=0; i<2e4; i++)
    std::getline(encoder_data, data);
  // read off the comment in the gyro file
  std::getline(gyro_data, data);  
  
  // establish initial conditions
  std::getline(encoder_data, data, ' ');
  double t0 = std::stod(data);
  time.push_back(0.0);
  std::getline(encoder_data, data, ' ');
  int tick0 = std::stoi(data);
  encoder_ticks.push_back(0);
  std::getline(encoder_data, data); // skip last column

  for (int i=0; i<4404; i++){
    std::getline(encoder_data, data, ' '); // time data
    time.push_back(std::stod(data) - t0);
    
    std::getline(encoder_data, data, ' '); // tick data
    encoder_ticks.push_back(std::stoi(data) - tick0);

    std::getline(encoder_data, data); // skip remaining data
  }

  for (int i=0; i< 4404; i++){
    std::getline(gyro_data, data, ' '); // ignore time data
    std::getline(gyro_data, data, ' '); // ignore temp data
    std::getline(gyro_data, data, ' '); // angular velocity data (deg/s)
    angular_velocity.push_back(M_PI*std::stof(data)/180.0);
    std::getline(gyro_data, data); // skip remaining data;
  }
  encoder_data.close();
  gyro_data.close();

  return std::tuple<std::vector<double>,std::vector<int>,std::vector<double>>(time, encoder_ticks, angular_velocity);
}
