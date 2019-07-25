For detailed information on the approach to pose estimation for a three-wheeled platform
and testing results see pose_estimation.html or, for the ipython notebook, see pose_estimation.ipynb.

platform:
  The platform class helps facilitate pose estimation of the three-wheeled platform described in
  pose_estimation.html. The pose is defined by (x,y,heading) where x and y
  refer to the position of the center of the rear axel. See tests.cpp for examples on usage of the
  platform class and testing routines.
  
  class platform
  {
    //
  }

  Constructor:
    platform() - default constructor creates a platform object with initial pose (0,0,0)

  Methods:
    std::tuple<float,float,flaot> estimate(float time, float steering_angle, int encoder_ticks, float angular_velocity)
      estimate the current pose

      parameters:
        float time - The time in seconds. Expected to be 0s at the time of creation of the platform object
        float steering_angle - steering wheel angle in radians
        int encoder_ticks - number of ticks recorded by the wheel encoder. Expected to be 0 at the time of creation
                            of the platform object
        float angular_velocity - rotational velocity of the platform about the Z-axis in rad/s measured by onboard gyro
      returns:
        std::tuple<float,float,float> estimated_pose - estimated pose at given time (x,y,heading) measured in (m,m,rad)   
    

