# Project Overview

Implement a Kalman Filter from scratch for combining data from a 9-DoF IMU and a time-of-flight distance sensor 
attached to the end-effector of a delta robot. The filter will be used to estimate the position and velocity of the 
end-effector in 3D space.

Results will be visualized with a 3D plot, written in C++ and compared to kinematics estimations from the robot's control system.

Learning Concepts:
- Kalman Filtering
- Reading I2C data from high-speed sensors
- Eigen Library (C++ Library for Linear Algebra and Matrices)
- Graphics and Plotting in C++

## Resources
- https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
- https://groups.seas.harvard.edu/courses/cs281/papers/unscented.pdf
- https://www.kalmanfilter.net/default.aspx
