
[image1]: ./images/kalmanequations.png "Kalman filter equations"
[image2]: ./images/motion_model.png "Linear motion model"
[image3]: ./images/q.png "Covariance matrix Q"
[image4]: ./images/hradar.png "H for radar measurements"
[image5]: ./images/hradar_lin.png "Linearized observation model for radar measurements"
[image6]: ./images/dataset1.png "Results with first data set"
[image7]: ./images/dataset2.png "Results with second data set"

# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found in the classroom lesson for this project.

Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Project Structure

An overview about the files related to the kalman filter.

* kalman_filter.cpp / kalman_filter.h 
  Implementation of the Kalman filter. Contain the prediction and update step. The `Update` method updates the state using a linear model. 
 `UpdateEKF` uses a nonlinear model which was linearized to update the states.
* FusionEKF.cpp / FusionEKF.h 
  This class initializes the Kalman filter and perform the sensor fusion with the laser and radar measurements. 
* measurement_package.h
  Data structure definitions for the measurement data
 * tools.cpp / tools.h
   Various utility functions such as RMSE or Jacobian computation
 * main.cpp
   Handle the incoming measurements and ground truth and prepares them for the Kalman filter. 
   Calls the `ProcessMeasurement` method of the FusionEKF class for each measurement. 
   Handles the communication with the simulator over a websocket. 
 
## Implementation

The Kalman filter consists of a prediction und update step. The follow images shows the equations for this steps:
![alt text][image1]

The motion model is given by

![alt text][image2]

For the covariance matrix Q is 

![alt text][image3]

The observation model for the laser H is 

<a href="https://www.codecogs.com/eqnedit.php?latex=\mathbf{H}&space;=&space;\begin{pmatrix}&space;1&space;&&space;0&&space;0&space;&&space;0\\&space;0&space;&&space;1&&space;0&space;&&space;0&space;\end{pmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mathbf{H}&space;=&space;\begin{pmatrix}&space;1&space;&&space;0&&space;0&space;&&space;0\\&space;0&space;&&space;1&&space;0&space;&&space;0&space;\end{pmatrix}" title="\mathbf{H} = \begin{pmatrix} 1 & 0& 0 & 0\\ 0 & 1& 0 & 0 \end{pmatrix}" /></a>

The observation model for the radar measurements is

![alt text][image4]

This observation model is nonlinear. To use these measurements with the Kalman filter, we need to linearize the observation model h, which 
leads to the extended Kalman filter. To deal with this non-linearity, we use a Taylor series expansion. From this Taylor series expansions, only the 
first order is evaluated. The nonlinear model is derived with respect to each state variable, which yields a Jacobian matrix and is used as the linearized observation model for the radar measurements. 

![alt text][image5]

For the radar measurements the same state transition model is used as for the laser measurements. 

I tried to prevent unneccessary computations by using intermediate results. An example can be found in the lines 139-149
where the covariance matrix Q is computed. To prevent the creation of the identity matrix each time in the update step, 
the idenity matrix is initialized in the `init` method. 

If the `ProcessMeasurement` method called for the first time, the state vector and matrix P are initialized with data depending on the sensor type. 

## Results 


Here are images of the simulator for both datasets:

#### 1. Dataset

![alt text][image6]

#### 2. Dataset

![alt text][image7]


The RMSE should be less or equal to [0.11, 0.11, 0.52, 0.52]. With the usage of both sensor types for the dataset 1 and dataset 2 we fullfill
the criteria. 
I tested how well the Kalman filter perform when we use only one sensor of the first data set. By using only one sensor we meet not the specified
criteria. I think the Kalman filter can fullfill the given criteria with only sensor when there are more measurements available. 
However, the Kalman filter is not far away from RMSE criteria, depending on the type of application the positions and velocities maybe good enough.


| Dataset | Sensors         		|    RMSE x | RMSE y | RMSE vx | RMSE vy | 
|:---------------------:|:---------------------:|:---------------------:|:---------------------:|:---------------------:|:---------------------:|
| 1| Laser & Radar | 0.0973 | 0.0855 | 0.4513 | 0.4399 |
| 2| Laser & Radar | 0.0740 | 0.0964 | 0.4466 | 0.4756 |
| 1| Laser only | 0.1473| 0.1153 | 0.6383 | 0.5346 |
| 1| Radar only  | 0.2302 | 0.3464 | 0.5835 | 0.8040 |


