[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

# The Project
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project I implemented a 2 dimensional particle filter in C++. My particle filter is given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter also gets observation and control data.

[//]: # (Image References)

[image1]: ./images/simulator.png "Simaluator"

# Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

# Interaction with the Term 2 Simulator
Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator
* ["sense_x"]
* ["sense_y"]
* ["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state
* ["previous_velocity"]
* ["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values
* ["sense_observations_x"]
* ["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation
* ["best_particle_x"]
* ["best_particle_y"]
* ["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label
* ["best_particle_associations"]

// for respective (x,y) sensed positions
* ["best_particle_sense_x"] <= list of sensed x positions
* ["best_particle_sense_y"] <= list of sensed y positions



# Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

## The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

## All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.


# Code Style
I tried to stick to the [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
In order to check the guidelines I installed cpplint using 
`pip install cpplint`


# Results
```
Success! Your particle filter passed!
```

The simulator visualizes the particle filter:
![alt text][image1]




