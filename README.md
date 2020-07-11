# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
---

## Goals

The goal of the project is to to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data are used. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

---
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.
---
## Project Rubric Criteria

### Compilation
#### The code compiles correctly
Code compiles without errors with cmake and make.

### Valid Trajectories
#### The car is able to drive at least 4.32 miles without incident
This creterion is achieved by using:
* smooth trajectory generation using Spline library (lines 208-318 of main.cpp);
* trajectory predicion of the vehicles around us with use of sensor fusion data (lines 113-141 of main.cpp) and function
`checkLane` in helpers.h (lines 159-191);
* behavior planning which takes into account the predicted trajectories of the surrounding vehicles (lines 144-205 of main.cpp).
#### The car drives according to the speed limit
This is achieved by setting the maximum velocity (`ref_vel`) - lines 296-299 of main.cpp.
#### Max Acceleration and Jerk are not exceeded
This criterion is achieved by incrementing and decrementing the speed with the pre-calculated values (main.cpp, lines 294 and 298).
#### Car does not have collisions
This is done by using trajectory prediction of the other cars (lines 113-141 of main.cpp and lines 159-191 of helpers.h)
#### The car stays in its lane, except for the time between changing lanes
The coordinates are generated according to the desired lane (lines 244-246 of main.cpp)
#### The car is able to change lanes
The car can smoothly change lanes thanks to the Spline library. The lane change trajectories are generated in lines 244-246 of main.cpp. The decision to change lanes is made if we are close to the car in front of us inside our lane, if it is safe to change lane (which means that all the cars in the adjacent lane are far enough - at least 30 meters ahead or 15 meters behind - this is performed by `checkLane` function) and if that lane is faster or it is empty or the cars are at least 50 meters ahead (main.cpp, lines 144-205). Also, I exclude the possibility to change lane twice in a row for security reasons (lines 155-156, 191, 202 of main.cpp). The idea is that if we have moved to the middle lane, we have to go some time in that lane before considering next lane change.

### Reflection
Path generation should yield us a smooth trajectory with comfortable acceleration and jerk. In order to do that splines can be used. In this case we need to choose evenly spaced points along the road and take into account the previous 
path traveled. We need to break the distance into parts such way that it gives us our required speed.  
Besided it, path generation should be done taking into account behavior of the surrounding vehicles. Lane change should be safe and optimal (the target lane should be faster).

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
