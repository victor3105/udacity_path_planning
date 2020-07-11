# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
---

## Goals

The goal of the project is to to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data are used.

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
