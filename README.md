# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
[model]: ./model-update-equations.png "Model update equations"
[vehicle-state]: ./vehicle-state.png "Vehicle State"
[legend]: ./legend.png "Legend"
[lane-curve-equations]: ./lane-curve-equations.png "Lane Curvature Polynomial"
[cost-function]: ./cost-function.png "Cost Function"

## Implementation Review

#### State vector

![vehicle-state]

![legend.png]

### The Model

The model consists of:
* vehicle state composed on 2D postion, direction speed, and errors in lane-keeping and direction.
* A set of vehicle dynamics equations to adjust state
* A reference trajectory that specifies the desired path to follow
* A derived lane tracking third-degree polynomial that specifies the path to be followed to match the reference trajectory
* Actuators to control speed and direction
* A cost function to identify the correct path to follow to match the reference trajectory and correct behaviour while driving

The state update equations are as follows:

![model]

The polynomial f(x) referenced in the state maps the curve of the road ahead. In this case a third degree polynomial is used (see below).

![lane-curve-equations]

The cost function consists of the error in direction, the difference to the refernce velocity (set to 40 mph), the magnitude of the actuators (steering and throttle), the magnitude of successive actuator changes, and the cross-track error. The effects of cross-track error on the cost was reduced to avoid over-correction (oscillation) in the final motion. The actual proportion of CTE was found through experimentation.

![cost-function]

#### Actuators

The actuators are steering (delta) and throttle (a).

#### Vehicle Model Update Equations

![model]

### Timestep Length & Elapsed Duration

I settled on N=10 and dt = 0.05 (50 ms). This results in a resolution equal to half the loop delay (100ms) and a set of actuations for the next 0.5 seconds of travel.

### Polynomial Fitting and MPC Processing



### Handling MPC Latency

The algorithm was adjusted for latency as follows:

* First remove the delay from the loop and adjust the model to run smoothly several times around the track. 
* Adjust N, dt and alpha (CTE factor) to accomplish this. The result of this was N=10, dt=0.1 and alpha=0.05.
* At this point, the MPC model returns the first actuation of the series
* Next add the delay back into the loop and examine its influence on the dynamics. 
* After adding the delay, choose a future actuation (not the first in the series). The rationale is that since there is a delay in actuation, the current actuation is no longer valid. Therefore some future actuation must be use.
* The result of this was choosing the second actuation in the series of future actuations and setting dt=0.05. The initial choice of dt=delay (100ms or 0.1), did not effect the correct behaviour.


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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

