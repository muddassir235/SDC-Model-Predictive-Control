# Self-Driving Car Model Predictive Control

## Implementation

### Model

The vehicle model I have used is a simple kinematic model based on the following equations. It is similar to the bicycle Model that we were taught in Lesson 12.

I consider the
- **(x, y)** coordinates
- The orientation of the vehicle **(ψ)**
- the current velocity magnitude **(v)**
- Steering actuation **(δ)**
- The throttle actuation or acceleration **(a)**

So when the yaw rate is close to zero I use the following update equations.
```
x[t+1] = x[t] + v[t] * cos(ψ[t]) * dt
y[t+1] = y[t] + v[t] * sin(ψ[t]) * dt
ψ[t+1] = ψ[t] + (v[t] * δ[t] * dt) / Lf
v[t+1] = v[t] + a[t] * dt
```

And when it has a substantial value I use a different set of equation which consider that difference.

```
ψ'[t] = (v[t] * δ) / Lf
x[t+1] = x[t] + v[t] * (sin(ψ[t] + ψ'[t] * dt) - sin(ψ[t])) / ψ'[t]
y[t+1] = y[t] + v[t] * (cos(ψ[t]) - cos(ψ[t] + ψ'[t] * dt)) / ψ'[t]
```

For calculating the cross track error, as I am doing all calculations in vehicle coordinates I use the **y** coordinate for corresponding to the current **x** position as the cross track error. I do this using a third order polynomial fitted to the waypoints of given by the simulator.

```C++
coeffs = polyfit(waypoint_xs, waypoint_ys, 3)
cte = ployeval(coeffs, x) - y // x and y are zero in reference to the vehicle
```

For calculating the error in the orientation of the vehicle I also used the same third order polynomial and found the tangent to it at the position of the vehicle.
```C++
eψ = -atan(c1 + c2 * x + c3 * x^2) // -atan(f'(x[t))
```

For updating the cross track Error I use the following update equations.
```C++
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(eψ) * dt // f(x) is the 3rd order polynomial function
```

For Updating the eψ I use the following equations
```
eψ[t+1] = ψ[t] - atan(f'(x[t])) + v[t] * δ[t] * dt / Lf
```

The initial state values **(x0, y0, ψ0, v0, cte0, eψ0)** are then fed into an optimizer along with waypoints **Ipopt**. The optimizer predict a path 7 steps into the future with dt = 0.1 using our kinematic vehicle model.

The constraints on the model are as follows
- The steering actuation is constrained between **-25** to **25** degree or (**-0.436332** to **0.436332** radians).
- The throttle value is constrained between 1 and -1

The **cost function** for the optimizer is as follows:
```
cost = sum(cte) +
       sum(eψ) +
       sum(v - 35) +
       15 * sum(δ) +
       15 * sum(a) +
       1000 * sum(dδ) +
       10 * sum(da)

// Here 35 m/s is the velocity we want achieve
```

### Timestep Length and Elapsed Duration (N & dt)
For Elapsed duration I selected **0.1 seconds** as it matched with the Latency and was giving good results on the track. A lower value wasn't giving good results on high speeds nor where higher values, it was mainly selected via trial and error

For timesteps I initially started off with the values given in the quiz solution (25) but soon realized from the plot on the simulator that they were too many. Then I switched to 10-12 but still at high speeds the car was predicting too far in the horizon
so I finally settled for **7 steps** which gave a fair and reasonable prediction into the future.

### Polynomial fitting and MPC Pre-processing
First of all I converted the waypoints given by the simulator from global coordinates into local ones in order for allowing the plotting of the waypoints and easy use of model predictive control equations. I used the following equation for transforming each waypoint coordinate.  

```
let, dx = xg - x0 // xg: global x
let, dy = yg - y0 // yg: global y

x = dx * cos(-psi) - dy * sin(-psi)
y = dx * sin(-psi) + dy * cos(-psi)
```

A 3rd order polynomial is then fit to these waypoints
```
coeffs = ployfit(ptsx, ptsy, 3)
```

### Latency

The latency in the model predictive control is handled by predicting the state 100 milliseconds into the future before feeding the state into the solver, this helps us incorporate latency into our system

The the prediction is done using the model discussed above,

```
// yaw rate
ψ'[t] = (v[t] * δ) / Lf

// when yaw rate is close to zero
x[t+1] = x[t] + v[t] * cos(ψ[t]) * dt
y[t+1] = y[t] + v[t] * sin(ψ[t]) * dt

// when yaw rate is substantial
x[t+1] = x[t] + v[t] * (sin(ψ[t] + ψ'[t] * dt) - sin(ψ[t])) / ψ'[t]
y[t+1] = y[t] + v[t] * (cos(ψ[t]) - cos(ψ[t] + ψ'[t] * dt)) / ψ'[t]

ψ[t+1] = ψ[t] + (v[t] * δ[t] * dt) / Lf
v[t+1] = v[t] + a[t] * dt

// future cte and eψ
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(eψ) * dt
eψ[t+1] = ψ[t] - atan(f'(x[t])) + v[t] * δ[t] * dt / Lf
```

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
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
