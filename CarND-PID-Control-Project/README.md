# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## About the project

In this project you'll revisit the lake race track from the Behavioral Cloning Project. This time, however, you'll implement a PID controller in C++ to maneuver the vehicle around the track!

The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

## Rubric Points

### *Student describes the effect of the P, I, D component of the PID algorithm in their implementation. Is it what you expected?*

* The **P (proportional)** component is the most important factor that influences the car's behavior. It corrects the steering angle to compensate the car's distance from the lane center (cross track error - CTE). The higher the magnitude of the CTE, the higher will be the steering value towards the lane center.

* The **D (differential)** component makes the car to approach the lane center smoothly and prevents the car from "wiggling" around the lane center. When using only the P and D components the car drives properly on straight lines and gentle curves, but fails on sharp turns.

* The **I (integral)** component accounts for bias in the driving system, such as a steering drift. I found this component very important in sharp turns, where the drifting effect is more noticeable.

Here are some example videos that show the difference between a PID and a P controller:

* [PID controller](https://youtu.be/3a5OGd0VbnA)
* [P controller](https://youtu.be/R5mhwg0tHHw)

### *Student discusses how they chose the final hyperparameters (P, I, D coefficients)*

The first step is to choose initial parameters manually. This is done by trial and error, but there are some [interesting heuristics](https://en.wikipedia.org/wiki/PID_controller#Manual_tuning) to guide this tuning process. The I coefficient is particularly challenging to tune, so it was set to zero (PD controller).

Once we have the initial coefficient values, we let the car drive and adjust the coefficients using the "Twiddle" algorithm (a.k.a. [coordinate descent](https://en.wikipedia.org/wiki/Coordinate_descent)). The Twiddle algorithm is called after a "settling" number of steps (= 100 steps) and every "twiddle period" steps (= 500 steps). The coefficients are updated until the sum of the increment vector ```dp``` is smaller than a tolerance value (= 0.01).

The throttle is also controlled by a PID controller with the same tuning strategies. There is a target speed defined and the error also takes into account the CTE:

```
double target_speed = 100.;
pid_throttle.UpdateError(speed - target_speed + speed * fabs(cte));
```

Note that CTE influence is proportional to speed (our car is more careful at high speeds).

After around 100,000 steps the **steering P, I, D coefficients** converged to values **0.0887991, 9.4545e-05, 0.957148** respectively. And the **throttle P, I, D coefficients** converged to values **0.0636811, 7.44242e-05, 0.705109** respectively. These coefficients allow the car to drive safely with top speeds around 75mph. With more careful parameter tuning higher speeds can be achieved.

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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
