This is my submission for Project 4 of Term2 of Udacity's Self-Driving Car Engineer Nanodegree Program.
Here the objective is to implement the PID (Proportional Integral and Differential) controller to control the steering angle and throttle of a car given its cross track error (distance of the vehicles current postion from the desired center of the lane).

## Rubrik points
### Your code should compile.
The code can be compiled without error using cmake and make.
```
jagkin@jagkin-Inspiron-N5050:~/Projects/Udacity_ND/CarND-PID-Control-Project/build$ make clean && make
[ 33%] Building CXX object CMakeFiles/pid.dir/src/PID.cpp.o
/home/jagkin/Projects/Udacity_ND/CarND-PID-Control-Project/src/PID.cpp: In member function ‘void PID::Optimizer(double)’:
/home/jagkin/Projects/Udacity_ND/CarND-PID-Control-Project/src/PID.cpp:218:15: warning: comparison between signed and unsigned integer expressions [-Wsign-compare]
     if (index == (dK.size() - 1))
               ^
[ 66%] Building CXX object CMakeFiles/pid.dir/src/main.cpp.o
/home/jagkin/Projects/Udacity_ND/CarND-PID-Control-Project/src/main.cpp: In lambda function:
/home/jagkin/Projects/Udacity_ND/CarND-PID-Control-Project/src/main.cpp:98:18: warning: unused variable ‘angle’ [-Wunused-variable]
           double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                  ^
[100%] Linking CXX executable pid
[100%] Built target pid
```
### The PID procedure follows what was taught in the lessons.
The controller and optimizer (using twiddle) implementation in src/PID.cpp follows the procedure taught in lessons.

### Describe the effect each of the P, I, D components had in your implementation.
#### Effect of P
Of the three coefficients P is probably the most important, it controls the immediate response to the current error observed. Increasing it makes the controller responsive but has the side effect of making it unstable (to overshoot beyond the center line).
#### Effect of I
It was (and is) difficult to see the exact effect of I component. For steering angle, the sum of ctes over time probably converges to some non-zero value and I component probably helps to drive that value to 0 by nudging the controller response. For throttle controller the absolute CTE was fed as input so the sum of errors will only increase indefinitely, for this reason I component was not used for throttle controller.
#### Effect of D
D component helps prevent the overshoot caused by very high P component. Setting it to high value makes the system a bit slow to respond in case of sudden increase in error for example around the corners.

### Describe how the final hyperparameters were chosen.
#### Steering angle controller
Initial values for Kp, Ki and Kd were set based on trial and error by keepig other coefficients constants.
I started with Kp  +0.5 (and Ki = Kd = 0) as the simulator starts with vehicle on slightly right of the center and needs to be moved to left. But the car then steered too hard to the left, I then decreased it till 0.1 when car seemed to stay in the lane but was oscillating a lot eventually going off the track.
I then held Kp at 0.1 and played around with Kd to counter the effect of Kp and arrived at Kd = 0.5 to keep the car on the track for one lap. I then played around with Ki (starting with 0.1 and then 0.01 and the 0.0001 and then 0.0005 and finally 0.0006) and chose the value which seemed to keep the car around the center most f the time.

#### Steering angle controller
Initial Kp = 3.0 and Kd = 0.1 were choosen again by trial and error by choosing values which did not make the car go off the track while keeping the speed high enough.

#### Tuning using twiddle
Twiddle algorithm was implemented to tune the parameters further. The function Optimizer() in src/PID.cpp implements the algorithm. It chooses dKp/dKi/dKd to be about 30% of initial Kp/Ki/Kd and then tweaks them to make the error (actually absolute cte) to lowest value. For each step, the algoritm let to run around a lap without collecting any error info (to allow it to reach steady state) and then errors are collected for a lap and compared against best error so far.
The threshold is set to 5% of sum of dKs.

Both steering angle and throttle controller were tuned simultaneously. 
logs/final_tuning.log contains the console output during tuning.


### The vehicle must successfully drive a lap around the track.
The vehicle completed several laps around the track.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

