# Car PID Controls

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

*made by [CJ](https://github.com/vssrcj)*

# Overview.

This project uses C++ to create a Proportional-Integral-Derivative Controller (PID) in order to drive a simulated car around a virtual track.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

# Results.

## P (proportional).

Only the P parameter is set in this video:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=8wJTvivkoyY" target="_blank">
 <img src="http://img.youtube.com/vi/8wJTvivkoyY/0.jpg" 
  alt="IMAGE ALT TEXT HERE" height="300"
 />
</a>

It is the most important parameter - it steers in the direction and in proportion to the distance from the center of the lane.
It does overshoot a lot though, and approaches the center smoothly.

## PD (proportional & differential)

The P and D parameters are set in this video:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=Lc1gx-2a1qg" target="_blank">
 <img src="http://img.youtube.com/vi/Lc1gx-2a1qg/0.jpg" 
  alt="IMAGE ALT TEXT HERE" height="300"
 />
</a>

The differential parameter is used no mitigate the overshooting.  This results in a much smoother approach to the center.

## PID (proportional & differential & integral)

The P, I, and D parameters are set in this video:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=JFefsbsfGYY" target="_blank">
 <img src="http://img.youtube.com/vi/JFefsbsfGYY/0.jpg" 
  alt="IMAGE ALT TEXT HERE" height="300"
 />
</a>

The integral parameter acts to counteract the bias.  It seems to help the car better steer around corners.

# Twiddle

Twiddle is a way to optimize all coefficients/parameters.

The result obtained are:
* Kp: 2.6
* Kd: 128
* Ki: 0.18

If you use these parameters, the car approaches the line rapidly.  It does it too well, and results in a jarring experience.

If you smooth these results (make them proportionally smalle), you get:
* Kp: 0.052
* Kd: 2.56
* Ki: 0.0035

Which provides the best experience:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=13qPcwBriBw" target="_blank">
 <img src="http://img.youtube.com/vi/13qPcwBriBw/0.jpg" 
  alt="IMAGE ALT TEXT HERE" height="300"
 />
</a>

This car moves around the track with a (0.5) throttle without any problems.
