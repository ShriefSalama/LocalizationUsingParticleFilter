# Overview
In this project we will use 2 dimensional particle filter in C++ to localize a car in a known map. 

## Project Introduction
### Inputs
- Initial noisy readings from GPS.
- Known map landmarks.
- Periodic observations.
- Periodic control dat,Ta.

### Outputs
- X,Y and Theta error between best particle and the actual car position.

## Filter Theory 
Particle filter theory is very easy to understand, Simply you have you car in a known map, and you receive periodic observations from the sensors equipped on this car. What you do is to assume that there's 

## Filter Implementation 
### Initialization 
in init state we shall define number of perticles which will affect our accuracy and filter speed, also we shall initialize each particle with its init position and heading (theta). 
Normally since we have initial GPS reading, we use it to define roughly each particle position.

### Predicition
As we receive periodic control data, which is Velocity and Yaw rate, we use it to update each particle position, So If we knew that Car moved one meter forward, then we should move all particle to one meter forward. 

### Weights' Updating 
Weight of each particle represents how much does this particle match all observation received from the actual car. 
In order to calculate it we need to do the following steps :
#### Observation transformation 
we are receiving observations/sensors reading from our car in CAR coordinate, in other words, it states that there an object in front of the car with 2 meters for example, so reading is (2,0).
Since we know the exact location (in MAP coordinate) and heading of our particles, then we will set this observation (2 meters in front of car) to each particle. this operation is called Observation transformation. 
Example : particle is located in position (3,4) from MAP reference and heading to north (upward), reading 2 meters in front of car means it will be in position (3,6) with respect to this particle. 

#### Observation Association 

#### Multi-variate gaussian distribution 

### Resampling

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter
