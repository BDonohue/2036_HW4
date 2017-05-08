#include <iostream>
#include <cmath>    // necessary for sin() and cos()
#include <math.h>  
#include "robot.h"
#define PI2 6.283185307179586


//----------------------------------
//CONSTRUCTOR
//----------------------------------

Robot::Robot(){
  initialized = false;
}


//----------------------------------
//SETTER
//----------------------------------

void Robot::initializeField(float x, float y, float orient, RobotField newfield){
  setField(newfield);  //set field first, because setPosition needs field
  setPosition(x,y);
  setOrientation(orient);
  initialized = true;  //initialization is true
}

void Robot::setPosition(float x, float y){
  //check that the parameters are within range
  if(x < 0){
    x = 0;
  } else if(x > field.getSizeX()){
    x = field.getSizeX();
  }

  if(y < 0){
    y = 0;
  } else if(y > field.getSizeY()){
    y = field.getSizeY();
  }

  xPosition = x;
  yPosition = y;
}

void Robot::setOrientation(float orient){
  orientation = orient;
  //if the orientation is greater than 2pi, divide until it's less than 2pi
  orientation = fmod(orientation,PI2);
}

void Robot::setField(RobotField f){
  field = f;
}


//----------------------------------
//GETTER
//----------------------------------

float Robot::getXPosition(){
  return xPosition;
}

float Robot::getYPosition(){
  return yPosition;
}

float Robot::getOrientation(){
  return orientation;
}

RobotField Robot::getField(){
  return field;
}


//----------------------------------
//INPUT/OUTPUT
//----------------------------------

void Robot::move(float dist, float steer, float motionError, float steeringError)
{
  //dist is how far the bot is going to move, motion error is added onto dist
  //steer is how much the bot is going to rotate, steering error is added onto steer

  //set the new orientation
  setOrientation(getOrientation() + steer + steeringError);
  //get and set the new X and Y position
  int newX = getXPosition() + cos(getOrientation())*(dist+motionError);
  int newY = getYPosition() + sin(getOrientation())*(dist+motionError);
  setPosition(newX,newY);  
}

//this function will only be used by the particle bots
double Robot::evaluateSensorReadingLikelihood(float sensorReadings[], int numSensors, float sensorSTDev)
{
  //sensorReadings is readings from actual robot
  //numSensors is the number of sensors, always 4
  //sensorSTDev is an error that is added to the likelihood equation

  int dx;
  int dy;
  double mult;
  float noisevar = sensorSTDev * sensorSTDev;
  double likelihood = 1;
  //check if initialized
  if(!initialized){
    return -1;
  }
  
  //loop through each sensor, calculate it's likelihood
  int distError;
  for(int i = 0; i < numSensors; i++){
    //find the distance between the particle's position to the sensor
    dx = getXPosition() - field.getSensorX(i);
    dy = getYPosition() - field.getSensorY(i);
    distError = sqrt(pow(dx,2) + pow(dy,2));
    //find the difference between the actual robot's reading and the bot particle's reading
    distError = distError - sensorReadings[i];
    //calculate the likelihood for this sensor, and multiple to the other sensors
    mult = exp(-(distError*distError)/(2.0*noisevar)) / sqrt((PI2) * noisevar);  
    likelihood *= mult;
  }
  //likelihood will return a small number, magnitudes of 10^-double digits
  return likelihood;
}

//this function will only be used by the actual robot
void Robot::getSensorReadings(float sensorReadings[], int numSensors, float sensorError)
{
  //sensor readings is the array which stores the distance from the bot's position to each of the sensors
  //numSensors are the number of sensors, which is always 4
  //sensor error is added onto the end of the dist

  //check if initilized
  if(!initialized){
    return;
  }
  float dx;
  float dy;
  float dist;
  //calculate the distance from the Robot's sensor to each Beacon, store them in sensorReadings
  for(int i = 0; i < numSensors; i++){
    dx = getXPosition() - field.getSensorX(i);
    dy = getYPosition() - field.getSensorY(i);
    dist = sqrt(pow(dx,2) + pow(dy,2));
    dist += sensorError;
    sensorReadings[i] = dist;
  }
}