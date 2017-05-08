#include "robotfield.h"


//----------------------------------
//CONSTRUCTOR
//----------------------------------

RobotField::RobotField()
{
    sizeX   = 100.0;
    sizeY   = 100.0;
    numSensors = 4;
    for (int k=0; k < numSensors; k++)
    {
        sensorX[k] = 0.0;
        sensorY[k] = 0.0;
    }

}

RobotField::RobotField(float x, float y, float X[4], float Y[4])
{
    sizeX   = x;
    sizeY   = y;
    numSensors = 4;
    for (int k=0; k < numSensors; k++)
    {
        sensorX[k] = X[k];
        sensorY[k] = Y[k];
    }
}


//----------------------------------
//GETTER
//----------------------------------

float RobotField::getSizeX(){
    return sizeX;
}

float RobotField::getSizeY(){
    return sizeY;
}

float RobotField::getSensorX(int x){
    return sensorX[x];
}

float RobotField::getSensorY(int y){
    return sensorY[y];
}