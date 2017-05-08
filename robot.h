#include "robotfield.h"

class Robot
{
  public:
  explicit Robot();
  //Setters
  void 	initializeField(float x, float y, float orient, RobotField newfield);  //In some ways, this is the constructor
  void 	setPosition(float x, float y);
  void  setOrientation(float orient);
  void  setField(RobotField f);
  //Getters
  float getXPosition();
  float getYPosition();
  float getOrientation();
  RobotField getField();
  //Functions
  void  move(float dist, float steer, float motionError, float steeringError);
  double evaluateSensorReadingLikelihood(float sensorReadings[], int numSensors, float sensorSTDev);
  void  getSensorReadings(float sensorReadings[], int numSensors, float sensorError);
  
 private:
  float xPosition;
  float yPosition;
  float orientation;
  RobotField field;
  int	initialized;  //check if initializeField was called in order to prevent errors
};