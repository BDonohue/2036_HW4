class RobotField
{
  public:
	explicit RobotField();
	RobotField(float x, float y, float X[4], float Y[4]);
	float getSizeX();			//returns the width of the arena
    float getSizeY();
    float getSensorX(int x);	//parameter x returns sensor[x] from the array, value needs to be between 0 and 3; returns x position of sensor
    float getSensorY(int y);	

  private:
  	float	sizeX;
	float	sizeY;
	int		numSensors;		
	float	sensorX[4];
	float	sensorY[4];
};
