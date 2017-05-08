//use g++ -std=c++11 -o main particlefilt.cpp robot.cpp robotfield.cpp robotfield.h robot.h
#include <iostream>		// provide cout, etc.
#include <limits>       // has the RAND_MAX value
#include <time.h>       // include time functions for random number
						//    initialization 
#include <cstdlib>  	// contains function prototype for rand()
#include <random>		// A more powerful random number generator class
						//    for Gaussian random numbers
#include "robot.h"		// Our robot class, used for both the "actual"
						//    robot and the particles
#define N_PARTICLES 	500
#define N_SENSORS       4
#define ITERATIONS  	10
#define FIELDX      	100.0
#define FIELDY      	100.0
#define PRINT       	true
#define SENSOR_STDEV    8.0
#define MOTION_STDEV    1.0
#define STEERING_STDEV  0.1

using namespace std;	// provide easy access to cout, cin, etc.

// two non-class utility functions to be defined later in this file
void printPositions(Robot actualRobot, Robot botParticle[], RobotField robotField, int N);
void resample(Robot botParticle[], Robot botParticleTemp[], int N, double w[]);

int main()
{
  // Define variables for utility use
  int	k, its;         // loop counters
  float	newX;           // temporary variables for initializing particles
  float	newY;
  float	newO;
  float xavg;           // for evaluating performance at end
  float yavg;

  // Define random number generators for Gaussian-distributed random numbers.
  // The following are defined in <random>.
  default_random_engine     	    generator;			    // set up the base random number generator object
  normal_distribution<double>	    motionNoise(0.0, MOTION_STDEV); // an object to return Gaussian random numbers based on generator
                                                                    //     parameters: mean, standard_deviation
  normal_distribution<double>     sensorNoise(0.0, SENSOR_STDEV);   // the sensor reading noise
  normal_distribution<double>     steeringNoise(0.0, STEERING_STDEV); // the steering noise
  // Define the sensor positions to be within 10 units of each corner (in the x, y directions)
  float	sensorArrayX[N_SENSORS] = {10.0, 10.0, FIELDX-10.0, FIELDX-10.0};   // x coefficients
  float	sensorArrayY[N_SENSORS]	= {10.0, FIELDY-10.0, 10.0, FIELDY-10.0};   // y coefficients
  //readings contain the readings from the actualy robot on how far away each sensor is to the bot
  float	readings[N_SENSORS];                                                
  //This contains the infomation on how big the grid is and where the sensors are on the grid
  RobotField  robotField((float) FIELDX, (float) FIELDY, sensorArrayX, sensorArrayY);
  //"Our" robot.  We want to find out it's position in the grid
  Robot	actualRobot;
  //The bot particles are the robots that are "simulated", this average position of the bots should resemble those of the actualrobot and several iterations
  Robot	botParticle[N_PARTICLES];
  //Temp is used to hold values when re-initiating bot-particles
  Robot	botParticleTemp[N_PARTICLES];
  //W is the likelihood of a particle representing the real robot
  double w[N_PARTICLES];


  //----------------------------------
  //Intialize the actual robot and the particles
  //----------------------------------

  srand(time(NULL));     // initialize random seed
  newX = rand() % static_cast<int>(robotField.getSizeX());     // get random coordinates
  newY = rand() % static_cast<int>(robotField.getSizeY());
  newO = ((rand() % 628) / 100.0);                        // and a random orientation
  //Give the robot a random location in the field
  actualRobot.initializeField(newX, newY, newO, robotField);
  //Do the same for all the particles
  for (k = 0;  k < N_PARTICLES; k++)
  {
    newX = rand() % static_cast<int>(robotField.getSizeX());
    newY = rand() % static_cast<int>(robotField.getSizeY());
    newO = ((rand() % 628) / 100.0);
    botParticle[k].initializeField(newX, newY, newO, robotField);
  }
  //Print out the intial map
  printPositions(actualRobot,botParticle,robotField,N_PARTICLES);
  

  //----------------------------------
  //Filter the particles
  //----------------------------------

  //Run the filtering through many iterations in order to get a more accurate answer
  for (its = 0; its < ITERATIONS; its++)     
  {
    //Move the robot to a new position on the map, calculate the distance to each sensor
    actualRobot.move(2.0,0.2, motionNoise(generator), steeringNoise(generator));
    actualRobot.getSensorReadings(readings, N_SENSORS, sensorNoise(generator));
    //Loop through the bot particles and calculate each particles likelihood
    for (k = 0; k < N_PARTICLES; k++)
	{
	  w[k] = botParticle[k].evaluateSensorReadingLikelihood(readings, N_SENSORS, SENSOR_STDEV);
	}
    //Resample the particles to receive new ones, and display them
    resample(botParticle, botParticleTemp, N_PARTICLES, w);
    printPositions(actualRobot, botParticle, robotField, N_PARTICLES);
	//Move the bot particles the same way the actual robot moved, that way they will scatter, giving us more data
    for(k = 0; k < N_PARTICLES; k++)
    {
      botParticle[k].move(2.0,0.2, motionNoise(generator), steeringNoise(generator));
    }
  }


  //----------------------------------
  //Examine final results
  //----------------------------------

  xavg = 0.0;
  yavg = 0.0;
  for (k = 0; k < N_PARTICLES; k++)
  {
    // sum up the x and y values from all particles (before final motion)
    xavg += botParticleTemp[k].getXPosition(); 
    yavg += botParticleTemp[k].getYPosition();
  }
  xavg = xavg / N_PARTICLES;                  // and calculate averages
  yavg = yavg / N_PARTICLES;
  // Print results and quit!
  cout << "actual position:    " << actualRobot.getXPosition() << ", " << actualRobot.getYPosition() << endl;
  cout << "predicted position: " << xavg << ", " << yavg << endl;
  return 0;
}


//----------------------------------
//Resample function
//----------------------------------

//botParticle[] is the simulated robots
//botParticleTemp[] is an empty array in order to switch values within simulated robots
//N is the number of particles
//W is an array of N size with the likelihood of each particle
void resample(Robot botParticle[], Robot botParticleTemp[], int N, double w[])
{
  float       maxW = 0.0;
  float       step;
  int         index;
  int         randMax;
  //Find the maximum likelihood of all the particles
  for(int i = 0; i < N; i++){
    if(maxW < w[i]){
      maxW = w[i];
    }
  }
  //The starting index is a ranodom number between 0 and N-1
  randMax = N - 1;
  index = rand() % randMax;
  // loop N times over k to randomly select particles
  for(int i = 0; i < N; i++){
    //Choose a random value between 0 and 2 * max of the likelihood
    step = (static_cast<float>(rand())/RAND_MAX) * 2.0 * maxW;
    //Loop through values until the step runs out; this is the randomly choosing a side of the circle portion 
    while(step > w[index]){
      step = step - w[index];
      index = (index + 1) % N;
    }
    //Store the newly choosen particle in botParticlTemp
    botParticleTemp[i] = botParticle[index];
  }
  //Put the botParticleTemp back into the botParticles
  for(int i = 0; i < N; i++){
  	botParticle[i] = botParticleTemp[i];
  }
}


//----------------------------------
//Print the array on terminal
//----------------------------------

#define COLS 76
#define ROWS 22

void printPositions(Robot actualRobot,Robot botParticle[], RobotField robotField, int N)
{
  int r;
  int c;
  int x;
  int y;
  float xscale;
  float yscale;

  int putdot;
  int k;

  float maxR;
  float maxC;

  if (!PRINT)
    return;

  maxR = robotField.getSizeX();
  maxC = robotField.getSizeY();

  xscale = COLS / maxC;
  yscale = ROWS / maxR;

  cout << "+";
  for (c = 0; c < COLS; c++)
    cout << "-";
  cout << "+" << endl;
  for (r = 0; r < ROWS; r++)
    {
      cout << "|";
      for (c = 0; c < COLS; c++)
	  {
	    x = static_cast<int> (actualRobot.getXPosition() * xscale);
	    y = static_cast<int> (actualRobot.getYPosition() * yscale);
	    if ((x == c) && (y == r))
	    {
	      cout << "O";
	    } else {
	    putdot = 0;
	    for (k = 0; k < N; k++)
	    {
		  x = static_cast<int> (botParticle[k].getXPosition() * xscale);
		  y = static_cast<int> (botParticle[k].getYPosition() * yscale);
		  if ((x == c) && (y == r))
		  {
		    putdot++;
		  }
	    }
	    switch (putdot)
	    {
	      case 0 :
		    cout << " ";
		    break;
	      case 1 :
		    cout << ".";
		    break;
	      case 2 :
		    cout << ":";
		    break;
	      case 3 :
		    cout << "=";
		    break;
	      case 4 :
		    cout << "%%";
		    break;
	      default :
		    cout << "#";
		    break;
	    }
	  }
	}
      cout << "|" << endl;
    }
  cout << "+";
  for (c = 0; c < COLS; c++)
    cout << "-";
  cout << "+" << endl << endl << endl;
}