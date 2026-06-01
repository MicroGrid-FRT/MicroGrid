// File:          microgrid_drive.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;


int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  Motor *leftMotor =
      robot->getMotor("Left Motor");

  Motor *rightMotor =
      robot->getMotor("Right Motor");
      
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    
    leftMotor->setVelocity(3.0);
    rightMotor->setVelocity(3.0);
    
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
