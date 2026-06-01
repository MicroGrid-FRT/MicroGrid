// File:          MicroGrid_Mk1_SIM_Conroller_v1.cpp
// Date:          05/22/2026
// Description:   Webots controller for version(s): "MicroGrid_Mk1_SIM_RevA"
// Author:        Nándor Sümeghi
// Modifications: none

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Keyboard.hpp>
#include <cmath>
#include <iostream>

using namespace webots;


int main(int argc, char **argv) {

  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  //Device Declarations
  
  //Right Arm
  
  Motor *rightcMotor=
    robot->getMotor("Continuous_Motor_Right");
  Motor *rightfMotor=
    robot->getMotor("Fixed_Motor_Right");
    
  PositionSensor *rightcPos=
    robot->getPositionSensor("Continuous_Position_Right");
  PositionSensor *rightfPos=
    robot->getPositionSensor("Fixed_Position_Right");
    
  //Left Arm
  
  Motor *leftcMotor=
    robot->getMotor("Continuous_Motor_Left");
  Motor *leftfMotor=
    robot->getMotor("Fixed_Motor_Left");
    
  PositionSensor *leftcPos=
    robot->getPositionSensor("Continuous_Position_Left");
  PositionSensor *leftfPos=
    robot->getPositionSensor("Fixed_Position_Left");
    
  //Body
  
  Gyro *bodyGyro=
    robot->getGyro("Body_Gyro");
  Accelerometer *bodyAccelerometer=
    robot->getAccelerometer("Body_Accelerometer");
    
  //External
  
  Keyboard *keyboard=
    robot->getKeyboard();
  
  //Device Initiallizations
  rightcMotor->setPosition(INFINITY);
  leftcMotor->setPosition(INFINITY);
  
  rightfPos->enable(timeStep);
  leftfPos->enable(timeStep);

  rightcMotor->setVelocity(0.0);
  leftcMotor->setVelocity(0.0);
  rightfMotor->setVelocity(3);
  leftfMotor->setVelocity(3);
  
  keyboard->enable(timeStep);

  // Main loop:
  while (robot->step(timeStep) != -1) {
  
    //Only input based control for now
    int key;
    
    while((key=keyboard->getKey())!=-1)
    {    
      switch(key)
      {
        case 'E':
        {
          if (leftcMotor->getVelocity()<10) leftcMotor->setVelocity(3);
          break;
        }
        case 'D':
        {
          if (leftcMotor->getVelocity()>-10) leftcMotor->setVelocity(-3);
          break;
        }
        case 'A':
        {
          if (leftfPos->getValue()<1.396) leftfMotor->setPosition(leftfPos->getValue()+0.0157);
          break;
        }
        case 'W':
        {
          if (leftfPos->getValue()>-1.396) leftfMotor->setPosition(leftfPos->getValue()-0.0157);
          break;
        }
        case 'V':
        {
          leftcMotor->setVelocity(0);
          leftfMotor->setPosition(0);
          break;
        }
        case 'P':
        {
          if (rightcMotor->getVelocity()<10) rightcMotor->setVelocity(3);
          break;
        }
        case 'L':
        {
          if (rightcMotor->getVelocity()>-10) rightcMotor->setVelocity(-3);
          break;
        }
        case '[':
        {
          if (rightfPos->getValue()<1.396) rightfMotor->setPosition(rightfPos->getValue()+0.0157);
          break;
        }
        case '\'':
        {
          if (rightfPos->getValue()>-1.396) rightfMotor->setPosition(rightfPos->getValue()-0.0157);
          break;
        }
        case 'M':
        {
          rightcMotor->setVelocity(0);
          rightfMotor->setPosition(0);
          break;
        }
      }
    }
      
  }


  delete robot;
  return 0;
}
