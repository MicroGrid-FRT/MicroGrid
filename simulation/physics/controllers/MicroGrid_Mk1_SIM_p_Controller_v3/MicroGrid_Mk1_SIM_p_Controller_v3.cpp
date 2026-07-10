// File:          MicroGrid_Mk1_SIM_p_Conroller_v3.cpp
// Date:          21/06/2026
// Description:   Webots controller for version(s): "MicroGrid_Mk1_SIM_RevD";
// Author:        Nándor Sümeghi
// Modifications: Introduce connectors

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
//#include <webots/Gyro.hpp>
//#include <webots/Accelerometer.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Connector.hpp>
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

  Connector *rightConnector =
    robot->getConnector("Right_Connector");

  //Left Arm

  Motor *leftcMotor=
    robot->getMotor("Continuous_Motor_Left");
  Motor *leftfMotor=
    robot->getMotor("Fixed_Motor_Left");

  PositionSensor *leftcPos=
    robot->getPositionSensor("Continuous_Position_Left");
  PositionSensor *leftfPos=
    robot->getPositionSensor("Fixed_Position_Left");

   Connector *leftConnector =
    robot->getConnector("Left_Connector");

  //Body

  /*Gyro *bodyGyro=
    robot->getGyro("Body_Gyro");
  Accelerometer *bodyAccelerometer=
    robot->getAccelerometer("Body_Accelerometer");*/

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

  leftConnector->enablePresence(timeStep);
  rightConnector->enablePresence(timeStep);

  keyboard->enable(timeStep);

  // Main loop:
  while (robot->step(timeStep) != -1) {

    //Only input based control for now
    int key;

    //Left was pressed, right was pressed
    bool lwp=false;
    bool rwp=false;

    while((key=keyboard->getKey())!=-1)
    {
      switch(key)
      {
        case 'E':
        {
          if(rightcMotor->getVelocity()<9 && leftcMotor->getVelocity()<9)
          {
            rightcMotor->setVelocity(rightcMotor->getVelocity()+0.1);
            leftcMotor->setVelocity(leftcMotor->getVelocity()+0.1);
          }
          break;
        }
        case 'D':
        {
          if(rightcMotor->getVelocity()>-9 && leftcMotor->getVelocity()>-9)
          {
            rightcMotor->setVelocity(rightcMotor->getVelocity()-0.1);
            leftcMotor->setVelocity(leftcMotor->getVelocity()-0.1);
          }
          break;
        }
        case 'R':
        {
          if(rightfPos->getValue()<1.335)
          rightfMotor->setPosition(rightfPos->getValue()+0.0523);
          break;
        }
        case 'F':
        {
          if(rightfPos->getValue()>-1.335)
          rightfMotor->setPosition(rightfPos->getValue()-0.0523);
          break;
        }
        case 'W':
        {
          if(leftfPos->getValue()>-1.335)
          leftfMotor->setPosition(leftfPos->getValue()-0.0523);
          break;
        }
        case 'S':
        {
          if(leftfPos->getValue()<1.335)
          leftfMotor->setPosition(leftfPos->getValue()+0.0523);
          break;
        }
        case 'A':
        {
          lwp=true;
          break;
        }
        case 'G':
        {
          rwp=true;
          break;
        }
        case 'L':
        {
          if (rightConnector->getPresence())
          {
            rightConnector->lock();
            if (rightConnector->isLocked()) std::cout<<"right connect"<<std::endl;
          }
          break;
          //else rightConnector->unlock();
        }
        case 'O':
        {
          rightConnector->unlock();
          break;
        }
        case 'J':
        {
          if (leftConnector->getPresence())
          {
            leftConnector->lock();
            if (leftConnector->isLocked()) std::cout<<"left connect"<<std::endl;
          }
          //else leftConnector->unlock();
          break;
        }
        case 'U':
        {
          leftConnector->unlock();
          break;
        }
      }
    }

    float lvel=leftcMotor->getVelocity();
    float rvel=rightcMotor->getVelocity();
    //std::cout<<lvel<<std::endl;
    if((lwp && rwp) || (!lwp && !rwp))
    {
      if(lvel>rvel) lvel=rvel;
      else rvel=lvel;
    }
    else if(lwp && lvel == rvel) lvel+=0.25;
    else if(rwp && rvel == lvel) rvel+=0.25;

    rightcMotor->setVelocity(rvel);
    leftcMotor->setVelocity(lvel);

  }


  delete robot;
  return 0;
}
