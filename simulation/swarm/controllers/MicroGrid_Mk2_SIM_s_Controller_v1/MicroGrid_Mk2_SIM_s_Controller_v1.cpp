// File:          MicroGrid_Mk1_SIM_Controller_v3.cpp
// Date:          06/08/2026
// Description:   Webots controller for version(s): "MicroGrid_Mk1_SIM_RevC"
// Author:        Nándor Sümeghi
// Modifications: Swarm behaviour overhaul

#include <webots/Supervisor.hpp>
#include <math.h>
#include <stdexcept>
#include <array>
#include <ctime>
#include <string>
using namespace webots;


//Robot strucutre
struct RobotData
{
    Node *robot;
    Field *translation;
    Field *rotation;
    Field *arm1rotation;
    Field *arm2rotation;
    Field *arm1position;
    Field *arm2position;

    //place in world
    double x;
    double y;
    double z;
    double xGoal;
    double yGoal;

    //direction in plane
    double heading;
    double heading_goal;

    //body roll position
    double roll;

    //arm states
    double arm1con; //axis through the body
    double arm1fix;
    double arm1fix_goal;
    double arm2con; //axis through the body
    double arm2fix;
    double arm2fix_goal;

    //motion
    double speed;
    double speed_goal;

    //important informations
    //r=0.1
    //arm length=0.3
    //arm radius=0.06

    //constructors
    RobotData(Supervisor &supervisor, const std::string &defname) :
    x(0), y(0), z(0.1), xGoal(0), yGoal(0), heading(0), heading_goal(0), roll(0), arm1con(0.14), arm1fix(-1), arm1fix_goal(-1), arm2con(0.14), arm2fix(-1), arm2fix_goal(-1), speed(0), speed_goal(0)
    {
        robot=supervisor.getFromDef(defname);
        if (!robot) throw std::runtime_error("Robot DEF '" + defname + "' not found.");

        translation=robot->getField("translation");
        rotation=robot->getField("rotation");

        if (!translation) throw std::runtime_error("translation field missing");
        if (!rotation) throw std::runtime_error("rotation field missing");

        Field *children=robot->getField("children");
        Node *joint1 = children->getMFNode(0);
        Node *joint2 = children->getMFNode(3);

        Node *arm1 = joint1->getField("endPoint")->getSFNode();
        Node *arm2 = joint2->getField("endPoint")->getSFNode();
        if (!joint1 || !joint2) throw std::runtime_error("joint node missing");

        arm1rotation=arm1->getField("rotation");
        arm2rotation=arm2->getField("rotation");
        if (!arm1rotation || !arm2rotation) throw std::runtime_error("arm rotation field missing");

        arm1position=arm1->getField("translation");
        arm2position=arm2->getField("translation");
        if (!arm1position || !arm2position) throw std::runtime_error("arm translation field missing");
    }

    RobotData(Node *node) :
    x(0), y(0), z(0.1), xGoal(0), yGoal(0), heading(0), heading_goal(0), roll(0), arm1con(0.14), arm1fix(-1), arm1fix_goal(-1), arm2con(0.14), arm2fix(-1), arm2fix_goal(-1), speed(0), speed_goal(0)
    {
        robot = node;

        if (!robot) throw std::runtime_error("Null robot node.");

        translation = robot->getField("translation");
        rotation = robot->getField("rotation");

        if (!translation) throw std::runtime_error("translation field missing");

        if (!rotation) throw std::runtime_error("rotation field missing");

        Field *children = robot->getField("children");

        Node *joint1 = children->getMFNode(0);
        Node *joint2 = children->getMFNode(3);

        Node *arm1 = joint1->getField("endPoint")->getSFNode();
        Node *arm2 = joint2->getField("endPoint")->getSFNode();

        arm1rotation = arm1->getField("rotation");
        arm2rotation = arm2->getField("rotation");

        if (!arm1rotation || !arm2rotation) throw std::runtime_error("arm rotation field missing");

        arm1position=arm1->getField("translation");
        arm2position=arm2->getField("translation");
        if (!arm1position || !arm2position) throw std::runtime_error("arm translation field missing");
    }

    //functions
    void manualMove(double px, double py, double pz)
    {
        x=px;
        y=py;
        z=pz;
    }

    void stepMove(double dt)
    {
        double distance=speed*dt;

        x+=distance*sin(-heading);
        y+=distance*cos(-heading);

        roll-=distance/0.1;
        arm1con+=distance/0.1;
        arm2con+=distance/0.1;
    }

    void stepTurn(double dt)
    {
        heading+=dt*(arm1fix-arm2fix);
    }

    std::array<double,3> position() const
    {
        return {x, y, z};

    }

    std::array<double,4> rotationAxisAngle() const
    {
        double w = cos(heading/2) * cos(roll/2);
        double x = cos(heading/2) * sin(roll/2);
        double y = sin(heading/2) * sin(roll/2);
        double z = sin(heading/2) * cos(roll/2);

        double angle = 2*acos(w);

        w=sqrt(std::max(1e-12, 1-w*w));

        if (w < 1e-6 || angle < 1e-6)
        {
            std::cout<<"DEBUG body: x="<<1<<", y="<<0<<", z="<<0<<", angle="<<0<<std::endl;
            return {1.0, 0.0, 0.0, 0.0};  // any valid axis is fine when angle = 0
        }

        std::cout<<"DEBUG body: x="<<x/w<<", y="<<y/w<<", z="<<z/w<<", angle="<<angle<<std::endl;

        return {x/w, y/w, z/w, angle};
    }

    std::array<double,4> arm1RotationAxisAngle()
    {
        if(arm1fix>M_PI*4/3) arm1fix=M_PI*4/3;
        else if(arm1fix<-M_PI*4/3) arm1fix=-M_PI*4/3;

        double w = cos(-arm1fix/2) * cos(arm1con/2);
        double x = cos(-arm1fix/2) * sin(arm1con/2);
        double y = -sin(-arm1fix/2) * sin(arm1con/2);
        double z = sin(-arm1fix/2) * cos(arm1con/2);

        double angle = 2 * acos(w);

        w = sqrt(std::max(1e-12, 1 - w*w));

        if (w < 1e-6 || angle < 1e-6)
        {
            std::cout<<"DEBUG arm1: x="<<1<<", y="<<0<<", z="<<0<<", angle="<<0<<std::endl;
            return {1.0, 0.0, 0.0, 0.0};  // any valid axis is fine when angle = 0
        }

        std::cout<<"DEBUG arm1: x="<<x/w<<", y="<<y/w<<", z="<<z/w<<", angle="<<angle<<std::endl;

        return {x/w, y/w, z/w, angle};
    }

    std::array<double,4> arm2RotationAxisAngle()
    {
        if(arm2fix>M_PI*4/3) arm2fix=M_PI*4/3;
        else if(arm2fix<-M_PI*4/3) arm2fix=-M_PI*4/3;

        double w = cos(arm2fix/2) * cos(arm2con/2);
        double x = cos(arm2fix/2) * sin(arm2con/2);
        double y = -sin(arm2fix/2) * sin(arm2con/2);
        double z = sin(arm2fix/2) * cos(arm2con/2);

        double angle = 2 * acos(w);

        w = sqrt(std::max(1e-12, 1 - w*w));

        if (w < 1e-6 || angle < 1e-6)
        {
            std::cout<<"DEBUG arm2: x="<<1<<", y="<<0<<", z="<<0<<", angle="<<0<<std::endl;
            return {1.0, 0.0, 0.0, 0.0};  // any valid axis is fine when angle = 0
        }

        std::cout<<"DEBUG arm2: x="<<x/w<<", y="<<y/w<<", z="<<z/w<<", angle="<<angle<<std::endl;

        return {x/w, y/w, z/w, angle};
    }

    std::array<double,3> armPositionReset(double x) const
    {
        return {x, 0, 0};
    }

    void applyData()
    {
        translation->setSFVec3f(position().data());
        rotation->setSFRotation(rotationAxisAngle().data());
        arm1rotation->setSFRotation(arm1RotationAxisAngle().data());
        arm2rotation->setSFRotation(arm2RotationAxisAngle().data());
        arm1position->setSFVec3f(armPositionReset(-0.16).data());
        arm2position->setSFVec3f(armPositionReset(0.16).data());
    }

    void toGoal(double dt)
    {
        if (speed_goal > 5.0) speed_goal = 5.0;
        else if (speed_goal < -5.0) speed_goal = -5.0;

        // ---------------- ROTATION CONTROL ----------------
        const double speedStep = 0.02;

        double speedError = speed_goal - speed;

        if (speedError > speedStep)
            speed += speedStep;
        else if (speedError < -speedStep)
            speed -= speedStep;
        else
            speed = speed_goal;

        if (speed<1) return;

        // ---------------- HEADING ERROR ----------------
        double error = heading_goal - heading;

        while (error > M_PI)  error -= 2.0 * M_PI;
        while (error < -M_PI) error += 2.0 * M_PI;

        // ---------------- CURRENT TURN RATE ----------------
        double currentTurn = arm1fix - arm2fix;

        // ---------------- DIRECT TURN RATE CONTROL ----------------
        // This is the key change: we control angular velocity, not "steering"
        const double Kp = 3.0;
        const double Kd = 2.0;

        double desiredTurn =
            Kp * error
            - Kd * currentTurn;

        // limit turn (important for stability)
        const double maxTurn = 2.5;

        if (desiredTurn > maxTurn) desiredTurn = maxTurn;
        if (desiredTurn < -maxTurn) desiredTurn = -maxTurn;

        // ---------------- ARM CONTROL (VERY IMPORTANT CHANGE) ----------------
        // Instead of slow incremental chasing, we directly move toward target
        double armError = desiredTurn - currentTurn;

        const double armGain = 0.25;

        double correction = armGain * armError;

        arm1fix += correction * 0.5;
        arm2fix -= correction * 0.5;

        // ---------- ARM LIMITS ----------
        const double armLimit = M_PI * 3.0 / 4.0;

        if (arm1fix > armLimit) arm1fix = armLimit;
        else if (arm1fix < -armLimit) arm1fix = -armLimit;

        if (arm2fix > armLimit) arm2fix = armLimit;
        else if (arm2fix < -armLimit) arm2fix = -armLimit;
    }

};
/*
to set turn, convert to quaternions:
 w = cos(heading/2) * cos(turn/2)
 x = cos(heading/2) * sin(turn/2)
 y = sin(heading/2) * sin(turn/2)
 z = sin(heading/2) * cos(turn/2)
then to axis-angle:
angle = 2*arccos(w)
s = sqrt(1 - w * w)
x = x/s
y = y/s
z = z/s
*/


std::vector<RobotData> spawnRobots( Supervisor &supervisor, int count, double goalX, double goalY, double worldRadius = 10, double minDistance = 0.7)
{
    std::vector<RobotData> robots;

    Node *prototype = supervisor.getFromDef("MICROGRID");

    if (!prototype) throw std::runtime_error("DEF MICROGRID not found.");

    std::string protoString = prototype->exportString();

    Node *root = supervisor.getRoot();
    Field *children = root->getField("children");

    // First robot already exists
    robots.emplace_back(prototype);

    // Create clones
    for (int i = 1; i < count; i++)
    {
        std::string defName = "MICROGRID_" + std::to_string(i);

        std::string robotString = "DEF " + defName + " " + protoString.substr(protoString.find("Robot"));

        children->importMFNodeFromString(-1, robotString);

        Node *newNode =
            children->getMFNode(children->getCount() - 1);

        robots.emplace_back(newNode);
    }

    // Random placement with minimum spacing
    for (auto &r : robots)
    {
        bool valid = false;

        while (!valid)
        {
            valid = true;

            double rx = (2.0 * rand() / double(RAND_MAX) - 1.0) * worldRadius;

            double ry = (2.0 * rand() / double(RAND_MAX) - 1.0) * worldRadius;

            for (const auto &other : robots)
            {
                if (&other == &r)
                    break;

                double dx = rx - other.x;
                double dy = ry - other.y;

                if (sqrt(dx * dx + dy * dy) < minDistance)
                {
                    valid = false;
                    break;
                }
            }

            if (valid)
            {
                r.manualMove(rx, ry, 0.1);
                r.xGoal=goalX;
                r.yGoal=goalY;

                r.heading = 2.0 * M_PI * rand() / double(RAND_MAX);

                r.heading_goal = r.heading;

                r.applyData();
            }
        }
    }

    return robots;
}


void updateVicsekGoal(RobotData& robot, const std::vector<RobotData>& robots, double radius)
{
    double vx = 0.0;
    double vy = 0.0;
    robot.heading_goal=0;
    int count=-1;

    for(const auto& other : robots)
    {
        double dx = other.x - robot.x;
        double dy = other.y - robot.y;

        double dist2 = dx*dx + dy*dy;

        if(dist2 <= radius*radius)
        {
            robot.heading_goal+=other.heading_goal;
        }
        count++;
    }

    double vicsekHeading = atan2(vy, vx);

    double gx = robot.xGoal - robot.x;
    double gy = robot.yGoal - robot.y;

    double goalHeading = atan2(gy, gx);

    double distance = sqrt(gx*gx + gy*gy);

    double goalWeight;
    double swarmWeight=0.3;
    if(count>5) swarmWeight/=2;
    goalWeight=sqrt(gx*gx + gy*gy)/100;
    if (goalWeight>0.9) goalWeight=0.9;
    swarmWeight=(1-goalWeight)*swarmWeight;
    goalWeight=1-swarmWeight;

    double hx =
        goalWeight  * cos(goalHeading) +
        swarmWeight * cos(vicsekHeading);

    double hy =
        goalWeight  * sin(goalHeading) +
        swarmWeight * sin(vicsekHeading);

    if (count > 3) swarmWeight *= 0.5;

    double noise =
        (((double)rand() / RAND_MAX) - 0.5) * 0.8;

    robot.heading_goal = atan2(hy, hx) + noise;

    if(distance < 2.0)
        robot.speed_goal = 0.0;
    else if(distance < 6.0)
        robot.speed_goal = 1.5;
    else
        robot.speed_goal = 5.0;
}


int main(int argc, char **argv)
{
    //M_PI - pi value
    //create Supervisor instance
    Supervisor supervisor;

    //initialize random
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    int timestep = supervisor.getBasicTimeStep();
    double dt=double(timestep)/1000;

    std::vector<RobotData> robots = spawnRobots(supervisor, 100, 0, 10, 10);


    // Main loop:
    while (supervisor.step(timestep) != -1)
    {
        for(int i=0; i<robots.size(); i++)
        {
            updateVicsekGoal(robots[i], robots, 5);
        }
        for(int i=0; i<robots.size(); i++)
        {
            robots[i].toGoal(dt);
            robots[i].stepTurn(dt);
            robots[i].stepMove(dt);
            robots[i].applyData();
        }
    };

    return 0;
}
