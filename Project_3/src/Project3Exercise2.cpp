///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Shreyas Poyrekar!!
//////////////////////////////////////

#include <iostream>
#include <fstream>


// The collision checker produced in project 2
#include "CollisionChecking.h"

// Your random tree planner
#include "RTP.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>

 namespace ob = ompl::base;
 namespace og = ompl::geometric;

// This is our state validity checker for checking if our point robot is in collision
bool isValidStatePoint(const ompl::base::State *state, const std::vector<Rectangle> &obstacles)
{
    // cast the abstract state type to the type we expect
     const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    // Extract x, y
    double x = se2state->getX();
    double y = se2state->getY();

    return isValidPoint(x, y, obstacles);
}

// This is our state validity checker for checking if our square robot is in collision
bool isValidStateSquare(const ompl::base::State *state, double sideLength, const std::vector<Rectangle> &obstacles)
{
    // cast the abstract state type to the type we expect
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
    double x = se2state->getX();
    double y = se2state->getY();
    double theta = se2state->getYaw();

    return isValidSquare(x, y, theta, sideLength, obstacles);
}


void planPoint(const std::vector<Rectangle> obstacles)
{
    // implementation of RTP to plan for a point robot.
    auto space(std::make_shared<ob::SE2StateSpace>());
    
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-2);  // x and y have a minimum of -2
    bounds.setHigh(2);  // x and y have a maximum of 2
    space->setBounds(bounds);
    
     // define a simple setup class
     og::SimpleSetup ss(space);
     
     // create a planner for the defined space
     auto planner(std::make_shared<og::RTP>(ss.getSpaceInformation()));
     ss.setPlanner(planner);
    
     // set state validity checking for this space
     ss.setStateValidityChecker(std::bind(isValidStatePoint,std::placeholders::_1,obstacles));
    
     // start state
     ompl::base::ScopedState<> start(space);
     start[0] = -1.0;
     start[1] = +1.3;
    
     // goal state
     ompl::base::ScopedState<> goal(space);
     goal[0] = 1.2;
     goal[1] = 1.2;
     
     // set the start and goal states
     ss.setStartAndGoalStates(start, goal);
  /*
     // this call is optional, but we put it in to get more output information
     ss.setup();
     ss.print();
  */
     // attempt to solve the problem within one second of planning time
     ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        // Apply some heuristics to simplify (and prettify) the solution
         ss.simplifySolution();

        // print the path to screen
        std::cout << "Found solution:" << std::endl;
        ompl::geometric::PathGeometric &path = ss.getSolutionPath();
        path.interpolate(50);
        path.printAsMatrix(std::cout);

    }
    else
        std::cout << "No solution found" << std::endl;
}

void planBox(const std::vector<Rectangle> &obstacles)
{
    // implementation of RTP to plan for a rotating square robot.
    auto space(std::make_shared<ob::SE2StateSpace>());
    
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-2);  // x and y have a minimum of -2
    bounds.setHigh(2);  // x and y have a maximum of 2
    space->setBounds(bounds);
    
     // define a simple setup class
     og::SimpleSetup ss(space);
     
     // create a planner for the defined space
     auto planner(std::make_shared<og::RTP>(ss.getSpaceInformation()));
     ss.setPlanner(planner);
    
     // set state validity checking for this space and set the sidelength to 0.25
     ss.setStateValidityChecker(std::bind(isValidStateSquare,std::placeholders::_1,0.25,obstacles));
    
    ompl::base::ScopedState<> start(space);
    start[0] = -1.8;
    start[1] = -0.9;
    start[2] = 0.0;

    ompl::base::ScopedState<> goal(space);
    goal[0] = 0.9;
    goal[1] = 0.9;
    goal[2] = 0.0;
     
     // set the start and goal states
     ss.setStartAndGoalStates(start, goal);
  
  /*
     // this call is optional, but we put it in to get more output information
     ss.setup();
     ss.print();
     */
  
     // attempt to solve the problem within one second of planning time
     ob::PlannerStatus solved = ss.solve(5.0);

    if (solved)
    {
        // Apply some heuristics to simplify (and prettify) the solution
         ss.simplifySolution();

        // print the path to screen
        std::cout << "Found solution:" << std::endl;
        ompl::geometric::PathGeometric &path = ss.getSolutionPath();
        path.interpolate(50);
        path.printAsMatrix(std::cout);

    }
    else
        std::cout << "No solution found" << std::endl;
    
}

void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    // vector of rectangles with your first environment.
    Rectangle obs;
    obs.x = -1.3;
    obs.y = -1.0;
    obs.width = 0.5;
    obs.height = 0.5;
    obstacles.push_back(obs);
    
    obs.x = 0.0;
    obs.y = -1.0;
    obs.width = 0.3;
    obs.height = 0.7;
    obstacles.push_back(obs);
    
    obs.x = -1.5;
    obs.y = -0.3;
    obs.width = 0.6;
    obs.height = 0.4;
    obstacles.push_back(obs);
    
    obs.x = 1.0;
    obs.y = 0.4;
    obs.width = 0.5;
    obs.height = 0.6;
    obstacles.push_back(obs);
}

void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
    // vector of rectangles with your second environment.
    Rectangle obs;
    obs.x = -2.0;
    obs.y = -1.3;
    obs.width = 2.9;
    obs.height = 0.1;
    obstacles.push_back(obs);

    obs.x = 0.8;
    obs.y = -1.2;
    obs.width = 0.1;
    obs.height = 1.5;
    obstacles.push_back(obs);

    obs.x = 0.0;
    obs.y = 0.3;
    obs.width = 0.9;
    obs.height = 0.1;
    obstacles.push_back(obs);

    obs.x = -0.9;
    obs.y = 1.2;
    obs.width = 2.9;
    obs.height = 0.1;
    obstacles.push_back(obs);

    obs.x = -0.9;
    obs.y = -0.3;
    obs.width = 0.1;
    obs.height = 1.5;
    obstacles.push_back(obs);

    obs.x = -0.9;
    obs.y = -0.4;
    obs.width = 0.9;
    obs.height = 0.1;
    obstacles.push_back(obs);
}

int main(int /* argc */, char ** /* argv */)
{
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;

        std::cin >> robot;
    } while (robot < 1 || robot > 2);

    do
    {
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) Rectangle obstacle" << std::endl;
        std::cout << " (2) Zag zag obstacle" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    switch (choice)
    {
        case 1:
            makeEnvironment1(obstacles);
            break;
        case 2:
            makeEnvironment2(obstacles);
            break;
        default:
            std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            planPoint(obstacles);
            break;
        case 2:
            planBox(obstacles);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}
