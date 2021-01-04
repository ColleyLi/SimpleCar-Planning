/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */
/* Modificator: Jianfeng Cui */

/**
 * Define the simple car properties.
*/

#include "SimpleCarPlanning.h"
#include "ValidityChecker.h"
#include "RRTX.h"

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <iostream>
#include <fstream>
#include <boost/math/constants/constants.hpp>


SimpleCarPlanning::SimpleCarPlanning()
{
    track_ = 0.2;
    // The extreme steering angle is set as 35[deg] / 180 * pi = 0.6[rad]
    max_steering_angle_ = 0.6;
    path_filename_ = "simple_car_path.txt";
    path_filename_app_ = "simple_car_path_app.txt";
    path_filename_geometric_ = "simple_car_path_geometric.txt";
    path_filename_geometric_app_ = "simple_car_path_geometric_app.txt";
}

void SimpleCarPlanning::propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    const auto* se2state = start -> as<ob::SE2StateSpace::StateType>();
    const double* pos = se2state -> as<ob::RealVectorStateSpace::StateType>(0) -> values;
    const double theta = se2state -> as<ob::SO2StateSpace::StateType>(1) -> value;
    const double* u = control -> as<oc::RealVectorControlSpace::ControlType>() -> values;

    result -> as<ob::SE2StateSpace::StateType>()->setXY(
        pos[0] + u[0] * duration * cos(theta),
        pos[1] + u[0] * duration * sin(theta)
        );
    result -> as<ob::SE2StateSpace::StateType>()->setYaw(
        theta + u[0] * duration * tan(u[1]) / wheelbase_
        );

    // Make sure the orientation is in SO(2)
    // So this act as the postPropagate if using ode methods
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1));
}

void SimpleCarPlanning::SimpleCarODE(const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
    const double* u = control -> as<oc::RealVectorControlSpace::ControlType>() -> values;
    const double theta = q[2];

    // Zero out qdot
    qdot.resize (q.size(), 0);

    qdot[0] = u[0] * cos(theta);
    qdot[1] = u[0] * sin(theta);
    qdot[2] = u[0] * tan(u[1]) / wheelbase_;
}

// This is a callback method invoked after numerical integration.
void SimpleCarPlanning::postPropagate(const ob::State* state, 
                const oc::Control* control, const double duration, ob::State* result)
{
    ompl::base::SO2StateSpace SO2;
 
    // Ensure that the car's resulting orientation lies between 0 and 2*pi.
    ompl::base::SE2StateSpace::StateType& s = *result->as<ompl::base::SE2StateSpace::StateType>();
    SO2.enforceBounds(s[1]);
}

void SimpleCarPlanning::plan()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10.0);
    bounds.setHigh(10.0);

    space -> setBounds(bounds);

    // create a control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0, -0.5);
    cbounds.setHigh(0, 0.5);
    cbounds.setLow(1, - max_steering_angle_);
    cbounds.setHigh(1, max_steering_angle_);

    cspace -> setBounds(cbounds);

    // construct an instance of space information from this control space
    auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

    // set state validity checking for this space

    // option: use a mannual hard-designed validity checker
    si -> setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
    // si -> setStateValidityCheckingResolution(0.03);
    // or possible option: use FCL provided by OmplApp
    // Note: It is found that OmplApp facilitates: FCLStateValidityChecker, RigidBodyGeometry
    // See graph legend http://ompl.kavrakilab.org/classompl_1_1app_1_1RigidBodyGeometry.html
    // RigidBodyGeometry operates map and robot meshes with FCLStateValidityChecker, and acts as
    // a base class for further specific scenarios
    // OmplApp provides compact and clear inheritation so for this is directly used in planWithApp()

    // Set state propagator

    // option: simply a propagate function with enforcebounds
    si -> setStatePropagator(propagate);
    si -> setup();
    // or option: use ode solver
    // auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(si, &SimpleCarODE));
    // si -> setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &postPropagate));
    // si -> setup();

    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(-8.0);
    start->setY(-8.0);
    start->setYaw(0.0);

    // create a goal state
    ob::ScopedState<ob::SE2StateSpace> goal(start);
    goal->setX(8.0);
    goal->setY(8.0);
    goal->setYaw(boost::math::constants::pi<double>());

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef -> setStartAndGoalStates(start, goal, 0.05);

    // create a planner for the defined space
    auto planner(std::make_shared<oc::RRT>(si));

    // set the problem we are trying to solve for the planner
    planner -> setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner -> setup();

    // print the settings for this space
    si -> printSettings(std::cout);

    // print the problem settings
    pdef -> print(std::cout);

    // attempt to solve the problem within ten seconds of planning time
    ob::PlannerStatus solved = planner -> ob::Planner::solve(10.0);

    if(solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef -> getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        // path->print(std::cout);
        // path -> as<oc::PathControl>() -> asGeometric().printAsMatrix(std::cout);
        path -> as<oc::PathControl>() -> printAsMatrix(std::cout);

        // // print something of PathControl to have an insight
        // auto controlDurations = path -> as<oc::PathControl>() -> getControlDurations();
        // for(std::vector<double>::iterator vit = controlDurations.begin(), 
        //                                 vend = controlDurations.end();vit != vend; vit++)
        // {
        //     std::cout << *vit << std::endl;
        // }

        // Save the path matrix into the path file
        // For SE(2) states there are three numbers per line: x, y, and θ
        std::ofstream path_file(path_filename_.c_str());
        if(!path_file){
            std::cerr << "Cannot open output path file!" << std::endl;
            exit(EXIT_FAILURE);
        }
        path -> as<oc::PathControl>() -> printAsMatrix(path_file);
    }
    else
        std::cout << "No solution found" << std::endl;
}
/*
void SimpleCarPlanning::planWithApp()
{
    oa::KinematicCarPlanning setup;

    std::string map_dir = "../config/";
    std::string robot_fname = std::string(map_dir + "kinematics_car.dae");
    std::string env_fname = std::string(map_dir + "Barriers_easy_env.dae");
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    // plan for kinematic car in SE(2)
    // ob::StateSpacePtr SE2(setup.getStateSpace());

    // set the bounds for the R^2 part of SE(2)
    // ob::RealVectorBounds bounds(2);
    // bounds.setLow(-10);
    // bounds.setHigh(10);
    // SE2->as<ob::SE2StateSpace>()->setBounds(bounds);

    // define start state
    ob::ScopedState<ob::SE2StateSpace> start(setup.getSpaceInformation());
    start->setX(80.0);
    start->setY(-400.0);
    start->setYaw(0.0);

    // define goal state
    ob::ScopedState<ob::SE2StateSpace> goal(start);
    goal->setX(500.0);
    goal->setY(-80.0);
    goal->setYaw(boost::math::constants::pi<double>()/2);

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal, .1);

    setup.setPlanner(std::make_shared<oc::RRT>(setup.getSpaceInformation()));

    setup.setup();

    ob::PlannerStatus solved = setup.solve(10.0);

    if(solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        oc::PathControl path = setup.getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        // path -> print(std::cout);
        // path.asGeometric().printAsMatrix(std::cout);
        path.printAsMatrix(std::cout);

        // Save the path matrix into the path file
        // For SE(2) states there are three numbers per line: x, y, and θ
        std::ofstream path_file(path_filename_app_.c_str());
        if(!path_file){
            std::cerr << "Cannot open output path file!" << std::endl;
            exit(EXIT_FAILURE);
        }
        // path.asGeometric().printAsMatrix(path_file);
        path.printAsMatrix(path_file);
    }
    else
        std::cout << "No solution found" << std::endl;
}*/

void SimpleCarPlanning::PlanGeometric()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(50);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));

    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(4.0);
    start->setY(4.0);
    start->setYaw(0.0);

    // create a goal state
    ob::ScopedState<ob::SE2StateSpace> goal(start);
    goal->setX(40.0);
    goal->setY(40.0);
    goal->setYaw(boost::math::constants::pi<double>());

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef -> setStartAndGoalStates(start, goal, 0.05);

    // create a planner for the defined space
    auto planner(std::make_shared<ompl::RRTX>(si));
    // auto planner(std::make_shared<og::RRTstar>(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();


    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(20.0);

    if(solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef -> getSolutionPath();
        // BSpline smoothless
        og::PathSimplifierPtr ps;
        auto path_geometric = path -> as<og::PathGeometric>();
        ps -> smoothBSpline(*path_geometric);
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        // path->print(std::cout);
        // path -> as<oc::PathControl>() -> asGeometric().printAsMatrix(std::cout);
        // path -> as<oc::PathControl>() -> printAsMatrix(std::cout);
        path -> as<og::PathGeometric>() -> printAsMatrix(std::cout);

        // Save the path matrix into the path file
        // For SE(2) states there are three numbers per line: x, y, and θ
        std::ofstream path_file(path_filename_geometric_.c_str());
        if(!path_file){
            std::cerr << "Cannot open output path file!" << std::endl;
            exit(EXIT_FAILURE);
        }
        path -> as<og::PathGeometric>() -> printAsMatrix(path_file);
    }
    else
        std::cout << "No solution found" << std::endl;
}
/*
void SimpleCarPlanning::PlanGeometricWithApp()
{
    oa::SE2RigidBodyPlanning setup;

    std::string map_dir = "../config/";
    std::string robot_fname = std::string(map_dir + "kinematics_car.dae");
    std::string env_fname = std::string(map_dir + "Barriers_easy_env.dae");
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    // plan for kinematic car in SE(2)
    // ob::StateSpacePtr SE2(setup.getStateSpace());

    // set the bounds for the R^2 part of SE(2)
    // ob::RealVectorBounds bounds(2);
    // bounds.setLow(-10);
    // bounds.setHigh(10);
    // SE2->as<ob::SE2StateSpace>()->setBounds(bounds);

    // define start state
    ob::ScopedState<ob::SE2StateSpace> start(setup.getSpaceInformation());
    start->setX(80.0);
    start->setY(-400.0);
    start->setYaw(0.0);

    // define goal state
    ob::ScopedState<ob::SE2StateSpace> goal(start);
    goal->setX(500.0);
    goal->setY(-80.0);
    goal->setYaw(boost::math::constants::pi<double>()/2);

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal, .1);

    setup.setPlanner(std::make_shared<ompl::RRTX>(setup.getSpaceInformation()));

    setup.setup();

    ob::PlannerStatus solved = setup.solve(10.0);

    if(solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        og::PathGeometric path = setup.getSolutionPath();

        // BSpline smoothless
        og::PathSimplifierPtr ps;
        ps -> smoothBSpline(path);
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        // path -> print(std::cout);
        // path.asGeometric().printAsMatrix(std::cout);
        path.printAsMatrix(std::cout);

        // Save the path matrix into the path file
        // For SE(2) states there are three numbers per line: x, y, and θ
        std::ofstream path_file(path_filename_geometric_app_.c_str());
        if(!path_file){
            std::cerr << "Cannot open output path file!" << std::endl;
            exit(EXIT_FAILURE);
        }
        // path.asGeometric().printAsMatrix(path_file);
        path.printAsMatrix(path_file);
    }
    else
        std::cout << "No solution found" << std::endl;
}*/