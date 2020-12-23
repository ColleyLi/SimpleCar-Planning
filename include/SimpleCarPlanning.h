/**
 * Define the simple car properties.
*/
#ifndef SIMPLE_CAR_PLANNING_H
#define SIMPLE_CAR_PLANNING_H

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/config.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

#define USE_PCL_COLLCHECK 1

#if USE_PCL_COLLCHECK == 1
// #include <omplapp/geometry/RigidBodyGeometry.h>
// #include <omplapp/apps/AppBase.h>
// #include <omplapp/config.h>
#include <omplapp/apps/KinematicCarPlanning.h>
namespace oa = ompl::app;
#endif

class SimpleCarPlanning{
    public:
        constexpr static double wheelbase_ = 0.3;
        double track_;
        double max_steering_angle_;

        SimpleCarPlanning();
        void static propagate(const ob::State *start, const oc::Control *control, 
                        const double duration, ob::State *result);
        void static SimpleCarODE(const oc::ODESolver::StateType& q, 
                        const oc::Control* control, oc::ODESolver::StateType& qdot);
        void static postPropagate(const ob::State* state, const oc::Control* control, 
                        const double duration, ob::State* result);
        void plan();
        void planWithApp();

        std::string path_filename_;
        std::string path_filename_app_;
};

#endif // SIMPLE_CAR_PLANNING_H