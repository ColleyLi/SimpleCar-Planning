#include "SimpleCarPlanning.h"
#include "ValidityChecker.h"

#include <ompl/config.h>
#include <iostream>

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    SimpleCarPlanning scp;
    // scp.plan();
    // scp.planWithApp();
    // scp.PlanGeometric();
    scp.PlanGeometricWithApp();

    return 0;
}