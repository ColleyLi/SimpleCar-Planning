/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

/* Author: Luis G. Torres, Jonathan Gammell */
/* Modificator: Jianfeng Cui */

/**
 * Define the state validity checker
*/
#include <ompl/base/StateValidityChecker.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <vector>

using namespace std;

#include "ValidityChecker.h"
#include "SimpleCarPlanning.h"


ValidityChecker::ValidityChecker(const ob::SpaceInformationPtr &si): ob::StateValidityChecker(si)
{}

bool ValidityChecker::isValid(const ob::State* state) const
{
    return this -> clearance(state) <= 1e-3;
}

double ValidityChecker::clearance(const ob::State* state) const
{
    const auto *se2state = state -> as<ob::SE2StateSpace::StateType>();
    const auto *pos = se2state -> as<ob::RealVectorStateSpace::StateType>(0);
    double x = pos -> values[0];
    double y = pos -> values[1];
    const auto *rot = se2state -> as<ob::SO2StateSpace::StateType>(1);

    // Define new map
    int dim = 50;
    std::vector<std::vector<double>> grid(dim,vector<double>(dim));;
        
    for(int i = 0; i < dim; i++) { 
        for(int j = 0; j < dim; j++) {
            if ((i > 0 && i < 5 && j > 20 && j < 25) || (i > 20 && i < 25 && j > 15 && j < 20)) {
                grid[i][j] = 0;
            }
            else {
                grid[i][j] = 1;
            }
        }
    }        
    double sum = 0;
    for(int j = x - 1; j < x + 2; j += 1) {
        for(int i = y - 1; i < y + 2; i += 1) {
            sum += grid[i][j];
        }
    }
    return abs(sum - 9);

    //return sqrt((x-0.0)*(x-0.0)+(y-0.0)*(y-0.0)) - 9;
}