/**
*   Manually define the map using the ValidityChecker
*   Author: Jianfeng Cui 
*/

#ifndef VALIDITY_CHECKER_H
#define VALIDITY_CHECKER_H

#include <ompl/base/StateValidityChecker.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>

namespace ob = ompl::base;

#define GRID_ROWS 200
#define GRID_COLS 200

class ValidityChecker: public ob::StateValidityChecker
{
    public:
        ValidityChecker(const ob::SpaceInformationPtr &si);
    
        virtual bool isValid(const ob::State* state) const;

        double clearance(const ob::State* state) const;

        std::vector<std::vector<int>> map_;
        std::vector<std::vector<int>> clr_matrix_;
};

#endif // VALIDITY_CHECKER_H