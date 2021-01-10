/* 
Manually define the map using the ValidityChecker
@Author: Jianfeng Cui 
*/

#include <ompl/base/StateValidityChecker.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include "ValidityChecker.h"


ValidityChecker::ValidityChecker(const ob::SpaceInformationPtr &si): ob::StateValidityChecker(si)
{
    map_.resize(GRID_ROWS);
    for(int i = 0; i < GRID_ROWS; i++){
        map_[i].resize(GRID_COLS);
    }
    for(int i = 0; i < GRID_ROWS; i++) { 
        for(int j = 0; j < GRID_COLS; j++) {
            if ((i >= 90 && i <= 100 && j >= 90 && j <= 100 ) || 
                    (i >= 80 && i <= 199 && j >= 50 && j <= 55) ||
                        (i >= 0 && i <= 100 && j >= 150 && j <= 155)){
                map_[i][j] = 0;
            }
            else {
                map_[i][j] = 1;
            }
        }
    }
}

bool ValidityChecker::isValid(const ob::State* state) const
{
    return this -> clearance(state) > 0.0;
}

double ValidityChecker::clearance(const ob::State* state) const
{
    const auto *se2state = state -> as<ob::SE2StateSpace::StateType>();
    const auto *pos = se2state -> as<ob::RealVectorStateSpace::StateType>(0);
    double x = pos -> values[0];
    double y = pos -> values[1];
    const auto *rot = se2state -> as<ob::SO2StateSpace::StateType>(1);

    if(x < 1 + 5 || x > 199 - 5 || y < 1 + 5 || y > 199 - 5){
        return 0;
    }

    int x_map = GRID_ROWS - round(y);
    int y_map = round(x);

    for(int j = x_map - 5; j < x_map + 5; j++)
    {
        for(int i = y_map - 5; i < y_map + 5; i++)
        {
            if(map_[j][i] == 0) return 0;
        }
    }

    // return sqrt((x-0.0)*(x-0.0)+(y-0.0)*(y-0.0)) - 9;
    // return map_[x_map][y_map] == 1;
    return 1;
}