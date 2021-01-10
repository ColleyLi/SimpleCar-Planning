/**
*   Manually define the map using the ValidityChecker
*   Author: Jianfeng Cui 
*/

#include <ompl/base/StateValidityChecker.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include "ValidityChecker.h"

ValidityChecker::ValidityChecker(const ob::SpaceInformationPtr &si): ob::StateValidityChecker(si)
{
    // Draw the map with obstacles(0) and available(1)
    map_.resize(GRID_ROWS);
    for(int i = 0; i < GRID_ROWS; i++){
        map_[i].resize(GRID_COLS);
    }
    for(int i = 0; i < GRID_ROWS; i++) { 
        for(int j = 0; j < GRID_COLS; j++) {
            if ((i >= 90 && i <= 100 && j >= 90 && j <= 100 ) || 
                    (i >= 80 && i <= 199 && j >= 50 && j <= 55) ||
                        (i >= 0 && i <= 100 && j >= 150 && j <= 155) ||
                            (i == 0 || i == 199) || (j == 0 || j == 199)){
                map_[i][j] = 0;
            }
            else {
                map_[i][j] = 1;
            }
        }
    }
    // Traverse the map to store the clearance of each grid
    clr_matrix_.resize(GRID_ROWS);
    for(int i = 0; i < GRID_ROWS; i++){
        clr_matrix_[i].resize(GRID_COLS);
    }
    for(int i = 0; i < GRID_ROWS; i++)
    { 
        for(int j = 0; j < GRID_COLS; j++)
        {
            // std::cout << "here i, j" << i << ' ' << j <<std::endl;
            if(map_[i][j] == 0)
            {
                clr_matrix_[i][j] = 0;
                continue;
            }
            int clr = 0;
            bool collision = 0;
            while(collision == 0){
                clr++;
                for(int jj = j - clr; jj < j + clr; jj++){
                    collision += !map_[i - clr][jj];
                    collision += !map_[i + clr][jj];
                }
                for(int ii = i - clr + 1; ii < i + clr - 1; ii++){
                    collision += !map_[ii][j - clr];
                    collision += !map_[ii][j + clr];
                }
            }
            clr_matrix_[i][j] = clr;
        }
    }
}

bool ValidityChecker::isValid(const ob::State* state) const
{
    return this -> clearance(state) > 10.0; // safety margin: 2x
}

double ValidityChecker::clearance(const ob::State* state) const
{
    // Design the collision checking and clearance calculation
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

    // for(int j = x_map - 5; j < x_map + 5; j++)
    // {
    //     for(int i = y_map - 5; i < y_map + 5; i++)
    //     {
    //         if(map_[j][i] == 0) return 0;
    //     }
    // }

    // return sqrt((x-0.0)*(x-0.0)+(y-0.0)*(y-0.0)) - 9;
    // return map_[x_map][y_map] == 1;
    return clr_matrix_[x_map][y_map];
}