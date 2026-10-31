// Header file for plan.cpp
#ifndef PLAN_H
#define PLAN_H

#include <vector>
#include "constants.h"

// A plan represents a sequence of cells the robot should traverse to catch the target.
struct Plan{

    std::vector<int> cells;   
    int catch_time = -1;      
    Cost cost = INF_COST;     
};

// A point on the target's trajectory that the robot can reach in time.
struct Goal{
    
    int time;  
    int cell;   
};

#endif 
