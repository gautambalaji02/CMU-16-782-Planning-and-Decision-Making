// Header file for plan_executor.cpp
#ifndef PLAN_EXECUTOR_H
#define PLAN_EXECUTOR_H

#include <vector>
#include "grid_map.h"
#include "plan.h"

// Class definition
class PlanExecutor{
    
    public:
        void setPlan(const GridMap& grid, const Plan& plan);

        // Writes the robot's next cell (1-based x, y) into action_ptr.
        void nextMove(const GridMap& grid, int robot_cell, int curr_time, int* action_ptr);

    private:
        Plan plan_;
        std::vector<Cost> cheapest_ahead_;   
        int index_ = 0;                      
};

#endif 
