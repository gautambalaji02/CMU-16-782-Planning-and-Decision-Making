// Header file for wait_cell_planner.cpp

#ifndef WAIT_CELL_PLANNER_H
#define WAIT_CELL_PLANNER_H

#include <vector>
#include "grid_map.h"
#include "reachability.h"
#include "plan.h"

// Class definition
class WaitCellPlanner{
    
    public:
        WaitCellPlanner(const GridMap& grid, const Reachability& reach);

        // Returns an invalid plan if no plan was found.
        Plan solve();

    private:
        // backward search from the trajectory to compute costs and deadlines
        void backwardSearch();

        // returns the best wait cell, or -1
        int forwardSearch();    
        
        // builds the final plan given the best wait cell
        Plan buildPlan(int wait_cell) const;

        const GridMap& grid_;
        const Reachability& reach_;
        Cost wait_cost_;                      

        // backward search from the trajectory
        std::vector<Cost> back_cost_;
        std::vector<int> deadline_;
        std::vector<unsigned char> back_dir_; 
        std::vector<char> back_closed_;

        // forward search from the robot
        std::vector<Cost> fwd_cost_;
        std::vector<int> fwd_steps_;
        std::vector<unsigned char> fwd_dir_; 
        std::vector<char> fwd_closed_;
};

#endif 
