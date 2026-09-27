// Implementation file for plan_executor.cpp
#include "../include/plan_executor.h"

#include <algorithm>

void PlanExecutor::setPlan(const GridMap& grid, const Plan& plan)
{
    plan_ = plan;
    index_ = 0;
    int n = (int)plan_.cells.size();
    cheapest_ahead_.assign(n, INF_COST);
    for (int k = n - 1; k >= 0; k--)
    {
        Cost here = grid.cost(plan_.cells[k]);
        cheapest_ahead_[k] = (k + 1 < n) ? std::min(here, cheapest_ahead_[k + 1]) : here;
    }
}

void PlanExecutor::nextMove(const GridMap& grid, int robot_cell, int curr_time, int* action_ptr)
{
    int next = robot_cell;                              
    if (plan_.catch_time >= 0)
    {
        int moves_left = (int)plan_.cells.size() - 1 - index_;
        int slack = (plan_.catch_time - curr_time) - moves_left;
        bool on_cheapest = grid.cost(plan_.cells[index_]) == cheapest_ahead_[index_];
        if (moves_left > 0 && !(slack > 0 && on_cheapest))
            index_++;                                
        next = plan_.cells[index_];               
    }
    action_ptr[0] = grid.cellX(next);
    action_ptr[1] = grid.cellY(next);
}
