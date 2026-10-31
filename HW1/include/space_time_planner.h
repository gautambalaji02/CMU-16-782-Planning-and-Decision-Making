// Header file for space_time_planner.cpp
#ifndef SPACE_TIME_PLANNER_H
#define SPACE_TIME_PLANNER_H

#include <vector>
#include "grid_map.h"
#include "reachability.h"
#include "plan.h"

// Class definition
class SpaceTimePlanner{
    
    public:
        SpaceTimePlanner(const GridMap& grid, const Reachability& reach);

        // Replaces `plan` if a cheaper one exists. Returns false if the
        // region was too large and the search was skipped.
        bool improve(Plan& plan);

    private:
        double estimateStates(Cost upper_bound) const;

        // multi-goal backward Dijkstra
        void computeClampedCost();             
        int earliestCatch(int cell, int t) const;
        Cost heuristic(int cell, int t) const;

        const GridMap& grid_;
        const Reachability& reach_;
        std::vector<Cost> clamped_cost_;       
};

#endif 
