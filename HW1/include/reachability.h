// Header file for reachability.cpp

#ifndef REACHABILITY_H
#define REACHABILITY_H

#include <vector>
#include "grid_map.h"
#include "plan.h"

// Class definition
class Reachability{
    
    public:
        Reachability(const GridMap& grid, int start_cell, int start_time);

        int start_cell;
        int start_time;
        std::vector<int> steps;      
        std::vector<Goal> goals;
        int min_cost;

    private:
        void breadthFirstSearch(const GridMap& grid);
        void collectGoals(const GridMap& grid);
};

#endif 
