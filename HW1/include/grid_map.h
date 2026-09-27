// Header file for grid_map.cpp
#ifndef GRID_MAP_H
#define GRID_MAP_H

#include "constants.h"
#include "plan.h"

// Class definition
class GridMap{

    public:
        // Default constructor
        GridMap() = default;
        GridMap(int* map, int collision_thresh, int x_size, int y_size,
                int* target_traj, int target_steps);

        
        int numCells() const;
        int targetSteps() const;

        int cost(int cell) const;
        bool isFree(int cell) const;                 
        int cellIndex(int x, int y) const;           
        int cellX(int cell) const;                   
        int cellY(int cell) const;
        int neighbor(int cell, int dir) const;       

        int targetCell(int t) const;                
        int chebyshevToTarget(int cell, int t) const;

        // Cost of executing `plan` from start_time: move along plan.cells and
        // spend all spare time waiting at the cheapest cell of the path.
        Cost planCost(const Plan& plan, int start_time) const;

    private:
    
        int* map_ = nullptr;
        int* traj_ = nullptr;
        int collision_thresh_ = 0;
        int x_size_ = 0;
        int y_size_ = 0;
        int target_steps_ = 0;
};

#endif 
