// Implementation file for grid_map.cpp
#include "../include/grid_map.h"

#include <algorithm>
#include <cstdlib>

// Constructor and member function implementations for the GridMap class.
GridMap::GridMap(int* map, int collision_thresh, int x_size, int y_size,
                 int* target_traj, int target_steps)
    : map_(map), traj_(target_traj), collision_thresh_(collision_thresh),
      x_size_(x_size), y_size_(y_size), target_steps_(target_steps)
{
}

// Returns the total number of cells in the grid.
int GridMap::numCells() const { return x_size_ * y_size_; }

// Returns the number of steps in the target trajectory.
int GridMap::targetSteps() const { return target_steps_; }

// Returns true if the cell is free (i.e., its cost is non-negative and less than the collision threshold), and false otherwise.
int GridMap::cost(int cell) const { return map_[cell]; }

// Returns the index of a cell given its x and y coordinates.
bool GridMap::isFree(int cell) const
{
    return map_[cell] >= 0 && map_[cell] < collision_thresh_;
}

// Returns the index of a cell given its x and y coordinates.
int GridMap::cellIndex(int x, int y) const { return GETMAPINDEX(x, y, x_size_, y_size_); }

// Returns the x coordinate of a cell.
int GridMap::cellX(int cell) const { return cell % x_size_ + 1; }

// Returns the y coordinate of a cell.
int GridMap::cellY(int cell) const { return cell / x_size_ + 1; }

// Returns the index of a neighboring cell in the specified direction, or -1 if the neighbor is out of bounds.
int GridMap::neighbor(int cell, int dir) const
{
    int x = cellX(cell) + DX[dir];
    int y = cellY(cell) + DY[dir];
    if (x < 1 || y < 1 || x > x_size_ || y > y_size_)
        return -1;
    return cellIndex(x, y);
}

// target_traj stores all x coordinates first, then all y coordinates
int GridMap::targetCell(int t) const
{
    return cellIndex(traj_[t], traj_[t + target_steps_]);
}

// Returns the Chebyshev distance from the given cell to the target cell at time t.
int GridMap::chebyshevToTarget(int cell, int t) const
{
    int dx = std::abs(cellX(cell) - traj_[t]);
    int dy = std::abs(cellY(cell) - traj_[t + target_steps_]);
    return std::max(dx, dy);
}

// Returns the cost of a plan starting from the given start_time.
Cost GridMap::planCost(const Plan& plan, int start_time) const
{
    int moves = (int)plan.cells.size() - 1;
    int waits = plan.catch_time - start_time - moves;
    if (waits < 0)
        return INF_COST;   // cannot reach the catch cell in time

    Cost total = 0;
    Cost cheapest = INF_COST;
    for (int k = 0; k <= moves; k++)
    {
        if (k < moves)
            total += cost(plan.cells[k]);   // pay for the cell we leave
        cheapest = std::min<Cost>(cheapest, cost(plan.cells[k]));
    }
    return total + (Cost)waits * cheapest;
}
