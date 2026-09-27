/*=================================================================
 *
 * reachability.cpp
 *
 *=================================================================*/
#include "../include/reachability.h"

#include <algorithm>

Reachability::Reachability(const GridMap& grid, int start_cell, int start_time)
    : start_cell(start_cell), start_time(start_time), min_cost(1){

    breadthFirstSearch(grid);
    collectGoals(grid);
}

// Plain BFS: every move takes one time step, whatever the cell costs.
void Reachability::breadthFirstSearch(const GridMap& grid){

    steps.assign(grid.numCells(), -1);
    std::vector<int> queue;
    queue.push_back(start_cell);
    steps[start_cell] = 0;
    min_cost = grid.cost(start_cell);

    for (size_t head = 0; head < queue.size(); head++)
    {
        int u = queue[head];
        for (int d = 0; d < NUM_DIRS; d++)
        {
            int v = grid.neighbor(u, d);
            if (v < 0 || steps[v] >= 0 || !grid.isFree(v))
                continue;
            steps[v] = steps[u] + 1;
            min_cost = std::min(min_cost, grid.cost(v));
            queue.push_back(v);
        }
    }
}

// The target is at targetCell(i) at time i. The robot can catch it there
// only if it can walk to that cell in at most (i - start_time) moves.
void Reachability::collectGoals(const GridMap& grid){
    
    goals.clear();
    for (int i = start_time + 1; i < grid.targetSteps(); i++)
    {
        int cell = grid.targetCell(i);
        if (steps[cell] >= 0 && steps[cell] <= i - start_time)
            goals.push_back({i, cell});
    }
}
