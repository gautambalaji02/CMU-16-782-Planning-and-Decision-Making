// Wait cell planner implementation.
#include "../include/wait_cell_planner.h"

#include <algorithm>
#include <functional>
#include <queue>

namespace{
// Priority queue entry, ordered by (key, tie) - smallest first.
struct QueueEntry{

    Cost key;
    int tie;
    int cell;
    bool operator>(const QueueEntry& other) const{

        if (key != other.key) return key > other.key;
        return tie > other.tie;
    }
};
typedef std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> MinQueue;

// cost of standing on a cell, measured relative to the wait cost m
Cost extraCost(const GridMap& grid, int cell, Cost m){

    return std::max<Cost>(0, grid.cost(cell) - m);
}
}  // namespace

WaitCellPlanner::WaitCellPlanner(const GridMap& grid, const Reachability& reach)
    : grid_(grid), reach_(reach), wait_cost_(reach.min_cost){}

Plan WaitCellPlanner::solve(){

    backwardSearch();
    int wait_cell = forwardSearch();
    if (wait_cell < 0)
        return Plan();
    return buildPlan(wait_cell);
}

// Backward multi-goal Dijkstra.
void WaitCellPlanner::backwardSearch(){

    const int n = grid_.numCells();
    const int t0 = reach_.start_time;
    back_cost_.assign(n, INF_COST);
    deadline_.assign(n, -1);
    back_dir_.assign(n, NO_PARENT);
    back_closed_.assign(n, 0);

    MinQueue open;
    for (const Goal& g : reach_.goals){

        Cost value = (Cost)(g.time - t0) * wait_cost_;
        if (value < back_cost_[g.cell] || (value == back_cost_[g.cell] && g.time > deadline_[g.cell])){

            back_cost_[g.cell] = value;
            deadline_[g.cell] = g.time;
            open.push({value, -g.time, g.cell});
        }
    }

    while (!open.empty()){

        QueueEntry top = open.top();
        open.pop();
        int u = top.cell;
        if (back_closed_[u] || top.key != back_cost_[u] || -top.tie != deadline_[u])
            continue;                                  
        back_closed_[u] = 1;

        int new_deadline = deadline_[u] - 1;
        for (int d = 0; d < NUM_DIRS; d++){

            int v = grid_.neighbor(u, d);
            if (v < 0 || reach_.steps[v] < 0 || back_closed_[v])
                continue;
            if (new_deadline < t0 + reach_.steps[v])
                continue;                               
            Cost value = back_cost_[u] + extraCost(grid_, v, wait_cost_);
            if (value < back_cost_[v] || (value == back_cost_[v] && new_deadline > deadline_[v])){

                back_cost_[v] = value;
                deadline_[v] = new_deadline;
                back_dir_[v] = (unsigned char)(NUM_DIRS - 1 - d);   
                open.push({value, -new_deadline, v});
            }
        }
    }
}

// Forward search from the robot to find the best wait cell.
int WaitCellPlanner::forwardSearch()
{
    const int n = grid_.numCells();
    const int t0 = reach_.start_time;
    const int s = reach_.start_cell;
    if (back_cost_[s] >= INF_COST)
        return -1;                                      

    fwd_cost_.assign(n, INF_COST);
    fwd_steps_.assign(n, 0);
    fwd_dir_.assign(n, NO_PARENT);
    fwd_closed_.assign(n, 0);

    MinQueue open;
    fwd_cost_[s] = 0;
    open.push({back_cost_[s], 0, s});

    Cost best_value = INF_COST;
    int best_cell = -1;
    while (!open.empty())
    {
        QueueEntry top = open.top();
        open.pop();
        int u = top.cell;
        if (fwd_closed_[u] || top.key != fwd_cost_[u] + back_cost_[u] || top.tie != fwd_steps_[u])
            continue;                                 
        if (top.key >= best_value)
            break;                                      
        fwd_closed_[u] = 1;

        if (grid_.cost(u) <= wait_cost_ && t0 + fwd_steps_[u] <= deadline_[u])
        {
            best_value = top.key;                     
            best_cell = u;
        }

        Cost new_cost = fwd_cost_[u] + extraCost(grid_, u, wait_cost_);
        int new_steps = fwd_steps_[u] + 1;
        for (int d = 0; d < NUM_DIRS; d++)
        {
            int v = grid_.neighbor(u, d);
            if (v < 0 || fwd_closed_[v] || back_cost_[v] >= INF_COST)
                continue;
            if (new_cost < fwd_cost_[v] || (new_cost == fwd_cost_[v] && new_steps < fwd_steps_[v]))
            {
                fwd_cost_[v] = new_cost;
                fwd_steps_[v] = new_steps;
                fwd_dir_[v] = (unsigned char)d;         
                open.push({new_cost + back_cost_[v], new_steps, v});
            }
        }
    }
    return best_cell;
}

// path1: follow forward parents from w back to the robot (then reverse)
// path2: follow backward pointers from w to the catch cell
Plan WaitCellPlanner::buildPlan(int wait_cell) const
{
    Plan plan;
    for (int c = wait_cell; ; )
    {
        plan.cells.push_back(c);
        if (fwd_dir_[c] == NO_PARENT)
            break;
        c = grid_.neighbor(c, NUM_DIRS - 1 - fwd_dir_[c]);
    }
    std::reverse(plan.cells.begin(), plan.cells.end());

    for (int c = wait_cell; back_dir_[c] != NO_PARENT; )
    {
        c = grid_.neighbor(c, back_dir_[c]);
        plan.cells.push_back(c);
    }
    plan.catch_time = deadline_[plan.cells.back()];
    plan.cost = grid_.planCost(plan, reach_.start_time);
    return plan;
}
