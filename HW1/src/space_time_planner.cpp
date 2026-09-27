// Implementation of the SpaceTimePlanner class.
#include "../include/space_time_planner.h"

#include <algorithm>
#include <functional>
#include <queue>
#include <utility>

// Constructor for the SpaceTimePlanner class. Initializes the grid and reachability information.
SpaceTimePlanner::SpaceTimePlanner(const GridMap& grid, const Reachability& reach)
    : grid_(grid), reach_(reach){}

// Estimates the number of (cell, time) states that could be explored given an upper bound on the cost.
double SpaceTimePlanner::estimateStates(Cost upper_bound) const{

    Cost last_time = std::min<Cost>(grid_.targetSteps() - 1,
                                    reach_.start_time + (upper_bound - 1) / reach_.min_cost);
    double total = 0;
    for (int c = 0; c < grid_.numCells(); c++)
    {
        if (reach_.steps[c] < 0)
            continue;
        Cost window = last_time - (reach_.start_time + reach_.steps[c]) + 1;
        if (window > 0)
            total += (double)window;
    }
    return total;
}

// Computes the clamped cost for each cell, which is a lower bound on the
// extra cost (above min_cost) needed to reach the target trajectory from that cell.
void SpaceTimePlanner::computeClampedCost(){

    const int n = grid_.numCells();
    clamped_cost_.assign(n, INF_COST);
    std::vector<char> closed(n, 0);
    typedef std::pair<Cost, int> Entry;          
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

    for (const Goal& g : reach_.goals)
    {
        clamped_cost_[g.cell] = 0;
        open.push({0, g.cell});
    }
    while (!open.empty())
    {
        Entry top = open.top();
        open.pop();
        int u = top.second;
        if (closed[u] || top.first != clamped_cost_[u])
            continue;
        closed[u] = 1;
        for (int d = 0; d < NUM_DIRS; d++)
        {
            int v = grid_.neighbor(u, d);
            if (v < 0 || reach_.steps[v] < 0 || closed[v])
                continue;
            Cost value = clamped_cost_[u] + (grid_.cost(v) - reach_.min_cost);
            if (value < clamped_cost_[v])
            {
                clamped_cost_[v] = value;
                open.push({value, v});
            }
        }
    }
}

// Returns the earliest time at which the robot can catch the target starting from (cell, t).
int SpaceTimePlanner::earliestCatch(int cell, int t) const{

    int lo = t + 1;
    int hi = grid_.targetSteps() - 1;
    if (lo > hi || hi - grid_.chebyshevToTarget(cell, hi) < t)
        return grid_.targetSteps();
    while (lo < hi)
    {
        int mid = (lo + hi) / 2;
        if (mid - grid_.chebyshevToTarget(cell, mid) >= t)
            hi = mid;
        else
            lo = mid + 1;
    }
    return lo;
}

Cost SpaceTimePlanner::heuristic(int cell, int t) const{

    if (clamped_cost_[cell] >= INF_COST)
        return INF_COST;
    int catch_time = earliestCatch(cell, t);
    if (catch_time >= grid_.targetSteps())
        return INF_COST;
    return clamped_cost_[cell] + (Cost)(catch_time - t) * reach_.min_cost;
}

// Attempts to improve the given plan using a time-layered search. Returns true if the plan was improved, false otherwise.
bool SpaceTimePlanner::improve(Plan& plan){

    if (estimateStates(plan.cost) > EXACT_SEARCH_MAX_STATES)
        return false;
    computeClampedCost();

    const int n = grid_.numCells();
    const int t0 = reach_.start_time;
    Cost best = plan.cost;
    int last_time = (int)std::min<Cost>(grid_.targetSteps() - 1,
                                        t0 + (best - 1) / reach_.min_cost);

    // All states ever created, stored in time order so the path can be
    // traced back at the end: state k = (state_cell[k], parent state_parent[k]).
    std::vector<int> state_cell, state_parent;

    // Per layer: cost of each cell (dense arrays, cleared after use),
    // position of the cell in state_cell (-1 = not in the layer yet), and
    // the heuristic of each state in the layer.
    std::vector<Cost> layer_cost[2] = {std::vector<Cost>(n, INF_COST), std::vector<Cost>(n, INF_COST)};
    std::vector<int> layer_index[2] = {std::vector<int>(n, -1), std::vector<int>(n, -1)};
    std::vector<Cost> cur_h, next_h;

    state_cell.push_back(reach_.start_cell);
    state_parent.push_back(-1);
    cur_h.push_back(heuristic(reach_.start_cell, t0));
    layer_cost[t0 % 2][reach_.start_cell] = 0;
    int layer_begin = 0, layer_end = 1;

    int catch_parent = -1, catch_cell = -1, catch_time = -1;

    for (int t = t0; t < last_time && layer_begin < layer_end; t++){

        std::vector<Cost>& cur_cost = layer_cost[t % 2];
        std::vector<Cost>& next_cost = layer_cost[(t + 1) % 2];
        std::vector<int>& next_index = layer_index[(t + 1) % 2];
        const int target_next = grid_.targetCell(t + 1);
        const int next_begin = (int)state_cell.size();
        next_h.clear();

        for (int k = layer_begin; k < layer_end; k++){

            int u = state_cell[k];
            if (cur_cost[u] + cur_h[k - layer_begin] >= best)
                continue;                               
            Cost value = cur_cost[u] + grid_.cost(u);   

            for (int d = 0; d <= NUM_DIRS; d++){

                int v = (d == NUM_DIRS) ? u : grid_.neighbor(u, d);
                if (v < 0 || !grid_.isFree(v))
                    continue;
                if (v == target_next){

                    if (value < best){

                        best = value;
                        catch_parent = k;
                        catch_cell = v;
                        catch_time = t + 1;
                        last_time = (int)std::min<Cost>(last_time, t0 + (best - 1) / reach_.min_cost);
                    }
                    continue;
                }
                int idx = next_index[v];
                if (idx < 0){

                    Cost h = heuristic(v, t + 1);
                    if (h >= INF_COST || value + h >= best)
                        continue;
                    next_index[v] = (int)state_cell.size();
                    state_cell.push_back(v);
                    state_parent.push_back(k);
                    next_h.push_back(h);
                    next_cost[v] = value;
                }
                else if (value < next_cost[v] || (value == next_cost[v] && v == u)){

                    // on equal cost prefer waiting (fewer moves, straighter paths)
                    if (value + next_h[idx - next_begin] >= best)
                        continue;
                    next_cost[v] = value;
                    state_parent[idx] = k;
                }
            }
        }
        // clear layer t so its arrays can be reused for layer t+2
        for (int k = layer_begin; k < layer_end; k++){
            cur_cost[state_cell[k]] = INF_COST;
            layer_index[t % 2][state_cell[k]] = -1;
        }
        layer_begin = next_begin;
        layer_end = (int)state_cell.size();
        std::swap(cur_h, next_h);
    }

    if (catch_cell < 0)
        return true;                                    

    // trace back, then drop the waits to get the spatial path
    std::vector<int> timed_path;
    timed_path.push_back(catch_cell);
    for (int k = catch_parent; k >= 0; k = state_parent[k])
        timed_path.push_back(state_cell[k]);
    std::reverse(timed_path.begin(), timed_path.end());

    Plan better;
    for (int c : timed_path)
        if (better.cells.empty() || better.cells.back() != c)
            better.cells.push_back(c);
    better.catch_time = catch_time;
    better.cost = grid_.planCost(better, t0);
    if (better.cost < plan.cost)
        plan = better;
    return true;
}
