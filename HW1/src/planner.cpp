/*=================================================================
 *
 * planner.cpp
 *
 * 
 * Note : Moved all the defined functions into constants.h
 *=================================================================*/
#include "../include/planner.h"
#include "../include/grid_map.h"
#include "../include/reachability.h"
#include "../include/wait_cell_planner.h"
#include "../include/space_time_planner.h"
#include "../include/plan_executor.h"

#include <chrono>
#include <cstdio>

void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    )
{
    // "static" variables keep their values between calls
    static bool planned = false;
    static GridMap grid;
    static PlanExecutor executor;

    if (!planned)
    {
        auto start = std::chrono::steady_clock::now();
        grid = GridMap(map, collision_thresh, x_size, y_size, target_traj, target_steps);
        int robot_cell = grid.cellIndex(robotposeX, robotposeY);

        Reachability reach(grid, robot_cell, curr_time);

        WaitCellPlanner wait_cell_planner(grid, reach);
        Plan plan = wait_cell_planner.solve();
        Cost decomposition_cost = plan.cost;

        SpaceTimePlanner space_time_planner(grid, reach);
        bool exact_ran = space_time_planner.improve(plan);

        executor.setPlan(grid, plan);
        planned = true;

        double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        printf("[planner] decomposition cost = %lld, exact search %s, final cost = %lld, "
               "catch at t = %d, planning time = %.3f s\n",
               decomposition_cost, exact_ran ? "ran (plan is optimal)" : "skipped (region too large)",
               plan.cost, plan.catch_time, seconds);
    }

    executor.nextMove(grid, grid.cellIndex(robotposeX, robotposeY), curr_time, action_ptr);
}
