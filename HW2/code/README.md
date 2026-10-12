# Homework 2: Planning for a High-DoF Arm

Sampling-based planners (RRT, RRT-Connect, RRT*, PRM) for an n-DOF planar arm.

## Contents

| Path | What it is |
|---|---|
| `src/planner.cpp` | Given code, the four planners and `main()` |
| `src/utility.cpp`, `include/utility.h` | Helpers shared by the planners: collision checking, sampling, the search tree, path shortcutting, plan output |
| `src/verifier.cpp` | Given verifier: checks that a plan is collision-free and starts/ends at the right configurations |
| `scripts/grader.py` | Given grader: runs every planner, verifies the result and writes a CSV |
| `scripts/visualizer.py` | Given visualizer: turns a plan file into a GIF |
| `maps/` | `map1.txt`, `map2.txt` |
| `output/` | Plan files, GIFs and grader results are written here |

## Requirements

- CMake 3.16.3 or newer and a C++14 compiler (g++ or clang)
- Python 3 with `numpy`, `pandas`, `matplotlib` and `pillow` (for the grader and visualizer)

## Building

The planner is split over two source files (`planner.cpp` and `utility.cpp`), so the single-file g++ command from the handout no longer works. Use one of the following.

**CMake (recommended)**, from the `code` directory:

```
mkdir build && cd build
cmake ..
cmake --build . --config Release
```

This builds `build/planner` and `build/verifier`. With CMake, the map and output directories are compiled in as absolute paths, so the executables can be run from any directory.

**g++**, from the `code/build` directory:

```
g++ -O2 -std=c++14 -I../include ../src/planner.cpp ../src/utility.cpp -o planner
g++ -O2 -std=c++14 ../src/verifier.cpp -o verifier
```

With g++, maps are read from `../maps` and plans written to `../output` relative to the current directory, so run the executables from `build/` or `scripts/`.

## Running the planner

```
./planner <mapFile> <numofDOFs> <startAngles> <goalAngles> <plannerID> <outputFile>
```

| Planner ID | Planner |
|---|---|
| 0 | RRT |
| 1 | RRT-Connect |
| 2 | RRT* |
| 3 | PRM |

- Angles are in radians, comma separated with no spaces.
- `mapFile` is a file name inside `maps/`.
- `outputFile` is written inside `output/`.

Example, from `build/`:

```
./planner map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 1 myOutput.txt
```

This writes the plan to `output/myOutput.txt`: the map path on the first line, then one comma-separated configuration per line. The planner also prints the number of nodes, waypoints and the path cost.

If a planner does not find a path within its sample limit, it prints a message and the program exits with an error, which the grader counts as a failure.

RRT* always runs its full 10,000 iterations to keep improving the path, so it takes several seconds (about 10–17 s on the given problems). The other planners usually finish well under a second.

## Verifying a plan

The verifier takes the plan file path relative to the current directory. From `build/`:

```
./verifier map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 ../output/myOutput.txt
```

It prints `Passed verifier!` and returns 0 if every configuration is collision-free and the plan starts at the start and ends at the goal.

## Visualizing a plan

From the `code` directory:

```
python scripts/visualizer.py myOutput.txt --gifFilepath myGif.gif
```

This reads `output/myOutput.txt` and writes `output/myGif.gif`. Add `--incPrev 1` to keep earlier arm poses on screen (slower).

## Running the grader

Build first, then from the `scripts` directory:

```
cd scripts
python grader.py
```

The grader runs all four planners on the test problems in `grader.py`, verifies each plan, makes a GIF of each, and writes `output/grader_out/grader_results.csv` (steps, cost, time and success for each run). The folder `output/grader_out/` must exist, since the planner does not create directories (`mkdir -p ../output/grader_out` if it is missing).

## Hyper-parameters

All are constants at the top of each planner function in `src/planner.cpp`.

| Parameter | Value | Used by |
|---|---|---|
| Step size | 0.5 rad | RRT, RRT-Connect, RRT* |
| Goal threshold | 0.1 rad | RRT, RRT-Connect, RRT* |
| Sample limit | 10,000 | RRT, RRT-Connect, RRT* |
| Goal bias | 0.1 | RRT, RRT* |
| Rewiring radius | 1.0 rad | RRT* |
| Roadmap samples, neighbours | 2000, k = 10 (k = 50 for start and goal) | PRM |
| Gaussian sampling sigma | π/6 | PRM |
| Edge collision-check resolution | π/90 rad | all |

RRT shortens its path afterwards by skipping waypoints that can be connected directly. PRM uses Gaussian sampling to place nodes near obstacles and A* to search the roadmap.
