// All the constants
#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <climits>

// Cell (x, y) -> index into the map array. x and y start at 1.
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

// Path costs can exceed the range of int on large maps.
typedef long long Cost;
const Cost INF_COST = LLONG_MAX / 4;

// 8-connected grid. The directions are ordered so that the opposite of
// direction d is (NUM_DIRS - 1 - d).
const int NUM_DIRS = 8;
const int DX[NUM_DIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
const int DY[NUM_DIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
const unsigned char NO_PARENT = 255;   // "no direction" marker for parent pointers

// The exact space-time search only runs if the estimated number of
// (x, y, t) states it could touch is below this value.
const double EXACT_SEARCH_MAX_STATES = 2.0e8;

#endif 
