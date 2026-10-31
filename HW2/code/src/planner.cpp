/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h>
#include <climits>
#include <cstdlib>
#include <queue>
#include <limits>

#include "utility.h"

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 10

#ifndef MAPS_DIR
#define MAPS_DIR "../maps"
#endif
#ifndef OUTPUT_DIR
#define OUTPUT_DIR "../output"
#endif


// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                GIVEN FUNCTIONS                                                    //
//                                                                                                                   //
//*******************************************************************************************************************//

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          DEFAULT PLANNER FUNCTION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

void planner(
    double* map,
	int x_size,
	int y_size,
	double* armstart_anglesV_rad,
	double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
		
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }
    }    
    *planlength = numofsamples;
    
    return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              RRT IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRT(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
    // planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);

	*plan = NULL;
	*planlength = 0;

	// Hyper parameters
	const int NUM_SAMPLES = 10000;
	const double stepSize = 0.5;		// Fixed extension length, in radians
	const double goalThreshold = 0.1;
	const double goalBias = 0.1;		// Fraction of samples aimed at the goal

	CollisionChecker checker(map, x_size, y_size, numofDOFs);
	ConfigSampler sampler(numofDOFs);
	Tree tree(numofDOFs, NUM_SAMPLES + 2);
	tree.addRoot(armstart_anglesV_rad);

	vector<double> q_rand(numofDOFs), q_new(numofDOFs);
	int goalIdx = -1;

	// Repeat for num_samples iterations to sample configurations in the C-space.
	for (int i = 0; i < NUM_SAMPLES; i++) {

		sampler.withGoalBias(q_rand.data(), armgoal_anglesV_rad, goalBias);

		// EXTEND
		double minDist;
		int nearestIndex = tree.nearest(q_rand.data(), minDist);
		if (!checker.tryExtend(tree.node(nearestIndex), q_rand.data(),
							   stepSize, q_new.data())) {
			i--;				// blocked - retry rather than spend the sample
			continue;
		}

		int newIndex = tree.addNode(q_new.data(), nearestIndex);

		// Close enough to the goal? 
		if (configDist(q_new.data(), armgoal_anglesV_rad, numofDOFs) < goalThreshold &&
			checker.canConnect(q_new.data(), armgoal_anglesV_rad)) {
			goalIdx = tree.addNode(armgoal_anglesV_rad, newIndex);
			break;
		}
	}

	if (goalIdx < 0) {
		printf("RRT: Failed to find a path to the goal within the sample limit.\n");
		return;
	}

	Path path = tree.extractPath(goalIdx);
	int beforeShortcut = (int)path.size();
	shortcutPath(path, checker);

	printf("RRT: %d nodes, shortened path from %d to %d waypoints, cost %f\n",
		   tree.size(), beforeShortcut, (int)path.size(), pathCost(path));
	emitPlan(path, plan, planlength);
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                         RRT CONNECT IMPLEMENTATION                                                //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRTConnect(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
    // planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);

	*plan = NULL;
	*planlength = 0;

	// Hyper parameters
	const int NUM_SAMPLES = 10000;
	const double goalThreshold = 0.1;
	const double stepSize = 0.5;		// Fixed extension length, in radians

	CollisionChecker checker(map, x_size, y_size, numofDOFs);
	ConfigSampler sampler(numofDOFs);

	// Two trees, one rooted at the start and one at the goal
	Tree treeStart(numofDOFs, NUM_SAMPLES + 2);
	Tree treeGoal(numofDOFs, NUM_SAMPLES + 2);
	treeStart.addRoot(armstart_anglesV_rad);
	treeGoal.addRoot(armgoal_anglesV_rad);

	vector<double> q_rand(numofDOFs), q_new(numofDOFs), q_conn(numofDOFs);

	// Looping for num_samples iterations to sample configurations in the C-space.
	for (int k = 0; k < NUM_SAMPLES; k++) {

		// Alternate which tree is extended and which one reaches back
		bool extendStartTree = (k % 2 == 0);
		Tree& treeCurr  = extendStartTree ? treeStart : treeGoal;
		Tree& treeOther = extendStartTree ? treeGoal : treeStart;

		sampler.uniform(q_rand.data());

		// EXTEND 
		double distCurr;
		int nearestIndex = treeCurr.nearest(q_rand.data(), distCurr);
		if (!checker.tryExtend(treeCurr.node(nearestIndex), q_rand.data(),
							   stepSize, q_new.data()))
			continue;			// blocked, try another sample

		int newIndexCurr = treeCurr.addNode(q_new.data(), nearestIndex);

		// CONNECT
		double distOther;
		int frontier = treeOther.nearest(q_new.data(), distOther);
		while (true) {

			// Have the trees met? 
			if (configDist(treeOther.node(frontier), q_new.data(), numofDOFs) < goalThreshold &&
				checker.canConnect(treeOther.node(frontier), q_new.data())) {

				// Which index is in which tree depends on who was extended this iteration.
				int startSideIdx = extendStartTree ? newIndexCurr : frontier;
				int goalSideIdx  = extendStartTree ? frontier : newIndexCurr;

				Path path = treeStart.extractPath(startSideIdx);

				// The goal side comes out root-first, so append it backwards
				Path goalSide = treeGoal.extractPath(goalSideIdx);
				for (int a = (int)goalSide.size() - 1; a >= 0; a--)
					path.push_back(goalSide[a]);

				printf("RRT-Connect: connected after %d iterations, trees %d + %d nodes, "
					   "%d waypoints, cost %f\n",
					   k + 1, treeStart.size(), treeGoal.size(),
					   (int)path.size(), pathCost(path));
				emitPlan(path, plan, planlength);
				return;
			}

			// take greedy step, or give up on this connection
			if (!checker.tryExtend(treeOther.node(frontier), q_new.data(),
								   stepSize, q_conn.data()))
				break;

			frontier = treeOther.addNode(q_conn.data(), frontier);
		}
	}

	printf("RRT-Connect: Failed to connect the two trees within the sample limit.\n");
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRTStar(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
    // planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);

	*plan = NULL;
	*planlength = 0;

	// Hyper parameters - RRT
	const int NUM_SAMPLES = 10000;
	const double stepSize = 0.5;		// Fixed extension length, in radians
	const double goalThreshold = 0.1;
	const double goalBias = 0.1;		// Fraction of samples aimed at the goal

	// Hyper parameters - RRT*
	const double r = 1.0;				// Radius for rewiring, in radians

	CollisionChecker checker(map, x_size, y_size, numofDOFs);
	ConfigSampler sampler(numofDOFs);
	Tree tree(numofDOFs, NUM_SAMPLES + 2);
	tree.addRoot(armstart_anglesV_rad);

	vector<double> q_rand(numofDOFs), q_new(numofDOFs);
	vector<int> nearby;

	// Looping for num_samples iterations to sample configurations in the C-space.
	for (int i = 0; i < NUM_SAMPLES; i++) {

		sampler.withGoalBias(q_rand.data(), armgoal_anglesV_rad, goalBias);

		// EXTEND 
		double minDist;
		int nearestIndex = tree.nearest(q_rand.data(), minDist);
		if (!checker.tryExtend(tree.node(nearestIndex), q_rand.data(),
							   stepSize, q_new.data())) {
			i--;
			continue;
		}

		// X_near
		tree.near(q_new.data(), r, nearby);

		// CHOOSE PARENT 
		int q_min = nearestIndex;
		double bestCost = tree.cost(nearestIndex) +
						  configDist(tree.node(nearestIndex), q_new.data(), numofDOFs);
		for (size_t a = 0; a < nearby.size(); a++) {
			int nearIdx = nearby[a];
			if (nearIdx == nearestIndex)
				continue;
			double c = tree.cost(nearIdx) +
					   configDist(tree.node(nearIdx), q_new.data(), numofDOFs);
			// Costtocome plus edge cost, and check that the edge is collision-free
			if (c < bestCost && checker.canConnect(tree.node(nearIdx), q_new.data())) {
				bestCost = c;
				q_min = nearIdx;
			}
		}

		// Add q_new under its cheapest parent 
		int newIndex = tree.addNode(q_new.data(), q_min);
		tree.setCost(newIndex, bestCost);

		// REWIRE 
		for (size_t a = 0; a < nearby.size(); a++) {
			int nearIdx = nearby[a];
			if (nearIdx == q_min)
				continue;

			double c = tree.cost(newIndex) +
					   configDist(tree.node(newIndex), tree.node(nearIdx), numofDOFs);
			if (c >= tree.cost(nearIdx))
				continue;

			// Never reparent onto a descendant.
			if (tree.isAncestor(nearIdx, newIndex))
				continue;

			if (checker.canConnect(tree.node(newIndex), tree.node(nearIdx))) {
				tree.setParent(nearIdx, newIndex);
				tree.setCost(nearIdx, c);
			}
		}
	}

	// Attach the goal at the end, not on first contact: RRT* keeps refining.
	int bestNode = -1;
	double bestTotal = std::numeric_limits<double>::max();
	for (int n = 0; n < tree.size(); n++) {
		double d = configDist(tree.node(n), armgoal_anglesV_rad, numofDOFs);
		if (d >= goalThreshold)
			continue;
		double total = tree.trueCost(n) + d;
		if (total < bestTotal && checker.canConnect(tree.node(n), armgoal_anglesV_rad)) {
			bestTotal = total;
			bestNode = n;
		}
	}

	if (bestNode < 0) {
		printf("RRT*: Failed to reach the goal within the sample limit.\n");
		return;
	}

	// The last waypoint must be the goal exactly 
	Path path = tree.extractPath(bestNode);
	path.push_back(vector<double>(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs));

	printf("RRT*: %d nodes, path of %d waypoints, cost %f\n",
		   tree.size(), (int)path.size(), pathCost(path));
	emitPlan(path, plan, planlength);
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              PRM IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerPRM(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    /* TODO: Replace with your implementation */
    // planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);

	*plan = NULL;
	*planlength = 0;

	// Hyper parameters
	const int num_samples = 2000;
	const int k = 10;
	const double sigma = PI / 6.0;		

	CollisionChecker checker(map, x_size, y_size, numofDOFs);
	ConfigSampler sampler(numofDOFs);

	// Step 1: Construct the roadmap graph 
	vector<vector<double>> samples;		// C_free
	vector<vector<int>> adj;			// adjacency list
	samples.reserve(num_samples);
	adj.reserve(num_samples);

	vector<double> q(numofDOFs);
	vector<int> neighbors;

	// Attach node idx to its nearest neighbours, and how many edges created.
	auto connectNode = [&](int idx, int numNeighbors) {
		kNearest(samples, samples[idx].data(), numNeighbors, idx, numofDOFs, neighbors);

		int numEdges = 0;
		for (size_t c = 0; c < neighbors.size(); c++) {
			int n = neighbors[c];
			if (std::find(adj[idx].begin(), adj[idx].end(), n) != adj[idx].end())
				continue;			
			if (checker.canConnect(samples[idx].data(), samples[n].data())) {
				adj[idx].push_back(n);
				adj[n].push_back(idx);	
				numEdges++;
			}
		}
		return numEdges;
	};

	auto addNode = [&](const double* config) {
		samples.push_back(vector<double>(config, config + numofDOFs));
		adj.push_back(vector<int>());
		return (int)samples.size() - 1;
	};

	// Gaussian sampling 
	while ((int)samples.size() < num_samples) {
		if (!sampler.gaussianPair(q.data(), sigma, checker))
			continue;
		connectNode(addNode(q.data()), k);
	}

	// Step 2: Connect the start and goal configurations to the roadmap graph.
	// Both are guaranteed collision-free: main() validates them before calling.
	// Add both endpoints before connecting either, so that a directly visible
	// start-goal pair gets its own edge.
	int startIdx = addNode(armstart_anglesV_rad);
	int goalIdx  = addNode(armgoal_anglesV_rad);

	// Endpoints get a larger neighbourhood: their positions are fixed, there is
	// only one attempt at attaching them, and Gaussian sampling leaves gaps.
	const int k_endpoints = 5 * k;

	if (connectNode(startIdx, k_endpoints) == 0) {
		printf("PRM: failed to connect the start configuration to the roadmap\n");
		return;
	}
	if (connectNode(goalIdx, k_endpoints) == 0) {
		printf("PRM: failed to connect the goal configuration to the roadmap\n");
		return;
	}

	// Step 3: A* over the roadmap, from startIdx to goalIdx.
	// Edge cost is C-space distance, so configDist is admissible and consistent.
	const int numNodes = (int)samples.size();
	const double INF = std::numeric_limits<double>::infinity();

	vector<double> g(numNodes, INF);	// cost of the best path found so far
	vector<int> parent(numNodes, -1);	// backpointers, for reconstructing the path
	vector<bool> closed(numNodes, false);

	auto heuristic = [&](int n) {
		return configDist(samples[n].data(), samples[goalIdx].data(), numofDOFs);
	};

	// OPEN list as a min-heap of (f = g + h, node). std::priority_queue is a
	// max-heap by default, hence the greater<> comparator.
	std::priority_queue<std::pair<double, int>,
						vector<std::pair<double, int>>,
						std::greater<std::pair<double, int>>> open;

	g[startIdx] = 0.0;
	open.push({heuristic(startIdx), startIdx});

	while (!open.empty()) {
		int s = open.top().second;
		open.pop();

		// No decrease-key, so improved nodes are re-pushed and stale copies remain.
		// The first pop of a node is the cheapest; skip the duplicates.
		if (closed[s])
			continue;
		closed[s] = true;

		if (s == goalIdx)			// with a consistent heuristic this is optimal
			break;

		for (size_t a = 0; a < adj[s].size(); a++) {
			int sp = adj[s][a];
			if (closed[sp])
				continue;
			double cost = g[s] + configDist(samples[s].data(), samples[sp].data(), numofDOFs);
			if (cost < g[sp]) {
				g[sp] = cost;
				parent[sp] = s;
				open.push({cost + heuristic(sp), sp});
			}
		}
	}

	if (!closed[goalIdx]) {
		// Both endpoints attached, but they landed in different components
		printf("PRM: start and goal are not connected in the roadmap\n");
		return;
	}

	// Walk the backpointers from the goal and flip the result. samples[startIdx]
	// and samples[goalIdx] are exact copies of the configurations passed in, so
	// main()'s equalDoubleArrays checks hold.
	Path path;
	for (int n = goalIdx; n != -1; n = parent[n])
		path.push_back(samples[n]);
	std::reverse(path.begin(), path.end());

	printf("PRM: found a path with %d waypoints, cost %f\n",
		   (int)path.size(), pathCost(path));
	emitPlan(path, plan, planlength);
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MAIN FUNCTION                                                      //
//                                                                                                                   //
//*******************************************************************************************************************//

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;

    std::string mapDirPath = MAPS_DIR;
    std::string mapFilePath = mapDirPath + "/" + argv[1];
    // g++ builds record a path relative to the process cwd (often build/).
    // visualization is run from the code directory, so store an absolute path.
#ifndef _WIN32
    char resolvedMapPath[PATH_MAX];
    if (realpath(mapFilePath.c_str(), resolvedMapPath) != nullptr) {
        mapFilePath = resolvedMapPath;
    }
#endif
    std::cout << "Reading problem definition from: " << mapFilePath << std::endl;
	tie(map, x_size, y_size) = loadMap(mapFilePath);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);

    std::string outputDir = OUTPUT_DIR;
	string outputFile = outputDir + "/" + argv[6];
	std::cout << "Writing solution to: " << outputFile << std::endl;

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;

    // Call the corresponding planner function
    if (whichPlanner == PRM)
    {
        plannerPRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRT)
    {
        plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTCONNECT)
    {
        plannerRRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTSTAR)
    {
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else
    {
        planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << mapFilePath << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}