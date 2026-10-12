// utility.h - helpers shared by the four planners in planner.cpp.

#ifndef UTILITY_H
#define UTILITY_H

#include <vector>
#include <random>

#ifndef PI
#define PI 3.141592654
#endif

// Defined in planner.cpp with the rest of the given code.
int IsValidArmConfiguration(double* angles, int numofDOFs, double* map,
							int x_size, int y_size);

// A path as a list of configurations, start first.
typedef std::vector<std::vector<double> > Path;

// Euclidean distance between two configurations.
double configDist(const double* a, const double* b, int numofDOFs);

// The map and the arm, so the planners stop passing four context values around.
class CollisionChecker {
public:
	CollisionChecker(double* map, int x_size, int y_size, int numofDOFs);

	int dof() const { return m_dof; }

	// Is this configuration collision-free?
	bool isValid(double* q) const;

	// Is the straight line between a and b free? Checks strictly between them,
	// so both endpoints must already be known valid.
	bool canConnect(double* a, double* b) const;

	// Step from 'from' towards 'towards' by at most stepSize, write it to out
	// and check it. False means blocked, which is what stops a greedy connect.
	bool tryExtend(double* from, double* towards, double stepSize, double* out) const;

private:
	double* m_map;
	int m_xSize, m_ySize, m_dof;
};

// The random engine and the ways of drawing a configuration. Owning the engine
// avoids the default-seeded local that made every run come out identical.
class ConfigSampler {
public:
	explicit ConfigSampler(int numofDOFs);

	// Each joint uniform over [0, 2*PI).
	void uniform(double* q);

	// The goal itself with probability bias, uniform otherwise.
	void withGoalBias(double* q, double* goal, double bias);

	// Gaussian sampling (Boor et al. 1999): keep a configuration only when
	// exactly one of a uniform/Gaussian pair is free, which lands it near an
	// obstacle boundary. False means nothing was written.
	bool gaussianPair(double* q, double sigma, const CollisionChecker& checker);

private:
	int m_dof;
	std::mt19937 m_engine;
	std::uniform_real_distribution<double> m_angle;
	std::uniform_real_distribution<double> m_coin;
};

// The search tree behind RRT, RRT-Connect and RRT*. Node n occupies
// [n*dof, (n+1)*dof) of one block, reserved up front so node() stays valid.
class Tree {
public:
	Tree(int numofDOFs, int reserveNodes);

	int size() const { return (int)(m_nodes.size() / m_dof); }
	double* node(int i) { return m_nodes.data() + (size_t)i * m_dof; }

	int addRoot(double* config);
	int addNode(double* config, int parentIdx);

	// Cost-to-come. Only RRT* tracks it.
	double cost(int i) const { return m_costs[i]; }
	void setCost(int i, double c) { m_costs[i] = c; }
	void setParent(int i, int p) { m_parents[i] = p; }

	// Nearest node to q; its distance goes into outDist.
	int nearest(double* q, double& outDist);

	// Every node within radius of q.
	void near(double* q, double radius, std::vector<int>& out);

	// Is 'candidate' on the parent chain of 'of'? Stops a rewire making a cycle.
	bool isAncestor(int candidate, int of) const;

	// Cost-to-come recomputed by walking parents, since a rewire leaves the
	// stored costs of descendants stale.
	double trueCost(int i);

	// Configurations from the root down to leaf.
	Path extractPath(int leaf);

private:
	int m_dof;
	std::vector<double> m_nodes;
	std::vector<int> m_parents;
	std::vector<double> m_costs;
};

// Sum of per-joint motion, the same cost the grader computes.
double pathCost(const Path& path);

// Greedy shortcutting: from each waypoint kept, jump to the farthest later one
// still reachable in a free straight line. Start and goal always survive.
void shortcutPath(Path& path, const CollisionChecker& checker);

// Hand the path back in the malloc'd layout main() expects.
void emitPlan(const Path& path, double*** plan, int* planlength);

// Indices of the k entries of 'points' closest to q, nearest first.
void kNearest(const std::vector<std::vector<double> >& points, const double* q,
			  int k, int skipIndex, int numofDOFs, std::vector<int>& out);

#endif // UTILITY_H
