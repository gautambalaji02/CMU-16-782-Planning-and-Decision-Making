// utility.cpp - implementation of the shared planner helpers.

#include "utility.h"

#include <math.h>
#include <vector>
#include <random>
#include <algorithm>
#include <limits>
#include <stdlib.h>

using std::vector;

double configDist(const double* a, const double* b, int numofDOFs) {
	double sum = 0.0;
	for (int j = 0; j < numofDOFs; j++) {
		double d = a[j] - b[j];
		sum += d * d;
	}
	return sqrt(sum);
}

// Wrap an angle into [0, 2*PI). Only the Gaussian sampler needs this.
static double wrapAngle(double a) {
	a = fmod(a, 2 * PI);
	if (a < 0) a += 2 * PI;
	return a;
}

// --- CollisionChecker ---

CollisionChecker::CollisionChecker(double* map, int x_size, int y_size, int numofDOFs)
	: m_map(map), m_xSize(x_size), m_ySize(y_size), m_dof(numofDOFs) {}

bool CollisionChecker::isValid(double* q) const {
	return IsValidArmConfiguration(q, m_dof, m_map, m_xSize, m_ySize) != 0;
}

bool CollisionChecker::canConnect(double* a, double* b) const {
	const double step = PI / 90.0;		// ~2 degrees per interpolation step
	int numSteps = (int)(configDist(a, b, m_dof) / step) + 1;
	vector<double> q(m_dof);
	for (int s = 1; s < numSteps; s++) {
		double t = (double)s / numSteps;
		for (int j = 0; j < m_dof; j++)
			q[j] = a[j] + t * (b[j] - a[j]);
		if (!isValid(q.data()))
			return false;
	}
	return true;
}

bool CollisionChecker::tryExtend(double* from, double* towards,
								 double stepSize, double* out) const {
	double d = configDist(from, towards, m_dof);
	if (d < 1e-9)
		return false;					// already there, nothing to add

	// A fixed length, not a fixed fraction: a fraction shrinks as the tree
	// densifies and the tree stops expanding. Land exactly on a close target.
	double scale = (d > stepSize) ? stepSize / d : 1.0;
	for (int j = 0; j < m_dof; j++)
		out[j] = from[j] + scale * (towards[j] - from[j]);

	// Endpoint first: one check instead of ~15, and it rejects most failures.
	return isValid(out) && canConnect(from, out);
}

// --- ConfigSampler ---

ConfigSampler::ConfigSampler(int numofDOFs)
	: m_dof(numofDOFs),
	  m_engine(std::random_device()()),
	  m_angle(0.0, 2 * PI),
	  m_coin(0.0, 1.0) {}

void ConfigSampler::uniform(double* q) {
	for (int j = 0; j < m_dof; j++)
		q[j] = m_angle(m_engine);
}

void ConfigSampler::withGoalBias(double* q, double* goal, double bias) {
	if (m_coin(m_engine) < bias) {
		for (int j = 0; j < m_dof; j++)
			q[j] = goal[j];
	} else {
		uniform(q);
	}
}

bool ConfigSampler::gaussianPair(double* q, double sigma,
								 const CollisionChecker& checker) {
	std::normal_distribution<double> gauss(0.0, sigma);
	vector<double> q1(m_dof), q2(m_dof);
	for (int j = 0; j < m_dof; j++) {
		q1[j] = m_angle(m_engine);
		// Wrapping keeps samples in [0, 2*PI) so configDist stays meaningful
		q2[j] = wrapAngle(q1[j] + gauss(m_engine));
	}

	bool v1 = checker.isValid(q1.data());
	bool v2 = checker.isValid(q2.data());
	if (v1 == v2)						// both free or both blocked
		return false;

	vector<double>& keep = v1 ? q1 : q2;
	for (int j = 0; j < m_dof; j++)
		q[j] = keep[j];
	return true;
}

// --- Tree ---

Tree::Tree(int numofDOFs, int reserveNodes) : m_dof(numofDOFs) {
	m_nodes.reserve((size_t)reserveNodes * numofDOFs);
	m_parents.reserve(reserveNodes);
	m_costs.reserve(reserveNodes);
}

int Tree::addRoot(double* config) {
	return addNode(config, 0);			// the root is its own parent
}

int Tree::addNode(double* config, int parentIdx) {
	m_nodes.insert(m_nodes.end(), config, config + m_dof);
	m_parents.push_back(parentIdx);
	m_costs.push_back(0.0);
	return size() - 1;
}

int Tree::nearest(double* q, double& outDist) {
	int best = 0;
	outDist = std::numeric_limits<double>::max();
	for (int n = 0; n < size(); n++) {
		double d = configDist(q, node(n), m_dof);
		if (d < outDist) {
			outDist = d;
			best = n;
		}
	}
	return best;
}

void Tree::near(double* q, double radius, vector<int>& out) {
	out.clear();
	for (int n = 0; n < size(); n++) {
		if (configDist(q, node(n), m_dof) <= radius)
			out.push_back(n);
	}
}

bool Tree::isAncestor(int candidate, int of) const {
	for (int a = of; a != 0; a = m_parents[a]) {
		if (a == candidate)
			return true;
	}
	return candidate == 0;				// the root is an ancestor of everything
}

double Tree::trueCost(int i) {
	double c = 0.0;
	for (int a = i; a != 0; a = m_parents[a])
		c += configDist(node(a), node(m_parents[a]), m_dof);
	return c;
}

Path Tree::extractPath(int leaf) {
	vector<int> indices;
	for (int n = leaf; ; n = m_parents[n]) {
		indices.push_back(n);
		if (n == 0)
			break;
	}
	std::reverse(indices.begin(), indices.end());

	Path path;
	path.reserve(indices.size());
	for (size_t i = 0; i < indices.size(); i++) {
		double* config = node(indices[i]);
		path.push_back(vector<double>(config, config + m_dof));
	}
	return path;
}

// --- Path helpers ---

double pathCost(const Path& path) {
	double total = 0.0;
	for (size_t i = 1; i < path.size(); i++) {
		for (size_t j = 0; j < path[i].size(); j++) {
			double d = fabs(path[i][j] - path[i - 1][j]);
			total += std::min(d, fabs(2 * PI - d));
		}
	}
	return total;
}

void shortcutPath(Path& path, const CollisionChecker& checker) {
	if (path.size() < 3)
		return;							// nothing between the endpoints to drop

	Path out;
	out.push_back(path.front());
	for (int i = 0; i < (int)path.size() - 1; ) {
		// Scan from the far end so each kept edge is the longest legal one
		int j = (int)path.size() - 1;
		while (j > i + 1 && !checker.canConnect(path[i].data(), path[j].data()))
			j--;
		out.push_back(path[j]);
		i = j;
	}
	path.swap(out);
}

void emitPlan(const Path& path, double*** plan, int* planlength) {
	*planlength = (int)path.size();
	*plan = (double**) malloc(path.size() * sizeof(double*));
	for (size_t i = 0; i < path.size(); i++) {
		int dof = (int)path[i].size();
		(*plan)[i] = (double*) malloc(dof * sizeof(double));
		for (int d = 0; d < dof; d++)
			(*plan)[i][d] = path[i][d];
	}
}

void kNearest(const vector<vector<double> >& points, const double* q,
			  int k, int skipIndex, int numofDOFs, vector<int>& out) {
	vector<std::pair<double, int> > cand;
	cand.reserve(points.size());
	for (int n = 0; n < (int)points.size(); n++) {
		if (n == skipIndex)				// a node is its own nearest neighbour
			continue;
		cand.push_back(std::make_pair(configDist(q, points[n].data(), numofDOFs), n));
	}

	// Only the first kk entries matter, so don't pay to order the rest
	int kk = std::min<int>(k, (int)cand.size());
	std::partial_sort(cand.begin(), cand.begin() + kk, cand.end());

	out.clear();
	for (int c = 0; c < kk; c++)
		out.push_back(cand[c].second);
}
