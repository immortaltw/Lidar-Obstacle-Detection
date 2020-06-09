#ifndef CLUSTER_H
#define CLUSTER_H

#include <vector>
#include "kdtree.h"

void proximity(const std::vector<std::vector<float>>& points,
				int id,
				KdTree* tree,
				float distanceTol,
				std::vector<int> &cluster,
				std::vector<bool>& processed);

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize);

#endif