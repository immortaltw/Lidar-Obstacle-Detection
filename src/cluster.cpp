#include <iostream>
#include <vector>
#include "kdtree.h"

void proximity(const std::vector<std::vector<float>>& points,
				int id,
				KdTree* tree,
				float distanceTol,
				std::vector<int> &cluster,
				std::vector<bool>& processed) {
	processed[id] = true;
	cluster.push_back(id);
	auto neighbors = tree->search(points[id], distanceTol);
	for (int n : neighbors) {
		if (!processed[n]) {
			proximity(points, n, tree, distanceTol, cluster, processed);
		}
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);
	for (int i=0; i<points.size(); ++i) {
		if (!processed[i]) {
			std::vector<int> new_cluster;
			proximity(points, i, tree, distanceTol, new_cluster, processed);
            if (new_cluster.size() > maxSize || new_cluster.size() < minSize) continue;
			clusters.push_back(new_cluster);
		}
	}
 
	return clusters;

}