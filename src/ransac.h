#ifndef RANSAC_H
#define RANSAC_H

#include "./render/render.h"
#include <unordered_set>
#include <random>
#include <vector>
#include <iterator>

template<typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations
	for (int i=0; i<maxIterations; ++i) {
		// Randomly sample subset and fit plane
		std::unordered_set<int> inliners;
		while (inliners.size() < 3) {
			inliners.insert(rand()%cloud->points.size());
		}

		// Measure distance between every point and fitted line
        // TODO: Use eigen
		auto itr = inliners.begin();
		float x1 = cloud->points[*itr].x;
		float y1 = cloud->points[*itr].y;
        float z1 = cloud->points[*itr].z;
		itr++;
		float x2 = cloud->points[*itr].x;
		float y2 = cloud->points[*itr].y;
        float z2 = cloud->points[*itr].z;
		itr++;
		float x3 = cloud->points[*itr].x;
		float y3 = cloud->points[*itr].y;
        float z3 = cloud->points[*itr].z;

        std::vector<float> v1 {x2 - x1, y2 - y1, z2 - z1};
        std::vector<float> v2 {x3 - x1, y3 - y1, z3 - z1};

        std::vector<float> v1_X_v2 = {v1[1]*v2[2]-v1[2]*v2[1], v1[2]*v2[0]-v1[0]*v2[2], v1[0]*v2[1]-v1[1]*v2[0]};
		float A = v1_X_v2[0];
		float B = v1_X_v2[1];
		float C = v1_X_v2[2];
        float D = -(A*x1+B*y1+C*z1);

		for (int j=0; j<cloud->points.size(); ++j) {
            // Skip points which generate the plane.
			if (inliners.count(j)>0) continue;

			// If distance is smaller than threshold count it as inlier
			float x_ = cloud->points[j].x;
			float y_ = cloud->points[j].y;
            float z_ = cloud->points[j].z;
			float dist = fabs(A * x_ + B * y_ + C * z_ + D) / sqrt(A * A + B * B + C * C);
			if (dist <= distanceTol) {
				inliersResult.insert(j);
			}
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

#endif