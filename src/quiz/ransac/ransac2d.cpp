/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>
#include <vector>
#include <iterator>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations 
	for (int i=0; i<maxIterations; ++i) {
		// Randomly sample subset and fit line
		std::unordered_set<int> inliners;
		while (inliners.size() < 2) {
			inliners.insert(rand()%cloud->points.size());
		}

		// Measure distance between every point and fitted line
		auto itr = inliners.begin();
		float x1 = cloud->points[*itr].x;
		float y1 = cloud->points[*itr].y;
		itr++;
		float x2 = cloud->points[*itr].x;
		float y2 = cloud->points[*itr].y;

		float A = x1 - x2;
		float B = y2 - y1;
		float C = x1 * y2 - x2 * y1;

		for (int j=0; j<cloud->points.size(); ++j) {
			if (inliners.count(j)>0) continue;

			// If distance is smaller than threshold count it as inlier
			float x3 = cloud->points[j].x;
			float y3 = cloud->points[j].y;
			float dist = fabs(A * x3 + B * y3 + C) / sqrt(A * A + B * B);
			if (dist <= distanceTol) {
				inliersResult.insert(j);
			}
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 100, 0.1);
	std::unordered_set<int> inliers = RansacPlane(cloud, 2500, 0.0008);


	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
