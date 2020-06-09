/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor 
  	Lidar *lidar = new Lidar(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    // renderPointCloud(viewer, cloud, "highway");

    // Create point processor
    auto *processor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,  pcl::PointCloud<pcl::PointXYZ>::Ptr> segCloud = processor->SegmentPlane(cloud, 100, 0.2);
    renderPointCloud(viewer, segCloud.first, "cars", Color(1, 0, 0));
    renderPointCloud(viewer, segCloud.second, "plane", Color(0, 1, 0));
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, .1 , Eigen::Vector4f (-15.0, -6.0, -2.0, 1), Eigen::Vector4f ( 15.0, 7.0, 5, 1));
    auto clouds = pointProcessorI->SegmentPlanePCL(filterCloud, 20, .5);
    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> cars = pointProcessorI->ClusteringPCL(clouds.first, .5, 10, 2000);
    int car_id = 0;
    for (auto car : cars) {
        Box b = pointProcessorI->BoundingBox(car);
        renderBox(viewer, b, car_id);
        // renderPointCloud(viewer ,car ,"car"+std::to_string(car_id), Color(1, 0, 0));
        car_id++;
    }
    renderPointCloud(viewer,clouds.first,"cars", Color(0, 0, 1));
    renderPointCloud(viewer,clouds.second,"plane", Color(0, 1, 0));
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, .1 , Eigen::Vector4f (-15.0, -6.0, -2.0, 1), Eigen::Vector4f ( 15.0, 7.0, 5, 1));
    auto clouds = pointProcessorI->SegmentPlane(filterCloud, 50, .03);
    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> cars = pointProcessorI->Clustering(clouds.first, 0.4, 20, 2000);
    int car_id = 0;
    for (auto car : cars) {
        Box b = pointProcessorI->BoundingBox(car);
        renderBox(viewer, b, car_id);
        // renderPointCloud(viewer ,car ,"car"+std::to_string(car_id), Color(1, 0, 0));
        car_id++;
    }
    renderPointCloud(viewer,clouds.first,"cars", Color(0, 0, 1));
    renderPointCloud(viewer,clouds.second,"plane", Color(0, 1, 0));
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    // cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}