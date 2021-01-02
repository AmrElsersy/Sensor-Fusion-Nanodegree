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
    Car car4( Vect3(20,-2,0), Vect3(7,4,4), Color(1,0,1), "ray2" );

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);
    cars.push_back(car4);


        renderHighway(viewer);
        egoCar.render(viewer);
    if(renderScene)
    {
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
        car4.render(viewer);
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
    bool renderBoxes = true;

    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    auto lidar = new Lidar(cars, 0);
    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();

    // renderRays(viewer, lidar->position, cloud);  
    // renderPointCloud(viewer, cloud, "cloudRay2");

    auto processPointCloud = new ProcessPointClouds<pcl::PointXYZ>();

    auto plane_and_obstacles = processPointCloud->SegmentPlane(cloud, 1000, 0.2);
    auto planeCloud = plane_and_obstacles.first;
    auto obstaclesCloud = plane_and_obstacles.second;

    // renderPointCloud(viewer, obstaclesCloud, "obstacles", Color(0,1,0));
    // renderPointCloud(viewer, planeCloud, "plane", Color(1,0,0));

    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(1,1,0), Color(0,1,1)};
    auto clusters = processPointCloud->Clustering(obstaclesCloud, 1.0, 4, 50);
    // auto clusters = processPointCloud->SersyClustering(obstaclesCloud, 1.0);

    for (int i = 0; i < clusters.size(); i++)
    {
        auto cluster = clusters[i];
        renderPointCloud(viewer, cluster, "cluster" + std::to_string(i), colors[i%5] );

        if (renderBoxes)
        {
            // Box box = processPointCloud->BoundingBox(cluster);
            BoxQ box = processPointCloud->BoundingBoxQ(cluster);
            renderBox(viewer, box, i, colors[i%5], 0.8);
        }
    }


}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* processPointCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    // Filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = processPointCloud->FilterCloud(cloud, 0.1, Eigen::Vector4f(-30,-6,-3,1), Eigen::Vector4f(30,6, 10,1));
    // renderPointCloud(viewer, filterCloud, "filter");

    // Segmentation
    auto plane_and_obstacles = processPointCloud->SegmentPlane(filterCloud, 100, 0.2);
    auto planeCloud = plane_and_obstacles.first;
    auto obstaclesCloud = plane_and_obstacles.second;

    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(1,1,0), Color(0,1,1)};
    // auto clusters = processPointCloud->Clustering(obstaclesCloud, 0.5, 20, 1000);
    auto clusters = processPointCloud->SersyClustering(obstaclesCloud, 0.5);
    
    for (int i = 0; i < clusters.size(); i++)
    {
        auto cluster = clusters[i];
        renderPointCloud(viewer, cluster, "cluster" + std::to_string(i), colors[i%5] );

        auto box = processPointCloud->BoundingBox(cluster);
        renderBox(viewer, box, i, colors[0], 1);
    }

    renderPointCloud(viewer, planeCloud, "cloud", Color(0,1,0));
    // renderPointCloud(viewer, cloud, "Cloud");
    
    // auto transform_cloud = processPointCloud->TransformPointCloud(cloud, -M_PI/4);
    // renderPointCloud(viewer, transform_cloud, "ray2");
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0.1, 0.1, 0.1);
    
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

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer Ray2"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI>* processPointCloud = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = processPointCloud->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    

    while (!viewer->wasStopped ())
    {       
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        cloud = processPointCloud->loadPcd((*streamIterator).string());
        cityBlock(viewer, processPointCloud, cloud);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}
