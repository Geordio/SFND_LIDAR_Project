/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

// void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
// {
//     // ----------------------------------------------------
//     // -----Open 3D viewer and display simple highway -----
//     // ----------------------------------------------------

//     // RENDER OPTIONS
//     bool renderScene = false;
//     std::vector<Car> cars = initHighway(renderScene, viewer);

//     // TODO:: Create lidar sensor
//     Lidar *lidar = new Lidar(cars, 0);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = lidar->scan();
//     // Vect3(0,0,0)
//     // renderRays(viewer,  Vect3(0,0,1), pointCloud);
//     std::__cxx11::string name = "PointCloud";
//     Color colorRed = Color(255, 0, 0);
//     Color colorGreen = Color(0, 255, 0);
//     Color colorWhite = Color(255, 255, 255);
//     Color colorBlue = Color(0, 0, 255);
//     // renderPointCloud(viewer, pointCloud, name, colorWhite);

//     // void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color)
//     // TODO:: Create point processor
//     // ProcessPointClouds<pcl::PointXYZ>* pointCloudProcessor = new ProcessPointClouds<pcl::PointXYZ>();

//     ProcessPointClouds<pcl::PointXYZ> pointCloudProcessor;
//     std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointCloudProcessor.SegmentPlane(pointCloud, 100, 0.2);

//     // renderPointCloud(viewer, segmentCloud.first, "ObstacleCloud", colorRed);
//     // renderPointCloud(viewer, segmentCloud.second, "PlaneCloud", colorGreen);

//     // run clustering on the obstacle cloud
//     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointCloudProcessor.Clustering(segmentCloud.first, 5, 3, 30);

//     cout << "returned to env" << endl; 

//     int clusterId = 0;
//     std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

//     for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
//     {
//         std::cout << "cluster size ";
//         pointCloudProcessor.numPoints(cluster);
//         renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);

//         // Box box = pointCloudProcessor.BoundingBox(cluster);
//         // renderBox(viewer, box, clusterId);
//         ++clusterId;
//     }
// }

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

/// new cityBlock method, to allow streaming of pcd data

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{

    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------


    // cout << "input points: " << inputCloud->size() << endl;

    // Experiment with the ? values and find what works best
    float voxel_size = 0.3;
    float cropBoxRearx = -10;
    float cropBoxSizey = 7;
    float cropBoxFromx = 30; 
    float cropBoxBottomz = -5; // setting a b
    float cropBoxTopZ = 0;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, voxel_size, Eigen::Vector4f(cropBoxRearx, -cropBoxSizey, cropBoxBottomz, 1), Eigen::Vector4f(cropBoxFromx, cropBoxSizey, cropBoxTopZ, 1));
    // pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, voxel_size, Eigen::Vector4f(-10, -7, -10, 1), Eigen::Vector4f(30, 7, 4, 1));
    // cout << "filtered points: " << filterCloud->size() << endl;
    // renderPointCloud(viewer, filterCloud, "filterCloud");
    std::__cxx11::string name = "PointCloud";

    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 1), Color(0, 0, 1)};
    // ProcessPointClouds<pcl::PointXYZ> pointCloudProcessor;
    int maxIterations = 50;
    float distanceTol = 0.5; // s
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, maxIterations, distanceTol);

    // renderPointCloud(viewer, segmentCloud.first, "ObstacleCloud", colorRed);
    renderPointCloud(viewer, segmentCloud.second, "PlaneCloud", Color(0, 1, 0));

    // run clustering on the obstacle cloud
    // maxSize and minSize not used for self implemented KDtree
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first,distanceTol);// 0.5);//, 3, 30);

    int clusterId = 0;


    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }

}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // new code to go with the new cityblock to handle streaming
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    // TODO update path
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    // flag to allow swithcing between processing a single frame for dev and debug purposes
    bool singleFrame = false;

    if (singleFrame)
    {
    // simpleHighway(viewer);
    // cityBlock(viewer);

}


    while (!viewer->wasStopped())
    {

        if (!singleFrame)
        {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        }
        viewer->spinOnce();
    }
}
