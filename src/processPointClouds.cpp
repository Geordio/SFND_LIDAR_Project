// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <unordered_set>
// #include "kdtree3d.h"
// #include "quiz/ransac/ransac2d.cpp"

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // std::cout << cloud->points.size() << std::endl;
}


/// filters the cloud by using voxel grid and then cropping the point cloud. 
/// ?? Should crop first then voxel? check processing timing
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  typename pcl::PointCloud<PointT>::Ptr filterCloud(new pcl::PointCloud<PointT>);

  // typename pcl::PointCloud<PointT>::Ptr filterCloud;
  // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  // Create the filtering object
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(filterRes, filterRes, filterRes);
  sor.filter(*filterCloud);
  // sor.filter (*cloud_filtered);

  // typename pcl::PointCloud<PointT>::Ptr regionCloud(new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> cropBox(true);
  // pcl::CropBox<PointT>* cropBox = new pcl::CropBox<PointT> (false);

  cropBox.setInputCloud(filterCloud);
  cropBox.setMax(maxPoint);
  cropBox.setMin(minPoint);
  cropBox.filter(*filterCloud);

  // crop the roof out, from the lesson solution
  std::vector<int> indices;
  pcl::CropBox<PointT> cropRoof(true);
  // pcl::CropBox<PointT>* cropBox = new pcl::CropBox<PointT> (false);

  cropBox.setInputCloud(filterCloud);
  cropBox.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  cropBox.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  cropBox.filter(indices);

  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};

  //iterate through the indices retruned from the roof crop, adding to inliers
  for (int point : indices)
    inliers->indices.push_back(point);

  // TODO: add the roof filer from the solution video
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(filterCloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*filterCloud);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  return filterCloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

  typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

  for (int index : inliers->indices)
  {
    planeCloud->points.push_back(cloud->points[index]);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacleCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
  // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(planeCloud, cloud);
  return segResult;
}

/// arguments
/// pointer to cloud
/// maxIterations
/// distanceTol increase value to allow wider spaced points to be clustered
template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // calculate the line coefficients
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered;

  int pointCloudSize = cloud->points.size();

  // For max iterations
  for (int index = 0; index < maxIterations; index++)
  {
    std::vector<int> samples;
    std::unordered_set<int> inliers;
    // std::unordered_set<int> samples2;

    // generate 2 random numbers
    int sampleIndex1 = rand() % pointCloudSize;
    int sampleIndex2 = rand() % pointCloudSize;

    // check that the generate random number is unique, and if not then create a new one until it is
    while (sampleIndex1 == sampleIndex2)
      sampleIndex2 = rand() % pointCloudSize;

    // generate a 3rd point for the plane representation
    int sampleIndex3 = rand() % pointCloudSize;

    // check that the generate random number is unique, and if not then create a new one until it is
    while ((sampleIndex1 == sampleIndex3) || (sampleIndex2 == sampleIndex3))
      sampleIndex3 = rand() % pointCloudSize;

    samples.insert(samples.begin(), sampleIndex1);
    // samples2.insert(sampleIndex1);

    // std::cout << "samples:\t" << sampleIndex1 << ", " << sampleIndex2 << std::endl;

    // probably better way to do this, but get it working first
    // define variables for x, y, z for each point and populate
    float x1 = cloud->points[sampleIndex1].x;
    float y1 = cloud->points[sampleIndex1].y;
    float z1 = cloud->points[sampleIndex1].z;

    float x2 = cloud->points[sampleIndex2].x;
    float y2 = cloud->points[sampleIndex2].y;
    float z2 = cloud->points[sampleIndex2].z;

    float x3 = cloud->points[sampleIndex3].x;
    float y3 = cloud->points[sampleIndex3].y;
    float z3 = cloud->points[sampleIndex3].z;
#// these are left over from the line implementation. Delete later
    // float A = y1 - y2;
    // float B = x2 - x1;
    // float C = (x1 * y2 - x2 * y1);

    float A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    float B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    float C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    float D = -(A * x1 + B * y1 + C * z1);

    // std::cout << "A: " << A << " ,B: " << B << ", C: " << C << std::endl;

    for (int j = 0; j < pointCloudSize; j++)
    {
      // check to see if the index was one of the ones selected randomly if not, then check the distance to the line / plane
      // if it was one of the gerated points then its and inlier and add to the inlier set
      if ((j != sampleIndex1) && (j != sampleIndex2) && (j != sampleIndex3))
      {

        float x = cloud->points[j].x;
        float y = cloud->points[j].y;
        float z = cloud->points[j].z;

        // first line is for a line.... second is a plane, commented out line implementation
        // float d = abs(A*x + B*y + C) / sqrt((pow(A,2) + (pow(B,2) )));

        // IMPORTANT LESSON LEARNT!!!! ABS and FABS. grrrrr
        float d = fabs(A * x + B * y + C * z + D) / sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2));

        // cout << "index:\t"<< j <<"\t, distance:\t" << d << endl;
        // check if the distance is smaller than the passed tolerance
        if (d < distanceTol)
        {
          inliers.insert(j);
        }
      }
      else
      {
        inliers.insert(j);
      }
    }

// check to see if this has the highest number of inliers in the iteration cycle
    if (inliers.size() > inliersResult.size())
    {
      inliersResult = inliers;
    }

  }

  return inliersResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  std::unordered_set<int> inliers = Ransac(cloud, maxIterations, distanceThreshold);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

  // from the inliers as an ordered set, generate a Pointcloud for the in liers and out liers
  for (int index = 0; index < cloud->points.size(); index++)
  {
    pcl::PointXYZI point = cloud->points[index];
    if (inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);  }


  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
  segResult.first = cloudOutliers;
  segResult.second = cloudInliers;

  return segResult;
}

template <typename PointT>


void ProcessPointClouds<PointT>::clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int> &cluster, std::vector<bool> &processed, KdTree_simple *tree, float distanceTol)
{
  processed[indice] = true;
  cluster.push_back(indice);

  std::vector<int> nearest = tree->search(points[indice], distanceTol);

  for (int id : nearest)
  {
    if (!processed[id])
      clusterHelper(id, points, cluster, processed, tree, distanceTol);
  }
}


/// minSize, maxSize no longer used since own implementation of KDtree, removed from signature
template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance)
{

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

  // need to replace with own KDTree implementation for project
  // TODO, expand the KDTree from the quiz to be 3d. need to cover x,y,z dimensions? i.e mod 3.

  // create tree object
  // add points
  // cluster    	euclideanCluster method returns std::vector<std::vector<int>> clusters
  // might be able to substitue in for the cluster_indicies?

  KdTree_simple *tree_simple = new KdTree_simple;
  std::vector<std::vector<float>> points;

  //
  for (int i = 0; i < cloud->points.size(); i++)
  {
    std::vector<float> vect({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});

    // cout << "vect: " << vect[0] << "," << vect[1] << "," << vect[2] << endl;
    // points_test.push_back((cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
    points.push_back(vect);
    tree_simple->insert(vect, i);
    
  }

  //TODO, include this in the loop above?
  // for (int i = 0; i < points.size(); i++)
  //   tree_simple->insert(points[i], i);

  std::vector<std::vector<int>> clusters_new;
  std::vector<bool> processed(points.size(), false);

  int i = 0;

  while (i < points.size())
  {
    if (processed[i])
    {
      i++;
      continue;
    }

    std::vector<int> cluster;
    clusterHelper(i, points, cluster, processed, tree_simple, clusterTolerance);
    clusters_new.push_back(cluster);
    i++;
  }

  // TODO convert the  "std::vector<std::vector<int>> clusters_new;"" to the expected return type std::vector<typename pcl::PointCloud<PointT>::Ptr>
  // (just because I'm trying to keep the interface the same...)
  // clusters is the current returned variable

  // iterate through the cluster indices, and add all the points that relate to the clusters to the clout cluster that will be returned
  // iterate through the point cloud
  for (std::vector<std::vector<int>>::const_iterator it = clusters_new.begin(); it != clusters_new.end(); ++it)
  {
    // create a new cloud that represents a cluster
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);

    // iterate through the points idx in the cluster
    for (std::vector<int>::const_iterator pit = it->begin(); pit != it->end(); ++pit)
    {
      cloud_cluster->points.push_back((*cloud)[*pit]);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.push_back(cloud_cluster);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  // std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

  return clusters;
}
template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

  std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}
