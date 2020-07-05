/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

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
	
	// TODO: Fill in this function

	// number of samples is 2 for the line
	int goal_samples = 2;

// Line Formula 
// Ax + By + C = 0

// (y1-y2)x + (x2-x1)y + (x1*y2 - x2*y1) = 0
// A = y1-y2
// B = x2-x1
// C = (x1*y2 - x2*y1)

// Point(x,y)
// Distance d = abs(Ax + By + C) / sqrt((pow(A,2) + (pow(B,2) ))
// calculate the line coefficients

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;

	int pointCloudSize = cloud->points.size();

	cout << "pointCloud size:\t" << pointCloudSize << endl;

	// For max iterations 
	for (int index = 0; index < maxIterations; index++) {
		std::vector<int> samples;
		std::unordered_set<int> inliers;
		// std::unordered_set<int> samples2;

	// generate 2 random numbers
		int sampleIndex1 = rand() % pointCloudSize; 
		int sampleIndex2 = rand() % pointCloudSize; 
		while (sampleIndex1 == sampleIndex2)
			sampleIndex2 = rand() % pointCloudSize; 

		samples.insert(samples.begin(),sampleIndex1);
		// samples2.insert(sampleIndex1);

		cout << "samples:\t" << sampleIndex1 << ", " << sampleIndex2 << endl;

		float x1 = cloud->points[sampleIndex1].x;
		float y1 = cloud->points[sampleIndex1].y;
			
		float x2 = cloud->points[sampleIndex2].x;
		float y2 = cloud->points[sampleIndex2].y;

		float A = y1-y2;
		float B = x2-x1;
		float C = (x1*y2 - x2*y1);

		cout << "A: " << A << " ,B: " << B << ", C: " << C << endl;		

		for (int j = 0; j < pointCloudSize; j++) {

			// check to see if the index was one of the ones selected randomly
			if ((j != sampleIndex1) && (j != sampleIndex2)) {

				float x = cloud->points[j].x;
				float y = cloud->points[j].y;

				float d = abs(A*x + B*y + C) / sqrt((pow(A,2) + (pow(B,2) )));

				cout << "index:\t"<< j <<"\t, distance:\t" << d << endl;
				// check if the distance is smaller than the passed tolerance
				if (d < distanceTol) {
					inliers.insert(j);	
				}
			}
			else {
				inliers.insert(j);
			}
		}
		cout << "----------------------------" << endl;
		cout << "Inliers: " << inliersResult.size() << endl;

		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

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
