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


//----------------------------------------------------------------
// Equation of a Plane through Three Points
// Ax+By+Cz+D=0
// point1=(x1,y1,z1)
// point2=(x2,y2,z2)
// point3=(x3,y3,z3)

// Use point1 as a reference and define two vectors on the plane v1 and v2 as follows:

//     Vector v1 travels from point1 to point2.
//     Vector v2 travels from point1 to point3

// model plane using 2 vectors, from point 1 to 2, and point 1 to 3
// v1=<x2−x1,y2−y1,z2−z1> 
// v2=<x3−x1,y3−y1,z3−z1>

// d=∣A∗x+B∗y+C∗z+D∣/sqrt(pow(A,2)+pow(B,2) +pow(C,2)).

// generate 3 random numbers to pick points from the cloud.
//-----------------------------------------------------------------

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

// generate a 3rd point for the plane representation
		int sampleIndex3 = rand() % pointCloudSize; 		

		while ((sampleIndex1 == sampleIndex3) || (sampleIndex2 == sampleIndex3))
			sampleIndex3 = rand() % pointCloudSize; 
 

		samples.insert(samples.begin(),sampleIndex1);
		// samples2.insert(sampleIndex1);

		cout << "samples:\t" << sampleIndex1 << ", " << sampleIndex2 << endl;

		float x1 = cloud->points[sampleIndex1].x;
		float y1 = cloud->points[sampleIndex1].y;
		float z1 = cloud->points[sampleIndex1].z;
			
		float x2 = cloud->points[sampleIndex2].x;
		float y2 = cloud->points[sampleIndex2].y;
		float z2 = cloud->points[sampleIndex2].z;

		float x3 = cloud->points[sampleIndex3].x;
		float y3 = cloud->points[sampleIndex3].y;
		float z3 = cloud->points[sampleIndex3].z;

		float A = y1-y2;
		float B = x2-x1;
		float C = (x1*y2 - x2*y1);

	


		A = (y2 - y1) * (z3 - z1) - (z2 - z1)*(y3 - y1);
		B = (z2 - z1) * (x3 - x1) - (x2 - x1)*(z3 - z1);
		C = (x2 - x1) * (y3 - y1) - (y2 - y1)*(x3 - x1);
	 float D = -(A * x1+ B * y1+C *z1);

// TODO: would be better to define v1 and v2 using points. ie  v1 = p2 - p1; v2 = p3-p1
// the resulitng vector form the couple of steps would be v1×v2=<i,j,k>. can then access th eappropriate element
// the above is the long winded way




// this dont work but left for ref. THESE DO WORK NOW. ISSUE WITH WRONG MINUS SIGN!!!!
		// std::vector<float> v1{1,2,3};
		// std::vector<float> v1{(x2 - x1),(y2 - y1),(z2 - z1)};
		// std::vector<float> v1{x2 - x1,y2 - y1, z2 - z1};

		cout << "A: " << A << " ,B: " << B << ", C: " << C << endl;		

		for (int j = 0; j < pointCloudSize; j++) {

			// check to see if the index was one of the ones selected randomly
			if ((j != sampleIndex1) && (j != sampleIndex2)) {

				float x = cloud->points[j].x;
				float y = cloud->points[j].y;
				float z = cloud->points[j].z;

// first line is for a line.... second is a plane
				// float d = abs(A*x + B*y + C) / sqrt((pow(A,2) + (pow(B,2) )));
				float d = abs(A*x + B*y +C*z  + D)/sqrt(pow(A,2) + pow(B,2) +pow(C,2));

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
	// NOTE CreateData3D for plane implementation, CreateData fr Line
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

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
