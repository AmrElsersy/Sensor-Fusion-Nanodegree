/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <cmath>

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

	// For max iterations 
	for (int i = 0; i < maxIterations; i++)
	{
		// Randomly sample subset and fit line
		auto size = cloud->points.size();
		auto point1 = cloud->points[rand() % size];
		auto point2 = cloud->points[rand() % size];

		// rand line cofficients ... A * x + B * y + C = 0 ....... (y1−y2)x+(x2−x1)y+(x1∗y2−x2∗y1)=0
		auto A = point1.y - point2.y;
		auto B = point2.x - point1.x;
		auto C = point1.x * point2.y - point2.x * point1.y;

		std::unordered_set<int> inliers;

		// Measure distance between every point and fitted line
		for (int j = 0; j < cloud->points.size(); j++)
		{
			auto point = cloud->points[j];
			// orthogonal distance d = |Ax+By+C|/sqrt(A^2+B^2)d=∣Ax+By+C∣/sqrt(A^2 +B^2 )
			auto d = fabs( A * point.x + B * point.y + C ) / hypot(A, B);

			// If distance is smaller than threshold count it as inlier
			if (d <= distanceTol)
				inliers.insert(j);
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}
	return inliersResult;
}


std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations 
	for (int i = 0; i < maxIterations; i++)
	{
		// Randomly sample subset and fit Plane
		auto size = cloud->points.size();
		auto point1 = cloud->points[rand() % size];
		auto point2 = cloud->points[rand() % size];
		auto point3 = cloud->points[rand() % size];

		// Plane Equation Ax+By+Cz+D=0 ....... v1×v2=<(y2−y1)(z3−z1)−(z2−z1)(y3−y1),
											// (z2-z1)(x3-x1)-(x2-x1)(z3-z1)
											// (x2-x1)(y3-y1)-(y2-y1)(x3-x1)>
		// A = i component, B = j component, C = k component of cross mult of v1 & v2 which are vectors from point1 to point2 & point3
		auto x1 = point1.x; 
		auto x2 = point2.x; 
		auto x3 = point3.x; 
		
		auto y1 = point1.y; 
		auto y2 = point2.y; 
		auto y3 = point3.y; 
		
		auto z1 = point1.z; 
		auto z2 = point2.z; 
		auto z3 = point3.z; 
		
		
		auto A = (y2 - y1)*(z3 - z1)-(z2 - z1)* (y3 - y1);
		auto B = (z2 - z1)*(x3 - x1)-(x2 - x1)* (z3 - z1);
		auto C = (x2 - x1)*(y3 - y1)-(y2 - y1)* (x3 - x1);
		auto D = -( A * x1 + B * y1 + C * z1 );

		std::unordered_set<int> inliers;

		// Measure distance between every point and fitted line
		for (int j = 0; j < cloud->points.size(); j++)
		{
			auto point = cloud->points[j];
			// orthogonal distance d = |Ax+By+C|/sqrt(A^2+B^2)d=∣Ax+By+C∣/sqrt(A^2 +B^2 )
			auto d = fabs( A * point.x + B * point.y + C * point.z + D ) / sqrt(A*A + B*B + C*C);

			// If distance is smaller than threshold count it as inlier
			if (d <= distanceTol)
				inliers.insert(j);
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}
	return inliersResult;
}


int main ()
{
	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();	

	// std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);

	std::unordered_set<int> inliers = Ransac3D(cloud, 10, 0.2);

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
