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

std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	//srand(time(NULL));
	
	// TODO: Fill in this function
	std::unordered_set<int> inliers;

	while(maxIterations>0)
	{
		// Randomly sample subset and fit line
		inliers.clear();
		while(inliers.size()<2)
		{
			inliers.insert(rand()%cloud->points.size());
		}
		auto constIter = inliers.cbegin();
		pcl::PointXYZ pointA(cloud->points[*constIter]);
		constIter++;
		pcl::PointXYZ pointB(cloud->points[*constIter]);

		// Ax + By + C = 0
		const float A(pointA.y - pointB.y);
		const float B(pointB.x-pointA.x);
		const float C(pointA.x*pointB.y - pointB.x*pointA.y);

		// Measure distance between every point and fitted line
		for(auto index = 0;index<cloud->points.size();++index)
		{
			auto point = cloud->points[index];
			const float distance = fabs(A*point.x + B*point.y + C)/ sqrtf(A*A + B*B);
			// If distance is smaller than threshold count it as inlier
			if (distance < distanceTol)
			{
				inliers.insert(index);
			}
		}
		inliersResult = inliersResult.size() < inliers.size() ? inliers : inliersResult;
		maxIterations--;
	}
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	//srand(time(NULL));
	
	// TODO: Fill in this function
	std::unordered_set<int> inliers;

	while(maxIterations>0)
	{
		// Randomly sample subset and fit line
		inliers.clear();
		while(inliers.size()<3)
		{
			inliers.insert(rand()%cloud->points.size());
		}
		auto constIter = inliers.cbegin();
		pcl::PointXYZ pointA(cloud->points[*constIter]);
		constIter++;
		pcl::PointXYZ pointB(cloud->points[*constIter]);
		constIter++;
		pcl::PointXYZ pointC(cloud->points[*constIter]);

		// Ax + By + Cz +D = 0
		Eigen::Vector3f v1(pointB.x-pointA.x,pointB.y-pointA.y,pointB.z-pointA.z);
		Eigen::Vector3f v2(pointC.x-pointA.x,pointC.y-pointA.y,pointC.z-pointA.z);
		auto crossProduct = v1.cross(v2); 

		const float A(crossProduct.x());
		const float B(crossProduct.y());
		const float C(crossProduct.z());
		const float D(-(A*pointA.x,B*pointA.y,C*pointA.z));

		// Measure distance between every point and fitted line
		for(auto index = 0;index<cloud->points.size();++index)
		{
			auto point = cloud->points[index];
			const float distance = fabs(A*point.x + B*point.y + C*point.z + D)/ sqrtf(A*A + B*B + C*C);
			// If distance is smaller than threshold count it as inlier
			if (distance < distanceTol)
			{
				inliers.insert(index);
			}
		}
		inliersResult = inliersResult.size() < inliers.size() ? inliers : inliersResult;
		maxIterations--;
	}
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 1000, 0.5);

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
