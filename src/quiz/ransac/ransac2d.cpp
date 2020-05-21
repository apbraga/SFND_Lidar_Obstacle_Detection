/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include<random>
#include<vector>

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
	for(int i = 0; i < maxIterations; i++){
	// Randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while(inliers.size()<3){
			inliers.insert(rand()%(cloud->points.size()));
		}
		//get coordinates from random points
		float x1,x2,x3,y1,y2,y3,z1,z2,z3;
		auto idx = inliers.begin();
		x1 = cloud->points[*idx].x;
		y1 = cloud->points[*idx].y;
		z1 = cloud->points[*idx].z;
		idx++;
		x2 = cloud->points[*idx].x;
		y2 = cloud->points[*idx].y;
		z2 = cloud->points[*idx].z;
		idx++;
		x3 = cloud->points[*idx].x;
		y3 = cloud->points[*idx].y;
		z3 = cloud->points[*idx].z;
		//calculate line components
		float v1[] = {x2-x1,y2-y1,z2-z1}; // Point1 -> Point2
		float v2[] = {x3-x1,y3-y1,z3-z1}; // Point1 -> Point3
		float normal[] = {v1[1]*v2[2]-v1[2]*v2[1], v1[2]*v2[0]-v1[0]*v2[2],v1[0]*v2[1]-v1[1]*v2[0]}; // cross product of v1 x v2
		
		float a = normal[0];
		float b = normal[1];
		float c = normal[2];
		float d = -(a*x1+b*y1+c*z1);

	// Measure distance between every point and fitted line
		for(int j = 0; j < cloud->points.size() ; j++){
			
			//skip if the cloud point is the inlier point
			if(inliers.count(j) > 0){
				continue;
			}
			
			pcl::PointXYZ point = cloud->points[j];
			float x3 = point.x;
			float y3 = point.y;
			float z3 = point.z;

			float distance = fabs(a*x3+b*y3+c*z3+d)/sqrt(a*a+b*b+c*c);
			// If distance is smaller than threshold count it as inlier
			if(distance <= distanceTol){
				inliers.insert(j);
			}

			if(inliers.size() > inliersResult.size()){
				inliersResult = inliers;
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.25);

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
