#ifndef RANSAC_H_
#define RANSAC_H_

#include <unordered_set>

template<typename PointT>
void Ransac3D(typename pcl::PointCloud<PointT>::Ptr& cloud, int& maxIterations, float& distanceThreshold, std::unordered_set<int>& inliersResult){
    srand(time(NULL));
	int cloud_size = cloud->points.size();
	auto points = cloud->points;
	// TODO: Fill in this function
	// For max iterations 
	while(maxIterations--){
	// Randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while(inliers.size()<3){
            inliers.insert(rand()%cloud_size);
		}
		//get coordinates from random points
		float x1,x2,x3,y1,y2,y3,z1,z2,z3;
		auto idx = inliers.begin();
		x1 = points[*idx].x;
		y1 = points[*idx].y;
		z1 = points[*idx].z;
		idx++;
		x2 = points[*idx].x;
		y2 = points[*idx].y;
		z2 = points[*idx].z;
		idx++;
		x3 = points[*idx].x;
		y3 = points[*idx].y;
		z3 = points[*idx].z;
		//calculate line components
		float v1[] = {x2-x1,y2-y1,z2-z1}; // Point1 -> Point2
		float v2[] = {x3-x1,y3-y1,z3-z1}; // Point1 -> Point3
		float normal[] = {v1[1]*v2[2]-v1[2]*v2[1], v1[2]*v2[0]-v1[0]*v2[2],v1[0]*v2[1]-v1[1]*v2[0]}; // cross product of v1 x v2
		
		float a = normal[0];
		float b = normal[1];
		float c = normal[2];
        float d = -(a*x1+b*y1+c*z1);
		float sqrt_abc = sqrt(a*a+b*b+c*c);
	// Measure distance between every point and fitted line
		for(int j = 0; j < cloud_size ; j++){
			
			//skip if the cloud point is the inlier point
			if(inliers.count(j) > 0){
				continue;
			}
			
			float x = points[j].x;
			float y = points[j].y;
			float z = points[j].z;

			float distance = fabs(a*x+b*y+c*z+d)/sqrt_abc;
			// If distance is smaller than threshold count it as inlier
			if(distance <= distanceThreshold){
				inliers.insert(j);
			}	

		}
		
		if(inliers.size() > inliersResult.size()){
				inliersResult = inliers;
		}

    }

	
    if(inliersResult.size() == 0){
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        //return -1;
    }
}

#endif