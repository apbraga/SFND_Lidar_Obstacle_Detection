// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(std::unordered_set<int>& processed, int index, std::vector<int>& cluster, float& distanceTol, KdTree& tree, typename pcl::PointCloud<PointT>::Ptr cloud){
	processed.insert(index);
	cluster.push_back(index);
    std::vector<float> point {cloud->points[index].x, cloud->points[index].y , cloud->points[index].z};
	std::vector<int> close_points = tree.search(point, distanceTol);
	while(close_points.size()>0){
		index = close_points.back();
		close_points.pop_back();
		if(processed.find(index) == processed.end()){
			proximity(processed, index, cluster, distanceTol, tree, cloud);
		}
	}
}


template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr& cloud, KdTree& tree, float& distanceTol)
{

	std::vector<std::vector<int>> clusters;
	std::unordered_set<int> processed;

	for(int i = 0; i < cloud->points.size(); i++){
		
		if(processed.find(i) == processed.end()){
			std::vector<int> cluster;
			proximity(processed, i, cluster, distanceTol, tree , cloud);
			clusters.push_back(cluster);
		}
	}
 
	return clusters;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    std::cout << cloud->points.size()<<std::endl;
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr voxel_result (new pcl::PointCloud<PointT>);
    //Voxel Filter
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes,filterRes,filterRes);
    sor.filter(*voxel_result);

    //Region of Interest
    typename pcl::PointCloud<PointT>::Ptr crop_result (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> crop(true);
    crop.setInputCloud(voxel_result);
    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    crop.filter(*crop_result);

    //Remove roof points
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(crop_result);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices){
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(crop_result);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*crop_result);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return crop_result;

}
/*
// Plane Segmentation using pcl library
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstacle (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr road (new pcl::PointCloud<PointT> ());
    //add road points to road point cloud
    for(int idx : inliers->indices){
        road->points.push_back(cloud->points[idx]);
    }
    //extract road points from lidar reading, isolation object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstacle);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle, road);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // DONE:: Fill in this function to find inliers for the cloud.
    //based on http://pointclouds.org/documentation/tutorials/planar_segmentation.html#planar-segmentation
    
    //initialize pointers for segmentation result
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    //setup the segmentation object using RANSAC ans its parameters
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    //Get planes and inliers from segmentation
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    //sanity check to confirm if plane was found
    //inliers are points fitted to the plane
    if(inliers->indices.size() == 0){
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        //return -1;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}
*/

//plane separation by scratch
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // DONE:: Fill in this function to find inliers for the cloud.
    //based on http://pointclouds.org/documentation/tutorials/planar_segmentation.html#planar-segmentation
    
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
			
			PointT point = cloud->points[j];
			float x = point.x;
			float y = point.y;
			float z = point.z;

			float distance = fabs(a*x+b*y+c*z+d)/sqrt(a*a+b*b+c*c);
			// If distance is smaller than threshold count it as inlier
			if(distance <= distanceThreshold){
				inliers.insert(j);
			}

			if(inliers.size() > inliersResult.size()){
				inliersResult = inliers;
			}

		}

	}
    if(inliersResult.size() == 0){
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        //return -1;
    }

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());
	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = {cloudOutliers, cloudInliers};
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    
    return segResult;
}


/* //Clustering using pcl library
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // DONE:: Fill in the function to perform euclidean clustering to group detected obstacles
    // http://pointclouds.org/documentation/tutorials/cluster_extraction.php
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec ;
    ec.setClusterTolerance(clusterTolerance); 
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    for( std :: vector < pcl :: PointIndices >:: const_iterator it = cluster_indices . begin (); it != cluster_indices . end (); ++ it ){
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for(std :: vector < int >:: const_iterator pit = it -> indices . begin (); pit != it -> indices . end (); ++ pit){
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);        
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
*/

// Clustering from scratch
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // DONE:: Fill in the function to perform euclidean clustering to group detected obstacles
    // http://pointclouds.org/documentation/tutorials/cluster_extraction.php

    //add cloud points to KDtree
    KdTree tree = KdTree();
    for(int i = 0 ; i < (cloud->points).size(); i++){
        std::vector<float> point {cloud->points[i].x, cloud->points[i].y , cloud->points[i].z};
        tree.insert(point,i);
    }
    // get clusters
    std::vector<std::vector<int>> clusters_indices = euclideanCluster(cloud, tree, clusterTolerance);
    // filter clusters
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    for(auto cluster : clusters_indices){
        if((cluster.size() > minSize ) && (cluster.size() < maxSize )){
            typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
            for(auto point_indx : cluster){
                clusterCloud->points.push_back(cloud->points[point_indx]);
            }
            clusters.push_back(clusterCloud);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
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

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    BoxQ box;

    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                    ///    the signs are different and the box doesn't get correctly oriented in some cases.
    /* // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloudSegmented);
    pca.project(*cloudSegmented, *cloudPCAprojection);
    std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
    std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
    // In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.
    */

   // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    box.bboxQuaternion = Eigen::Quaternionf(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    box.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}