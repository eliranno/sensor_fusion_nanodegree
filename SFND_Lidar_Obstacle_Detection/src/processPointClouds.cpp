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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>);
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes,filterRes,filterRes);
    sor.filter(*filteredCloud);

    pcl::CropBox<PointT> cb(true);
    cb.setMin(minPoint);
    cb.setMax(maxPoint);
    cb.setInputCloud(filteredCloud);
    
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

    cb.filter(*cloudRegion);





    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

  typename pcl::ExtractIndices<PointT> extract;

  typename pcl::PointCloud<PointT>::Ptr cloudPlane(new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr cloudObstacle(new pcl::PointCloud<PointT>);
  
  for(int index : inliers->indices)
  {
      cloudPlane->points.push_back(cloud->points[index]);
  }


  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);

  //extrct the inliners
  extract.filter(*cloudPlane);

  extract.setNegative(true);
  extract.filter(*cloudObstacle);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudPlane, cloudObstacle);
  return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    typename pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);

    if(inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a plannar model for the given dataset" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(auto it=clusterIndices.begin();it!=clusterIndices.end();++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for(auto pit = it->indices.begin();pit!=it->indices.end();++pit)
        {
            cloudCluster->points.push_back(cloud->points[*pit]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);
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

template<typename PointT>
typename std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	//srand(time(NULL));
	
	// TODO: Fill in this function
	std::unordered_set<int> inliers;


    typename pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);

	while(maxIterations--)
	{
		// Randomly sample subset and fit line
		inliers.clear();
		while(inliers.size()<3)
		{
			inliers.insert(rand()%cloud->points.size());
		}
		auto constIter = inliers.cbegin();
		PointT pointA(cloud->points[*constIter]);
		constIter++;
		PointT pointB(cloud->points[*constIter]);
		constIter++;
		PointT pointC(cloud->points[*constIter]);

		// Ax + By + Cz +D = 0
		Eigen::Vector3f v1(pointB.x-pointA.x,pointB.y-pointA.y,pointB.z-pointA.z);
		Eigen::Vector3f v2(pointC.x-pointA.x,pointC.y-pointA.y,pointC.z-pointA.z);
		auto crossProduct = v1.cross(v2); 

		const float A(crossProduct.x());
		const float B(crossProduct.y());
		const float C(crossProduct.z());
		const float D(static_cast<float> (-(A*pointA.x+B*pointA.y+C*pointA.z)));

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
	}
	// Return indicies of inliers from fitted line with most inliers

    for(auto i=0;i<cloud->points.size();++i)
    {
        if(inliersResult.count(i) > 0)
        {
            cloud1->points.push_back(cloud->points[i]);
        }
        else
        {
            cloud2->points.push_back(cloud->points[i]);
        }
    }

	return {cloud1,cloud2};
}

template <typename PointT>
typename std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Cluster(typename pcl::PointCloud<PointT>::Ptr cloud,const double distanceTol,const int min,const int max)
{

    // compose the kd tree
    KdTree tree;
    std::vector<std::vector<float>> points;

    for(auto index = 0;index<cloud->points.size();++index)
    {
        auto point = cloud->points[index];
        const std::vector<float> p {point.x,point.y,point.z};
        tree.insert(p,index);
        points.push_back({point.x,point.y,point.z});
    }

    // call euclideanCluster to produce a vector of clusters point id

    auto clusterIdList = euclideanCluster(points,&tree,distanceTol);

    typename std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    
    for(auto cluster : clusterIdList )
    {
        if(cluster.size() < min || cluster.size() > max)
        {
            // cluster is too small/big just ignore it
            continue;
        }
        typename pcl::PointCloud<PointT>::Ptr clusterPointCloud(new pcl::PointCloud<PointT>);
        for(auto index : cluster)
        {
            clusterPointCloud->points.push_back(cloud->points[index]);
        }
        clusters.push_back(clusterPointCloud);
    }
    return clusters;
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;
	std::unordered_set<int> processedIndices;
	std::vector<int> cluster;

	std::function <void(int)> helper = [&cluster,points,tree,distanceTol,&processedIndices,&helper](int pos)
	{
		processedIndices.insert(pos);
		cluster.push_back(pos);
		auto nearest = tree->search(points[pos],distanceTol);
		for(auto pointId : nearest)
		{
			if(processedIndices.count(pointId) == 0)
			{
				helper(pointId);
			}
		}
	};

	for(auto index = 0; index < points.size();++index)
	{
		cluster.clear();
		if(processedIndices.count(index) == 0)
		{
			helper(index);
			clusters.push_back(cluster);
		}
	}
 
	return clusters;
}