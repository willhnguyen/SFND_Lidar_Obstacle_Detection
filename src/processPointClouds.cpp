// PCL lib Functions for processing point clouds 

#include <unordered_set>
#include <queue>

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

    // Downsample point cloud by voxel resolution
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>());

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    // Downsample point cloud by cropping view
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new typename pcl::PointCloud<PointT>());

    pcl::CropBox<PointT> cb;
    cb.setInputCloud(cloudFiltered);
    cb.setMin(minPoint);
    cb.setMax(maxPoint);
    cb.filter(*cloudRegion);

    // Downsample by clearing car roof points
    typename pcl::PointCloud<PointT>::Ptr cloudRegionNoRoof(new typename pcl::PointCloud<PointT>());

    cb.setInputCloud(cloudRegion);
    cb.setMin(Eigen::Vector4f(-2, -1.5, -1.5, 1));
    cb.setMax(Eigen::Vector4f(2, 1.5, 1, 1));
    cb.setNegative(true);
    cb.filter(*cloudRegionNoRoof);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegionNoRoof;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    extract.setNegative(false);
    extract.filter(*planeCloud);
    
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	srand(time(NULL));
    auto cloud_size = cloud->size();

    std::unordered_set<int> inliersResult;
    inliersResult.reserve(cloud_size);

	// RANSAC Implementation
    std::unordered_set<int> inliersSet;
    inliersSet.reserve(cloud_size);

	while(maxIterations--)
	{
        inliersSet.clear();
		
		// Randomly sample subset and fit line
		int a_index = rand() % cloud_size;
		int b_index = rand() % cloud_size;
		int c_index = rand() % cloud_size;
		inliersSet.insert(a_index);
		inliersSet.insert(b_index);
		inliersSet.insert(c_index);

		PointT& a = cloud->points[a_index];
		PointT& b = cloud->points[b_index];
		PointT& c = cloud->points[c_index];

		float x1 = a.x, y1 = a.y, z1 = a.z,
		      x2 = b.x, y2 = b.y, z2 = b.z,
		      x3 = c.x, y3 = c.y, z3 = c.z;

		// Rays: v1 = (b - a), v2 = (c - a)
		float xv1 = x2 - x1, yv1 = y2 - y1, zv1 = z2 - z1,
		      xv2 = x3 - x1, yv2 = y3 - y1, zv2 = z3 - z1;
		
		// Cross Product: u = v1 x v2
		float uA = yv1 * zv2 - yv2 * zv1,
		      uB = zv1 * xv2 - zv2 * xv1,
			  uC = xv1 * yv2 - xv2 * yv1,
			  uD = -(uA * x1 + uB * y1 + uC * z1);

		// Measure distance between every point and fitted line
		for (int i = 0; i < cloud_size; ++i)
		{
            if (inliersSet.find(i) != inliersSet.end()) {
                continue;
            }

			// If distance is smaller than threshold count it as inlier
			PointT& p = cloud->points[i];
			float distance = fabs(uA * p.x + uB * p.y + uC * p.z + uD) / sqrt(uA * uA + uB * uB + uC * uC);
			if (distance < distanceThreshold) {
				inliersSet.insert(i);
			}
		}

		// Return indices of inliers from fitted line with most inliers
		if (inliersSet.size() > inliersResult.size()) {
			inliersResult = inliersSet;
		}
	}

    std::vector<int> &indices = inliers->indices;
    indices.reserve(inliersResult.size());
    for (int i : inliersResult) {
        indices.push_back(i);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template<typename PointT>
std::vector<float> ProcessPointClouds<PointT>::vectorize(PointT& p) {
    return {p.x, p.y, p.z};
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, KdTree &tree, std::vector<bool>& processed_points, typename pcl::PointCloud<PointT>::Ptr cluster, int idx)
{
    if (processed_points[idx]) {
        return;
    }
    processed_points[idx] = true;

    cluster->points.push_back(cloud->points[idx]);
    std::vector<float> p = vectorize(cloud->points[idx]);
    std::vector<int> nearby_points = tree.search(p, clusterTolerance);
    for (int nearby_point_idx : nearby_points)
        if (!processed_points[nearby_point_idx])
            proximity(cloud, clusterTolerance, tree, processed_points, cluster, nearby_point_idx);
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    int n_points = cloud->points.size();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Generate KD Tree
    KdTree tree;
    for (int i = 0; i < n_points; ++i) {
        std::vector<float> p = vectorize(cloud->points[i]);
        tree.insert(p, i);
    }

    // // Euclidean Clustering (queue method, BFS)
    // std::vector<bool> processed_points(n_points, false);
    // std::queue<int> proximity_queue;
    // for (int i = 0; i < n_points; ++i)
    // {
    //     if (!processed_points[i])
    //     {
    //         typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
    //         // Breadth-First Search Styled Travesal of KD Tree
    //         proximity_queue.push(i);
    //         while (!proximity_queue.empty())
    //         {
    //             int point_idx = proximity_queue.front();
    //             proximity_queue.pop();
    //             // If visited move on, else mark visited and process
    //             if (processed_points[point_idx])
    //                 continue;
    //             processed_points[point_idx] = true;
    //             // Add point to cluster and visit its neighbors
    //             cluster->points.push_back(cloud->points[point_idx]);
    //             std::vector<float> p = vectorize(cloud->points[point_idx]);
    //             std::vector<int> nearby_points = tree.search(p, clusterTolerance);
    //             for (int nearby_point_idx : nearby_points)
    //                 if (!processed_points[nearby_point_idx])
    //                     proximity_queue.push(nearby_point_idx);
    //         }
    //         // Include clusters if they fall within cluster size limits
    //         if (cluster->points.size() >= minSize && cluster->points.size() <= maxSize) {
    //             clusters.push_back(cluster);
    //         }
    //     }
    // }

    // Euclidean Clustering (call stack method, DFS)
    std::vector<bool> processed_points(n_points, false);
    for (int i = 0; i < n_points; ++i)
    {
        if (!processed_points[i])
        {
            typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
            proximity(cloud, clusterTolerance, tree, processed_points, cluster, i);
            // Include clusters if they fall within cluster size limits
            if (cluster->points.size() >= minSize && cluster->points.size() <= maxSize) {
                clusters.push_back(cluster);
            }
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