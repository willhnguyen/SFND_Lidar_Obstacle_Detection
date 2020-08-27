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

	// For max iterations
	while(maxIterations--)
	{
		std::unordered_set<int> inliers;
		
		// Randomly sample subset and fit line
		int a_index = rand() % cloud->size();
		int b_index = rand() % cloud->size();
		int c_index = rand() % cloud->size();
		inliers.insert(a_index);
		inliers.insert(b_index);
		inliers.insert(c_index);

		pcl::PointXYZ& a = cloud->points[a_index];
		pcl::PointXYZ& b = cloud->points[b_index];
		pcl::PointXYZ& c = cloud->points[c_index];

		double x1 = a.x, y1 = a.y, z1 = a.z,
		       x2 = b.x, y2 = b.y, z2 = b.z,
		       x3 = c.x, y3 = c.y, z3 = c.z;

		// Rays: v1 = (b - a), v2 = (c - a)
		double xv1 = x2 - x1, yv1 = y2 - y1, zv1 = z2 - z1,
		       xv2 = x3 - x1, yv2 = y3 - y1, zv2 = z3 - z1;
		
		// Cross Product: u = v1 x v2
		double uA = yv1 * zv2 - yv2 * zv1,
		       uB = zv1 * xv2 - zv2 * xv1,
			   uC = xv1 * yv2 - xv2 * yv1,
			   uD = -(uA * x1 + uB * y1 + uC * z1);

		// Measure distance between every point and fitted line
		for (int i = 0; i < cloud->size(); ++i)
		{
			// If distance is smaller than threshold count it as inlier
			pcl::PointXYZ& p = cloud->points[i];
			float distance = fabs(uA * p.x + uB * p.y + uC * p.z + uD) / sqrt(uA * uA + uB * uB + uC * uC);
			if (distance < distanceTol) {
				inliers.insert(i);
			}
		}

		// Return indicies of inliers from fitted line with most inliers
		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}
	std::cout << "RANSAC inliers count: " << inliersResult.size() << " out of " << cloud->size() << " points" << std::endl;
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	
	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.5);

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
