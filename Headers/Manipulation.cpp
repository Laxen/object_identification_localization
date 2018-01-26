#include "Manipulation.h"

/*
   Manipulation::Manipulation() {

   }
 */

/**
  Computes normals for a cloud
  @param cloud The cloud to compute normals for, this is also the output cloud
  @param search_surface The cloud to be used when searching for neighbors
 */
void
Manipulation::compute_normals(Point_Cloud_N::Ptr cloud, Point_Cloud_N::Ptr search_surface, double radius_search) {
	pcl::NormalEstimationOMP<Point_N,Point_N> normal_estimator;
	normal_estimator.setRadiusSearch(radius_search);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setSearchSurface(search_surface);
	normal_estimator.compute(*cloud);
}

/*
   Uses RANSAC to find the largest plane
   @param cloud The cloud to find the plane in
 */
pcl::ModelCoefficients::Ptr
Manipulation::find_plane(Point_Cloud_N::Ptr cloud, int max_iterations, double distance_threshold) {
	pcl::SACSegmentation<Point_N> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::PointCloud<Point_N>::Ptr cloud_plane (new pcl::PointCloud<Point_N> ());
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

	// Set parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (max_iterations);
	seg.setDistanceThreshold (distance_threshold);

	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);

	return coefficients;
}

/*
   Uses RANSAC to remove the largest plane
   @param cloud The cloud to remove the plane in
 */
pcl::ModelCoefficients::Ptr
Manipulation::remove_plane(Point_Cloud_N::Ptr cloud, int max_iterations, double distance_threshold) {
	pcl::SACSegmentation<Point_N> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::PointCloud<Point_N>::Ptr cloud_plane (new pcl::PointCloud<Point_N> ());
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

	// Set parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (max_iterations);
	seg.setDistanceThreshold (distance_threshold);

	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0)
	{
		std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
		return coefficients;
	}

	// Extract plane inliers (only used for printing number of inliers in plane)
	pcl::ExtractIndices<Point_N> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.setNegative (false);
	extract.filter (*cloud_plane);
	std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

	// Remove plane inliers, save the rest in output
	extract.setNegative (true);
	extract.filter (*cloud);

	return coefficients;
}

/**
  Extracts clusters from the cloud and returns them in a vector
  @param cloud The cloud to extract clusters from
  @return A vector of clusters
 */
std::vector<Manipulation::Point_Cloud_N::Ptr>
Manipulation::extract_clusters(Point_Cloud_N::Ptr cloud, double tolerance, int min_size, int max_size) {
	pcl::search::KdTree<Point_N>::Ptr tree (new pcl::search::KdTree<Point_N>);
	tree->setInputCloud(cloud);

	// Set up EuclideanClusterExtraction and extract cluster indices
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<Point_N> ec;
	ec.setClusterTolerance (tolerance);
	ec.setMinClusterSize (min_size);
	ec.setMaxClusterSize (max_size);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	// Extract pointclouds from cluster indices and store in cluster_vector
	std::vector<Point_Cloud_N::Ptr> cluster_vector;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		Point_Cloud_N::Ptr cloud_cluster (new Point_Cloud_N);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			cloud_cluster->points.push_back (cloud->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		cluster_vector.push_back(cloud_cluster);
	}

	return cluster_vector;
}

/**
  Joins clusters into a single point cloud
  @param clusters Vector of clusters
  @return Point cloud with all clusters
 */
Manipulation::Point_Cloud_N::Ptr
Manipulation::join_clusters(std::vector<Point_Cloud_N::Ptr> clusters) {
	Point_Cloud_N::Ptr cloud (new Point_Cloud_N);

	for(int i = 0; i < clusters.size(); i++) {
		*cloud += *clusters[i];
	}

	return cloud;
}

/**
  Downsamples a cloud
  @param cloud The cloud to be downsampled
  @param leaf_size The leaf size to be used when downsampling
  @return The downsampled cloud
 */
Manipulation::Point_Cloud_N::Ptr
Manipulation::downsample_cloud(Point_Cloud_N::Ptr cloud, double leaf_size) {
	Point_Cloud_N::Ptr cloud_ds (new Point_Cloud_N);
	pcl::VoxelGrid<Point_N> vg;

	vg.setInputCloud(cloud);
	vg.setLeafSize(leaf_size, leaf_size, leaf_size);
	vg.filter(*cloud_ds);

	return cloud_ds;
}

/**
  Subtracts one cloud from the other using octree
  @param cloud1 The cloud that the other cloud is subtracted from
  @param cloud2 The cloud that is subtracted
  @param leaf_size The size of the octree leaves
  @return The subtracted point cloud
 */
Manipulation::Point_Cloud_N::Ptr
Manipulation::subtract_clouds(Point_Cloud_N::Ptr cloud1, Point_Cloud_N::Ptr cloud2, double leaf_size) {
	Point_Cloud_N::Ptr cloud1_subtracted (new Point_Cloud_N);

	pcl::octree::OctreePointCloudChangeDetector<Point_N> octree (leaf_size);
	octree.setInputCloud(cloud2);
	octree.addPointsFromInputCloud();
	octree.switchBuffers();
	octree.setInputCloud(cloud1);
	octree.addPointsFromInputCloud();
	std::vector<int> indices;
	octree.getPointIndicesFromNewVoxels(indices);

	pcl::PointIndices::Ptr indices_cloud (new pcl::PointIndices());
	indices_cloud->indices = indices;
	pcl::ExtractIndices<Point_N> extract;
	extract.setInputCloud(cloud1);
	extract.setIndices(indices_cloud);
	extract.setNegative(false);
	extract.filter(*cloud1_subtracted);

	return cloud1_subtracted;
}

/**
  Removes statistical outliers from cloud
  @param cloud The cloud to remove outliers from
  @param mean_k The number of neighbors to use
  @param std_dev_mul The multiplier to use for std dev
 */
void
Manipulation::remove_statistical_outliers(Point_Cloud_N::Ptr cloud, int mean_k, double std_dev_mul) {
	pcl::StatisticalOutlierRemoval<Point_N> sorfilter;
	sorfilter.setInputCloud(cloud);
	sorfilter.setMeanK(mean_k);
	sorfilter.setStddevMulThresh(std_dev_mul);
	sorfilter.filter(*cloud);
}

/**
  Splits a cloud into two clouds, one that is visible to the viewpoint and one that is occluded
  @param cloud The cloud to check for occlusions in
  @param leaf_size Leaf size used in downsampling the cloud
  @param viewpoint Eigen::Vector3d which defines the viewpoint
  @param visible_cloud[out] Part of the cloud that is visible
  @param occluded_cloud[out] Part of the cloud that is occluded
  @param occluded_indices[out] The indices of the occluded points
 */
void
Manipulation::compute_occlusion(Point_Cloud_N::Ptr cloud, double leaf_size, Eigen::Vector3d viewpoint, Point_Cloud_N::Ptr visible_cloud, Point_Cloud_N::Ptr occluded_cloud, std::vector<int>* occluded_indices) {
	pcl::octree::OctreePointCloudSearch<Point_N> octree (leaf_size);
	octree.setInputCloud(cloud);
	octree.defineBoundingBox();
	octree.addPointsFromInputCloud();

	Eigen::Vector3f direction;
	Eigen::Vector3f start_point;
	std::vector<int> indices;
	for(int i = 0; i < cloud->width; i++) {
		start_point(0) = cloud->points[i].x;
		start_point(1) = cloud->points[i].y;
		start_point(2) = cloud->points[i].z;

		direction(0) = viewpoint(0) - cloud->points[i].x;
		direction(1) = viewpoint(1) - cloud->points[i].y;
		direction(2) = viewpoint(2) - cloud->points[i].z;

		octree.getIntersectedVoxelIndices(start_point, direction, indices);
		int n_occlusions = indices.size();

		for(int j = 0; j < indices.size(); j++) {
			// If points are very close to each-other, we do not consider the occlusion
			if(fabs(cloud->points[indices[j]].z - cloud->points[i].z) <= leaf_size)
			{
				n_occlusions--;
			}
		}

		if(n_occlusions == 0) {
			visible_cloud->points.push_back(cloud->points[i]);
		} else {
			occluded_cloud->points.push_back(cloud->points[i]);
			occluded_indices->push_back(i);
		}
	}
}

/**
  Computes the inliers of cloud target in cloud source, using the distance_threshold
  @param source The cloud that target should fit into
  @param target The cloud to compute inliers for
  @param distance_threshold The largest threshold for counting as an inlier
  @param inliers[out] The vector of inliers
  @param outliers[out] The vector of outliers
 */
void
Manipulation::compute_inliers(Point_Cloud_N::Ptr source, Point_Cloud_N::Ptr target, double distance_threshold, std::vector<int>* inliers, std::vector<int>* outliers) {
	pcl::KdTreeFLANN<Point_N> tree;
	tree.setInputCloud(source);

	for(int i = 0; i < target->points.size(); i++) {
		std::vector<int> nn_index(1);
		std::vector<float> nn_dist(1);
		tree.nearestKSearch(target->points[i], 1, nn_index, nn_dist);

		if(nn_dist[0] < distance_threshold*distance_threshold) {
			inliers->push_back(i);
		} else {
			outliers->push_back(i);
		}
	}
}

bool
Manipulation::is_centered (Point_Cloud_N::Ptr cluster, double dist_thr) {
	// Check if the cluster is centered
	for (int i = 0; i < cluster->points.size(); i++)
	{
		Point_N p = cluster->points[i]; 
		if (sqrt (pow (p.x, 2) + pow (p.y, 2)) < dist_thr)
		{
			return true;
		}
	}
	
	return false;
}

/**
  Loads a point cloud
  @param path The path to the cloud
  @param cloud The cloud pointer to use for storing the cloud
  @return 0 if error, 1 if successful
 */
int
Manipulation::load_cloud(std::string path, Point_Cloud_N::Ptr cloud) {
	std::cout << "Loading cloud \"" << path << "\"\n" << std::endl;

	int load_successful = pcl::io::loadPCDFile<Point_N>(path, *cloud);
	if(load_successful < 0) {
		pcl::console::print_error("Cloud could not be loaded!\n");
		return 0;
	}
	else {
		pcl::console::print_info("Cloud successfully loaded!\n");
		return 1;
	}
}

/**
  Loads a point cloud with RGB data
  @param path The path to the cloud
  @param cloud The cloud pointer to use for storing the cloud
  @return 0 if error, 1 if successful
 */
int
Manipulation::load_cloud_rgba(std::string path, Point_Cloud_RGBA::Ptr cloud) {
	std::cout << "Loading cloud \"" << path << "\"\n";

	int load_successful = pcl::io::loadPCDFile<Point_RGBA>(path, *cloud);
	if(load_successful < 0) {
		pcl::console::print_error("Cloud could not be loaded!\n");
		return 0;
	}
	else {
		pcl::console::print_info("Cloud successfully loaded!\n");
		return 1;
	}
}
