#ifndef MANIPULATION_H
#define MANIPULATION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree2buf_base.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>

class Manipulation {
	private:
		typedef pcl::PointNormal Point_N;
		typedef pcl::PointXYZRGB Point_RGBA;
		typedef pcl::PointCloud<Point_N> Point_Cloud_N;
		typedef pcl::PointCloud<Point_RGBA> Point_Cloud_RGBA;

	public:
		/**
			Computes normals for a cloud
			@param cloud The cloud to compute normals for, this is also the output cloud
			@param search_surface The cloud to be used when searching for neighbors
		 */
		void
			compute_normals(Point_Cloud_N::Ptr cloud, Point_Cloud_N::Ptr search_surface, double radius_search);

		/*
			 Uses RANSAC to find the largest plane
			 @param cloud The cloud to find the plane in
		 */
		pcl::ModelCoefficients::Ptr
			find_plane(Point_Cloud_N::Ptr cloud, int max_iterations, double distance_threshold);

		/*
			 Uses RANSAC to remove the largest plane
			 @param cloud The cloud to remove the plane in
		 */
		pcl::ModelCoefficients::Ptr
			remove_plane(Point_Cloud_N::Ptr cloud, int max_iterations, double distance_threshold);

		/**
			Extracts clusters from the cloud and returns them in a vector
			@param cloud The cloud to extract clusters from
			@return A vector of clusters
		 */
		std::vector<Point_Cloud_N::Ptr>
			extract_clusters(Point_Cloud_N::Ptr cloud, double tolerance, int min_size, int max_size);

		/**
			Joins clusters into a single point cloud
			@param clusters Vector of clusters
			@return Point cloud with all clusters
		 */
		Point_Cloud_N::Ptr
			join_clusters(std::vector<Point_Cloud_N::Ptr> clusters);

		/**
			Downsamples a cloud
			@param cloud The cloud to be downsampled
			@param leaf_size The leaf size to be used when downsampling
			@return The downsampled cloud
		 */
		Point_Cloud_N::Ptr
			downsample_cloud(Point_Cloud_N::Ptr cloud, double leaf_size);

		/**
			Subtracts one cloud from the other using octree
			@param cloud1 The cloud that the other cloud is subtracted from
			@param cloud2 The cloud that is subtracted
			@param leaf_size The size of the octree leaves
			@return The subtracted point cloud
		 */
		Point_Cloud_N::Ptr
			subtract_clouds(Point_Cloud_N::Ptr cloud1, Point_Cloud_N::Ptr cloud2, double leaf_size);

		/**
			Removes statistical outliers from cloud
			@param cloud The cloud to remove outliers from
			@param mean_k The number of neighbors to use
			@param std_dev_mul The multiplier to use for std dev
		 */
		void
			remove_statistical_outliers(Point_Cloud_N::Ptr cloud, int mean_k, double std_dev_mul);

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
			compute_occlusion(Point_Cloud_N::Ptr cloud, double leaf_size, Eigen::Vector3d viewpoint, Point_Cloud_N::Ptr visible_cloud, Point_Cloud_N::Ptr occluded_cloud, std::vector<int>* occluded_indices);

		/**
			Computes the inliers of cloud target in cloud source, using the distance_threshold
			@param source The cloud that target should fit into
			@param target The cloud to compute inliers for
			@param distance_threshold The largest threshold for counting as an inlier
			@param inliers[out] The vector of inliers
			@param outliers[out] The vector of outliers
		 */
		void
			compute_inliers(Point_Cloud_N::Ptr source, Point_Cloud_N::Ptr target, double distance_threshold, std::vector<int>* inliers, std::vector<int>* outliers);

		bool
			is_centered (Point_Cloud_N::Ptr cluster, double dist_thr);

	/**
		Loads a point cloud
		@param path The path to the cloud
		@param cloud The cloud pointer to use for storing the cloud
		@return 0 if error, 1 if successful
	 */
	int
	load_cloud(std::string path, Point_Cloud_N::Ptr cloud);

	/**
		Loads a point cloud with RGB data
		@param path The path to the cloud
		@param cloud The cloud pointer to use for storing the cloud
		@return 0 if error, 1 if successful
	 */
	int
	load_cloud_rgba(std::string path, Point_Cloud_RGBA::Ptr cloud);
};

#endif
