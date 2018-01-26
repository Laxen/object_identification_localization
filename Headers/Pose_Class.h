#ifndef POSE_CLASS_H
#define POSE_CLASS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/print.h>
#include "sample_consensus_prerejective.h"
#include <pcl/common/time.h>
#include <pcl/features/fpfh_omp.h>
#include "Manipulation.h"
#include "Access_Results.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include "pose_data.h"

class Pose_Class {
	private:
		typedef pcl::PointNormal Point_N;
		typedef pcl::PointCloud<Point_N> Point_Cloud_N;

		typedef pcl::FPFHSignature33 Feature;
		typedef pcl::PointCloud<Feature> Feature_Cloud;

		typedef pcl::visualization::PointCloudColorHandlerCustom<Point_N> ColorHandler_N;

		Manipulation manipulation;

		int VISUALIZE_CLUSTERS_REALTIME;

		double LEAF_SIZE; // Used for downsampling (0.003 for small objects, 0.010 for large)

		int POSE_MAX_ITERATIONS;
		double POSE_MAX_CORRESPONDENCE_DISTANCE;
		double POSE_CORRESPONDENCE_RANDOMNESS;
		double POSE_SIMILARITY_THRESHOLD;
		double POSE_INLIER_FRACTION;
		double POSE_INVERSE_INLIER_FRACTION;

		double POSE_FINAL_SIMILARITY_THRESHOLD;

		double FEATURE_RADIUS_SEARCH; // Radius for feature estimation

		double PLANE_DISTANCE_THRESHOLD; // Distance from point to plane to count as inlier to plane
		int PLANE_MAX_ITERATIONS; // Max RANSAC iterations for finding plane
		double BACKGROUND_SEGMENTATION_DISTANCE; // Max distance between two points when subtracting clouds

		double CLUSTER_TOLERANCE;
		int CLUSTER_MIN_SIZE;
		int CLUSTER_MAX_SIZE;

		double NORMAL_RADIUS_SEARCH; // Radius for nearest neighbors

		pcl::visualization::PCLVisualizer* visu_ptr;
		int visualization_mode;

		int print_mode;

		/**
			Computes features for a cloud
			@param cloud The cloud to compute features for
			@return The feature cloud
		 */
		Feature_Cloud::Ptr
		compute_features(Point_Cloud_N::Ptr cloud);

		/**
			Estimates the pose of an object in a scene
			@param object The object to estimate pose for
			@param scene The scene to find the object in
			@param object_features The features for the object
			@param scene_features The features for the scene
			@param transformation[out] The transformation (pose) of the object
			@param all_transformations[out] Vector with all transformations with inliers above inlier_fraction
			@return Vector of all inliers
		 */
		std::vector<int>
		estimate_pose(	Point_Cloud_N::Ptr object, 
				Point_Cloud_N::Ptr scene, 
				Feature_Cloud::Ptr object_features, 
				Feature_Cloud::Ptr scene_features, 
				Eigen::Matrix<float,4,4,Eigen::DontAlign>* transformation,
				int max_iterations, 
				int correspondence_randomness, 
				double similarity_threshold, 
				double max_correspondence_distance, 
				double inlier_fraction, 
				std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> >* all_transformations);

		/**
			Matches an object to all clusters and returns a vector of vector of poses of the object in each cluster
			@param object The object to estimate pose of
			@param object_features The feature cloud of the object
			@param clusters Vector of clusters
			@param object_to_cluster If true, matches object to cluster, otherwise matches clusters to object
			@param transformations[out] Vector of vector of transformations for each cluster
		 */
		void
		estimate_poses_for_clusters(	Point_Cloud_N::Ptr object, 
						Feature_Cloud::Ptr object_features, 
						std::vector<Point_Cloud_N::Ptr> clusters, 
						bool object_to_cluster, 
						std::vector<std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > >* transformations);

		/**
			Match clusters to object, and match object to passed clusters
			@param object The object to match with
			@param object_features Feature cloud for the object
			@param cluster Vector of all clusters
			@param transformations[out] Vector of vector of transformations for each cluster
		 */
		void
		match_cluster_to_object_to_cluster(	Point_Cloud_N::Ptr object, 
							Feature_Cloud::Ptr object_features, 
							std::vector<Point_Cloud_N::Ptr> clusters, 
							std::vector<std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > >* transformations);

		/**
			Takes transformations from each scene cluster, and puts them in pose clusters
			@param transformations Poses to be clustered. Rows = Scene clusters, Cols = (tr,inl) pairs for that scene cluster.
			@param clustered_poses[out] Rows = Pose clusters, Cols = (tr,inl) for that pose cluster. First col in each row contains (tr_avg, inl_avg) for that pose cluster.
		 */
		void
		cluster_poses(	std::vector<std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > >* transformations, 
				std::vector<std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > >* clustered_poses);

		/**
			Takes the average pose of each pose cluster, compares it to the other average poses and computes an accuracy based on a couple of parameters between them.
			Saves the result as a new vector with ((pose_avg, inliers_avg), accuracy)
			@param clustered_poses The clustered poses (pose, inliers)
			@param filtered_poses[out] The filtered poses ((pose, inliers), accuracy)
		 */
		void
		filter_clustered_poses(	std::vector<std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > >* clustered_poses, 
					std::vector<std::pair<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double>, double> >* filtered_poses);

		/**
			Finds points with persistent features in cloud
			@param cloud The cloud to find persistent features in
			@param output_indices The indices of the points with persistent features
		 */
		void
		compute_persistent_features(Point_Cloud_N::Ptr cloud, boost::shared_ptr<std::vector<int> > output_indices);

		/**
			Compares two filtered poses in regards to inliers (first.second is inliers)
		 */
		static bool
		filtered_poses_compare(	const std::pair<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double>, double>& firstElem, 
					const std::pair<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double>, double>& secondElem);

		/**
			Compares pose structures
		 */
		static bool
		poses_comparator(const Pose_Data& pose1, const Pose_Data& pose2);

		/**
			Writes pose data to file
			@param scene_name The name of the scene (name of file to save)
			@param poses Pose data as a vector of pose structs
		 */
		void
		write_pose_data(std::string scene_name, int cluster_index, std::vector<Pose_Data> poses);

	public:
		Pose_Class();
		Pose_Class(std::string config_path);

		/**
			Estimates the pose of an object in a scene using the full cluster->object->cluster pipeline, with pose clustering and pose cluster filtering
			@param object The object which pose should be estimated
			@param scene The cluster the object should fit into (CHANGE NAME TO CLUSTER)
			@return Vector of filtered poses sorted by inlier fraction
		 */
		std::vector<std::pair<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double>, double> >
		estimate_full_pose(	Point_Cloud_N::Ptr object, 
					Point_Cloud_N::Ptr cluster);

		/**
			Estimates the pose for all views in the cluster and writes the pose data
			@param views The views to estimate the pose for (can be merged views)
			@param model_names The names for each model (needs to match with views)
			@param model_view_indices The view indices for each view (not sure how this will work in merged views)
			@param cluster The cluster to estimate the pose in
			@param scene_name The name of the original scene, name of main folder in pose estimation results
			@param cluster_index The index of the cluster, name of subfolder in pose estimation results
		 */
		void
		estimate_pose_multiple_views(	std::vector<std::vector<Point_Cloud_N::Ptr> > views,
						std::vector<std::string> model_names,
						std::vector<std::vector<int> > model_view_indices,
						Point_Cloud_N::Ptr cluster,
						std::string scene_name,
						int cluster_index,
						int max_models = 3,
						int max_views = 3);

		/**
			Estimates the pose for all merged views in the cluster and writes the pose data
			@param views The merged views to estimate the pose for
			@param model_names The model name for each merged view
			@param model_view_indices The view indices used to merge each view
			@param cluster The cluster to estimate the pose in
			@param scene_name The name of the original scene, name of main folder in pose estimation results
			@param cluster_index The index of the cluster, name of subfolder in pose estimation results
			@param full_models The full CAD model for each view
		 */
		void
		estimate_pose_merged_views(	std::vector<Point_Cloud_N::Ptr> views,
						std::vector<std::string> model_names,
						std::vector<std::vector<int> > model_view_indices,
						Point_Cloud_N::Ptr cluster,
						std::string scene_name,
						int cluster_index,
						std::vector<Point_Cloud_N::Ptr> full_models);

		/**
			Set the visualizer to be used when showing clouds
			@param visu Pointer to the visualizer
		 */
		void
		set_visualizer(pcl::visualization::PCLVisualizer* visu);

		/**
			Set the visualization mode
			0 = No visualization
			1 = Visualize clustered and filtered poses
			2 = Visualize clustered poses
			3 = Visualize clustered and filtered poses with occlusion
			4 = Visualize clustered and filtered poses with inliers
			@param vis_mode The visualization mode
		 */
		void
		set_visualization_mode(int vis_mode);

		/**
			Set the print mode
			0 = No information printed
			1 = All information printed
			@param p_mode The print mode
		 */
		void
		set_print_mode(int p_mode);
};

#endif
