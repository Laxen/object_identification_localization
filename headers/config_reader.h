#ifndef CONFIG_READER_H
#define CONFIG_READER_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <set>
#include <exception>
#include <iostream>

class Config_Reader
{
	public:
		// Add_Model
		int resolution;
		int tessellation_level; 
		float view_angle;
		bool optimal_view_angle;
		float radius_tessellated_sphere;
		double scale_factor;
		float leaf_size;
		double cluster_tolerance;
		int min_cluster_size;
		int max_cluster_size;
		double search_radius_mls;
		int mean_k;
		float std_dev_mul_thresh;
		float bad_normals_threshold; 
		bool use_k_search;
		int k_search_normals;
		bool use_radius_search;
		double radius_search_normals;
		bool scale;
		bool largest_cluster_extraction;
		bool downsample;
		bool smooth;
		bool remove_outliers;
		bool view_processed_clouds;
		bool view_normals;
		float normal_magnitude;
		bool view_complete_model;
		bool view_graph;
		bool avg_glb_feature;

		// System
		std::string save_path;
		int pose_visualization_mode;
		int pose_print_mode;
		float hint_view_weight;
		float hint_normal_weight;
		float hint_feature_weight;
		float hint_distinguish_weight;
		double hint_below_plane_threshold;
		int hint_misalignment_angle_threshold;
		bool hint_view_search_results;
		bool hint_normalize_search_results;
		bool hint_print_info;
		bool rdf_enabled;
		std::string rdf_url;
		std::string rdf_repo_name;
		bool use_centered_cluster;
		double centered_threshold;
		int max_id_views;
		int max_pose_views;
		int max_merged_views;
		bool visualize_merged_views;
		std::string web_service_username;
		std::string web_service_password;
		std::string hand_web_service_url;

		// Pose
		int pose_max_iterations;
		double pose_max_correspondence_distance;
		double pose_correspondence_randomness;
		double pose_similarity_threshold;
		double pose_inlier_fraction;
		double pose_inverse_inlier_fraction;
		double pose_feature_radius_search;
		int visualization_mode;
		int print_mode;

		// Segmentation
		double segmentation_leaf_size;
		double segmentation_plane_distance_threshold;
		int segmentation_plane_max_iterations;
		double segmentation_background_segmentation_distance;
		double segmentation_normal_radius_search;
		double segmentation_cluster_tolerance;
		int segmentation_cluster_min_size;
		int segmentation_cluster_max_size;

		/**
		  Loads all parameters associated with adding a new model to the system (Rendering views)
		  @param filename The name of the config file (.ini)
		*/
		void 
		add_model_load_config (const std::string &filename);

		/**
			Loads all parameters associated with the system
			@param filename The name of the config file (.ini)
		 */
		void
		system_load_config (const std::string &filename);

		/**
			Loads all parameters associated with the librealsense_capturer
			@param filename The name of the config file (.ini)
		 */
		void
		capturer_load_config (const std::string &filename);

		/**
			Loads all parameters associated with the background_creator
			@param filename The name of the config file (.ini)
		 */
		void
		background_load_config (const std::string &filename);
};

#endif
