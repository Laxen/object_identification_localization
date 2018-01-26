#include "Config_reader.h"

namespace pt = boost::property_tree;

/**
	Loads all parameters associated with adding a new model to the system (Rendering views)
	@param filename The name of the config file (.ini)
 */
void
Config_reader::add_model_load_config (const std::string &filename)
{
	// Create empty property tree object
	pt::ptree tree;

	// Parse the INI into the property tree.
	pt::read_ini (filename, tree);

	// Read all config parameters associated with adding a new model 
	resolution = tree.get<int> ("Add_Model-Default.resolution");
	tessellation_level = tree.get<int> ("Add_Model-Default.tessellation_level"); 
	radius_tessellated_sphere = tree.get<float> ("Add_Model-Default.radius_tessellated_sphere");
	if (tree.get<std::string> ("Add_Model-Default.view_angle") == "optimal")
	{
		optimal_view_angle = true;
	}
	else
	{
		view_angle = tree.get<float> ("Add_Model-Default.view_angle");
		optimal_view_angle = false;
	}

	scale_factor = tree.get<double> ("Add_Model-Default.scale_factor");
	leaf_size = tree.get<float> ("Add_Model-Advanced.leaf_size");
	cluster_tolerance = tree.get<double> ("Add_Model-Advanced.cluster_tolerance");
	min_cluster_size = tree.get<int> ("Add_Model-Advanced.min_cluster_size");
	max_cluster_size = tree.get<int> ("Add_Model-Advanced.max_cluster_size");
	search_radius_mls = tree.get<double> ("Add_Model-Advanced.search_radius_mls");
	mean_k = tree.get<int> ("Add_Model-Advanced.mean_k");
	std_dev_mul_thresh = tree.get<float> ("Add_Model-Advanced.std_dev_mul_thresh");
	bad_normals_threshold = tree.get<float> ("Add_Model-Advanced.bad_normals_threshold"); 
	use_k_search = tree.get<bool> ("Add_Model-Advanced.use_k_search");
	k_search_normals = tree.get<int> ("Add_Model-Advanced.k_search_normals");
	use_radius_search = tree.get<bool> ("Add_Model-Advanced.use_radius_search");
	radius_search_normals = tree.get<double> ("Add_Model-Advanced.radius_search_normals");
	largest_cluster_extraction = tree.get<bool> ("Add_Model-Advanced.largest_cluster_extraction");
	downsample = tree.get<bool> ("Add_Model-Advanced.downsample");
	smooth = tree.get<bool> ("Add_Model-Advanced.smooth");
	remove_outliers = tree.get<bool> ("Add_Model-Advanced.remove_outliers");
	view_processed_clouds = tree.get<bool> ("Add_Model-Advanced.view_processed_clouds");
	view_normals = tree.get<bool> ("Add_Model-Advanced.view_normals");
	normal_magnitude = tree.get<float> ("Add_Model-Advanced.normal_magnitude");
	view_complete_model = tree.get<bool> ("Add_Model-Advanced.view_complete_model");
	view_graph = tree.get<bool> ("Add_Model-Advanced.view_graph");
	avg_glb_feature = tree.get<bool> ("Add_Model-Advanced.avg_glb_feature");
}

/**
	Loads all parameters associated with the system
	@param filename The name of the config file (.ini)
 */
void
Config_reader::system_load_config (const std::string &filename)
{
	// Create empty property tree object
	pt::ptree tree;

	// Parse the INI into the property tree.
	pt::read_ini (filename, tree);

	// Read all config parameters associated with adding a new model 
	save_path = tree.get<std::string> ("System-Default.save_path");

	pose_visualization_mode = tree.get<int> ("System-Advanced.pose_visualization_mode");
	pose_print_mode = tree.get<int> ("System-Advanced.pose_print_mode");
	
	hint_view_weight = tree.get<float> ("System-Default.hint_view_weight");
	hint_normal_weight = tree.get<float> ("System-Default.hint_normal_weight");
	hint_feature_weight = tree.get<float> ("System-Default.hint_feature_weight");
	hint_distinguish_weight = tree.get<float> ("System-Default.hint_distinguish_weight");
	hint_below_plane_threshold = tree.get<double> ("System-Advanced.hint_below_plane_threshold");
	hint_misalignment_angle_threshold = tree.get<int> ("System-Advanced.hint_misalignment_angle_threshold");
	hint_view_search_results = tree.get<bool> ("System-Advanced.hint_view_search_results");
	hint_normalize_search_results = tree.get<bool> ("System-Advanced.hint_normalize_search_results");
	hint_print_info = tree.get<bool> ("System-Advanced.hint_print_info");

	rdf_enabled = tree.get<bool> ("System-Default.rdf_enabled");
	rdf_url = tree.get<std::string> ("System-Default.rdf_url");
	rdf_repo_name = tree.get<std::string> ("System-Default.rdf_repo_name");

	use_centered_cluster = tree.get<bool> ("System-Default.use_centered_cluster");
	centered_threshold = tree.get<double> ("System-Default.centered_threshold");

	max_id_views = tree.get<int> ("System-Default.max_id_views");
	max_pose_views = tree.get<int> ("System-Default.max_pose_views");
	max_merged_views = tree.get<int> ("System-Default.max_merged_views");

	visualize_merged_views = tree.get<bool> ("System-Advanced.visualize_merged_views");

	web_service_username = tree.get<std::string> ("System-Default.web_service_username");
	web_service_password = tree.get<std::string> ("System-Default.web_service_password");
	hand_web_service_url = tree.get<std::string> ("System-Default.hand_web_service_url");
}
