#include "Hint_system_class.h"

/** 
  Constructor. 
*/
Hint_system_class::Hint_system_class (void)
{
	view_search_results_ = false;
	normalize_search_results_ = false;
	view_valid_nodes_ = false;
	print_info_ = false;
	show_utilities_ = false;
	weights_[0] = 0.222;
	weights_[1] = 0.222;
	weights_[2] = 0.222;
	weights_[3] = 0.333;
	BELOW_PLANE_THRESHOLD = -0.01;
	MISALIGNMENT_ANGLE_THRESHOLD = 30;
	PI_CONST = atan (1.0) * 4; // Pi
	Z_AXIS_MISALIGNMENT_THRESHOLD = cos (MISALIGNMENT_ANGLE_THRESHOLD * PI_CONST / 180);
	VALID_AXIS_THR = 15;
	visu_ptr_set = false;
	
	// Load T_CtoH
	Access_Results ar;
	boost::filesystem::path p = ar.path_to_calibration_results();
	p += "/T_CtoH";
	std::ifstream infile(p.string().c_str());
	std::string line;
	int row = 0;
	while(std::getline(infile, line)) 
	{
		std::vector<double> elements;
		while(line.find(",") != -1) 
		{
			int comma_idx = line.find(",");
			double element = std::atof(line.substr(0, comma_idx).c_str());
			elements.push_back(element);
			line.erase(0, comma_idx+1);
		}

		T_CtoH.row(row) << elements[0], elements[1], elements[2], elements[3];
		row++;
	}
	
	// Swap the last row with the last column in T_CtoH
	T_CtoH (0,3) = T_CtoH (3,0); 
	T_CtoH (1,3) = T_CtoH (3,1); 
	T_CtoH (2,3) = T_CtoH (3,2); 
	T_CtoH (3,0) = 0.0;
	T_CtoH (3,1) = 0.0;
	T_CtoH (3,2) = 0.0;
}

/**
  Sets the weights used for summing up the utilities
  @param weights Array containing the weights. Default is equal weights.
*/
void
Hint_system_class::set_weights (float weights[4])
{
	for (int i = 0; i < 4; i++)
	{
		weights_[i] = weights[i];
	}
}

/**
  Sets the below plane threshold
  @param value The value of the threshold in meters
*/
void
Hint_system_class::set_below_plane_threshold (double value)
{
	BELOW_PLANE_THRESHOLD = -value;
}

/**
  Sets the misalignment angle threshold
  @param value The value of the threshold in degrees
*/
void
Hint_system_class::set_misalignment_angle_threshold (double value)
{
	MISALIGNMENT_ANGLE_THRESHOLD = value;
	Z_AXIS_MISALIGNMENT_THRESHOLD = cos (MISALIGNMENT_ANGLE_THRESHOLD * PI_CONST / 180);
}

/**
  Sets the valid axis threshold angle which determines if a node is near a previous camera view.
  @param value The value of the threshold in degrees
*/
void
Hint_system_class::set_valid_axis_threshold (float value)
{
	VALID_AXIS_THR = value;
}

/**
  Set if the valid nodes int the graph should be shown
  @param view_valid_nodes Set this to true to show the valid nodes
*/
void 
Hint_system_class::set_view_valid_nodes (bool view_valid_nodes)
{
	view_valid_nodes_ = view_valid_nodes;
}	

/**
  Sets if the results of the search should be shown 
  @param view_search_results Set this to true to show the results
  @param view_search_results Set this to true to normalize the weighted sum of the utilities
*/
void
Hint_system_class::set_view_search_results (bool view_search_results, bool normalize_search_results)
{
	view_search_results_ = view_search_results;
	normalize_search_results_ = normalize_search_results;
}

/**
  Pass true to show info in terminal during execution
  @param print_info Set this to true to show info in terminal
*/
void
Hint_system_class::set_print_info (bool print_info)
{
	print_info_ = print_info;
}

/**
  Pass true to show info about the utilities in the graph
  @param print_info Set this to true to show utilities
*/
void
Hint_system_class::set_show_utilities (bool show_utilities)
{
	show_utilities_ = show_utilities;
}

/**
  Set the visualizer object to be used when visualizing
  @param visu The visualizer pointer
 */
void
Hint_system_class::set_visualizer(pcl::visualization::PCLVisualizer* visu) 
{
	visu_ptr_set = true;
	visu_ptr = visu;
}

/**
  Checks if the complete model has been loaded for the given model name
  @param model_name The name of the model
  @return A pair object where the first value indicates if the complete model has been loaded and the second value contains the center of mass of the model (if any)
*/
std::pair<bool, Hint_system_class::PointT>
Hint_system_class::complete_model_loaded (std::string model_name)
{
	std::pair<bool, PointT> p;
	
	// Search through the model_centers vector to see if it contains any model with model_name
	bool model_loaded = false;
	int index;
	for (int i = 0; i < model_centers.size(); i++)
	{
		if (model_centers[i].first == model_name)
		{
			model_loaded = true;
			index = i;
			break;
		}
	}
	
	if (model_loaded)
	{
		// Model has been loaded!
		p.first = true;
		p.second = model_centers[index].second;
	}
	else
	{
		p.first = false;
	}
	
	return p;
}

/**
  Loads the complete point cloud model and computes the center of mass
  @param model_name The name of the model
  @param pose The estimated pose
  @return The center of mass of the complete model
*/
Hint_system_class::PointT
Hint_system_class::compute_model_center (std::string model_name, Eigen::Matrix<float,4,4,Eigen::DontAlign> pose)
{
	std::pair<bool, PointT> loaded = complete_model_loaded (model_name);
	PointT center_of_mass;
	if (!(loaded.first))
	{
		// Complete model has not yet been loaded
		if (print_info_)
		{
			std::cout << "Model not loaded!" << std::endl;
		}
		
		Access_Model_data amd;
		PointCloudT::Ptr complete_model (new PointCloudT);
		amd.load_complete_model (model_name, complete_model);
		pcl::transformPointCloud (*complete_model, *complete_model, pose);
	
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid (*complete_model, centroid);
		
   		center_of_mass.x = centroid[0];
		center_of_mass.y = centroid[1];
		center_of_mass.z = centroid[2];
		std::pair<std::string, PointT> temp (model_name, center_of_mass);
		model_centers.push_back (temp);
    }
    else
    {
    	if (print_info_)
    	{
    		std::cout << "Model has already been loaded!" << std::endl;
    	}
    	
    	// Complete model has been loaded
    	center_of_mass = loaded.second;
    }
    
    return center_of_mass;
}

/**
  Loads the pose estimation results 
  @param scene_name The name of the scene
  @param cluster_index The cluster index of the scene
  @param access_results Object for Retrieving the result data
  @param pose_results Vector containing all the data from pose estimation
*/
int
Hint_system_class::load_pose_estimation_results (	std::string scene_name, 
													int cluster_index, 
													Access_Results access_results, 
													std::vector<Pose_estimation_results> &pose_results )
{
	std::vector<std::string> object_names;
	std::vector<std::vector<int> > view_indices;
	std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> > poses;
	std::vector<double> inlier_fractions;
	std::vector<double> accuracies;
	std::cout << "\nLoading pose results..." << std::endl;
	access_results.load_pose_estimation_results (scene_name, cluster_index, object_names, view_indices, poses, inlier_fractions, accuracies);
	std::cout << "Done\n" << std::endl;
	
	if (object_names.size() == 0)
	{
		// No results for given scene and cluster
		return 1;
	}
	
	for (int i = 0; i < object_names.size(); i++)
	{
		Pose_estimation_results per;
		per.name = object_names[i];
		
		Eigen::Matrix<float,4,4,Eigen::DontAlign> temp = poses[i]; // Must have a temp variable here 
		per.pose = temp;
		per.inlier_fraction = inlier_fractions[i];
		per.accuracy = accuracies[i];
		per.model_center = compute_model_center (object_names[i], poses[i]);
		
		// Load view-graph and add transformation from pose estimation
		per.graph.load_graph (object_names[i]);
		per.graph.add_transformation (poses[i]);
		
		pose_results.push_back (per);
	}
	
	if (print_info_)
	{
		for (int i = 0; i < model_centers.size(); i++)
		{
			std::cout << model_centers[i].first << std::endl;
		}
	}	
	
	return 0;
}

/**
  Loads the plane from the scene
  @param scene_name The name of the scene
  @return The plane coefficients
*/
pcl::ModelCoefficients
Hint_system_class::load_plane (std::string scene_name)
{
	Access_Results access_results;
	pcl::ModelCoefficients plane;
	access_results.load_segmentation_plane(scene_name + "/merged", plane);
	
	// Check if the normal is pointing upwards
	if (plane.values[2] > 0)
	{
		// Flip normal
		plane.values[0] *= -1;
		plane.values[1] *= -1;
		plane.values[2] *= -1;
		plane.values[3] *= -1;
	} 
	
	return plane;
}

/**
  Views the valid nodes in the graph. Valid nodes are nodes that are above the plane in the scene
  @param scene_name The name of the scene
  @param per The pose estimation result
  @param plane The plane coefficients
*/
void
Hint_system_class::view_valid_nodes (	std::string scene_name, 
										Pose_estimation_results per, 
										pcl::ModelCoefficients plane )
{	
	// Mark nodes that are below the plane with red spheres
	std::vector<float> r_vec_valid (per.graph.get_size (), 1.0);
	std::vector<float> g_vec_valid (per.graph.get_size (), 0.0);
	std::vector<float> b_vec_valid (per.graph.get_size (), 0.0);
	std::vector<bool> valid_nodes = per.graph.get_valid_nodes ();
	for (int i = 0; i < valid_nodes.size(); i++)
	{	
		if (valid_nodes[i])
		{
			r_vec_valid[i] = 0.0;
			g_vec_valid[i] = 1.0;
		}
	}
	
	if (!visu_ptr_set)
	{
		std::cout << "\nPlease set a visualizer in order to view the valid nodes using the set_visualizer function!\n" << std::endl;
		return;
	}
	
	per.graph.add_graph_to_viewer (*visu_ptr, 0.015, r_vec_valid, g_vec_valid, b_vec_valid, 0, false);
	
	// Add initial view node and rotation
	PointT p = per.graph.get_viewpoint (per.initial_view);
	visu_ptr->addSphere (p, 0.015, 1.0, 1.0, 1.0, "initial_view");
	Eigen::Affine3f a;
	Eigen::Matrix<float,4,4,Eigen::DontAlign> t;
	t.block <3,3> (0,0) = per.graph.get_rotation (per.initial_view).cast <float> ();
	t (0,3) = p.x;
	t (1,3) = p.y;
	t (2,3) = p.z;
	a.matrix () = t;
	visu_ptr->addCoordinateSystem (0.1, a, "rotation_initial_view");
	
	// Add complete model
	Access_Model_data access_model_data;
	PointCloudT::Ptr complete_model (new PointCloudT);
	access_model_data.load_complete_model (per.name, complete_model);
	pcl::transformPointCloud (*complete_model, *complete_model, per.pose);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler_complete_model (complete_model, 0, 0, 255);
	visu_ptr->addPointCloud<PointT> (complete_model, color_handler_complete_model, "complete_model");
	
	// Add scene
	Access_Results ar;
	PointCloudT_color::Ptr scene (new PointCloudT_color);
	ar.load_original_merged_scene (scene_name, scene);
	visu_ptr->addPointCloud<PointT_color> (scene);
	
	// Add plane and camera coordinate system
	visu_ptr->addPlane (plane);
	visu_ptr->addCoordinateSystem (0.1);
	visu_ptr->addCube (-0.01, 0.01, -0.01, 0.01, -0.01, 0.01, 1.0, 1.0, 1.0, "cube", 0);
	visu_ptr->spin ();
	
	visu_ptr->removeAllPointClouds();
	visu_ptr->removeAllShapes ();
	visu_ptr->removeAllCoordinateSystems ();
}

/**
  Checks if the complete model with the given transformation is below the plane
  @param per The pose estimation result
  @param plane The plane coefficients
  @return True if the model is below the plane and false otherwise
*/
bool
Hint_system_class::model_below_plane (Pose_estimation_results per, pcl::ModelCoefficients plane)
{
	// Load and transform complete model
	Access_Model_data access_model_data;
	PointCloudT::Ptr complete_model (new PointCloudT);
	access_model_data.load_complete_model (per.name, complete_model);
	pcl::transformPointCloud (*complete_model, *complete_model, per.pose);
	
	// Check if any points are below a certain threshold
	for (PointCloudT::iterator it = complete_model->begin(); it != complete_model->end(); it++)
	{
		if ((plane.values[0] * it->x + plane.values[1] * it->y + plane.values[2] * it->z + plane.values[3]) < BELOW_PLANE_THRESHOLD)
		{
			// Point is below plane, the pose is invalid!
			return true;
		}
	}
	
	// No points in the complete model were below the plane
	return false;
}

/**
  Aligns the camera z-axis towards the cluster center of mass
  @param camera_center The camera center
  @param model_center The model center of mass
  @return The correctly aligned z-axis viewpoint 
*/
Eigen::Vector3f
Hint_system_class::align_axis_to_cluster_center (PointT camera_center, PointT model_center)
{
	Eigen::Vector3f correct_z_axis;
	correct_z_axis (0) = model_center.x - camera_center.x;
	correct_z_axis (1) = model_center.y - camera_center.y;
	correct_z_axis (2) = model_center.z - camera_center.z;
	correct_z_axis.normalize();
	
	/*
	correct_z_axis (0) = camera_center.x - model_center.x;
	correct_z_axis (1) = camera_center.y - model_center.y;
	correct_z_axis (2) = camera_center.z - model_center.z;
	*/
	
	return correct_z_axis;
}

/**
  Finds the views that best aligns with the current and previous camera axes
  @param per The pose estimation result
*/
void
Hint_system_class::find_aligned_views (Pose_estimation_results &per)
{
	for (int i = 0; i < (previous_positions.size() + 1); i++)
	{
		Eigen::Vector3f cam_axis;
		int aligned_view;
		if (i == 0)
		{
			PointT camera_center;
			camera_center.x = 0;
			camera_center.y = 0;
			camera_center.z = 0;
			cam_axis = align_axis_to_cluster_center (camera_center, per.model_center);
			aligned_view = per.graph.find_aligned_view (cam_axis);
			per.initial_view = aligned_view;
			per.graph.set_valid_node (aligned_view, true);
		}
		else
		{
			Eigen::Vector3f temp = previous_positions[i-1].block <3,1> (0,3);
			PointT camera_center;
			camera_center.x = temp (0);
			camera_center.y = temp (1);
			camera_center.z = temp (2);
			cam_axis = align_axis_to_cluster_center (camera_center, per.model_center);
			aligned_view = per.graph.find_aligned_view (cam_axis);
			if (std::find(per.identified_views.begin(), per.identified_views.end(), aligned_view) != per.identified_views.end())
			{
				continue;
			}
			
			per.graph.set_valid_node (aligned_view, false);
		}
		
		per.identified_views.push_back (aligned_view);
	}
}

/**
  Computes the inlier fraction of the pose. (Source code from pcl::SampleConsensusPrerejective::getFitness)
  @param per The pose estimation result
*/
double
Hint_system_class::get_inlier_fraction (PointCloud_N::Ptr source, PointCloud_N::Ptr target)
{
	/*
	// Find inlier fraction from source to target
	std::vector<int> inliers;
	pcl::KdTreeFLANN<Point_N> kdtree;
	kdtree.setInputCloud (source);

	// Use squared distance for comparison with NN search results
	//const float max_range = corr_dist_threshold_ * corr_dist_threshold_;
	const float max_range = 0.0045 * 0.0045;
	 
	// For each point in the source dataset
	for (size_t i = 0; i < target->points.size (); ++i)
	{
		// Find its nearest neighbor in the target
		std::vector<int> nn_indices (1);
		std::vector<float> nn_dists (1);
		kdtree.nearestKSearch (target->points[i], 1, nn_indices, nn_dists);

		// Check if point is an inlier
		if (nn_dists[0] < max_range)
		{
			// Update inliers
			inliers.push_back (static_cast<int> (i));
		}
	}
	
	double inlier_frac1 = static_cast<double> (inliers.size ()) / static_cast<double> (source->points.size ());
	
	// Find inlier fraction from targe to source
	inliers.clear ();
	kdtree.setInputCloud (target);
	 
	// For each point in the source dataset
	for (size_t i = 0; i < source->points.size (); ++i)
	{
		// Find its nearest neighbor in the target
		std::vector<int> nn_indices (1);
		std::vector<float> nn_dists (1);
		kdtree.nearestKSearch (source->points[i], 1, nn_indices, nn_dists);

		// Check if point is an inlier
		if (nn_dists[0] < max_range)
		{
			// Update inliers
			inliers.push_back (static_cast<int> (i));
		}
	}
	
	double inlier_frac2 = static_cast<double> (inliers.size ()) / static_cast<double> (target->points.size ());
	
	return (inlier_frac1 + inlier_frac2) / 2;
	*/
	
	/*
	// Find inlier fraction from source to target
	std::vector<int> inliers;
	pcl::KdTreeFLANN<Point_N> kdtree;
	kdtree.setInputCloud (target);

	// Use squared distance for comparison with NN search results
	//const float max_range = corr_dist_threshold_ * corr_dist_threshold_;
	const float max_range = 0.0045 * 0.0045;
	 
	// For each point in the source dataset
	for (size_t i = 0; i < source->points.size (); ++i)
	{
		// Find its nearest neighbor in the target
		std::vector<int> nn_indices (1);
		std::vector<float> nn_dists (1);
		kdtree.nearestKSearch (source->points[i], 1, nn_indices, nn_dists);

		// Check if point is an inlier
		if (nn_dists[0] < max_range)
		{
			// Update inliers
			inliers.push_back (static_cast<int> (i));
		}
	}
	
	return (static_cast<double> (inliers.size ()) / static_cast<double> (source->points.size ()));
	*/
	
	//*
	// Find inlier fraction from source to target
	std::vector<int> inliers;
	pcl::KdTreeFLANN<Point_N> kdtree;
	kdtree.setInputCloud (source);

	// Use squared distance for comparison with NN search results
	//const float max_range = corr_dist_threshold_ * corr_dist_threshold_;
	const float max_range = 0.0045 * 0.0045;
	 
	// For each point in the source dataset
	for (size_t i = 0; i < target->points.size (); ++i)
	{
		// Find its nearest neighbor in the target
		std::vector<int> nn_indices (1);
		std::vector<float> nn_dists (1);
		kdtree.nearestKSearch (target->points[i], 1, nn_indices, nn_dists);

		// Check if point is an inlier
		if (nn_dists[0] < max_range)
		{
			// Update inliers
			inliers.push_back (static_cast<int> (i));
		}
	}
	
	return (static_cast<double> (inliers.size ()) / static_cast<double> (target->points.size ()));
	//*/
}

/**
  Recomputes the inlier fraction for the new views. If the recomputed inlier fraction is too low then the pose result is marked as invalid.
  @param per The pose estimation result
  @param scene_name The name of the scene
  @param cluster_index The cluster index
  @return True if the model has low inlier fraction and false otherwise
*/
bool
Hint_system_class::low_inlier_fraction (Pose_estimation_results &per, std::string scene_name, int cluster_index)
{
	Access_Results ar;
	Access_Model_data amd;
	double inlier_thr = 0.1;
	
	// Load source cloud
	PointCloud_N::Ptr source_cloud (new PointCloud_N);
	ar.load_segmentation_merged_cluster (scene_name, cluster_index, source_cloud);
	
	// Load and transform target cloud
	PointCloud_N::Ptr target_cloud (new PointCloud_N);
	amd.load_merged_views (per.name, per.identified_views, target_cloud);
	pcl::transformPointCloud (*target_cloud, *target_cloud, per.pose);
	
	per.inlier_fraction = get_inlier_fraction (source_cloud, target_cloud);
	if (per.inlier_fraction < inlier_thr)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/**
  Checks if identified views and poses is in agreement with the camera position and viewpoint
  @param scene_name The name of the scene
  @param cluster_index The cluster index
  @param pose_results Vector containing all the data from pose estimation
  @param valid_pose_results Vector containing all valid results from pose estimation
  @param invalid_pose_results Vector containing all invalid results from pose estimation
*/
int
Hint_system_class::camera_validation_test (	std::string scene_name,
											int cluster_index,
											std::vector<Pose_estimation_results> &pose_results, 
											std::vector<Pose_estimation_results> &valid_pose_results,
											std::vector<Pose_estimation_results> &invalid_pose_results )
{
	// Load plane for scene
	pcl::ModelCoefficients plane = load_plane (scene_name);
	
	for (int i = 0; i < pose_results.size(); i++)
	{
		// Determine valid nodes in the graph
		pose_results[i].graph.find_valid_nodes (plane, previous_positions, pose_results[i].model_center, VALID_AXIS_THR);
		
		// Find the views that aligns best with current and previous camera z-axis
		find_aligned_views (pose_results[i]);

		if (view_valid_nodes_)
		{
			// View the valid nodes in the graph
			view_valid_nodes (scene_name, pose_results[i], plane);
		}
		
		// Check if the model or some part of it is below the plane
		if (model_below_plane (pose_results[i], plane))
		{
			if (print_info_)
			{
				std::cout << "The complete model for the given pose is below the plane!\n" << std::endl; 
			}
			
			pose_results[i].invalid_pose_position = true;
			invalid_pose_results.push_back (pose_results[i]);
			continue;
		}
		
		// Check the inlier fraction for the new view
		if (low_inlier_fraction (pose_results[i], scene_name, cluster_index))
		{
			if (print_info_)
			{
				std::cout << "The inlier fraction for the new view is too low!\n" << std::endl; 
			}
			
			pose_results[i].low_inlier_fraction = true;
			invalid_pose_results.push_back (pose_results[i]);
			continue;
		}
		
		// The pose is valid! 
		valid_pose_results.push_back (pose_results[i]);
	}
	
	if (valid_pose_results.size() == 0)
	{
		// No valid poses were found!
		return 1;
	}
	else
	{
		// Sort valid pose results
		//std::sort ();
		return 0;
	}
}

/**
  Comparator used to sort Pose_estimation_results objects
  @param per1 The first Pose_estimation_results object
  @param per2 The second Pose_estimation_results object
*/
bool
Hint_system_class::poses_comparator (const Pose_estimation_results& per1, const Pose_estimation_results& per2) 
{
  	return per1.inlier_fraction > per2.inlier_fraction;
}

/**
  Comparator used to sort vectors with Pose_estimation_results objects
  @param vec1 The first vector with Pose_estimation_results object
  @param vec2 The second vector with Pose_estimation_results object
*/
bool
Hint_system_class::poses_comparator_2 (const std::vector<Pose_estimation_results>& vec1, const std::vector<Pose_estimation_results>& vec2) 
{
  	return vec1[0].inlier_fraction > vec2[0].inlier_fraction;
}

/**
  Sorts the valid pose results according to the model names
  @param valid_pose_results Vector containing the valid pose results
  @param return_vec Vector with all identified models and their best poses
  @return vector containing all the (similar) models
*/
std::vector<std::string>
Hint_system_class::sort_valid_pose_results (std::vector<Pose_estimation_results> &valid_pose_results, std::vector<Pose_Data> &return_vec)
{
	// Extract all models present in the valid results
	std::vector<std::string> models;
	models.push_back (valid_pose_results[0].name);
	for (int i = 1; i < valid_pose_results.size(); i++)
	{
		std::vector<std::string>::iterator it = std::find (models.begin(), models.end(), valid_pose_results[i].name);
		if (it == models.end())
		{
			// Model name not present in models
	   		models.push_back (valid_pose_results[i].name);
		}
	}
	
	// Build a vector for each model containing only the valid pose results for each model
	std::vector<std::vector<Pose_estimation_results> > vec;
	for (int i = 0; i < models.size(); i++)
	{
		std::vector<Pose_estimation_results> temp;
		for (int j = 0; j < valid_pose_results.size(); j++)
		{
			if (valid_pose_results[j].name == models[i])
			{
				temp.push_back (valid_pose_results[j]);
			}
		}
		
		// Sort temp
		std::sort (temp.begin(), temp.end(), poses_comparator);
		
		vec.push_back (temp);
	}
	
	// Sort vec
	std::sort (vec.begin(), vec.end(), poses_comparator_2);
	
	// Add models to similar_models_
	for (int i = 0; i < vec.size(); i++)
	{
		similar_models_.push_back (vec[i][0]);
	}
	
	// Add results to return_vec
	for (int i = 0; i < vec.size(); i++)
	{
		Pose_Data pd;
		pd.model_name = vec[i][0].name;
		pd.transformation = vec[i][0].pose;
		pd.accuracy = vec[i][0].accuracy;
		return_vec.push_back (pd);
	}
	
	// Concatenate all vectors into the final sorted vector
	std::vector<Pose_estimation_results> sorted_vec;
	for (int i = 0; i < vec.size(); i++)
	{
		sorted_vec.insert (sorted_vec.end(), vec[i].begin(), vec[i].end());
	}
	
	valid_pose_results = sorted_vec;
	
	return models;
}
	  
/**
  Normalizes the vector between 1.0 and 0.0  
  @param vec The vector to normalize
*/	
void 
Hint_system_class::normalize (std::vector<float> &vec)
{			
	// Sort vec
	std::vector<float> temp = vec;
	std::sort(temp.begin(), temp.end());
	
	float max_value = temp.back();
	float min_value = temp.front();
	for (int i = 0; i < vec.size(); i++)
	{
		vec[i] = (vec[i] - min_value) / (max_value - min_value);
	}
}

/**
  Combines the distinguish-utilities between similar models. Each view is assigned the maximum utility between the similar models. 
  @param distinguish_utilities_vec The distinguish-utilities vector for the similar_models
  @return Vector containing the combined and normalized distinguish-utilities
*/
std::vector<float> 
Hint_system_class::combine_distinguish_utilities (std::vector<std::vector<float> > distinguish_utilities_vec)
{
	std::vector<float> combined_utilities;
	for (int i = 0; i < distinguish_utilities_vec[0].size(); i++)
	{
		std::vector<float> temp;
		for (int j = 0; j < distinguish_utilities_vec.size(); j++)
		{
			temp.push_back (distinguish_utilities_vec[j][i]);
		}
		
		// Sort temp 
		std::sort (temp.begin(), temp.end());
		
		// Add the smallest (worst) distinguish_utility to combined_utilities
		combined_utilities.push_back (temp.front());
	}
	
	return combined_utilities;
}

/**
  Computes a weighted sum of all utilities. If no similar models were detected, the distinguish-utility-weight is split and added equaly 
  to the view-utility -and feature-utility-weight.
  @return Vector containing the summed utilities
*/
std::vector<float>
Hint_system_class::sum_utilities ()
{
	std::vector<float> summed_utilities (view_utilities_.size());
	if (distinguish_utilities_.size() == 0)
	{
		//
		// No similar models, add only view_utilities_, feature_utilities_ and normal_utilities_ to the weighted sum
		//
		
		// Add an equal amount of the distinguish-utility weight to the view-utility weight and feature-utility weight
		weights_[0] += weights_[3]/3;
		weights_[1] += weights_[3]/3;
		weights_[2] += weights_[3]/3;
		
		// Sum utilities
		for (int i = 0; i < view_utilities_.size(); i++)
		{
			summed_utilities[i] = view_utilities_[i] * weights_[0] + normal_utilities_[i] * weights_[1] + feature_utilities_[i] * weights_[2];
		}
	}
	else
	{
		// Sum utilities
		for (int i = 0; i < view_utilities_.size(); i++)
		{
			summed_utilities[i] = view_utilities_[i] * weights_[0] + normal_utilities_[i] * weights_[1] + feature_utilities_[i] * weights_[2] + distinguish_utilities_[i] * weights_[3];
		}
	}
	
	return summed_utilities;
}

/**
  Computes the next and optimal position in robot base coordinate system
  @param graph The view-graph
  @param scene_name the name of the scene
*/
void
Hint_system_class::compute_new_positions (View_graph graph, std::string scene_name)
{
	Eigen::Matrix<float,4,4,Eigen::DontAlign> t_next;
	Eigen::Matrix3d rotation1 = graph.get_rotation (next_view_);
	PointT viewpoint1 = graph.get_viewpoint (next_view_);
	t_next.block <3,3> (0,0) = rotation1.cast <float> ();
	t_next (0,3) = viewpoint1.x;
	t_next (1,3) = viewpoint1.y;
	t_next (2,3) = viewpoint1.z;
	t_next (3,3) = 1.0;
	
	Eigen::Matrix<float,4,4,Eigen::DontAlign> t_optimal;
	Eigen::Matrix3d rotation2 = graph.get_rotation (better_view_);
	PointT viewpoint2 = graph.get_viewpoint (better_view_);
	t_optimal.block <3,3> (0,0) = rotation2.cast <float> ();
	t_optimal (0,3) = viewpoint2.x;
	t_optimal (1,3) = viewpoint2.y;
	t_optimal (2,3) = viewpoint2.z;
	t_optimal (3,3) = 1.0;
	
	/*
	// Set up change of bases matrix C and compute next view and optimal view in the robot base coordinate system
	Eigen::Matrix<float,4,4,Eigen::DontAlign> C = T_HtoB * T_CtoH;
	next_position_ = C * t_next;
	optimal_position_ = C * t_optimal;
	*/
	
	//*
	// Set up change of bases matrix C and compute next and optimal view for the robot hand in the robot base coordinate system
	Eigen::Matrix<float,4,4,Eigen::DontAlign> C = T_HtoB * T_CtoH;
	next_position_ = C * t_next * T_CtoH.inverse ();
	optimal_position_ = C * t_optimal * T_CtoH.inverse ();
	//*/
}

/**
  Loads the current hand position in robot base coordinates
  @param scene_name The name of the scene
*/
void
Hint_system_class::load_T_HtoB (std::string scene_name)
{
	// Load T_HtoB
	Access_Results ar;
	boost::filesystem::path seg_path = ar.path_to_segmentation_results (); 
	std::string path (seg_path.string() + "/" + scene_name + "/robot_data.csv");
	std::ifstream ifs (path.c_str());
	std::string line;
	std::vector<double> elements;
	std::getline(ifs, line);

	while(true) 
	{
		int comma_idx = line.find(",");
		if(comma_idx == -1) break;

		double element = std::atof(line.substr(0, comma_idx).c_str());
		line.erase(0, comma_idx+1);

		elements.push_back(element);
	}

	// Quaternion loaded as w, x, y, z
	double w = elements[3];
	double x = elements[4];
	double y = elements[5];
	double z = elements[6];

	Eigen::Matrix3f R;
	R << 1-2*(pow(y,2) + pow(z,2)), 2*(x*y+w*z), 2*(x*z-w*y),
	  2*(x*y-w*z), 1-2*(pow(x,2)+pow(z,2)), 2*(y*z+w*x),
	  2*(x*z+w*y), 2*(y*z-w*x), 1-2*(pow(x,2)+pow(y,2));

	T_HtoB.block(0,0,3,3) = R.transpose().eval(); // This needs to be transposed, quat2rotm in matlab uses the equations above but transposed, that is why this needs to be transposed as well
	T_HtoB.row(3) << 0,0,0,1;
	T_HtoB.col(3) << elements[0]/1000.0, elements[1]/1000.0, elements[2]/1000.0, 1;
}

/**
  Loads the previous camera positions in robot base coordinates and transforms them to camera base coordinates
  @param scene_name The name of the scene
*/
void
Hint_system_class::load_previous_camera_position (std::string scene_name)
{
	Access_Results ar;
	int scene_index = std::strtol (scene_name.c_str(), NULL, 10) - 1;
	std::stringstream ss;
	while (scene_index >= 0)
	{
		ss.str ("");
		ss << scene_index;
		Eigen::Matrix<float,4,4,Eigen::DontAlign> prev_pos = ar.load_previous_camera_position (ss.str ());
		
		// Change to camera coordiante basis
		Eigen::Matrix<float,4,4,Eigen::DontAlign> C = T_HtoB * T_CtoH;
		prev_pos = C.inverse() * prev_pos;
		
		previous_positions.push_back (prev_pos);
		scene_index--;
	}
	
	/*
	for (int i = 0; i < previous_positions.size(); i++)
	{
		printf ("\n");
		printf ("            | %6.3f %6.3f %6.3f | \n", previous_positions[i] (0,0), previous_positions[i] (0,1), previous_positions[i] (0,2));
		printf ("        R = | %6.3f %6.3f %6.3f | \n", previous_positions[i] (1,0), previous_positions[i] (1,1), previous_positions[i] (1,2));
		printf ("            | %6.3f %6.3f %6.3f | \n", previous_positions[i] (2,0), previous_positions[i] (2,1), previous_positions[i] (2,2));
		printf ("\n");
		printf ("        t = < %3.3f, %3.3f, %3.3f >\n", previous_positions[i] (0,3), previous_positions[i] (1,3), previous_positions[i] (2,3));
	}
	*/
		
}

/**
  Computes the current camera position in robot base coordinates
*/
void
Hint_system_class::compute_current_camera_position ()
{
	current_position = T_HtoB * T_CtoH;
}

/**
  Loads and computes the cluster center of mass
  @param scene_name The name of the scene
  @param cluster_index The cluster index
/  
void 
Hint_system_class::cluster_center_of_mass (std::string scene_name, int cluster_index)
{
	// Load cluster
	Access_Results ar;
	PointCloud_N::Ptr cluster_cloud (new PointCloud_N);
	ar.load_segmentation_merged_cluster (scene_name, cluster_index, cluster_cloud);
	
	// Compute center of mass
	Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*cluster_cloud, centroid);
    cluster_center.x = centroid[0];
    cluster_center.y = centroid[1];
    cluster_center.z = centroid[2];
}
*/

/**
  Adds the segmentation results to the viewer
  @param viewer The visualization object
  @param scene_name The name of the scene
  @param per The pose estimation results 
*/
void
Hint_system_class::add_segmentation_results_to_viewer (pcl::visualization::PCLVisualizer &viewer, std::string scene_name, Pose_estimation_results per)
{
	Access_Results access_results;
	Access_Model_data access_model_data;

	if (scene_name == "0")
	{
		// Load single complete, unsegmented scene, initial view and better view
		PointCloudT_color::Ptr scene_original (new PointCloudT_color);
		access_results.load_original_scene_single (scene_name, scene_original);
		viewer.addPointCloud<PointT_color> (scene_original, "scene_original");
	
		PointCloud_N::Ptr initial_view_cloud (new PointCloud_N);
		PointCloud_N::Ptr better_view_cloud (new PointCloud_N);
		std::vector<PointCloud_N::Ptr> views;
		std::vector<int> indices;
		indices.push_back (per.initial_view);
		indices.push_back (better_view_);

		access_model_data.load_model_views_original_pose (per.name, views, indices);
		initial_view_cloud = views[0];
		better_view_cloud = views[1];
	
		// Transform initial_view_cloud to match the identified cluster in the scene
		PointCloud_N::Ptr initial_view_cloud_transformed (new PointCloud_N);
		pcl::transformPointCloud (*initial_view_cloud, *initial_view_cloud_transformed, per.pose);
		pcl::visualization::PointCloudColorHandlerCustom<Point_N> color_handler_initial_view (initial_view_cloud_transformed, 255, 0, 0);
		viewer.addPointCloud<Point_N> (initial_view_cloud_transformed, color_handler_initial_view, "initial_view_cloud_transformed");
	
		// Transform better_view_cloud to match the identified cluster in the scene
		PointCloud_N::Ptr better_view_cloud_transformed (new PointCloud_N);
		pcl::transformPointCloud (*better_view_cloud, *better_view_cloud_transformed, per.pose);
		pcl::visualization::PointCloudColorHandlerCustom<Point_N> color_handler_better_view (better_view_cloud_transformed, 0, 255, 0);
		viewer.addPointCloud<Point_N> (better_view_cloud_transformed, color_handler_better_view, "better_view_cloud_transformed");
	}
	else
	{
		// Load merged complete, unsegmented scene, initial view and better view
		Access_Results access_results;
		PointCloudT_color::Ptr merged_scene_original (new PointCloudT_color);
		access_results.load_original_merged_scene (scene_name, merged_scene_original);
		viewer.addPointCloud<PointT_color> (merged_scene_original, "merged_scene_original");
		
		if (per.identified_views.size() == 1)
		{
			// No previous identified views
			PointCloud_N::Ptr initial_view_cloud (new PointCloud_N);
			PointCloud_N::Ptr better_view_cloud (new PointCloud_N);
			std::vector<PointCloud_N::Ptr> views;
			std::vector<int> indices;
			indices.push_back (per.initial_view);
			indices.push_back (better_view_);

			access_model_data.load_model_views_original_pose (per.name, views, indices);
			initial_view_cloud = views[0];
			better_view_cloud = views[1];
	
			// Transform initial_view_cloud to match the identified cluster in the scene
			PointCloud_N::Ptr initial_view_cloud_transformed (new PointCloud_N);
			pcl::transformPointCloud (*initial_view_cloud, *initial_view_cloud_transformed, per.pose);
			pcl::visualization::PointCloudColorHandlerCustom<Point_N> color_handler_initial_view (initial_view_cloud_transformed, 255, 0, 0);
			viewer.addPointCloud<Point_N> (initial_view_cloud_transformed, color_handler_initial_view, "initial_view_cloud_transformed");
	
			// Transform better_view_cloud to match the identified cluster in the scene
			PointCloud_N::Ptr better_view_cloud_transformed (new PointCloud_N);
			pcl::transformPointCloud (*better_view_cloud, *better_view_cloud_transformed, per.pose);
			pcl::visualization::PointCloudColorHandlerCustom<Point_N> color_handler_better_view (better_view_cloud_transformed, 0, 255, 0);
			viewer.addPointCloud<Point_N> (better_view_cloud_transformed, color_handler_better_view, "better_view_cloud_transformed");
		}
		else
		{
			PointCloud_N::Ptr merged_view_cloud (new PointCloud_N);
			PointCloud_N::Ptr better_view_cloud (new PointCloud_N);
			std::vector<PointCloud_N::Ptr> views;
			std::vector<int> indices_merged_views;
			std::vector<int> indices_views;
			for (int i = 0; i < per.identified_views.size(); i++)
			{
				indices_merged_views.push_back (per.identified_views[i]);
			}
		
			indices_views.push_back (better_view_);
			access_model_data.load_merged_views (per.name, indices_merged_views, merged_view_cloud);
			access_model_data.load_model_views_original_pose (per.name, views, indices_views);
			better_view_cloud = views[0];
			
			// Transform merged_view_cloud to match the identified cluster in the scene
			PointCloud_N::Ptr merged_view_cloud_transformed (new PointCloud_N);
			pcl::transformPointCloud (*merged_view_cloud, *merged_view_cloud_transformed, per.pose);
			pcl::visualization::PointCloudColorHandlerCustom<Point_N> color_handler_merged_view_cloud (merged_view_cloud_transformed, 255, 0, 0);
			viewer.addPointCloud<Point_N> (merged_view_cloud_transformed, color_handler_merged_view_cloud, "merged_view_cloud_transformed");
	
			// Transform better_view_cloud to match the identified cluster in the scene
			PointCloud_N::Ptr better_view_cloud_transformed (new PointCloud_N);
			pcl::transformPointCloud (*better_view_cloud, *better_view_cloud_transformed, per.pose);
			pcl::visualization::PointCloudColorHandlerCustom<Point_N> color_handler_better_view (better_view_cloud_transformed, 0, 255, 0);
			viewer.addPointCloud<Point_N> (better_view_cloud_transformed, color_handler_better_view, "better_view_cloud_transformed");
		}
	}
}

/**
  Views the results of the search. In the first viewer, the red node is the starting view-node and the green node is the better view-node. 
  Blue nodes are nodes that have been visited when searching for a better node. The different utilities are shown in the second viewer.
  A green node indicates a high utility while a red node indicates a low utility. The original scene with hint-info is shown in the third viewer.   
  @param scene_name The name of the scene
  @param per The pose estimation results
*/
void
Hint_system_class::view_search_results (std::string scene_name, Pose_estimation_results &per)
{
	Access_Model_data access_model_data;
	std::vector<float> r_vec_search (per.graph.get_size (), 0.1);
	std::vector<float> g_vec_search (per.graph.get_size (), 0.1);
	std::vector<float> b_vec_search (per.graph.get_size (), 0.1);
	
	if (show_utilities_)
	{
		// Create first viewer with two viewports 
		pcl::visualization::PCLVisualizer visu ("viewer");
		int vp_1;
		int vp_2;
		visu.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
		visu.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	
		//
		// Plot search information in first viewport
		//
	
		// Assign colors for nodes according to the function description
		r_vec_search[per.initial_view] = 1.0;
		g_vec_search[per.initial_view] = 0.0;
		b_vec_search[per.initial_view] = 0.0;
		r_vec_search[better_view_] = 0.0;
		g_vec_search[better_view_] = 1.0;
		b_vec_search[better_view_] = 0.0;
	
		std::vector<bool> visited_nodes = per.graph.get_visited_nodes ();
		for (int i = 0; i < visited_nodes.size(); i++)
		{
			if (visited_nodes[i] && i != per.initial_view && i != better_view_)
			{
				r_vec_search[i] = 0.0;
				g_vec_search[i] = 0.0;
				b_vec_search[i] = 1.0;
			}
		}
	
		// Add graph displaying search information to viewport vp_1
		per.graph.add_graph_to_viewer (visu, 0.015, r_vec_search, g_vec_search, b_vec_search, vp_1, false);
		visu.addText ("Search results", 10, 10, 18, 1.0, 1.0, 1.0, "text1", vp_1);
	
		//
		// Plot summed utility information in second viewport
		//
	
		// Assign colors for nodes according to the utilities. High utility => green, low utility => red.
		std::vector<float> r_vec (per.graph.get_size (), 0.0);
		std::vector<float> g_vec (per.graph.get_size (), 0.0);
		std::vector<float> b_vec (per.graph.get_size (), 0.0);
		for (int i = 0; i < per.graph.get_size (); i++)
		{
			r_vec[i] = 1.0 - weighted_sum_utilities_[i];
			g_vec[i] = weighted_sum_utilities_[i];
		}
	
		per.graph.add_graph_to_viewer (visu, 0.015, r_vec, g_vec, b_vec, vp_2, false);
		visu.addText ("Summed utilities", 10, 10, 18, 1.0, 1.0, 1.0, "text2", vp_2);
										
		// Add model with correct transformation to viewer
		PointCloudT::Ptr cloud (new PointCloudT);
		access_model_data.load_complete_model (per.name, cloud);
		pcl::transformPointCloud (*cloud, *cloud, per.pose);
	
		visu.addPointCloud<PointT> (cloud, "cloud1", vp_1);
		visu.addPointCloud<PointT> (cloud, "cloud2", vp_2);
		//visu.resetCameraViewpoint ("cloud1");
		visu.spinOnce ();
	
		//
		// Create second viewer with four viewports displaying all utilities
		//
	
		pcl::visualization::PCLVisualizer visu2 ("viewer2");
		int vp2_1;
		int vp2_2;
		int vp2_3;
		int vp2_4;
		if (distinguish_utilities_.size() == 0)
		{
			visu2.createViewPort (0.0, 0, 0.33, 1.0, vp2_1);
			visu2.createViewPort (0.33, 0, 0.66, 1.0, vp2_2);
			visu2.createViewPort (0.66, 0, 1.0, 1.0, vp2_3);
		}
		else
		{
			visu2.createViewPort (0.0, 0, 0.25, 1.0, vp2_1);
			visu2.createViewPort (0.25, 0, 0.5, 1.0, vp2_2);
			visu2.createViewPort (0.5, 0, 0.75, 1.0, vp2_3);
			visu2.createViewPort (0.75, 0, 1.0, 1.0, vp2_4);
		}
	
		// Plot view-utility information in first viewport. Assign colors for nodes according to the view-utilities. High utility => green, low utility => red.
		for (int i = 0; i < per.graph.get_size (); i++)
		{
			r_vec[i] = 1.0 - view_utilities_[i];
			g_vec[i] = view_utilities_[i];
		}
	
		per.graph.add_graph_to_viewer (visu2, 0.015, r_vec, g_vec, b_vec, vp2_1, false);
		visu2.addText ("View-utilities", 10, 10, 18, 1.0, 1.0, 1.0, "text1", vp2_1);
		
		// Plot normal-utility information in second viewport. Assign colors for nodes according to the normal-utilities. High utility => green, low utility => red.
		for (int i = 0; i < per.graph.get_size (); i++)
		{
			r_vec[i] = 1.0 - normal_utilities_[i];
			g_vec[i] = normal_utilities_[i];
		}
	
		per.graph.add_graph_to_viewer (visu2, 0.015, r_vec, g_vec, b_vec, vp2_2, false);
		visu2.addText ("Normal-utilities", 10, 10, 18, 1.0, 1.0, 1.0, "text2", vp2_2);
	
		// Plot feature-utility information in third viewport. Assign colors for nodes according to the feature-utilities. High utility => green, low utility => red.
		for (int i = 0; i < per.graph.get_size (); i++)
		{
			r_vec[i] = 1.0 - feature_utilities_[i];
			g_vec[i] = feature_utilities_[i];
		}
	
		per.graph.add_graph_to_viewer (visu2, 0.015, r_vec, g_vec, b_vec, vp2_3, false);
		visu2.addText ("Feature-utilities", 10, 10, 18, 1.0, 1.0, 1.0, "text3", vp2_3);
	
		if (distinguish_utilities_.size() == 0)
		{
			// Add model to viewer
			visu2.addPointCloud<PointT> (cloud, "cloud1", vp2_1);
			visu2.addPointCloud<PointT> (cloud, "cloud2", vp2_2);
			visu2.addPointCloud<PointT> (cloud, "cloud3", vp2_3);
			//visu2.resetCameraViewpoint ("cloud1");
			visu2.spinOnce ();
		}
		else
		{
			// Plot distinguish-utility information in fourth viewport. Assign colors for nodes according to the distinguish-utilities. High utility => green, low utility => red.
			for (int i = 0; i < per.graph.get_size (); i++)
			{
				r_vec[i] = 1.0 - distinguish_utilities_[i];
				g_vec[i] = distinguish_utilities_[i];
			}
	
			per.graph.add_graph_to_viewer (visu2, 0.015, r_vec, g_vec, b_vec, vp2_4, false);
			visu2.addText ("Distinguish-utilities", 10, 10, 18, 1.0, 1.0, 1.0, "text4", vp2_4);
	
			// Add model to viewer
			visu2.addPointCloud<PointT> (cloud, "cloud1", vp2_1);
			visu2.addPointCloud<PointT> (cloud, "cloud2", vp2_2);
			visu2.addPointCloud<PointT> (cloud, "cloud3", vp2_3);
			visu2.addPointCloud<PointT> (cloud, "cloud4", vp2_4);
			//visu2.resetCameraViewpoint ();
			visu2.spinOnce ();
		}
	}
	
	//
	// Create a third viewer displaying hint information in the scene
	//
	
	if (!visu_ptr_set)
	{
		std::cout << "\nPlease set a visualizer in order to view the search results using the set_visualizer function!\n" << std::endl;
		return;
	}
	
	add_segmentation_results_to_viewer (*visu_ptr, scene_name, per);
	
	// Add graph to scene according to the new transformation
	std::fill (r_vec_search.begin (), r_vec_search.end (), 0.1);
	std::fill (g_vec_search.begin (), g_vec_search.end (), 0.1);
	std::fill (b_vec_search.begin (), b_vec_search.end (), 0.1);
	r_vec_search[better_view_] = 0.0;
	g_vec_search[better_view_] = 0.0;
	b_vec_search[better_view_] = 1.0;
	std::vector<bool> valid_nodes = per.graph.get_valid_nodes ();
	for (int i = 0; i < valid_nodes.size(); i++)
	{
		if (valid_nodes[i] && i != better_view_)
		{
			r_vec_search[i] = 1.0 - weighted_sum_utilities_[i];
			g_vec_search[i] = weighted_sum_utilities_[i];
		}
	}
	
	// Mark identified views with white spheres
	for (int i = 0; i < per.identified_views.size(); i++)
	{
		r_vec_search[per.identified_views[i]] = 1.0;
		g_vec_search[per.identified_views[i]] = 1.0;
		b_vec_search[per.identified_views[i]] = 1.0;
	}
	per.graph.add_graph_to_viewer (*visu_ptr, 0.015, r_vec_search, g_vec_search, b_vec_search, 0, false);
	
	// Add path to better_view_
	std::stringstream ss;
	int new_generation_index = 0;
	for (int i = 0; i < trajectory_.size() - 1; i++)
	{
		ss.str ("");
		ss << "trajectory" << new_generation_index << i;
		visu_ptr->addLine (per.graph.get_viewpoint (trajectory_[i]), per.graph.get_viewpoint (trajectory_[i+1]), 0.0, 0.0, 1.0, ss.str (), 0);
	}
	
	// Add optimal position and rotation for camera to scene
	Eigen::Affine3f a;
	Eigen::Matrix<float,4,4,Eigen::DontAlign> t;
	Eigen::Matrix3d rotation = per.graph.get_rotation (better_view_);
	PointT viewpoint = per.graph.get_viewpoint (better_view_);
	t.block <3,3> (0,0) = rotation.cast <float> ();
	t (0,3) = viewpoint.x;
	t (1,3) = viewpoint.y;
	t (2,3) = viewpoint.z;
	t (3,3) = 1.0;
	a.matrix () = t;
	visu_ptr->addCoordinateSystem (0.1, a, "rotation_better_view");
	
	// Add current camera positiona and rotation
	visu_ptr->addCoordinateSystem (0.1);
	visu_ptr->addCube (-0.01, 0.01, -0.01, 0.01, -0.01, 0.01, 1.0, 1.0, 1.0, "cube", 0);
	
	// Add previous camera positions and rotations to scene
	for (int i = 0; i < previous_positions.size(); i++)
	{
		a.matrix () = previous_positions[i];
		ss.str ("");
		ss << "prev_pos" << i;
		visu_ptr->addCoordinateSystem (0.05, a, ss.str ());
		visu_ptr->addCube (	previous_positions[i](0,3) - 0.005, 
							previous_positions[i](0,3) + 0.005,
							previous_positions[i](1,3) - 0.005,
						  	previous_positions[i](1,3) + 0.005,
						   	previous_positions[i](2,3) - 0.005,
						    previous_positions[i](2,3) + 0.005, 1.0, 1.0, 1.0, ss.str (), 0);
	}
	
	// Add text to scene showing the next and optimal position in robot base coordinate system  
	ss.str ("");
	ss << "Next position: <" << next_position_(0,3)*1000 << ", " << next_position_(1,3)*1000 << ", " << next_position_(2,3)*1000 << ">";
	visu_ptr->addText (ss.str (), 10, 40, 20, 1.0, 1.0, 1.0, "next_pos");
	ss.str ("");
	ss << "Optimal position: <" << optimal_position_(0,3)*1000 << ", " << optimal_position_(1,3)*1000 << ", " << optimal_position_(2,3)*1000 << ">";
	visu_ptr->addText (ss.str (), 10, 20, 20, 1.0, 1.0, 1.0, "optimal_pos");
	
	// Add text to scene showing the inlier match for the models
	int y_pos = 980;
	for (int i = 0; i < similar_models_.size(); i++)
	{
		ss.str ("");
		ss << similar_models_[i].name << ": " << similar_models_[i].accuracy;
		visu_ptr->addText (ss.str (), 10, y_pos, 20, 1.0, 1.0, 1.0);
		y_pos -= 20;
	}
	
	visu_ptr->spin ();
	
	std::string input;
	std::cout << "Optimal view OK? (y/n)\n" << std::endl;
	std::cin >> input;
	while (input != "y")
	{
		// Remove better_view_cloud
		visu_ptr->removePointCloud ("better_view_cloud_transformed");
		
		// Change color of better_view node
		ss.str ("");
		ss << "Sphere" << better_view_ << "viewport" << 0;
		visu_ptr->updateSphere (viewpoint, 0.02, 0.1, 0.1, 0.1, ss.str ());
		
		// Update valid nodes for view-graph
		per.graph.set_valid_node (better_view_, false);
		
		// There is no way in pcl::visualizer to remove or update lines so the best way to "remove" them is to make them completely black
		new_generation_index++;
		for (int i = 0; i < trajectory_.size() - 1; i++)
		{
			ss.str ("");
			ss << "trajectory" << new_generation_index << i;
			if (per.graph.is_neighbor (trajectory_[i], trajectory_[i+1]))
			{
				visu_ptr->addLine (per.graph.get_viewpoint (trajectory_[i]), per.graph.get_viewpoint (trajectory_[i+1]), 1.0, 1.0, 1.0, ss.str (), 0);
			}
			else
			{
				visu_ptr->addLine (per.graph.get_viewpoint (trajectory_[i]), per.graph.get_viewpoint (trajectory_[i+1]), 0.0, 0.0, 0.0, ss.str (), 0);
			}
		}
		
		// Search graph for better view
		trajectory_ = per.graph.search_for_better_view (per.initial_view);
		better_view_ = trajectory_.front ();
		next_view_ = trajectory_[trajectory_.size() - 2];
		
		// Add new path to better_view_
		new_generation_index++;
		for (int i = 0; i < trajectory_.size() - 1; i++)
		{
			ss.str ("");
			ss << "trajectory" << new_generation_index << i;
			visu_ptr->addLine (per.graph.get_viewpoint (trajectory_[i]), per.graph.get_viewpoint (trajectory_[i+1]), 0.0, 0.0, 1.0, ss.str (), 0);
		}
		
		// Transform better_view_cloud to match the identified cluster in the scene
		std::vector<int> indices;
		std::vector<PointCloud_N::Ptr> views;
		PointCloud_N::Ptr better_view_cloud (new PointCloud_N);
		PointCloud_N::Ptr better_view_cloud_transformed (new PointCloud_N);
		indices.push_back (better_view_);
		access_model_data.load_model_views_original_pose (per.name, views, indices);
		better_view_cloud = views[0];
		pcl::transformPointCloud (*better_view_cloud, *better_view_cloud_transformed, per.pose);
		pcl::visualization::PointCloudColorHandlerCustom<Point_N> color_handler_better_view (better_view_cloud_transformed, 0, 255, 0);
		visu_ptr->addPointCloud<Point_N> (better_view_cloud_transformed, color_handler_better_view, "better_view_cloud_transformed");
		
		// Add new position and rotation for camera to scene
		rotation = per.graph.get_rotation (better_view_);
		viewpoint = per.graph.get_viewpoint (better_view_);
		t.block <3,3> (0,0) = rotation.cast <float> ();
		t (0,3) = viewpoint.x;
		t (1,3) = viewpoint.y;
		t (2,3) = viewpoint.z;
		t (3,3) = 1.0;
		a.matrix () = t;
		visu_ptr->removeCoordinateSystem ("rotation_better_view");
		visu_ptr->addCoordinateSystem (0.1, a, "rotation_better_view");
		ss.str ("");
		ss << "Sphere" << better_view_ << "viewport" << 0;
		visu_ptr->updateSphere (viewpoint, 0.02, 0.0, 0.0, 1.0, ss.str ());
		
		// Compute next and optimal position for robot arm
		compute_new_positions (per.graph, scene_name);
		ss.str ("");
		ss << "Next position: <" << next_position_(0,3)*1000 << ", " << next_position_(1,3)*1000 << ", " << next_position_(2,3)*1000 << ">";
		visu_ptr->updateText (ss.str (), 10, 40, 20, 1.0, 1.0, 1.0, "next_pos");
		ss.str ("");
		ss << "Optimal position: <" << optimal_position_(0,3)*1000 << ", " << optimal_position_(1,3)*1000 << ", " << optimal_position_(2,3)*1000 << ">";
		visu_ptr->updateText (ss.str (), 10, 20, 20, 1.0, 1.0, 1.0, "optimal_pos");
		visu_ptr->spin ();

		std::cout << "Optimal view OK? (y/n)\n" << std::endl;
		std::cin >> input;
	}
	
	visu_ptr->removeAllPointClouds();
	visu_ptr->removeAllShapes ();
	visu_ptr->removeAllCoordinateSystems ();
}

/**
  Saves the current camera position
  @param scene_name The name of the scene
*/
void 
Hint_system_class::save_current_camera_position (std::string scene_name)
{
	// Add content of the current camera position
	std::stringstream ss;
	Access_Results ar;
	for (int row = 0; row < 4; row++)
	{
		for (int col = 0; col < 4; col++)
		{
			ss << current_position (row,col) << ",";
		}
	}
	
	// Add path to Hint_system_results in Results
	boost::filesystem::path p = ar.path_to_hint_system_results ();

	// Add path to scene
	p /= scene_name;

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory for scene
		if (!boost::filesystem::create_directory(p))
		{
			ss.str ("");
			ss << "ERROR: Could not create directory for " << scene_name << " in Hint_system_results!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}
	
	p /= "current_camera_position.csv";
	if (!boost::filesystem::exists(p))
	{
		// Save current camera position
		std::ofstream csv_file;
		csv_file.open (p.string().c_str());
		csv_file << ss.str ();
		csv_file.close();
	}
}

/**
  Saves the results
  @param scene_name The name of the scene
  @param cluster_index The cluster index
  @param valid_pose_results The valid pose results
  @param invalid_pose_results The invalid pose results
  @param flag A flag telling the function which variables to save
*/
void
Hint_system_class::save_results ( 	std::string scene_name, 
									int cluster_index, 
									std::vector<Pose_estimation_results> valid_pose_results, 
									std::vector<Pose_estimation_results> invalid_pose_results,
									int flag )
{
	Access_Results access_results;
	std::stringstream ss;	
	std::vector<std::string> valid_pose_results_str_vec; 
	std::vector<std::string> invalid_pose_results_str_vec; 
	std::vector<std::string> weighted_sum_utilities_str_vec; 
	std::vector<std::string> trajectory_str_vec;
	std::vector<std::string> next_position_str_vec;
	
	// Add content from the invalid pose results
	for (int i = 0; i < invalid_pose_results.size(); i++)
	{
		Pose_estimation_results invalid_per = invalid_pose_results[i];
		ss.str ("");
		ss << invalid_per.name << ",";
	
		for (int j = 0; j < invalid_per.identified_views.size(); j++)
		{
			ss << invalid_per.identified_views[j] << "+";
		}
		ss << ",";
	
		for (int row = 0; row < 4; row++)
		{
			for ( int col = 0; col < 4; col++)
			{
				ss << invalid_per.pose(row, col) << ",";
			}
		}
	
		ss << invalid_per.inlier_fraction << "," << invalid_per.accuracy << ",";
	
		if (invalid_per.invalid_pose_position)
		{
			ss << " Invalid pose position!";
		}
		else if (invalid_per.low_inlier_fraction)
		{
			ss << " Low inlier fraction for aligned views!";
		}

		invalid_pose_results_str_vec.push_back (ss.str ());
	}
	
	if (flag == 1)
	{
		// Add content from the valid pose results
		for (int i = 0; i < valid_pose_results.size(); i++)
		{
			Pose_estimation_results valid_per = valid_pose_results[i];
			ss.str ("");
			ss << valid_per.name << ",";
		
			for (int j = 0; j < valid_per.identified_views.size(); j++)
			{
				ss << valid_per.identified_views[j] << "+";
			}
			ss << ",";
		
			for (int row = 0; row < 4; row++)
			{
				for ( int col = 0; col < 4; col++)
				{
					ss << valid_per.pose(row, col) << ",";
				}
			}
		
			ss << valid_per.inlier_fraction << "," << valid_per.accuracy << ",";
			valid_pose_results_str_vec.push_back (ss.str ());
		}
		
		// Add content from the weighted sum utilities results
		for (int i = 0; i < weighted_sum_utilities_.size(); i++)
		{
			ss.str ("");
			ss << i << "," << weighted_sum_utilities_[i] << ",";
			weighted_sum_utilities_str_vec.push_back (ss.str ());
		}
	
		// Add content from the trajectory results
		for (int i = 0; i < trajectory_.size(); i++)
		{
			ss.str ("");
			ss << trajectory_[i] << ",";
			trajectory_str_vec.push_back (ss.str ());
		}
	
		//
		// Add content for robot data 
		//
	
		// Transform next_position_
		ss.str ("");
		ss 	<< next_position_(0,3)*1000 << "," 
			<< next_position_(1,3)*1000 << "," 
			<< next_position_(2,3)*1000 << ","; 
		
		// Convert rotation matrix of next_position_ to Euler angles
	
	
		next_position_str_vec.push_back (ss.str ());
	}
	
	access_results.save_hint_system_results (	scene_name, 
												cluster_index, 
												valid_pose_results_str_vec, 
												invalid_pose_results_str_vec, 
												weighted_sum_utilities_str_vec, 
												trajectory_str_vec,
												next_position_str_vec );
}

/**
  Finds a new view given an identified scene.  
  @param cluster_index The index of the cluster of the scene
  @return The vector containing all identified models with their best pose and the next position for the robot hand
  @return  containing all identified models with their best pose and the next position for the robot hand
*/
Hint_System_Data
Hint_system_class::find_new_view (std::string scene_name, int cluster_index)
{
	Hint_System_Data hs_data;
	
	// Load current robot hand position and previous camera positions in robot base coordinates
	load_T_HtoB (scene_name);
	load_previous_camera_position (scene_name);
	
	// Compute and save current camera position in robot base coordinates
	compute_current_camera_position ();
	save_current_camera_position (scene_name);
	
	/*
	// Load the cluster point cloud and compute the center of mass for it
	cluster_center_of_mass (scene_name, cluster_index);
	*/
	
	//
	// Load results from pose estimation and reject the invalid results
	//
	
	Access_Results access_results;
	std::vector<Pose_estimation_results> pose_results;
	if (load_pose_estimation_results (scene_name, cluster_index, access_results, pose_results) > 0)
	{
		std::cout << "\nNo pose estimation results for scene: " << scene_name << ", cluster: " << cluster_index << "\n" << std::endl;
		previous_positions.clear ();
		hs_data.valid_results = false;
		return hs_data;
	}
	
	// Reject invalid poses
	std::vector<Pose_estimation_results> valid_pose_results;
	std::vector<Pose_estimation_results> invalid_pose_results;
	if (camera_validation_test (scene_name, cluster_index, pose_results, valid_pose_results, invalid_pose_results) > 0)
	{
		std::cout << "\nNo valid pose estimation results for scene: " << scene_name << ", cluster: " << cluster_index << "\n" << std::endl;
		
		// Save results from Hint system
		save_results (scene_name, cluster_index, valid_pose_results, invalid_pose_results, 0);
		previous_positions.clear ();
		hs_data.valid_results = false;
		return hs_data;
	}
	
	// Sort the valid pose results and extract the similar models
	std::vector<Pose_Data> pd;
	std::vector<std::string> similar_model_names = sort_valid_pose_results (valid_pose_results, pd);
	hs_data.pd = pd;
	
	// Extract the best valid pose
	Pose_estimation_results best_pose_result = valid_pose_results[0];
	
	//
	// Search for better view given the best valid pose estimation result
	//
	
	/*
	// Load similar models from the results of identification
	std::vector<std::string> similar_models;
	access_results.load_similar_models (scene_name, cluster_index, best_pose_result.name, similar_models); 
	*/
	
	if (print_info_)
	{
		std::cout << "Valid models:" << std::endl;
		for (int i = 0; i < valid_pose_results.size(); i++)
		{
			std::cout << "\t" << valid_pose_results[i].name << std::endl;
		}
		
		std::cout << "Similar models: " << std::endl;
		for (int i = 0; i < similar_model_names.size(); i++)
		{
			std::cout << "\t" << similar_model_names[i] << std::endl;
		}
	
		std::cout << "\nBest pose: " << best_pose_result.name << std::endl;
	}
	
	// Load utilities from Model_data
	Access_Model_data access_model_data;
	view_utilities_ = access_model_data.load_view_utilities (best_pose_result.name);
	feature_utilities_ = access_model_data.load_feature_utilities (best_pose_result.name);
	normal_utilities_ = access_model_data.load_normal_utilities (best_pose_result.name);
	std::vector<std::vector<float> > distinguish_utilities_vec = access_model_data.load_distinguish_utilities (best_pose_result.name, similar_model_names); 
	
	distinguish_utilities_.clear ();
	if (distinguish_utilities_vec.size() > 1)
	{
		// Combine distinguish_utilities_vec into one vector
		distinguish_utilities_ = combine_distinguish_utilities (distinguish_utilities_vec);
		
		// Normalize vector
		normalize (distinguish_utilities_);
	}
	else if (distinguish_utilities_vec.size() == 1)
	{
		distinguish_utilities_ = distinguish_utilities_vec[0];
		
		// Normalize vector
		normalize (distinguish_utilities_);
	}
	
	// Combine all utilities into a weighted sum
	weighted_sum_utilities_ = sum_utilities ();
	//weighted_sum_utilities_[12] = 0.0;
	normalize (weighted_sum_utilities_);
	
	// Add weighted utilities to graph
	best_pose_result.graph.add_utilities (weighted_sum_utilities_);
	
	// Search graph for better view
	trajectory_ = best_pose_result.graph.search_for_better_view (best_pose_result.initial_view);
	better_view_ = trajectory_.front ();
	next_view_ = trajectory_[trajectory_.size() - 2];
	
	// Compute new position for robot arm
	compute_new_positions (best_pose_result.graph, scene_name);
	
	if (view_search_results_)
	{
		if (normalize_search_results_)
		{
			normalize (weighted_sum_utilities_);
		}
		
		// View the search results
		view_search_results (scene_name, best_pose_result);
	}
	
	// Save results from Hint system
	save_results (scene_name, cluster_index, valid_pose_results, invalid_pose_results, 1);
	
	// Clear global variables for next run
	weighted_sum_utilities_.clear ();
	trajectory_.clear ();
	similar_models_.clear ();
	previous_positions.clear ();
	
	hs_data.next_position = next_position_;
	hs_data.valid_results = true;
	
	return hs_data;
}




















