#include "access_results.h"

/**
  Useful commands:

	-path.string () returns a string of the path (e.g. /home/user/...)

	-path.filename () returns the name of the current file (e.g. a path with string "/home/user/object_identification_localization" will
	 return "object_identification_localization" when calling filename () )
*/

/**
  Returns the path to Data in /object_identification_localization
*/
boost::filesystem::path
Access_Results::path_to_data (void)
{
	// Current path
	boost::filesystem::path p(boost::filesystem::current_path());

	// Search parent directories for object_identification_localization
	while (true)
	{
		p = p.parent_path();

		if (p.filename() == "object_identification_localization")
		{
			break;
		}
	}

	// Add path to directory named "Data"
	p /= "data";

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory
		if (!boost::filesystem::create_directory(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory " << p << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}

	return p;
}

/**
  Returns the path to Results in /Data
*/
boost::filesystem::path
Access_Results::path_to_results (void)
{
	// Path to Data
	boost::filesystem::path p = path_to_data ();

	// Add path to pcd_scenes
	p /= "results";

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory
		if (!boost::filesystem::create_directory(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory " << p << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}

	return p;
}

/**
  Returns the path to Hint_system_results in /Results
*/
boost::filesystem::path
Access_Results::path_to_hint_system_results (void)
{
	// Add path to Results
	boost::filesystem::path p = path_to_results ();

	// Add path to Hint_system_results
	p /= "hint_system_results";

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory for hint system results
		if (!boost::filesystem::create_directory(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory " << p << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}

	return p;
}

/**
  Returns the path to Identification_results in /Results
*/
boost::filesystem::path
Access_Results::path_to_identification_results (void)
{
	// Add path to Results
	boost::filesystem::path p = path_to_results ();

	// Add path to Identification_results
	p /= "identification_results";

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory for identification results
		if (!boost::filesystem::create_directory(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory " << p << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}

	return p;
}

/**
  Returns the path to Pose_estimation_results in /Results
*/
boost::filesystem::path
Access_Results::path_to_pose_estimation_results (void)
{
	// Add path to Results
	boost::filesystem::path p = path_to_results ();

	// Add path to Pose_estimation_results
	p /= "pose_estimation_results";

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory for pose estimation results
		if (!boost::filesystem::create_directory(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory " << p << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}

	return p;
}

/**
  Returns the path to Segmentation_results in /Results
*/
boost::filesystem::path
Access_Results::path_to_segmentation_results (void)
{
	// Add path to Results
	boost::filesystem::path p = path_to_results ();

	// Add path to Segmentation_results
	p /= "segmentation_results";

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory
		if (!boost::filesystem::create_directory(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory " << p << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}

	return p;
}

/**
  Returns the path to the latest Segmentation_results in /Results
*/
boost::filesystem::path
Access_Results::path_to_latest_segmentation_results (void) {
	// Add path to Segmentation_results
	boost::filesystem::path seg_path = path_to_segmentation_results ();

	// Read latest path
	boost::filesystem::path p = seg_path;
	p += "/latest_path";

	std::ifstream ifs(p.c_str());
	std::string name;
	std::getline(ifs, name);

	// Add to seg_path and return
	seg_path += "/" + name;
	return seg_path;
}

/**
  Returns the path to Calibration_results in /Results
*/
boost::filesystem::path
Access_Results::path_to_calibration_results (void) {
	// Add path to Results
	boost::filesystem::path p = path_to_results ();

	// Add path to Calibration_results
	p /= "calibration_results";

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory
		if (!boost::filesystem::create_directory(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory " << p << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}

	return p;
}

/**
  Returns the path to single scene in Segmentation_results
*/
boost::filesystem::path
Access_Results::path_to_scene_in_segmentation_results (std::string scene_name)
{
	// Add path to Segmentation_Results
	boost::filesystem::path p = path_to_segmentation_results ();

	// Add path to Pose_estimation_results
	p /= scene_name + "/single";

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory
		if (!boost::filesystem::create_directory(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory " << p << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}

	return p;
}

/**
  Returns the path to scene in /Identification_results
*/
boost::filesystem::path
Access_Results::path_to_scene_in_identification_results (std::string scene_name)
{
	// Add path to Identification_results
	boost::filesystem::path p = path_to_identification_results ();

	// Add path to scene_name
	p /= scene_name;

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory
		if (!boost::filesystem::create_directory(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory " << p << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}

	return p;
}

/**
  Loads the results from identification of cluster: @param cluster_index, in scene: @param scene_name
  @param scene_name The name of the scene
  @param cluster_index The cluster index
  @param models Empty vector. Adds the identified models to the vector where the most likely model is the top one (index = 0)
  @param model_view_indices Empty vector. Adds the identified views to each model where the best view is the top one (index = 0)
  @param model_score Empty vector. Adds the score to the identified views. A good match has low score (~0.05-0.2)
  @param cluster Empty point cloud. Loads the point cloud of the cluster in Segmentation_results
*/
void
Access_Results::load_identification_results (	std::string scene_name,
						int cluster_index,
						std::vector<std::string> &models,
						std::vector<std::vector<int> > &model_view_indices,
						std::vector<std::vector<float> > &model_scores,
						PointCloud_N::Ptr cluster )
{
	// Add path to scene Identification_results
	boost::filesystem::path p = path_to_scene_in_identification_results (scene_name);
	std::stringstream ss;
	ss << "cluster" << cluster_index;
	p /= ss.str ();

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		std::stringstream ss;
		ss << "ERROR: No directory found for cluster" << cluster_index << " for scene " << scene_name << " in Identification_results!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}

	//
	// Load scene cloud from Segmentation_results
	//

	boost::filesystem::path p_cluster = path_to_scene_in_segmentation_results (scene_name);
	ss.str ("");
	ss << p_cluster.string () << "/cluster" << cluster_index << ".pcd";
	if (pcl::io::loadPCDFile<Point_N> (ss.str (), *cluster) < 0 )
	{
		std::stringstream ss;
		ss << "ERROR: Could not load cluster " << cluster_index << " in /Segmentation_results!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}

	//
	// Load data in CSV file
	//

	ss.str ("");
	ss << p.string () << "/ident_res.csv";
	std::ifstream csv_file (ss.str().c_str());

	std::string line;
	while (std::getline (csv_file, line))
	{
		std::stringstream buffer (line);
		std::string value;
		std::vector<std::string> data;

		while(std::getline(buffer, value, ','))
		{
		   data.push_back(value);
		}

		models.push_back (data[0]);
		std::vector<int> view_indices;
		for (int i = 1; i < data.size(); i += 2)
		{
			view_indices.push_back (std::strtol (data[i].c_str(), NULL, 10));
		}

		model_view_indices.push_back (view_indices);

		std::vector<float> view_scores;
		for (int i = 2; i < data.size(); i += 2)
		{
			view_scores.push_back (std::strtof (data[i].c_str(), NULL));
		}

		model_scores.push_back (view_scores);
	}
}

/**
  Loads the results from identification of cluster: @param cluster_index, in scene: @param scene_name, and return an ID_Data object
  @param scene_name The name of the scene
  @param cluster_index The cluster index
	@param id_data The vector of ID_Data objects
*/
void
Access_Results::load_identification_results_id_data_format(std::string scene_name, int cluster_index, std::vector<ID_Data> &id_data) {
	std::vector<std::string> models;
	std::vector<std::vector<int> > model_view_indices;
	std::vector<std::vector<float> > model_scores;
	PointCloud_N::Ptr cluster (new PointCloud_N);
	load_identification_results(scene_name, cluster_index, models, model_view_indices, model_scores, cluster);

	for(int i = 0; i < models.size(); i++) {
		ID_Data idd;
		idd.model_name = models[i];
		idd.view_indices = model_view_indices[i];
		idd.scores = model_scores[i];
		id_data.push_back(idd);
	}
}

/**
  Loads the latest results from identification
  @param cluster_index The index of the cluster
  @param models Empty vector. Adds the identified models to the vector where the most likely model is the top one (index = 0)
  @param model_view_indices Empty vector. Adds the identified views to each model where the best view is the top one (index = 0)
  @param model_score Empty vector. Adds the score to the identified views. A good match has low score (~0.05-0.2)
  @param cluster Empty point cloud. Loads the point cloud of the cluster in Segmentation_results
*/
std::string
Access_Results::load_latest_identification_results (int cluster_index,
							std::vector<std::string> &models,
							std::vector<std::vector<int> > &model_view_indices,
							std::vector<std::vector<float> > &model_scores,
							PointCloud_N::Ptr cluster )
{
	// Get latest scene name
	boost::filesystem::path p = path_to_segmentation_results ();
  	p += "/latest_path";
  	std::ifstream ifs (p.c_str());
  	std::string name;
  	std::getline (ifs, name);

  	// Load latest identification results
  	load_identification_results (name, cluster_index, models, model_view_indices, model_scores, cluster);

  	return name;
}

/**
	Loads the similar models from a given scene
	@param scene_name[in] The name of the scene
	@param cluster_index[in] The index of the cluster
	@param object The best matching object from pose estimation
	@param similar_models[out] Vector containing the similar models
*/
void
Access_Results::load_similar_models (std::string scene_name, int cluster_index, std::string object, std::vector<std::string> &similar_models)
{
	// Add path to scene in Identification_results
	boost::filesystem::path p = path_to_scene_in_identification_results (scene_name);
	std::stringstream ss;
	ss << "cluster" << cluster_index << "/similar_models.csv";
	p /= ss.str ();

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// The file similar_models.csv does not exist. No similar models were detected.
		return;
	}

	std::ifstream csv_file (p.c_str());

	std::string line;
	bool val = false;
	while (std::getline (csv_file, line))
	{
		if (line == object)
		{
			val = true;
		}
		else
		{
			similar_models.push_back (line);
		}
	}

	if (!val)
	{
		similar_models.clear ();
	}
}

/**
	Loads the similar models from the latest identitication results
	@param cluster_index[in] The index of the cluster
	@param object The best matching object from pose estimation
	@param similar_models[out] Vector containing the similar models
	@return The name of the latest scene
*/
std::string
Access_Results::load_similar_models_latest_results (int cluster_index, std::string object, std::vector<std::string> &similar_models)
{
	// Get latest scene name
	boost::filesystem::path p = path_to_segmentation_results ();
  	p += "/latest_path";
  	std::ifstream ifs (p.c_str());
  	std::string name;
  	std::getline (ifs, name);

  	// Load similar models from the latest identification results
  	load_similar_models (name, cluster_index, object, similar_models);

  	return name;
}

/**
	Loads the results from pose estimation and returns a Pose_Data object
	@param scene_name Name of the scene to load pose data from
  @param cluster_index The index of the cluster to load pose data from
	@param pose_data[out] Vector of Pose_Data objects
*/
void
Access_Results::load_pose_estimation_results_pose_format(std::string scene_name, int cluster_index, std::vector<Pose_Data> &pose_data) {
	std::vector<std::string> model_names;
	std::vector<std::vector<int> > view_indices;
	std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> > poses;
	std::vector<double> inlier_fractions;
	std::vector<double> accuracies;
	load_pose_estimation_results(scene_name, cluster_index, model_names, view_indices, poses, inlier_fractions, accuracies);

	for(int i = 0; i < model_names.size(); i++) {
		Pose_Data p_d;
		p_d.model_name = model_names[i];
		p_d.view_indices = view_indices[i];
		p_d.transformation = poses[i];
		p_d.inlier_fraction = inlier_fractions[i];
		p_d.accuracy = accuracies[i];
		pose_data.push_back(p_d);
	}
}

/**
	Loads the results from pose estimation
	@param scene_name Name of the scene to load pose data from
  @param cluster_index The index of the cluster to load pose data from
	@param model_names[out] Vector of each model name
	@param view_indices[out] Vector of vector of view indices. Outer vector is for model name, inner vector is a list of views for that model.
	@param poses[out] Vector of poses as Eigen::Matrix<float,4,4,Eigen::DontAlign> objects
	@param inlier_fractions[out] Vector of inlier fractions
	@param accuracies[out] Vector of accuracies
*/
void
Access_Results::load_pose_estimation_results(std::string scene_name, int cluster_index, std::vector<std::string> &model_names, std::vector<std::vector<int> > &view_indices, std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> > &poses, std::vector<double> &inlier_fractions, std::vector<double> &accuracies) {
	std::string data_path = path_to_pose_estimation_results().string();
	data_path += "/" + scene_name + "/" + boost::lexical_cast<std::string>(cluster_index) + ".csv";

  std::ifstream ifs(data_path.c_str());
  std::string line;

  while(std::getline(ifs, line)) {
    int c = line.find(",");
    std::string model_name = line.substr(0, c);
    line.erase(0, c+1);

		std::vector<int> v_i;
    c = line.find(",");
		std::string view_index_str = line.substr(0, c).c_str();
		while(true) {
			int plus_index = view_index_str.find("+");
			if(plus_index == -1) break;

	    int view_index = std::atoi(view_index_str.substr(0, plus_index).c_str());
			view_index_str.erase(0, plus_index+1);

			v_i.push_back(view_index);
		}
    line.erase(0, c+1);

    Eigen::Matrix<float,4,4,Eigen::DontAlign> pose;
    for(int row = 0; row < 4; row++) {
      for(int col = 0; col < 4; col++) {
        c = line.find(",");
        pose(row,col) = std::atof(line.substr(0, c).c_str());
        line.erase(0, c+1);
      }
    }

    c = line.find(",");
    double inlier_fraction = std::atof(line.substr(0, c).c_str());
    line.erase(0, c+1);

    c = line.find(",");
    double accuracy = std::atof(line.substr(0, c).c_str());
    line.erase(0, c+1);

    model_names.push_back(model_name);
    view_indices.push_back(v_i);
    poses.push_back(pose);
    inlier_fractions.push_back(inlier_fraction);
    accuracies.push_back(accuracy);
  }
}

/**
  Loads segmentation data with certain name specified by scene_name
  @param scene_name Name of the segmentation folder in Data/Results/Segmentation_results
  @param scene_original The original scene
  @param scene The downsampled and clustered scene
  @param clusters Vector of clusters found in the scene
*/
void
Access_Results::load_segmentation_results(std::string scene_name, Access_Results::PointCloud_N::Ptr scene_original, Access_Results::PointCloud_N::Ptr scene, std::vector<Access_Results::PointCloud_N::Ptr> &clusters) {
	boost::filesystem::path p = path_to_segmentation_results();
	p += "/" + scene_name + "/";

	if(pcl::io::loadPCDFile<Point_N>(p.string() + "scene_original.pcd", *scene_original) < 0 ) {
		std::stringstream ss;
		ss << "ERROR: Could not load scene_original in /segmentation_results!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	if(pcl::io::loadPCDFile<Point_N>(p.string() + "scene.pcd", *scene) < 0 ) {
		std::stringstream ss;
		ss << "ERROR: Could not load scene in /segmentation_results!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}

  // Load clusters
  std::ostringstream path;
  int i = 0;
  while(true) {
    path.str("");
    path << p.string() << "cluster" << i << ".pcd";
    if(boost::filesystem::exists(path.str())) {
      PointCloud_N::Ptr cluster (new PointCloud_N);
			if(pcl::io::loadPCDFile<Point_N>(path.str(), *cluster) < 0 ) {
				std::stringstream ss;
				ss << "ERROR: Could not load cluster " << i << " in /segmentation_results!\n\n";
				pcl::console::print_error(ss.str().c_str());
				std::exit (EXIT_FAILURE);
			}
      clusters.push_back(cluster);
    } else {
      break;
    }
    i++;
  }
}

void 
Access_Results::load_all_segmentation_results_original_scenes(std::vector<PointCloudT_color::Ptr>& original_scenes, std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> >& robot_data) {
	// Load all clouds in latest_path
	boost::filesystem::path seg_path = path_to_segmentation_results();
	boost::filesystem::path latest_path = seg_path;
	latest_path += "/latest_path";
	std::ifstream ifs(latest_path.c_str());
	std::string line;
	while(std::getline(ifs, line)) {
		PointCloudT_color::Ptr cloud (new PointCloudT_color);
		if(pcl::io::loadPCDFile<PointT_color>(seg_path.string() + "/" + line + "/single/scene_original.pcd", *cloud) < 0 ) {
			std::stringstream ss;
			ss << "ERROR: Could not load scene_original in /segmentation_results!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
		original_scenes.push_back(cloud);

		std::string robot_data_path = seg_path.string() + "/" + line + "/robot_data.csv";
		Eigen::Matrix<float,4,4,Eigen::DontAlign> data = get_robot_data_matrix(robot_data_path);
		robot_data.push_back(data);

		//data_paths.push_back(seg_path.string() + "/" + line + "/robot_data.csv");
		//clouds.push_back(cloud);
	}
	ifs.close();
}

/**
  Loads the merged cluster in segmentation results
  @param scene_name The name of the scene
  @param cluster_index The cluster index
  @param cluster The cluster point cloud
*/
void
Access_Results::load_segmentation_merged_cluster (std::string scene_name, int cluster_index, PointCloud_N::Ptr cluster)
{
	// Add path to segmentation results
	boost::filesystem::path p = path_to_segmentation_results ();
	std::stringstream ss;
	ss << "/" << scene_name << "/merged/cluster" << cluster_index << ".pcd";
	p += ss.str ();
	
	if (pcl::io::loadPCDFile<Point_N> (p.string (), *cluster) < 0)
	{
		ss.str ("");
		ss << "ERROR: Could not load merged cluster in " << scene_name << " in /segmentation_results!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
}

/**
	Loads the plane from the segmentation results of "scene_name" (note that scene_name needs to specify "single" or "merged" as well!)
	@param scene_name The name of the scene (with single or merged subfolder specified)
	@param plane The ModelCoefficients to store the plane coefficients in
*/
void
Access_Results::load_segmentation_plane(std::string scene_name, pcl::ModelCoefficients &plane) 
{
	boost::filesystem::path p = path_to_segmentation_results();
	p += "/" + scene_name + "/plane";
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		std::stringstream ss;
		ss << "ERROR: No file found named " + p.string() + "\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	
	std::ifstream ifs(p.string().c_str());
	std::string line;
	std::getline(ifs, line);

	for(int i = 0; i < 4; i++) {
		int idx = line.find(",");
		plane.values.push_back(std::atof(line.substr(0, idx).c_str()));
		line.erase(0, idx+1);
	}
}

/**
	Loads the latest single segmentation plane
	@param plane The ModelCoefficients pointer to store the plane coefficients in
*/
void
Access_Results::load_latest_single_segmentation_plane(pcl::ModelCoefficients &plane) 
{
	boost::filesystem::path p = path_to_segmentation_results();
	p += "/latest_path";
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		std::stringstream ss;
		ss << "ERROR: No file found named " << p.string() << "\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	
	std::ifstream ifs(p.c_str());
	std::string name;
	std::getline(ifs, name);

	load_segmentation_plane(name + "/single", plane);
}

/**
	Loads the latest merged segmentation plane
	@param plane The ModelCoefficients pointer to store the plane coefficients in
*/
void
Access_Results::load_latest_merged_segmentation_plane(pcl::ModelCoefficients &plane) 
{
	boost::filesystem::path p = path_to_segmentation_results();
	p += "/latest_path";
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		std::stringstream ss;
		ss << "ERROR: No file found named " << p.string() << "\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	
	std::ifstream ifs(p.c_str());
	std::string name;
	std::getline(ifs, name);

	load_segmentation_plane(name + "/merged", plane);
}

/**
  Loads original scene (single) with certain name specified by scene_name
  @param scene_name Name of the segmentation folder in Data/Results/Segmentation_results
  @param scene_original The original scene
*/
void
Access_Results::load_original_scene_single (std::string scene_name, PointCloudT_color::Ptr scene_original)
{
	// Add path to scene in Segmentation_results
	boost::filesystem::path p = path_to_segmentation_results();
	p += "/" + scene_name + "/single/scene_original.pcd";

	if (pcl::io::loadPCDFile<PointT_color>(p.string (), *scene_original) < 0 )
	{
		std::stringstream ss;
		ss << "ERROR: Could not load original_scene in " << scene_name << " in /segmentation_results!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
}

/**
  Loads original merged scene with certain name specified by scene_name
  @param scene_name Name of the segmentation folder in Data/Results/Segmentation_results
  @param merged_scene_original The original scene
*/
void
Access_Results::load_original_merged_scene (std::string scene_name, PointCloudT_color::Ptr merged_scene_original)
{
	// Add path to scene in Segmentation_results
	boost::filesystem::path p = path_to_segmentation_results();
	p += "/" + scene_name + "/merged/scene_original.pcd";
	//p += "/" + scene_name + "/single/scene_original.pcd";

	if (pcl::io::loadPCDFile<PointT_color>(p.string (), *merged_scene_original) < 0 )
	{
		std::stringstream ss;
		ss << "ERROR: Could not load original_scene in " << scene_name << " in /segmentation_results!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
}

/**
  Loads latest single (non-merged) segmentation data specified by latest_path
  @param scene_original The original scene
  @param scene The downsampled and clustered scene
  @param clusters Vector of clusters found in the scene
	@return The name of the latest path
*/
std::string
Access_Results::load_latest_single_segmentation_results(Access_Results::PointCloud_N::Ptr scene_original, Access_Results::PointCloud_N::Ptr scene, std::vector<Access_Results::PointCloud_N::Ptr> &clusters) {
	boost::filesystem::path p = path_to_segmentation_results();
  p += "/latest_path";
  std::ifstream ifs(p.c_str());
  std::string name;
  std::getline(ifs, name);

  load_segmentation_results(name + "/single", scene_original, scene, clusters);

	return name;
}

/**
  Loads latest merged segmentation data specified by latest_path
  @param scene_original The original scene
  @param scene The downsampled and clustered scene
  @param clusters Vector of clusters found in the scene
	@return The name of the latest path
*/
std::string
Access_Results::load_latest_merged_segmentation_results(Access_Results::PointCloud_N::Ptr scene_original, Access_Results::PointCloud_N::Ptr scene, std::vector<Access_Results::PointCloud_N::Ptr> &clusters) {
	boost::filesystem::path p = path_to_segmentation_results();
  p += "/latest_path";
  std::ifstream ifs(p.c_str());
  std::string name;
  std::getline(ifs, name);

  load_segmentation_results(name + "/merged", scene_original, scene, clusters);

	return name;
}

/**
  Saves the identification results for each cluster in each scene.
  @param scene The scene name
  @param cluster_index The cluster index
  @param model_names The identified model names
  @param scores_vec The view-scores of the identified models
  @param index_queries_vec The view-index of the identified models
*/
void
Access_Results::save_identification_results (	std::string scene,
						int cluster_index,
						std::vector<std::string> model_names,
						std::vector<std::vector<float> > scores_vec,
						std::vector<std::vector<int> > index_queries_vec,
						std::vector<std::string> similar_models )
{
	// Add path to Identification_results in Results
	boost::filesystem::path p = path_to_identification_results ();

	// Add path to scene
	p /= scene;

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory for scene
		if (!boost::filesystem::create_directory(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory for " << scene << " in identification_results!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}

	// Add path to cluster
	std::stringstream ss;
	ss << "cluster" << cluster_index;
	p /= ss.str ();

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory for cluster
		if (!boost::filesystem::create_directory(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory for " + ss.str () + " in " << scene << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}

	ss.str ("");
	ss << p.string () + "/ident_res.csv";

	// Create CSV file
	std::ofstream csv_file;
	csv_file.open (ss.str().c_str());

	// Add content to CSV file
	std::vector<std::string> string_vec;
	for (int i = 0; i < model_names.size(); i++)
	{
		ss.str ("");
		ss << model_names[i] << ",";
		for (int j = 0; j < 10; j++)
		{
			ss << index_queries_vec[i][j] << ",";
			ss << scores_vec[i][j] << ",";
		}

		string_vec.push_back (ss.str ());

		if (i >= 9)
		{
			break;
		}
	}

	for (int i = 0; i < string_vec.size(); i++)
	{
		csv_file << string_vec[i] << "\n";
	}

	csv_file.close();

	if (similar_models.size() != 0)
	{
		// Save similar models to CSV file
		ss.str ("");
		ss << p.string () + "/similar_models.csv";

		// Create CSV file
		std::ofstream csv_file_2;
		csv_file_2.open (ss.str().c_str());

		// Add content to CSV file
		std::vector<std::string> string_vec_2;
		for (int i = 0; i < similar_models.size(); i++)
		{
			string_vec_2.push_back (similar_models[i]);
		}

		for (int i = 0; i < string_vec_2.size(); i++)
		{
			csv_file_2 << string_vec_2[i] << "\n";
		}

		csv_file_2.close();
	}
}

/**
  Comparator used to sort Pose_Data objects
  @param pose1 The first Pose_Data object
  @param pose2 The second Pose_Data object
*/
bool
Access_Results::poses_comparator(const Pose_Data& pose1, const Pose_Data& pose2) 
{
  	return pose1.inlier_fraction > pose2.inlier_fraction;
}

/**
  Loads the valid pose resutls obtained from the hint system
  @param scene_name The name of the scene
  @param cluster_index The cluster index
  @param pose_data[out] Vector of Pose_Data objects
  @return 0 if there was valid pose data for the given scene and cluster and 1 otherwise
*/
int
Access_Results::load_hint_system_valid_poses (std::string scene_name, int cluster_index, std::vector<Pose_Data> &pose_data)
{
	std::vector<std::string> model_names;
	std::vector<std::vector<int> > view_indices;
	std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> > poses;
	std::vector<double> inlier_fractions;
	std::vector<double> accuracies;
	
	// Load valid poses from file
	boost::filesystem::path data_path = path_to_hint_system_results();
	std::stringstream ss;
	ss << scene_name << "/cluster" << cluster_index << "/" << "valid_pose_results.csv";
	data_path /= ss.str();
	
	// Check if there exists any valid pose data
	if (!boost::filesystem::exists(data_path))
	{
		return 1;
	}
	
	std::ifstream ifs(data_path.string().c_str());
	std::string line;

	while(std::getline(ifs, line)) 
	{
	
		std::string model_name;
		std::vector<int> view_indices;
		Eigen::Matrix<float,4,4,Eigen::DontAlign> transformation;
		double inlier_fraction;
		double accuracy;
		
		int c = line.find(",");
		model_name = line.substr(0, c);
		line.erase(0, c+1);

		std::vector<int> v_i;
		c = line.find(",");
		std::string view_index_str = line.substr(0, c).c_str();
		while(true) 
		{
			int plus_index = view_index_str.find("+");
			if(plus_index == -1) 
			{
				break;
			}
			
			int view_index = std::atoi(view_index_str.substr(0, plus_index).c_str());
			view_index_str.erase(0, plus_index+1);

			view_indices.push_back(view_index);
		}
		
		line.erase(0, c+1);

		for(int row = 0; row < 4; row++) 
		{
	  		for(int col = 0; col < 4; col++) 
	  		{
				c = line.find(",");
				transformation(row,col) = std::atof(line.substr(0, c).c_str());
				line.erase(0, c+1);
	  		}
		}

		c = line.find(",");
		inlier_fraction = std::atof(line.substr(0, c).c_str());
		line.erase(0, c+1);

		c = line.find(",");
		accuracy = std::atof(line.substr(0, c).c_str());
		line.erase(0, c+1);
		
		Pose_Data p_d;
		p_d.model_name = model_name;
		p_d.view_indices = view_indices;
		p_d.transformation = transformation;
		p_d.inlier_fraction = inlier_fraction;
		p_d.accuracy = accuracy;
		pose_data.push_back(p_d);
	}
	
	// Sort poses list after inlier fractions
	std::sort(pose_data.begin(), pose_data.end(), poses_comparator);
	
	return 0;
}

/**
  Loads the previous camera positions
  @param scene The name of the scene
  @return The transformation of the camera position
*/
Eigen::Matrix<float,4,4,Eigen::DontAlign>
Access_Results::load_previous_camera_position (std::string scene)
{
	// Add path to Hint_system_results in Results
	boost::filesystem::path p = path_to_hint_system_results ();

	// Add path to scene
	p /= scene;

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		std::stringstream ss;
		ss << "ERROR: Could not find directory for " << scene << " in hint_system_results!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	
	//
	// Load CSV-file
	//
	
	// Add path to CSV-file
	p /= "current_camera_position.csv";
	
	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		std::stringstream ss;
		ss << "ERROR: Could not find file: " << p.string () << "\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	
	// Load transformation
	std::ifstream csv_file (p.c_str());
	std::string line;
	int c;
	Eigen::Matrix<float,4,4,Eigen::DontAlign> pose;
	while (std::getline (csv_file, line))
	{
		for(int row = 0; row < 4; row++) 
		{
			for(int col = 0; col < 4; col++) 
			{
				c = line.find(",");
				pose (row,col) = std::atof (line.substr(0, c).c_str());
				line.erase(0, c+1);
			}
		}
	}
	
	return pose;
}

/**
  Save hint system results
  @param scene The name of the scene
  @param cluster_index The cluster index
  @param valid_pose_results Vector containing each line of the CSV-file for the valid pose results
  @param invalid_pose_results Vector containing each line of the CSV-file for the invalid pose results
  @param weighted_sum_utilities Vector containing each line of the CSV-file for the weighted, summed utilities
  @param trajectory Vector containing each line of the CSV-file for the trajectory
  @param next_position Vector containing each line of the CSV-file for the new position of the robot arm
*/
void
Access_Results::save_hint_system_results (	std::string scene,
						int cluster_index,
						std::vector<std::string> valid_pose_results,
						std::vector<std::string> invalid_pose_results,
						std::vector<std::string> weighted_sum_utilities,
						std::vector<std::string> trajectory,
						std::vector<std::string> next_position )
{
	// Add path to Hint_system_results in Results
	boost::filesystem::path p = path_to_hint_system_results ();

	// Add path to scene
	p /= scene;

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory for scene
		if (!boost::filesystem::create_directory(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory for " << scene << " in hint_system_results!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}

	// Add path to cluster
	std::stringstream ss;
	ss << "cluster" << cluster_index;
	p /= ss.str ();

	// Check if such path exists
	if (!boost::filesystem::exists(p))
	{
		// Path does not exist, create directory for cluster
		if (!boost::filesystem::create_directory(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory for " + ss.str () + " in " << scene << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}

	if (valid_pose_results.size() == 0)
	{
		// Save invalid pose results
		save_csv_file (p.string () + "/invalid_pose_results.csv", invalid_pose_results);
	}
	else
	{
		// Save valid pose results
		save_csv_file (p.string () + "/valid_pose_results.csv", valid_pose_results);

		if (invalid_pose_results.size() != 0)
		{
			// Save invalid pose results
			save_csv_file (p.string () + "/invalid_pose_results.csv", invalid_pose_results);
		}

		// Save weighted sum utilities
		save_csv_file (p.string () + "/weighted_sum_utilities.csv", weighted_sum_utilities);

		// Save trajectory
		save_csv_file (p.string () + "/trajectory.csv", trajectory);

		// Save new position
		save_csv_file (p.string () + "/next_position.csv", next_position);
	}
}

/**
  Save file as CSV
  @param file_name The name of the file
  @param str_vec Vector containing each line in the CSV-file
*/
void
Access_Results::save_csv_file (std::string file_name, std::vector<std::string> str_vec)
{
	std::ofstream csv_file;
	csv_file.open (file_name.c_str());

	for (int i = 0; i < str_vec.size(); i++)
	{
		csv_file << str_vec[i] << "\n";
	}

	csv_file.close();
}

/**
  Loads robot data from file and formats it into a Eigen::Matrix4f transformation
  @param path The path to the data file (CSV)
  @return The Eigen::Matrix4f transformation
 */
Eigen::Matrix<float,4,4,Eigen::DontAlign> 
Access_Results::get_robot_data_matrix(std::string path) {
	// Load CSV data and save in vector
	std::ifstream ifs (path.c_str());
	std::string line;
	std::vector<double> elements;
	std::getline(ifs, line);

	while(true) {
		int comma_idx = line.find(",");
		if(comma_idx == -1) break;

		double element = std::atof(line.substr(0, comma_idx).c_str());
		line.erase(0, comma_idx+1);

		elements.push_back(element);
	}

	// Quaternion loaded as x, y, z, w
	double w = elements[3];
	double x = elements[4];
	double y = elements[5];
	double z = elements[6];

	Eigen::Matrix3f R;
	R << 1-2*(pow(y,2) + pow(z,2)), 2*(x*y+w*z), 2*(x*z-w*y),
	  2*(x*y-w*z), 1-2*(pow(x,2)+pow(z,2)), 2*(y*z+w*x),
	  2*(x*z+w*y), 2*(y*z-w*x), 1-2*(pow(x,2)+pow(y,2));

	Eigen::Matrix<float,4,4,Eigen::DontAlign> T_HtoB;
	T_HtoB.block(0,0,3,3) = R.transpose().eval(); // Should be transposed because the equations for R should also be transposed
	T_HtoB.row(3) << 0.0,0.0,0.0,1.0;
	T_HtoB.col(3) << elements[0]/1000.0, elements[1]/1000.0, elements[2]/1000.0, 1.0;

	return T_HtoB;
}

/**
  Loads T_CtoH from CSV file and saves as Eigen::Matrix<float,4,4,Eigen::DontAlign>
  @return The T_CtoH transformation
 */
Eigen::Matrix<float,4,4,Eigen::DontAlign> 
Access_Results::get_T_CtoH() {
	Eigen::Matrix<float,4,4,Eigen::DontAlign> T_CtoH;

	boost::filesystem::path p = path_to_calibration_results();
	p += "/T_CtoH";
	if(!boost::filesystem::exists(p)) {
		std::stringstream ss;
		ss << "ERROR: Could not find calibration file at " << p << "!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}

	std::ifstream infile(p.string().c_str());
	std::string line;
	int row = 0;
	while(std::getline(infile, line)) {
		std::vector<double> elements;
		while(line.find(",") != -1) {
			int comma_idx = line.find(",");
			double element = std::atof(line.substr(0, comma_idx).c_str());
			elements.push_back(element);
			line.erase(0, comma_idx+1);
		}

		T_CtoH.row(row) << elements[0], elements[1], elements[2], elements[3];
		row++;
	}

	return T_CtoH.transpose().eval(); // MATLAB has everything the other way around, transpose before returning!
}

/**
  Saves the current camera position
  @param scene_name The name of the scene
  @param T_HtoB The transformation from robot hand to robot base
  @param T_CtoH The transformation from camera to robot hand
*/
void 
Access_Results::save_current_camera_position (	std::string scene_name, 
												Eigen::Matrix<float,4,4,Eigen::DontAlign> T_HtoB,
												Eigen::Matrix<float,4,4,Eigen::DontAlign> T_CtoH )
{
	// Compute current camera posisiton
	Eigen::Matrix<float,4,4,Eigen::DontAlign> current_position = T_HtoB * T_CtoH;

	// Add content of the current camera position
	std::stringstream ss;
	for (int row = 0; row < 4; row++)
	{
		for (int col = 0; col < 4; col++)
		{
			ss << current_position (row,col) << ",";
		}
	}
	
	// Add path to Hint_system_results in Results
	boost::filesystem::path p = path_to_hint_system_results ();

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
	
	// Save current camera position
	p /= "current_camera_position.csv";
	std::ofstream csv_file;
	csv_file.open (p.string().c_str());
	csv_file << ss.str ();
	csv_file.close();
}

/**
  Clears all results 
*/
void
Access_Results::clear_results ()
{
	// Get path to hint_system_results, identification_results, pose_estimation_results and segmentation_results
	Access_Results ar;
	boost::filesystem::path p_segmentation_results = ar.path_to_segmentation_results ();
	boost::filesystem::path p_identification_results = ar.path_to_identification_results ();
	boost::filesystem::path p_pose_estimation_results = ar.path_to_pose_estimation_results ();
	boost::filesystem::path p_hint_system_results = ar.path_to_hint_system_results ();
	
	if (boost::filesystem::exists(p_segmentation_results))
	{
		// Remove directory and all its contents
		boost::filesystem::remove_all (p_segmentation_results);
		
		// Create empty directory
		if (!boost::filesystem::create_directory(p_segmentation_results))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory " << p_segmentation_results << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}	
	}
	
	if (boost::filesystem::exists(p_identification_results))
	{
		boost::filesystem::remove_all (p_identification_results);
		
		// Remove directory and all its contents
		boost::filesystem::remove_all (p_identification_results);
		
		// Create empty directory
		if (!boost::filesystem::create_directory(p_identification_results))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory " << p_identification_results << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}
	
	if (boost::filesystem::exists(p_pose_estimation_results))
	{
		boost::filesystem::remove_all (p_pose_estimation_results);
		
		// Remove directory and all its contents
		boost::filesystem::remove_all (p_pose_estimation_results);
		
		// Create empty directory
		if (!boost::filesystem::create_directory(p_pose_estimation_results))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory " << p_pose_estimation_results << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}
	
	if (boost::filesystem::exists(p_hint_system_results))
	{
		boost::filesystem::remove_all (p_hint_system_results);
		
		// Remove directory and all its contents
		boost::filesystem::remove_all (p_hint_system_results);
		
		// Create empty directory
		if (!boost::filesystem::create_directory(p_hint_system_results))
		{
			std::stringstream ss;
			ss << "ERROR: Could not create directory " << p_hint_system_results << "!\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
	}
}































