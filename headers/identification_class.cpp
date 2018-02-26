
#include "identification_class.h"

/**
  Loads all model data  
*/
void
Identification_Class::load_model_data ()
{
	Access_Model_Data access;
	
	// Get all models
	std::vector<std::string> model_names = access.get_model_names ();
	
	// Load all model data for each model
	for (int i = 0; i < model_names.size(); i++)
	{
		model_data m;
		m.name = model_names[i];
		
		// Load complete model
		PointCloudT::Ptr model (new PointCloudT);
		access.load_complete_model (model_names[i], model);
		m.complete_model = model;
		
		// Load global features
		FeatureCloudT::Ptr features (new FeatureCloudT);
		access.load_global_features (model_names[i], features);
		m.feature_cloud = features;
		
		// Load model graph
		View_Graph g;
		g.load_graph (model_names[i]);
		m.graph = g;
		
		models.push_back (m);
	}
}

/**
  Constructor
*/
Identification_Class::Identification_Class (void)
{
	show_res_ = false;
	save_identification_results_ = true;
	VERY_GOOD_MATCH_ = 0.1f;
	GOOD_MATCH_ = 0.15f;
	MEDIUM_MATCH_ = 0.2f;
	SIMILAR_MATCH_ = 0.1f;
	
	// Load model data 
	std::cout << "Loading model data for identification..." << std::endl;
	load_model_data ();
	std::cout << "Done\n" << std::endl;
	if(models.size() == 0) {
		std::stringstream ss;
		ss << "ERROR: No models found!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	std::cout << "Models loaded: " << models.size() << "\n" << std::endl;
}

/**
  Set if program should show identification results
  @param show_res True if show results, false otherwise
*/
void
Identification_Class::show_results (bool show_res)
{
	show_res_ = true;
}

/**
  Set if program should save identification results
  @param save_identification_results True if save resutls, false otherwise
*/
void 
Identification_Class::set_save_results (bool save_identification_results)
{
	save_identification_results_ = save_identification_results;
}	

/**
  Estimates a ESF feature for a point cloud cluster
  @param cluster The cluster object containing the point cloud
*/
void 
Identification_Class::estimate_feature (cluster_data &cluster)
{	
	// Estimate global ESF feature for cluster
	FeatureEstimationT feature_estimator;
	FeatureCloudT::Ptr feature (new FeatureCloudT ());
	feature_estimator.setInputCloud (cluster.cloud);
	feature_estimator.compute (*feature);
	
	cluster.feature = feature->points[0];
}

/**
  Computes the l1-norm of the difference between two histograms
  @param f1 The first histogram
  @param f2 the second histogram 
*/
float 
Identification_Class::l1_norm (FeatureT f1, FeatureT f2)
{
	float sum = 0.0f;
	for (int i = 0; i < 640; i++)
	{
		sum += std::abs (f1.histogram[i] - f2.histogram[i]);
	}
	return sum;
}

/**
  Finds the best matching model features for a cluster
  @param cluster The cluster object containing the point cloud
*/
void 
Identification_Class::best_matching_features (cluster_data &cluster)
{	
	for (int i = 0; i < models.size(); i++)
	{
		// L1 norm
		std::vector<pcl::Correspondence> model_cluster_corrs;
		for (int j = 0; j < models[i].feature_cloud->points.size(); j++)
		{
			float diff = l1_norm (cluster.feature, models[i].feature_cloud->points[j]);
			pcl::Correspondence corr (j, static_cast<int> (1), diff); 
			model_cluster_corrs.push_back (corr); 
		}
		
		// Sort model_cluster_corrs in ascending order with respect to the feature distance
		std::sort(model_cluster_corrs.begin(), model_cluster_corrs.end(), less_than_key());
		
		models[i].correspondences = model_cluster_corrs;
	}
	
	// Sort models in ascending order with respect to the best matching feature
	std::sort(models.begin(), models.end());
	
	// Check if there are any similar models
	int index = 1;
	std::vector<std::string> sm;
	sm.push_back (models[0].name);
	while (std::abs (models[0].correspondences[0].distance - models[index].correspondences[0].distance) < SIMILAR_MATCH_)
	{
		sm.push_back (models[index].name);
		index++;
		if (index >= models.size())
		{
			break;
		}
	}
	cluster.similar_models = sm;
}

/**
  Prints the identification results to terminal
  @param cluster The cluster object containing the point cloud
*/
void 
Identification_Class::results_console (cluster_data cluster)
{
	for (int i = models.size()-1; i >= 0 ; i--)
	{
		std::stringstream ss;
		std::cout << "Model: " << models[i].name;
			
		if (models[i].correspondences[0].distance < VERY_GOOD_MATCH_)
		{
			ss << "  VERY GOOD MATCH";
			pcl::console::print_value (ss.str().c_str());
			std::cout << "\t\nBest 10 matching views:\n" << std::endl;
		} 
		else if (models[i].correspondences[0].distance < GOOD_MATCH_)
		{
			ss << "  GOOD MATCH";
			pcl::console::print_value (ss.str().c_str());
			std::cout << "\t\nBest 10 matching views:\n" << std::endl;
		} 
		else if (models[i].correspondences[0].distance < MEDIUM_MATCH_)
		{
			ss << "  MEDIUM MATCH";
			pcl::console::print_value (ss.str().c_str());
			std::cout << "\t\nBest 10 matching views:\n" << std::endl;
		}
		else
		{
			ss << "  BAD MATCH";
			pcl::console::print_value (ss.str().c_str());
			std::cout << "\t\nBest 10 matching views:\n" << std::endl;
		}
		
		for (int j = 0; j < 10; j++)
		{
			printf ("\tView index: %d, feature distance: %10.10f\n", models[i].correspondences[j].index_query, models[i].correspondences[j].distance);
		}
		std::cout << "\n" << std::endl;
	}
	
	// Highlight the identified model
	std::stringstream ss;
	ss << "\nIDENTIFIED MODEL: " << models[0].name << "\n\n\n";
	pcl::console::print_value (ss.str().c_str());
	
	if (cluster.similar_models.size() > 1)
	{
		// Print out information about similar models
		std::cout << "Objects with similar matches:\n" << std::endl;
		for (int i = 0; i < cluster.similar_models.size(); i++)
		{
			std::cout << "\t" << cluster.similar_models[i] << std::endl;
		}
		std::cout << "\n\nPress Q to continue\n\n" << std::endl;
	}
}

/**
  Shows the identification results
  @param cluster The cluster object containing the point cloud
*/
void 
Identification_Class::results_viewer (cluster_data cluster)
{
	//
	// Create a viewer with graph information
	//
	
	Access_Model_Data access;
	
	// Add graph
	pcl::visualization::PCLVisualizer viewer1 ("Viewer1");
	std::vector<float> r_vec (models[0].graph.get_size (), 1.0);
	std::vector<float> g_vec (models[0].graph.get_size (), 1.0);
	std::vector<float> b_vec (models[0].graph.get_size (), 1.0);
	r_vec[models[0].correspondences[0].index_query] = 0.0;
	g_vec[models[0].correspondences[0].index_query] = 1.0;
	b_vec[models[0].correspondences[0].index_query] = 0.0;
	models[0].graph.add_graph_to_viewer (viewer1, 0.01, r_vec, g_vec, b_vec, 0, false);
	
	// Add complete_model
	viewer1.addPointCloud<PointT> (models[0].complete_model);
	viewer1.spin ();
	
	//
	// Create a second viewer with view point cloud information
	//

	std::vector<PointCloud_N::Ptr> views;
	std::vector<int> indices;
	indices.push_back (models[0].correspondences[0].index_query);
	access.load_model_views (models[0].name, views, indices);
	
	pcl::visualization::PCLVisualizer viewer2 ("Viewer2");
	int vp_1;
	int vp_2;
	viewer2.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	viewer2.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*cluster.cloud, centroid);
	PointCloud_N::Ptr cluster_demean (new PointCloud_N);
	pcl::demeanPointCloud<Point_N> (*cluster.cloud, centroid, *cluster_demean);
	viewer2.addPointCloud<Point_N> (cluster_demean, "view_cluster", vp_1);
	viewer2.addPointCloud<Point_N> (views[0], "view_identified", vp_2);
	viewer2.addText ("Scene", 10, 10, 18, 1.0, 1.0, 1.0, "text1", vp_1);
	viewer2.addText ("Identified view", 10, 10, 18, 1.0, 1.0, 1.0, "text2", vp_2);
	viewer2.addCoordinateSystem (0.1, "co1", vp_1);
	viewer2.addCoordinateSystem (0.1, "co2", vp_2);
	
	viewer2.spin ();
}

/**
  Saves the identification results to file
  @param cluster The cluster object containing the point cloud
*/
void
Identification_Class::save_results (cluster_data cluster)
{
	// Save identification results to CSV file. Only the 10 best models are saved along with their 10 best views and score (feature distance)
	Access_Results access;
	std::vector<std::string> model_names;
	std::vector<std::vector<float> > scores_vec;
	std::vector<std::vector<int> > index_queries_vec;
	for (int i = 0; i < models.size(); i++)
	{
		model_names.push_back (models[i].name);
		
		std::vector<float> scores;
		std::vector<int> index_queries;
		for (int j = 0; j < 10; j++)
		{
			scores.push_back (models[i].correspondences[j].distance);
			index_queries.push_back (models[i].correspondences[j].index_query);
		}
		scores_vec.push_back (scores);
		index_queries_vec.push_back (index_queries);
	}
	
	access.save_identification_results (cluster.scene, cluster.index, model_names, scores_vec, index_queries_vec, cluster.similar_models);
}

/**
  Identifies an unknown cluster 
  @param scene_name The name of the scene
  @param cluster_index The cluster index
  @param cluster_cloud The point cloud of the unknown cluster
*/
void
Identification_Class::identify (std::string scene_name, int cluster_index, PointCloud_N::Ptr cluster_cloud)
{			
	std::cout << "Identifying scene: " << scene_name << ", cluster: " << cluster_index << "\n" << std::endl;
	
	cluster_data cluster;
	cluster.scene = scene_name;
	cluster.index = cluster_index;
	cluster.cloud = cluster_cloud;
	
	// Estimate feature for cluster
	estimate_feature (cluster);
	
	// Search for best matching features in the feature clouds for each model
	best_matching_features (cluster);
	
	if (save_identification_results_)
	{
		// Save identification results 
		std::cout << "Saving results..." << std::endl;
		save_results (cluster);
		std::cout << "Done\n" << std::endl;
	}
	
	std::cout << "\nIdentification of scene: " << scene_name << ", cluster: " << cluster_index << " complete!\n" << std::endl;
	
	if (show_res_)
	{
		// Output results from the feature matching to console
		results_console (cluster);
		
		// View results graphically
		results_viewer (cluster);	
	}
}






















