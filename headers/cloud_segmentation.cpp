#include "cloud_segmentation.h"

Cloud_Segmentation::Cloud_Segmentation(Config_Reader conf) {
	LEAF_SIZE = conf.segmentation_leaf_size;

	PLANE_DISTANCE_THRESHOLD = conf.segmentation_plane_distance_threshold;
	PLANE_MAX_ITERATIONS = conf.segmentation_plane_max_iterations;
	BACKGROUND_SEGMENTATION_DISTANCE = conf.segmentation_background_segmentation_distance;

	NORMAL_RADIUS_SEARCH = conf.segmentation_normal_radius_search;

	CLUSTER_TOLERANCE = conf.segmentation_cluster_tolerance;
	CLUSTER_MIN_SIZE = conf.segmentation_cluster_min_size;
	CLUSTER_MAX_SIZE = conf.segmentation_cluster_max_size;

	background_set = false;

	visualize = false;
}

/**
	Visualizes a single cloud once
	@param cloud The cloud to visualize
 */
void
Cloud_Segmentation::visualize_cloud(Point_Cloud_N::Ptr cloud) {
	if(visualize) {
		visu_ptr->addPointCloud(cloud, ColorHandler_N(cloud, 0.0, 255.0, 0.0), "cloud");
		visu_ptr->spin();
		visu_ptr->removeAllPointClouds();
	}
}

/**
	Segments a cloud into clusters
	@param scene_original_rgba The RGBA cloud of the scene that should be segmented
	@param name The name of the cloud (main folder in Segmentation_results)
	@param sub_name The sub name of the cloud (single or merged)
 */
void
Cloud_Segmentation::segment(Point_Cloud_RGBA::Ptr scene_original_rgba, std::string name, std::string sub_name) {
	Point_Cloud_N::Ptr scene (new Point_Cloud_N);
	Point_Cloud_N::Ptr scene_original (new Point_Cloud_N);

	Access_Results ar;
	Manipulation manipulation;

	if(visualize)
		visu_ptr->addCoordinateSystem(0.1);

	pcl::copyPointCloud(*scene_original_rgba, *scene_original);
	pcl::copyPointCloud(*scene_original, *scene);

	// Remove background or plane
	pcl::ModelCoefficients::Ptr plane (new pcl::ModelCoefficients);
	if(background_set) {
		pcl::console::print_info("\t Removing background in scene\n");

		plane = manipulation.find_plane(scene, PLANE_MAX_ITERATIONS, PLANE_DISTANCE_THRESHOLD);

		Eigen::Matrix<float,4,4,Eigen::DontAlign> T_background = T_CtoH.inverse() * cloud_transformation.inverse()*background_transformation * T_CtoH;
		Point_Cloud_N::Ptr background_temp (new Point_Cloud_N);
		pcl::transformPointCloud(*background, *background_temp, T_background);
		visu_ptr->addPointCloud(background_temp, ColorHandler_N(background_temp, 255.0, 0.0, 0.0), "cloud");
		visu_ptr->addPointCloud(scene, ColorHandler_N(scene, 0.0, 255.0, 0.0), "cloud2");
		visu_ptr->spin();
		visu_ptr->removeAllPointClouds();
		scene = manipulation.subtract_clouds(scene, background_temp, BACKGROUND_SEGMENTATION_DISTANCE);
	} else {
		pcl::console::print_info("\t Removing plane in scene\n");
		plane = manipulation.remove_plane(scene, PLANE_MAX_ITERATIONS, PLANE_DISTANCE_THRESHOLD);
	}
	visualize_cloud(scene);

	pcl::console::print_info("\t Downsampling scene\n");
	scene = manipulation.downsample_cloud(scene, LEAF_SIZE);
	visualize_cloud(scene);

	pcl::console::print_info("\t Computing scene normals\n");
	manipulation.compute_normals(scene, scene_original, NORMAL_RADIUS_SEARCH);

	pcl::console::print_info("\t Extracting clusters in scene\n");
	std::vector<Point_Cloud_N::Ptr> clusters;
	clusters = manipulation.extract_clusters(scene, CLUSTER_TOLERANCE, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);
	scene = manipulation.join_clusters(clusters);
	visualize_cloud(scene);

	pcl::console::print_info("\t Removing statistical outliers from each cluster\n");
	for(int i = 0; i < clusters.size(); i++) {
		manipulation.remove_statistical_outliers(clusters[i], 12, 1.0); // std_dev_mul was 0.5 before 2017-11-28
	}
	scene = manipulation.join_clusters(clusters);
	visualize_cloud(scene);

	// Save clouds
	boost::filesystem::path seg_res = ar.path_to_segmentation_results();
	boost::filesystem::path p = seg_res;
	p += "/" + name;
	boost::filesystem::create_directory(p);
	p += "/" + sub_name;
	boost::filesystem::create_directory(p);

	pcl::io::savePCDFileBinaryCompressed(p.string() + "/scene_original.pcd", *scene_original_rgba);
	pcl::io::savePCDFileBinaryCompressed(p.string() + "/scene.pcd", *scene);
	for(int i = 0; i < clusters.size(); i++) {
		pcl::io::savePCDFileBinaryCompressed(p.string() + "/cluster" + boost::lexical_cast<std::string>(i) + ".pcd", *clusters[i]);
	}

	// Save plane
	//if(!background_set) {
		std::ofstream ofs;
		ofs.open((p.string() + "/plane").c_str());
		for(int i = 0; i < plane->values.size(); i++) {
			ofs << plane->values[i] << ",";
		}
		ofs.close();
	//}

	// Update latest_path if this is a single segmentation
	if(sub_name == "single") {
		// Read all data in latest_path and save to latest_path_data, only do this when single cloud
		seg_res += "/latest_path";
		std::ifstream ifs(seg_res.string().c_str());
		std::string latest_path_data;
		std::string line;
		while(std::getline(ifs, line)) {
			latest_path_data += line + "\n";
		}
		ifs.close();

		// Write new latest path and then all previous data
		std::ofstream ofs;
		ofs.open(seg_res.string().c_str(), std::ios::out);
		ofs << name << "\n" << latest_path_data;
		ofs.close();
	}
}

/**
	Set the visualizer object to be used when visualizing
	@param visu The visualizer pointer
 */
void
Cloud_Segmentation::set_visualizer(pcl::visualization::PCLVisualizer* visu) {
	visualize = true;
	visu_ptr = visu;
}

/**
  Sets the background to be subtracted, if no background is set plane removal is used
  @param b The background cloud
  @param trans The transformation for the background
  @param T_CtoH The hand to camera transformation
*/
void
Cloud_Segmentation::set_background_data(Point_Cloud_N::Ptr b, Eigen::Matrix<float,4,4,Eigen::DontAlign> trans, Eigen::Matrix<float,4,4,Eigen::DontAlign> T_CtoH) {	
	background_set = true;
	background = b; 
	background_transformation = trans;
	this->T_CtoH = T_CtoH;
}

/**
	Sets the transformation matrix for the current cloud. This is used when segmenting background.
	This transformation needs to be set every time before segment() is called if using background segmentation.
	@param trans The transformation for the current cloud
*/
void 
Cloud_Segmentation::set_cloud_transformation_matrix(Eigen::Matrix<float,4,4,Eigen::DontAlign> trans) {
	cloud_transformation = trans;
}
