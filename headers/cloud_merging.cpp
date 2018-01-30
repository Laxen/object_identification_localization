#include "cloud_merging.h"

/**
	Merges a vector of clouds into the same coordinate system as the first cloud in the vector, using the corresponding vector of robot data. The result is saved in the latest segmentation data folder. 
	@param clouds Vector of all clouds that should be merged
	@param robot_data Vector of robot data (hand position) for each cloud
*/
void
Cloud_Merging::merge(std::vector<Point_Cloud_RGBA::Ptr> clouds, std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> > robot_data) {
	Access_Results ar;

	if(visualize) {
		std::cout << "Showing clouds before merging..." << '\n';
		for(int i = 0; i < clouds.size(); i++) {
			visu_ptr->addPointCloud<Point_RGBA>(clouds[i], boost::lexical_cast<std::string>(i));
		}
		visu_ptr->spin();
		visu_ptr->removeAllPointClouds();
	}

	// Find and import T_CtoH
	Eigen::Matrix<float,4,4,Eigen::DontAlign> T_CtoH = ar.get_T_CtoH();
	std::cout << "T_CtoH" << '\n';
	std::cout << T_CtoH << '\n';
	std::cout << "---" << '\n';

	// Create camera transformations
	std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> > camera_transformations;
	Eigen::Matrix<float,4,4,Eigen::DontAlign> T_origin = robot_data[0]; // Hand pos for camera that all other scenes will be merged to
	camera_transformations.push_back(Eigen::Matrix<float,4,4,Eigen::DontAlign>::Identity());
	for(int i = 1; i < robot_data.size(); i++) {
		Eigen::Matrix<float,4,4,Eigen::DontAlign> T_next = robot_data[i];
		Eigen::Matrix<float,4,4,Eigen::DontAlign> T = T_CtoH.inverse() * T_origin.inverse()*T_next * T_CtoH;
		camera_transformations.push_back(T);
	}

	Point_Cloud_RGBA::Ptr merged_clouds (new Point_Cloud_RGBA);
	for(int i = 0; i < clouds.size(); i++) {
		pcl::transformPointCloud(*clouds[i], *clouds[i], camera_transformations[i]);
		*merged_clouds += *clouds[i];
	}

	if(visualize) {
		visu_ptr->addPointCloud<Point_RGBA>(merged_clouds, "clouds");
		visu_ptr->spin();
	}

	// Save in Segmentation_results
	std::cout << "Saving cloud..." << '\n';
	boost::filesystem::path latest_seg = ar.path_to_latest_segmentation_results();
	latest_seg += "/merged";
	boost::filesystem::create_directory(latest_seg);
	pcl::io::savePCDFileBinaryCompressed(latest_seg.string() + "/scene_original.pcd", *merged_clouds);
	std::cout << "Done!" << '\n';
}

/**
	Set the visualizer object to be used when visualizing
	@param visu The visualizer pointer
 */
void
Cloud_Merging::set_visualizer(pcl::visualization::PCLVisualizer* visu) {
	visualize = true;
	visu_ptr = visu;
}
