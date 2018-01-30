#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include "../headers/access_results.h"
#include <pcl/common/transforms.h>
#include "../headers/config_reader.h"
#include "../headers/manipulation.h"
#include "../headers/robot_data_fetcher.h"

typedef pcl::PointNormal Point_N;
typedef pcl::PointXYZRGB Point_RGBA;
typedef pcl::PointCloud<Point_N> Point_Cloud_N;
typedef pcl::PointCloud<Point_RGBA> Point_Cloud_RGBA;

pcl::visualization::PCLVisualizer visu("Visualizer");
typedef pcl::visualization::PointCloudColorHandlerCustom<Point_N> ColorHandler_N;

int cloud_idx = 0;

int
main (int argc, char **argv) {
	Access_Results ar;
	Manipulation m;
	Config_Reader cf;
	cf.background_load_config("../../config.ini");

	Robot_Data_Fetcher fetcher;
	fetcher.set_auth_details(cf.web_service_username, cf.web_service_password);
	fetcher.set_url(cf.hand_web_service_url);

	// Find and import T_CtoH
	Eigen::Matrix4f T_CtoH = ar.get_T_CtoH();
	std::cout << "T_CtoH" << '\n';
	std::cout << T_CtoH << '\n';
	std::cout << "---" << '\n';

	Eigen::Matrix4f T_origin;
	Point_Cloud_RGBA::Ptr merged_cloud (new Point_Cloud_RGBA);

	while(true) {
		std::string orig_cloud_path = cf.save_path + boost::lexical_cast<std::string>(cloud_idx) + ".pcd";
		std::string orig_flag_path = cf.save_path + "." + boost::lexical_cast<std::string>(cloud_idx) + "_flag";
		std::string orig_robotdata_path = cf.save_path + boost::lexical_cast<std::string>(cloud_idx) + ".csv";
		std::cout << "Waiting for cloud at " << orig_cloud_path << '\n';
		while(!boost::filesystem::exists(orig_cloud_path)) {}; // Hang until cloud is found
		while(!boost::filesystem::exists(orig_flag_path)) {}; // Hang until flag is found
		std::cout << "Cloud found!" << '\n';
		std::cout << "Fetching robot data... " << '\n';
		fetcher.fetch_data(orig_robotdata_path);
		while(!boost::filesystem::exists(orig_robotdata_path)) {}; // Hang until robot data is found
		std::cout << "Robot data found!" << '\n';

		Eigen::Matrix4f T = ar.get_robot_data_matrix(orig_robotdata_path);
		if(cloud_idx == 0) {
			T_origin = T;
			boost::filesystem::copy_file(orig_robotdata_path, cf.save_path + "background.csv", boost::filesystem::copy_option::overwrite_if_exists);
		}

		Point_Cloud_RGBA::Ptr cloud (new Point_Cloud_RGBA);
		m.load_cloud_rgba(orig_cloud_path, cloud);

		if(cloud_idx > 0) {
			// Create camera transformations
			Eigen::Matrix4f T_C2toC1 = T_CtoH.inverse() * T_origin.inverse()*T * T_CtoH;
			pcl::transformPointCloud(*cloud, *cloud, T_C2toC1);
		}
		*merged_cloud += *cloud;

		// Save in Segmentation_results
		std::cout << "Saving cloud..." << '\n';
		pcl::io::savePCDFileBinaryCompressed(cf.save_path + "background.pcd", *merged_cloud);
		std::cout << "Done!" << '\n';

		// Visualize
		visu.addPointCloud<Point_RGBA>(merged_cloud, "merged_cloud");
		visu.spin();
		visu.removeAllPointClouds();


		cloud_idx++;
	}

}
