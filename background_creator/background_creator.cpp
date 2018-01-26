#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include "../../Headers/Access_Results.h"
#include <pcl/common/transforms.h>

typedef pcl::PointNormal Point_N;
typedef pcl::PointXYZRGB Point_RGBA;
typedef pcl::PointCloud<Point_N> Point_Cloud_N;
typedef pcl::PointCloud<Point_RGBA> Point_Cloud_RGBA;

pcl::visualization::PCLVisualizer visu("Visualizer");
typedef pcl::visualization::PointCloudColorHandlerCustom<Point_N> ColorHandler_N;

//std::string share_path = "./simulated_share/";
std::string share_path = "/home/robot/pointclouds/";
int cloud_idx = 0;

/**
  Loads a point cloud with RGB data
  @param path The path to the cloud
  @param cloud The cloud pointer to use for storing the cloud
  @return 0 if error, 1 if successful
 */
int 
load_cloud_rgba(std::string path, Point_Cloud_RGBA::Ptr cloud) {
	std::cout << "Loading cloud \"" << path << "\"\n" << std::endl;

	int load_successful = pcl::io::loadPCDFile<Point_RGBA>(path, *cloud);
	if(load_successful < 0) {
		pcl::console::print_error("Cloud could not be loaded!\n");
		return 0;
	}
	else {
		pcl::console::print_info("Cloud successfully loaded!\n");
		return 1;
	}
}

/**
  Loads robot data from file and formats it into a Eigen::Matrix<float,4,4,Eigen::DontAlign> transformation
  @param path The path to the data file (CSV)
  @return The Eigen::Matrix<float,4,4,Eigen::DontAlign> transformation
 */
Eigen::Matrix<float,4,4,Eigen::DontAlign> 
get_robot_data_matrix_quaternion(std::string path) {
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

int
main (int argc, char **argv) {
	Access_Results ar;

	// Find and import T_CtoH
	Eigen::Matrix4f T_CtoH = ar.get_T_CtoH();
	std::cout << "T_CtoH" << '\n';
	std::cout << T_CtoH << '\n';
	std::cout << "---" << '\n';

	Eigen::Matrix4f T_origin;
	Point_Cloud_RGBA::Ptr merged_cloud (new Point_Cloud_RGBA);

	while(true) {
		std::string orig_cloud_path = share_path + boost::lexical_cast<std::string>(cloud_idx) + ".pcd";
		std::string orig_robotdata_path = share_path + boost::lexical_cast<std::string>(cloud_idx) + ".csv";
		std::cout << "Waiting for cloud at " << orig_cloud_path << '\n';
		while(!boost::filesystem::exists(orig_cloud_path)) {}; // Hang until cloud is found
		std::cout << "Cloud found!" << '\n';
		std::cout << "Waiting for robot data at " << orig_robotdata_path << '\n';
		while(!boost::filesystem::exists(orig_robotdata_path)) {}; // Hang until robot data is found
		std::cout << "Robot data found!" << '\n';

		Eigen::Matrix4f T = ar.get_robot_data_matrix(orig_robotdata_path);
		if(cloud_idx == 0) {
			T_origin = T;
			boost::filesystem::copy_file(orig_robotdata_path, "background.csv");
		}

		Point_Cloud_RGBA::Ptr cloud (new Point_Cloud_RGBA);
		load_cloud_rgba(orig_cloud_path, cloud);

		if(cloud_idx > 0) {
			// Create camera transformations
			Eigen::Matrix4f T_C2toC1 = T_CtoH.inverse() * T_origin.inverse()*T * T_CtoH;
			pcl::transformPointCloud(*cloud, *cloud, T_C2toC1);
		}
		*merged_cloud += *cloud;

		// Save in Segmentation_results
		std::cout << "Saving cloud..." << '\n';
		pcl::io::savePCDFileBinaryCompressed("background.pcd", *merged_cloud);
		std::cout << "Done!" << '\n';

		// Visualize
		visu.addPointCloud<Point_RGBA>(merged_cloud, "merged_cloud");
		visu.spin();
		visu.removeAllPointClouds();


		cloud_idx++;
	}

}
