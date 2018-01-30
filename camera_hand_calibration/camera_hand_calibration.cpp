#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ppf_registration.h>
#include "../headers/access_results.h"

typedef pcl::PointNormal Point_N;
typedef pcl::PointXYZRGB Point_RGBA;
typedef pcl::PointCloud<Point_N> Point_Cloud_N;
typedef pcl::PointCloud<Point_RGBA> Point_Cloud_RGBA;

typedef pcl::visualization::PointCloudColorHandlerCustom<Point_N> ColorHandler_N;

pcl::visualization::PCLVisualizer visu("Visu");
std::vector<std::vector<Point_N> > points;

int viewport_index = 0;
std::vector<int> viewports;

/**
  Loads a point cloud
  @param path The path to the cloud
  @param cloud The cloud pointer to use for storing the cloud
  @return 0 if error, 1 if successful
 */
int load_cloud(std::string path, Point_Cloud_RGBA::Ptr cloud) {
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

// Callback function for registerPointPickingCallback
void pp_callback(const pcl::visualization::PointPickingEvent& event, void* visu_number_void)
{
	if(event.getPointIndex() != -1)
	{
		float x,y,z;
		event.getPoint(x,y,z);
		Point_N point;
		point.x = x;
		point.y = y;
		point.z = z;

		std::ostringstream oss;
		oss << "sphere_" << event.getPointIndex();
		visu.addSphere(point, 0.004, 255, 0, 0, oss.str(), viewports[viewport_index]);
		visu.updateCamera();

		std::string point_ok;
		std::cout << "\nIs point OK? (y/n)" << '\n';
		std::cin >> point_ok;
		if(point_ok == "n") {
			visu.removeShape(oss.str());
		} else {
			points[viewport_index].push_back(point);
			viewport_index++;
			if(viewport_index >= viewports.size())
				viewport_index = 0;
		}
	}
}

std::string point_vector_to_string(std::vector<Point_N> P) {
	std::ostringstream P_str;
	for(int i = 0; i < P.size(); i++) {
		Point_N p = P[i];
		P_str << p.x << " " << p.y << " " << p.z << " ";
	}
	return P_str.str();
}

/**
  Loads robot data from file and saves it as a string where each element is separated by a space. This works for both Euler and quaternion format.
  @param path The path to the robot data CSV file
 */
std::string get_robot_data_string(std::string path) {
	std::ifstream ifs (path.c_str());
	std::string line;
	std::ostringstream data_string;
	while(std::getline(ifs, line)) {
		while(line.find(",") != -1) {
			int comma_idx = line.find(",");
			double element = std::atof(line.substr(0, comma_idx).c_str());
			line.erase(0, comma_idx+1);
			data_string << element << " ";
		}
	}
	return data_string.str();
}

/**
  Loads robot data from file (quaternion format) and formats it into a Eigen::Matrix4f transformation
  @param path The path to the data file (quaternion CSV)
  @return The Eigen::Matrix4f transformation
 */
Eigen::Matrix4f
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

	// Quaternion loaded as w, x, y, z
	double w = elements[3];
	double x = elements[4];
	double y = elements[5];
	double z = elements[6];

	Eigen::Matrix3f R;
	R << 1-2*(pow(y,2) + pow(z,2)), 2*(x*y+w*z), 2*(x*z-w*y),
	  2*(x*y-w*z), 1-2*(pow(x,2)+pow(z,2)), 2*(y*z+w*x),
	  2*(x*z+w*y), 2*(y*z-w*x), 1-2*(pow(x,2)+pow(y,2));

	Eigen::Matrix4f T_HtoB;
	T_HtoB.block(0,0,3,3) = R.transpose().eval(); // This needs to be transposed, but I'm unsure why
	T_HtoB.row(3) << 0,0,0,1;
	T_HtoB.col(3) << elements[0]/1000.0, elements[1]/1000.0, elements[2]/1000.0, 1;

	std::cout << T_HtoB << std::endl;

	return T_HtoB;
}

/**
  Loads robot data from file (Euler format) and formats it into a Eigen::Matrix4f transformation
  @param path The path to the data file (Euler CSV)
  @return The Eigen::Matrix4f transformation
 */
Eigen::Matrix4f
get_robot_data_matrix(std::string path) {
	// Load CSV data and save in vector
	std::ifstream ifs (path.c_str());
	std::string line;
	std::vector<std::vector<double> > elements;
	for(int i = 0; i < 2; i++) { // 2 lines in each data file
		std::getline(ifs, line);
		std::vector<double> row;
		while(line.find(",") != -1) {
			int comma_idx = line.find(",");
			double element = std::atof(line.substr(0, comma_idx).c_str());
			line.erase(0, comma_idx+1);

			row.push_back(element);
		}
		elements.push_back(row);
	}

	// Convert rotations from degrees to radians
	std::vector<double> row = elements[1];
	for(int n = 0; n < row.size(); n++) {
		double element = row[n];
		row[n] = element * M_PI / 180.0;
	}
	elements[1] = row;

	// Build T_HtoB
	Eigen::Matrix3f RZ, RY, RX;
	RZ << cos(elements[1][2]), -sin(elements[1][2]), 0,
	   sin(elements[1][2]), cos(elements[1][2]), 0,
	   0, 0, 1;
	RY << cos(elements[1][1]), 0, sin(elements[1][1]),
	   0, 1, 0,
	   -sin(elements[1][1]), 0, cos(elements[1][1]);
	RX << 1, 0, 0,
	   0, cos(elements[1][0]), -sin(elements[1][0]),
	   0, sin(elements[1][0]), cos(elements[1][0]);
	Eigen::Matrix3f R_HtoB = RZ*RY*RX;
	Eigen::Matrix4f T_HtoB;
	T_HtoB.block(0,0,3,3) = R_HtoB;
	T_HtoB.row(3) << 0,0,0,1;
	T_HtoB.col(3) << elements[0][0], elements[0][1], elements[0][2], 1;

	return T_HtoB;
}

void
save_points() {
	for(int i = 0; i < points.size(); i++) {
		std::ofstream ofs;
		std::ostringstream path;
		path << "P" << i << ".csv";
		ofs.open(path.str().c_str());

		for(int n = 0; n < points[i].size(); n++) {
			Point_N p = points[i][n];

			ofs << p.x << "," << p.y << "," << p.z << ",\n";
		}

		ofs.close();
	}
}

int main(int argc, char** argv) {
	Access_Results ar;

	// Load clouds
	std::vector<Point_Cloud_RGBA::Ptr> clouds;
	std::vector<std::string> paths;
	int load_data_index = -1;
	for(int i = 1; i < argc; i++) {
		if(strcmp(argv[i], "-l") == 0) {
			load_data_index = i+1;
			break;
		} else {
			// Load cloud data
			Point_Cloud_RGBA::Ptr cloud (new Point_Cloud_RGBA);
			load_cloud(argv[i], cloud);
			clouds.push_back(cloud);
			paths.push_back(argv[i]);

			std::vector<Point_N> P;
			points.push_back(P);

			viewports.push_back(i);
		}
	}

	// Create viewports
	double viewport_size = 1.0 / viewports.size();
	for(int i = 0; i < clouds.size(); i++) {
		visu.createViewPort(viewport_size*i, 0.0, viewport_size*(i+1), 1.0, viewports[i]);
	}
	visu.registerPointPickingCallback(pp_callback);

	// Load point data
	if(load_data_index != -1) {
		std::cout << "Loading point data..." << '\n';
		for(int n = 0; n < clouds.size(); n++) {
			std::ifstream ifs (argv[load_data_index+n]);
			std::string line;
			while(std::getline(ifs, line)) {
				std::vector<double> elements;
				while(line.find(",") != -1) {
					int comma_idx = line.find(",");
					double element = std::atof(line.substr(0, comma_idx).c_str());
					line.erase(0, comma_idx+1);
					elements.push_back(element);
				}

				Point_N p;
				p.x = elements[0];
				p.y = elements[1];
				p.z = elements[2];
				std::ostringstream name;
				name << p.x << p.y << p.z;
				visu.addSphere(p, 0.004, 255, 0, 0, name.str(), viewports[n]);

				points[n].push_back(p);
			}
		}
	}

	// Add clouds and spin, during this spin the user picks points
	for(int i = 0; i < clouds.size(); i++) {
		std::ostringstream oss;
		oss << "cloud" << i;
		visu.addPointCloud<Point_RGBA>(clouds[i], oss.str(), viewports[i]);
	}
	visu.spin();
	visu.removeAllPointClouds();

	// Save point data
	std::cout << "Save point data? (y/n)" << '\n';
	std::string save;
	std::cin >> save;
	if(save == "y") {
		save_points();
	}

	// Convert point vector to Matlab matrix string
	std::string points_string = "[";
	for(int i = 0; i < points.size(); i++) {
		std::vector<Point_N> P = points[i];
		std::string P_str = point_vector_to_string(P);
		points_string += P_str + ";";
	}
	points_string += "]";

	// Load robot data from file and create a string for MATLAB, and a Matrix4f for visualization
	std::string robot_data_string = "[";
	std::vector<Eigen::Matrix4f> T_HtoB;
	for(int i = 0; i < clouds.size(); i++) {
		std::string data_path = paths[i];
		data_path.erase(data_path.end()-3, data_path.end());
		data_path.append("csv");
		std::string data_string = get_robot_data_string(data_path);
		robot_data_string += data_string + ";";

		Eigen::Matrix4f data_matrix = get_robot_data_matrix_quaternion(data_path);
		T_HtoB.push_back(data_matrix);
	}
	robot_data_string += "]";

	std::cout << points_string << '\n';
	std::cout << robot_data_string << '\n';

	// Call Matlab script, this will create T_CtoH
	std::ostringstream command;
	command << "./run_calibration_pointpicking.sh /usr/local/MATLAB/MATLAB_Runtime/v92 '" << points_string << "' '" << robot_data_string << "'";
	std::system(command.str().c_str());

	// Find and import T_CtoH
	std::cout << "Importing T_CtoH..." << '\n';
	Eigen::Matrix4f T_CtoH = ar.get_T_CtoH();
	std::cout << T_CtoH << '\n';

	// Save T_CtoH to Calibration results
	std::cout << "Copying T_CtoH to calibration results..." << '\n';
	boost::filesystem::path calib_path = ar.path_to_calibration_results();
	calib_path += "/T_CtoH";
	boost::filesystem::copy_file("T_CtoH", calib_path, boost::filesystem::copy_option::overwrite_if_exists);

	std::cout << "Visualizing result..." << '\n';
	// Transform
	for(int i = 1; i < clouds.size(); i++) {
		Eigen::Matrix4f T_C2toC1 = T_CtoH.inverse() * T_HtoB[0].inverse()*T_HtoB[i] * T_CtoH;
		pcl::transformPointCloud(*clouds[i], *clouds[i], T_C2toC1);
	}

	// Visualize
	visu.removeAllPointClouds();
	visu.addPointCloud<Point_RGBA>(clouds[0], "cloud0", viewports[0]);
	for(int i = 1; i < clouds.size(); i++) {
		std::ostringstream oss;
		oss << "cloud" << i;
		visu.addPointCloud<Point_RGBA>(clouds[i], oss.str(), viewports[0]);
	}
	visu.spin();

	// Compute transformations and visualize
	/*
	bool visualize_rgb = true;
	while(true) {
		if(visualize_rgb) {
			visu.removeAllPointClouds();
			visu.addPointCloud<Point_RGBA>(clouds[0], "cloud0", viewports[0]);
			for(int i = 1; i < clouds.size(); i++) {
				std::ostringstream oss;
				oss << "cloud" << i;
				visu.addPointCloud<Point_RGBA>(clouds[i], oss.str(), viewports[0]);
			}
			visu.spin();

			visualize_rgb = false;
		} else {
			Point_Cloud_N::Ptr temp (new Point_Cloud_N);
			pcl::copyPointCloud(*clouds[0], *temp);
			visu.removeAllPointClouds();
			visu.addPointCloud<Point_N>(temp, ColorHandler_N(temp, 0.0, 255.0, 0.0), "clouds0", viewports[0]);
			for(int i = 1; i < clouds.size(); i++) {
				std::ostringstream oss;
				oss << "cloud" << i;
				pcl::copyPointCloud(*clouds[i], *temp);
				visu.addPointCloud<Point_N>(temp, ColorHandler_N(temp, 255.0, 0.0, 0.0), "cloudsi", viewports[0]);
			}
			visu.spin();

			visualize_rgb = true;
		}
	}
	*/
}
