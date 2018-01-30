#include "../../headers/robot_data_fetcher.h"

std::string save_path = "data.csv";

int main(int argc, char** argv) {
	Robot_Data_Fetcher rdf;
	rdf.set_auth_details("Default User", "robotics");
	rdf.set_url("http://192.168.0.3/rw/rapid/tasks/T_ROB_L/motion?resource=robtarget&tool0&wobj=wobj0");

	// Wait for point cloud in share folder
	std::cout << "Fetching robot data..." << '\n';
	rdf.fetch_data(save_path);
	std::cout << "Robot data saved as " << save_path << "!" << '\n';
}
