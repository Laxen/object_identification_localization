
#include "../Headers/Hint_system_class.h"
#include "../Headers/pose_data.h"


int main (int argc, char** argv)
{
	//*
	// Variables
	bool view_search_results = true;
	bool normalize_search_results = false;
	bool view_valid_nodes = false;
	bool print_info = true;
	bool show_utilities = true;
	//float weights[] = {0.167, 0.167, 0.333, 0.333};
	//float weights[] = {0.0, 0.333, 0.333, 0.333};
	//float weights[] = {0.25, 0.25, 0.25, 0.25};
	float weights[] = {0.222, 0.222, 0.222, 0.333};
	pcl::visualization::PCLVisualizer visu("Visu");
	
	// Create object for hint system and receive a new camera position
	Hint_system_class hint_system;
	hint_system.set_weights (weights);
	hint_system.set_below_plane_threshold (0.01);
	hint_system.set_misalignment_angle_threshold (30);
	hint_system.set_valid_axis_threshold (20);
	hint_system.set_view_search_results (view_search_results, normalize_search_results);
	hint_system.set_view_valid_nodes (view_valid_nodes);
	hint_system.set_print_info (print_info);
	hint_system.set_show_utilities (show_utilities);
	hint_system.set_visualizer (&visu);

	//std::pair<std::vector<Pose_Data>, Eigen::Matrix<float,4,4,Eigen::DontAlign> > temp = hint_system.find_new_view (argv[1], std::strtol (argv[2], NULL, 10));
	Hint_System_Data hs_data = hint_system.find_new_view (argv[1], std::strtol (argv[2], NULL, 10));
	//*/
	
	/*
	// Create object for hint system and receive a new camera position
	Hint_system_class hint_system;
	float weights[] = {0.33, 0.33, 0.33};
	pcl::visualization::PCLVisualizer visu("Visu");
	hint_system.set_weights (weights);
	hint_system.set_below_plane_threshold (0.01);
	hint_system.set_misalignment_angle_threshold (30);
	hint_system.set_view_search_results (true, true);
	hint_system.set_visualizer (&visu);

	std::string scene_name (argv[1]);
	int cluster_index = std::strtol (argv[2], NULL, 10);
	hint_system.find_new_view (scene_name, cluster_index);
	
	// Load valid pose data
	Access_Results ar;
	std::vector<Pose_Data> pose_data;
	if (ar.load_hint_system_valid_poses (scene_name, cluster_index, pose_data) > 0)
	{
		std::cout << "No data loaded" << std::endl;
	}
	*/
	
	if (hs_data.valid_results)
	{
		for (int i = 0; i < hs_data.pd.size(); i++)
		{
			std::cout << "\n" << hs_data.pd[i].model_name << ":" << std::endl;
			printf ("\tScore: %3.3f\n", hs_data.pd[i].accuracy);
			printf ("\tPose:\n");
			printf ("\t            | %6.3f %6.3f %6.3f | \n", hs_data.pd[i].transformation (0,0), hs_data.pd[i].transformation (0,1), hs_data.pd[i].transformation (0,2));
			printf ("\t        R = | %6.3f %6.3f %6.3f | \n", hs_data.pd[i].transformation (1,0), hs_data.pd[i].transformation (1,1), hs_data.pd[i].transformation (1,2));
			printf ("\t            | %6.3f %6.3f %6.3f | \n", hs_data.pd[i].transformation (2,0), hs_data.pd[i].transformation (2,1), hs_data.pd[i].transformation (2,2));
			printf ("\n");
			printf ("\t        t = < %0.3f, %0.3f, %0.3f >\n", hs_data.pd[i].transformation (0,3), hs_data.pd[i].transformation (1,3), hs_data.pd[i].transformation (2,3));
		}
		std::cout << "\n" << std::endl;
	}
	else
	{
		std::cout << "No valid hint system results" << std::endl;
	}
	
	return (0);
}
























