
#include "../headers/hint_system_class.h"
#include "../headers/pose_data.h"


int main (int argc, char** argv)
{
	// Variables
	bool view_search_results = true;
	bool normalize_search_results = false;
	bool view_valid_nodes = false;
	bool print_info = true;
	bool show_utilities = true;
	float weights[] = {0.222, 0.222, 0.222, 0.333};
	pcl::visualization::PCLVisualizer visu("Visu");
	
	// Create object for hint system and receive a new camera position
	Hint_System_Class hint_system;
	hint_system.set_weights (weights);
	hint_system.set_below_plane_threshold (0.01);
	hint_system.set_misalignment_angle_threshold (30);
	hint_system.set_valid_axis_threshold (20);
	hint_system.set_view_search_results (view_search_results, normalize_search_results);
	hint_system.set_view_valid_nodes (view_valid_nodes);
	hint_system.set_print_info (print_info);
	hint_system.set_show_utilities (show_utilities);
	hint_system.set_visualizer (&visu);
	Hint_System_Data hs_data = hint_system.find_new_view (argv[1], std::strtol (argv[2], NULL, 10));
	
	
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
























