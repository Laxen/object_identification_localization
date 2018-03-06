#include "../headers/access_model_data.h"

int main(int argc, char** argv)
{
	if (argc > 2)
	{
		pcl::console::print_error ("\nOnly one input file is allowed!\n");
		std::exit (EXIT_FAILURE);
	}
	else if (argc == 1)
	{
		pcl::console::print_error ("Please select an input file!\n\n");
		std::cout << "Usage: remove_model INPUT\n\nINPUT is the specified added CAD model placed in the CAD_models folder.\n\n" << std::endl;
		std::exit (EXIT_FAILURE);
	}
	
	// Add path to model_data
	Access_Model_Data amd;
	boost::filesystem::path p_model = amd.path_to_model_data ();
	
	// Add path to model
	std::string model_name (argv[1]);
	p_model /= model_name;
	
	// Check if such path exists
	if (!boost::filesystem::exists(p_model))
	{
		std::stringstream ss;
		ss << "ERROR: Could not find " << model_name << " in model_data!\n\n";
		pcl::console::print_error(ss.str().c_str());
		std::exit (EXIT_FAILURE);
	}
	
	std::cout << "\nRemoving " << model_name << " from model_data..." << std::endl;
	
	// Remove model folder in CAD_models
	boost::filesystem::remove_all (p_model);
	
	// Remove distinguish utilities associated with the given model for all other models
	std::vector<std::string> model_names = amd.get_model_names();
	for (int i = 0; i < model_names.size(); i++)
	{
		// Add path to model in model_data
		boost::filesystem::path p = amd.path_to_model_in_model_data (model_names[i]);
		
		// Add path to distinguish utilities
		p /= "distinguish_utilities/" + model_name + "_distinguish_utilities.csv";
		
		// Check if such path exists
		if (!boost::filesystem::exists(p))
		{
			std::stringstream ss;
			ss << "ERROR: Could not find " + p.string() + "\n\n";
			pcl::console::print_error(ss.str().c_str());
			std::exit (EXIT_FAILURE);
		}
		
		// Remove the distinguish utility file
		boost::filesystem::remove (p);
	}
	
	std::cout << "done\n\nThe model: " << model_name << " was successfully removed from model_data\n" << std::endl;

	return (0);	
}




















