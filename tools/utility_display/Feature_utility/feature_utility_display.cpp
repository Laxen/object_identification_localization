
#include "../../../headers/access_model_data.h"
#include "../../../headers/view_graph.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal Point_N;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<Point_N> PointCloud_N;


int 
main (int argc, char** argv)
{
	if (argc > 2)
	{
		pcl::console::print_error ("\nOnly one input file is allowed!\n");
		std::exit (EXIT_FAILURE);
	}
	else if (argc == 1)
	{
		pcl::console::print_error ("Please select an input file!\n\n");
		std::cout << "Usage: feature_utility_display INPUT\n\nINPUT is the specified added CAD model placed in the CAD_models folder.\n\n" << std::endl;
		std::exit (EXIT_FAILURE);
	}
	
	Access_Model_Data amd;
	View_Graph graph;
	
	// Check if model exists (has been added)
	std::vector<std::string> names = amd.get_model_names();
	std::string model_name (argv[1]);
	if (std::find(names.begin(), names.end(), model_name) == names.end())
	{
		std::stringstream ss;
		ss << "The model " << model_name << " could not be found!\n\n";
		pcl::console::print_error (ss.str().c_str());
		std::cout << "Make sure that the model is added before using this program!\n\n" << std::endl;
		std::exit (EXIT_FAILURE);
	}
	
	// Load feature utilities 
	std::vector<float> utilities = amd.load_feature_utilities (model_name);
	
	// Load view-graph
	graph.load_graph (model_name);
	
	// Load complete model
	PointCloudT::Ptr complete_model (new PointCloudT);
	amd.load_complete_model (model_name, complete_model);
	
	//
	// Display view utilities with the view-graph
	//
	
	// Assign colors for nodes according to the utilities. High utility => green, low utility => red.
	std::vector<float> r_vec (graph.get_size (), 0.0);
	std::vector<float> g_vec (graph.get_size (), 0.0);
	std::vector<float> b_vec (graph.get_size (), 0.0);
	for (int i = 0; i < utilities.size (); i++)
	{
		r_vec[i] = 1.0 - utilities[i];
		g_vec[i] = utilities[i];
	}

	pcl::visualization::PCLVisualizer viewer ("viewer");
	graph.add_graph_to_viewer (viewer, 0.015, r_vec, g_vec, b_vec, 0, false);
	viewer.addPointCloud<PointT> (complete_model);
	std::cout << "\nPress Q to continue\n" << std::endl;
	viewer.spin ();
	
	//
	// Display utilities with colored point clouds
	//
	
	// Fill vector with random integers
	std::vector<int> indices;
	int rand_int = rand() % 80;
	indices.push_back (rand_int);
	for (int i = 0; i < 24; i++)
	{
		do
		{
			rand_int = rand() % 80; // Random number between 0 and 79
		}
		while (std::find(indices.begin(), indices.end(), rand_int) != indices.end());
		
		indices.push_back (rand_int);
	}
	
	std::vector<PointCloud_N::Ptr> views;
	amd.load_model_views (model_name, views, indices);
	
	// Create viewer with 25 viewports (5x5)
	pcl::visualization::PCLVisualizer viewer2 ("viewer2");
	float delta = 0.2;
	float x_min = 0.0, x_max = delta, y_min = 0.0, y_max = delta;
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			int vp = j + i*5 + 1;
			viewer2.createViewPort (x_min, y_min, x_max, y_max, vp);
			x_min += delta;
			x_max += delta;
		}
		
		y_min += delta;
		y_max += delta;
		x_min = 0.0;
		x_max = delta;
	}
	
	// Add views to all viewports with the color associated with the utility value. Green => high utility, red => low utility
	for (int i = 0; i < 25; i++)
	{
		r_vec[i] = 255 - 255*utilities[indices[i]];
		g_vec[i] = 255*utilities[indices[i]];
	}
	
	std::stringstream ss;
	for (int i = 0; i < 25; i++)
	{
		ss.str ("");
		ss << "view" << i;
		int vp = i + 1;
		//pcl::visualization::PointCloudColorHandlerCustom<Point_N> color_handler (views[i], 0, 255, 0);
		pcl::visualization::PointCloudColorHandlerCustom<Point_N> color_handler (views[i], r_vec[i], g_vec[i], b_vec[i]);
		viewer2.addPointCloud<Point_N> (views[i], color_handler, ss.str (), vp);
	}
	
	viewer2.spin ();
	
	return 0;
}
















