
#include "../../Headers/access_model_data.h"
#include "../../Headers/View_graph.h"
#include "../../Headers/Normal_utility.h"
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal Point_N;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<Point_N> PointCloud_N;

bool view_normal_utility = true;
float bad_normal_thr = 20;
float sphere_radius = 0.35;


int 
main (int argc, char** argv)
{
	Access_Model_Data amd;
	std::vector<std::string> model_names;
	
	if (argc > 1)
	{
		std::string input (argv[1]);
		model_names.push_back (input);
	}
	else
	{
		model_names = amd.get_model_names ();
	}
	
	for (int i = 0; i < model_names.size(); i++)
	{
		// Load views
		std::vector<PointCloud_N::Ptr> views;
		std::vector<int> indices;
		amd.load_model_views (model_names[i], views, indices);
		
		// Generate normal utilities
		std::cout << "Generating normal utilities..." << std::endl;
		Normal_utility nu;
		std::vector<float> normal_utilities;
		PointT v_p;
		v_p.x = 0.0;
		v_p.y = 0.0;
		v_p.z = -sphere_radius;
		normal_utilities = nu.generate_normal_utilities (views, v_p, bad_normal_thr, 1);
		nu.save_normal_utilities (model_names[i], normal_utilities);
		std::cout << "Done\n" << std::endl;
	
		if (view_normal_utility)
		{
			//
			// View normal utilities
			//
	
			View_graph graph;
			graph.load_graph (model_names[i]);
			pcl::visualization::PCLVisualizer viewer ("viewer");
			std::vector<float> r_vec (graph.get_size (), 0.0);
			std::vector<float> g_vec (graph.get_size (), 0.0);
			std::vector<float> b_vec (graph.get_size (), 0.0);

			// Plot normal-utility information. Assign colors for nodes according to the normal-utilities. High utility => green, low utility => red.
			for (int j = 0; j < graph.get_size (); j++)
			{
				r_vec[j] = 1.0 - normal_utilities[j];
				g_vec[j] = normal_utilities[j];
			}

			graph.add_graph_to_viewer (viewer, 0.015, r_vec, g_vec, b_vec, 0, false);
			PointCloudT::Ptr complete_model (new PointCloudT);
			amd.load_complete_model (model_names[i], complete_model);
			viewer.addPointCloud (complete_model);
			viewer.spin ();
		}
	}
	
	return 0;
}
















