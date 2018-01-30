
#include "../headers/identification_class.h"
#include "../headers/access_results.h"

typedef pcl::PointNormal Point_N;
typedef pcl::PointCloud<Point_N> PointCloud_N;

int main (int argc, char** argv)
{
	// Create identification object
	Identification_Class ident;
	ident.show_results (true);
	ident.set_save_results (false);
	
	// Load data from Segmentation
	Access_Results access;
	PointCloud_N::Ptr scene_original (new PointCloud_N);
	PointCloud_N::Ptr scene (new PointCloud_N);
	std::vector<PointCloud_N::Ptr> clusters;
	std::string scene_name;
	
	if (argc < 3)
	{
		if (argc == 2)
		{
			scene_name = std::string (argv[1]);
			access.load_segmentation_results (scene_name + "/single", scene_original, scene, clusters);
		}
		else
		{
			scene_name = access.load_latest_single_segmentation_results (scene_original, scene, clusters);
		}
		
		// Identify each cluster
		for (int i = 0; i < clusters.size(); i++)
		{
			ident.identify (scene_name, i, clusters[i]);
		}
	}
	else if (argc == 3)
	{
		scene_name = access.load_latest_single_segmentation_results (scene_original, scene, clusters);
		int cluster_index = std::strtol (argv[2], NULL, 10);
		ident.identify (scene_name, cluster_index, clusters[cluster_index]);
	}
	
	return (0);
}
