
#include "../../headers/access_model_data.h"
#include "../../headers/view_feature_score.h"

typedef pcl::PointNormal Point_N;
typedef pcl::PointCloud<Point_N> PointCloud_N;

int main (int argc, char** argv) {

	// Get all model names in Model_data
	Access_Model_Data access;
	std::vector<std::string> model_names = access.get_model_names ();
	
	View_Feature_Score vfs;
	for (int i = 0; i < model_names.size(); i++)
	{
		std::cout << "Generating feature utilities for " << model_names[i] << "..." << std::endl;
		
		// Load model views
		std::vector<PointCloud_N::Ptr> views;
		std::vector<int> indices;
		access.load_model_views (model_names[i], views, indices);
		
		// Compute feature utilities
		std::vector<double> utilities;
		utilities = vfs.compute_view_score_normalized(views);
		
		// Save feature utilities
		access.save_feature_utilities (model_names[i], utilities);
		
		std::cout << "Done\n" << std::endl;
	}

	return (0);
}
