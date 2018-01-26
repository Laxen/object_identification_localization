#include "../../Headers/Access_Model_data.h"
#include <pcl/features/fpfh_omp.h>

typedef pcl::PointNormal Point_N;
typedef pcl::PointCloud<Point_N> PointCloud_N;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

int main (int argc, char** argv)
{
	// Load models
	Access_Model_data access;
	std::vector<std::string> model_names = access.get_model_names ();
	//std::vector<std::string> model_names;
	//model_names.push_back ("wood_object");
	
	for (int i = 0; i < model_names.size(); i++)
	{
		std::cout << "\nComputing local features for " << model_names[i] << std::endl;
		
		// Load model views
		std::vector<PointCloud_N::Ptr> views;
		std::vector<int> indices;
		access.load_model_views_original_pose (model_names[i], views, indices);
		
		std::vector<FeatureCloudT::Ptr> features_vec;
		for (int j = 0; j < views.size(); j++)
		{
			// Estimate local FPFH features
			FeatureCloudT::Ptr features (new FeatureCloudT);
			pcl::FPFHEstimationOMP<Point_N, Point_N, FeatureT> feature_estimator;

			feature_estimator.setRadiusSearch(0.01);
			feature_estimator.setInputCloud(views[j]);
			feature_estimator.setInputNormals(views[j]);
			feature_estimator.compute(*features);
		
			features_vec.push_back (features);
		}
		
		// Save local features
		access.save_local_features (model_names[i], features_vec);
		std::cout << "Done\n" << std::endl;
	}
	
	return (0);
}
