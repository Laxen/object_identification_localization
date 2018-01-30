
#include "../../Headers/Similar_object_recognition.h"
#include "../../Headers/access_model_data.h"

typedef pcl::ESFSignature640 FeatureT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

int main (int argc, char** argv)
{
	Access_Model_Data access;
	Similar_object_recognition sor;
	
	std::vector<std::string> model_names = access.get_model_names ();
	std::vector<FeatureCloudT::Ptr> model_features;
	for (int i = 0; i < model_names.size(); i++)
	{
		FeatureCloudT::Ptr features (new FeatureCloudT);
		access.load_global_features (model_names[i], features);
		model_features.push_back (features);
	}
	
	for (int i = 0; i < model_names.size(); i++)
	{
		std::cout << model_names[i] << std::endl;
		sor.add_model (model_names[i], model_features[i]);
	}
	
	return(0);
}



















