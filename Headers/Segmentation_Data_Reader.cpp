#include "Segmentation_Data_Reader.h"

/**
  Loads a point cloud
  @param path The path to the cloud
  @param cloud The cloud pointer to use for storing the cloud
  @return 0 if error, 1 if successful
*/
int
Segmentation_Data_Reader::load_cloud(std::string path, Segmentation_Data_Reader::Point_Cloud_N::Ptr cloud) {
  int load_successful = pcl::io::loadPCDFile<Point_N>(path, *cloud);
  if(load_successful < 0) {
    pcl::console::print_error("Cloud could not be loaded!\n");
    return 0;
  }
  else {
    return 1;
  }
}

/**
  Reads segmentation data
  @param name Name of the segmentation folder in Data/Results/Segmentation_results
  @param scene_original The original scene
  @param scene The downsampled and clustered scene
  @param clusters Vector of clusters found in the scene
*/
void
Segmentation_Data_Reader::read_data_with_name(std::string name, Segmentation_Data_Reader::Point_Cloud_N::Ptr scene_original, Segmentation_Data_Reader::Point_Cloud_N::Ptr scene, std::vector<Segmentation_Data_Reader::Point_Cloud_N::Ptr>* clusters) {
	boost::filesystem::path p(boost::filesystem::current_path());
  while(true) {
    p = p.parent_path();
    if(p.filename() == "masters_thesis")
      break;
  }
  p += "/Data/Results/Segmentation_results/" + name + "/";

  load_cloud(p.string() + "scene_original.pcd", scene_original);
  load_cloud(p.string() + "scene.pcd", scene);

  // Load clusters
  std::ostringstream path;
  int i = 0;
  while(true) {
    path.str("");
    path << p.string() << "cluster" << i << ".pcd";
    if(boost::filesystem::exists(path.str())) {
      Point_Cloud_N::Ptr cluster (new Point_Cloud_N);
      load_cloud(path.str(), cluster);
      clusters->push_back(cluster);
    } else {
      break;
    }
    i++;
  }
}

/**
  Reads latest segmentation data from latest cloud captured (single cloud, not merged cloud)
  @param scene_original The original scene
  @param scene The downsampled and clustered scene
  @param clusters Vector of clusters found in the scene
*/
void
Segmentation_Data_Reader::read_latest_data_single(Segmentation_Data_Reader::Point_Cloud_N::Ptr scene_original, Segmentation_Data_Reader::Point_Cloud_N::Ptr scene, std::vector<Segmentation_Data_Reader::Point_Cloud_N::Ptr>* clusters) {
	boost::filesystem::path p(boost::filesystem::current_path());
  while(true) {
    p = p.parent_path();
    if(p.filename() == "masters_thesis")
      break;
  }
  p += "/Data/Results/Segmentation_results/latest_path";
  std::ifstream ifs(p.c_str());
  std::string name;
  std::getline(ifs, name);
  name += "/single"

  read_data_with_name(name, scene_original, scene, clusters);
}

/**
  Reads latest segmentation data from latest cloud captured (single cloud, not merged cloud)
  @param scene_original The original scene
  @param scene The downsampled and clustered scene
  @param clusters Vector of clusters found in the scene
*/
void
Segmentation_Data_Reader::read_latest_data_single(Segmentation_Data_Reader::Point_Cloud_N::Ptr scene_original, Segmentation_Data_Reader::Point_Cloud_N::Ptr scene, std::vector<Segmentation_Data_Reader::Point_Cloud_N::Ptr>* clusters) {
	boost::filesystem::path p(boost::filesystem::current_path());
  while(true) {
    p = p.parent_path();
    if(p.filename() == "masters_thesis")
      break;
  }
  p += "/Data/Results/Segmentation_results/latest_path";
  std::ifstream ifs(p.c_str());
  std::string name;
  std::getline(ifs, name);
  name += "/merged"

  read_data_with_name(name, scene_original, scene, clusters);
}
