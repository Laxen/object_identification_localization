#ifndef SEGMENTATION_DATA_READER_H
#define SEGMENTATION_DATA_READER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/visualization/pcl_visualizer.h>

class Segmentation_Data_Reader {
  private:
    typedef pcl::PointNormal Point_N;
    typedef pcl::PointCloud<Point_N> Point_Cloud_N;

    /**
      Loads a point cloud
      @param path The path to the cloud
      @param cloud The cloud pointer to use for storing the cloud
      @return 0 if error, 1 if successful
    */
    int
    load_cloud(std::string path, Point_Cloud_N::Ptr cloud);

  public:
    /**
      Reads segmentation data
      @param name Name of the segmentation folder in Data/Results/Segmentation_results
      @param scene_original The original scene
      @param scene The downsampled and clustered scene
      @param clusters Vector of clusters found in the scene
    */
    void
    read_data_with_name(std::string name, Point_Cloud_N::Ptr scene_original, Point_Cloud_N::Ptr scene, std::vector<Point_Cloud_N::Ptr>* clusters);

    /**
      Reads latest segmentation data from latest cloud captured (single cloud, not merged cloud)
      @param scene_original The original scene
      @param scene The downsampled and clustered scene
      @param clusters Vector of clusters found in the scene
    */
    void
    read_latest_data_single(Point_Cloud_N::Ptr scene_original, Point_Cloud_N::Ptr scene, std::vector<Point_Cloud_N::Ptr>* clusters);

    /**
      Reads latest segmentation data from latest merged cloud captured
      @param scene_original The original scene
      @param scene The downsampled and clustered scene
      @param clusters Vector of clusters found in the scene
    */
    void
    read_latest_data_merged(Point_Cloud_N::Ptr scene_original, Point_Cloud_N::Ptr scene, std::vector<Point_Cloud_N::Ptr>* clusters);
};

#endif
