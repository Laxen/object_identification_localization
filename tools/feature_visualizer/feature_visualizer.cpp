#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/fpfh_omp.h>
#include "../manipulation/manipulation.hpp"

typedef pcl::FPFHSignature33 Feature;
typedef pcl::PointCloud<Feature> Feature_Cloud;

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> ColorHandler_N;

int main(int argc, char** argv) {
  Point_Cloud_N::Ptr cloud (new Point_Cloud_N);
  Point_Cloud_N::Ptr cloud_original (new Point_Cloud_N);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<Point_N>(argv[1], *cloud);

  pcl::copyPointCloud(*cloud, *cloud_original);

  cloud = downsample_cloud(cloud, 0.003);
  pcl::copyPointCloud(*cloud, *cloud_rgb);
  uint8_t r = 255;
  uint8_t g = r;
  uint8_t b = r;
  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  for(int i = 0; i < cloud_rgb->width; i++) {
    cloud_rgb->points[i].rgb = *reinterpret_cast<float*>(&rgb);
  }
  //compute_normals(cloud, cloud_original, 0.004);

  Feature_Cloud::Ptr features (new Feature_Cloud);
  pcl::FPFHEstimationOMP<Point_N, Point_N, Feature> feature_estimator;
  feature_estimator.setRadiusSearch(0.010);
  feature_estimator.setInputCloud(cloud);
  feature_estimator.setInputNormals(cloud);
  feature_estimator.compute(*features);

  std::cout << "cloud: " << cloud->size() << std::endl;
  std::cout << "features: " << features->size() << std::endl;

  pcl::visualization::PCLVisualizer visu("Visualizer");
  visu.addPointCloud(cloud_rgb, ColorHandler_N(cloud_rgb, 0.0, 255.0, 0.0), "cloud_rgb");
  visu.addPointCloudNormals<Point_N>(cloud, 1, 0.005f, "cloud_normals");
  visu.spin();

  for(int f = 0; f < 3; f++) {
    for(int i = 0; i < cloud_rgb->size(); i++) {
      uint8_t red = (features->points[i].histogram[f*11] + features->points[i].histogram[f*11+1] + features->points[i].histogram[f*11+2]) / 100.0 * 255;
      uint8_t green = (features->points[i].histogram[f*11+3] + features->points[i].histogram[f*11+4] + features->points[i].histogram[f*11+5] + features->points[i].histogram[f*11+6]) / 100.0 * 255;
      uint8_t blue = (features->points[i].histogram[f*11+7] + features->points[i].histogram[f*11+8] + features->points[i].histogram[f*11+9] + features->points[i].histogram[f*11+10]) / 100.0 * 255;

      uint32_t rgb = ((uint32_t)red << 16 | (uint32_t)green << 8 | (uint32_t)blue);

      cloud_rgb->points[i].rgb = *reinterpret_cast<float*>(&rgb);
    }

    visu.removeAllPointClouds();
    visu.addPointCloud(cloud_rgb, "cloud_rgb");
    visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 22, "cloud_rgb");
    visu.spin();
  }
}
