#ifndef CLOUD_SEGMENTATION_H_
#define CLOUD_SEGMENTATION_H_

#include <pcl/io/pcd_io.h>
#include "Manipulation.h"
#include "access_results.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

class Cloud_Segmentation {
	private:
		typedef pcl::PointNormal Point_N;
		typedef pcl::PointXYZRGB Point_RGBA;
		typedef pcl::PointCloud<Point_N> Point_Cloud_N;
		typedef pcl::PointCloud<Point_RGBA> Point_Cloud_RGBA;

		double LEAF_SIZE; // Used for downsampling (0.003 for small objects, 0.010 for large)

		double PLANE_DISTANCE_THRESHOLD; // Distance from point to plane to count as inlier to plane
		int PLANE_MAX_ITERATIONS; // Max RANSAC iterations for finding plane
		double BACKGROUND_SEGMENTATION_DISTANCE; // Max distance between two points when subtracting clouds

		double NORMAL_RADIUS_SEARCH; // Radius for nearest neighbors

		double CLUSTER_TOLERANCE;
		int CLUSTER_MIN_SIZE;
		int CLUSTER_MAX_SIZE;

		pcl::visualization::PCLVisualizer* visu_ptr;
		typedef pcl::visualization::PointCloudColorHandlerCustom<Point_N> ColorHandler_N;

		Point_Cloud_N::Ptr background;
		bool background_set;
		Eigen::Matrix<float,4,4,Eigen::DontAlign> cloud_transformation;
		Eigen::Matrix<float,4,4,Eigen::DontAlign> background_transformation;
		Eigen::Matrix<float,4,4,Eigen::DontAlign> T_CtoH;

		bool visualize;

		/**
		  Loads a point cloud
		  @param path The path to the cloud
		  @param cloud The cloud pointer to use for storing the cloud
		  @return 0 if error, 1 if successful
		 */
		int
		load_cloud(std::string path, Point_Cloud_N::Ptr cloud);

		/**
		  Loads a point cloud with RGB data
		  @param path The path to the cloud
		  @param cloud The cloud pointer to use for storing the cloud
		  @return 0 if error, 1 if successful
		 */
		int
		load_cloud_rgba(std::string path, Point_Cloud_RGBA::Ptr cloud);

		/**
		  Visualizes a single cloud once
		  @param cloud The cloud to visualize
		 */
		void
		visualize_cloud(Point_Cloud_N::Ptr cloud);

	public:
		Cloud_Segmentation();

		/**
		  Segments a cloud into clusters
		  @param scene_original_rgba The RGBA cloud of the scene that should be segmented
		  @param name The name of the cloud (main folder in Segmentation_results)
		  @param sub_name The sub name of the cloud (single or merged)
		 */
		void
		segment(Point_Cloud_RGBA::Ptr scene_original_rgba, std::string name, std::string sub_name);

		/**
		  Sets the background to be subtracted, if no background is set plane removal is used
		  @param b The background cloud
		  @param trans The transformation for the background
		  @param T_CtoH The hand to camera transformation
		*/
		void
		set_background_data(Point_Cloud_N::Ptr b, Eigen::Matrix<float,4,4,Eigen::DontAlign> trans, Eigen::Matrix<float,4,4,Eigen::DontAlign> T_CtoH);

		/**
		  Set the visualizer object to be used when visualizing
		  @param visu The visualizer pointer
		 */
		void
		set_visualizer(pcl::visualization::PCLVisualizer* visu);

		/**
			Sets the transformation matrix for the current cloud. This is used when segmenting background.
			This transformation needs to be set every time before segment() is called if using background segmentation.
			@param trans The transformation for the current cloud
		*/
		void 
		set_cloud_transformation_matrix(Eigen::Matrix<float,4,4,Eigen::DontAlign> trans);

};

#endif
