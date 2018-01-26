#ifndef CLOUD_MERGING_H_
#define CLOUD_MERGING_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ppf_registration.h>
#include "Access_Results.h"

class Cloud_Merging {
	private:
		typedef pcl::PointNormal Point_N;
		typedef pcl::PointXYZRGB Point_RGBA;
		typedef pcl::PointCloud<Point_N> Point_Cloud_N;
		typedef pcl::PointCloud<Point_RGBA> Point_Cloud_RGBA;

		pcl::visualization::PCLVisualizer* visu_ptr;
		bool visualize;
	public:
		/**
			Merges a vector of clouds into the same coordinate system as the first cloud in the vector, using the corresponding vector of robot data. The result is saved in the latest segmentation data folder. 
			@param clouds Vector of all clouds that should be merged
			@param robot_data Vector of robot data (hand position) for each cloud
		*/
		void
		merge(std::vector<Point_Cloud_RGBA::Ptr> clouds, std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> > robot_data);

		/**
			Set the visualizer object to be used when visualizing
			@param visu The visualizer pointer
		 */
		void
		set_visualizer(pcl::visualization::PCLVisualizer* visu);
};

#endif
