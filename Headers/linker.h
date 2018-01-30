#ifndef LINKER_H_
#define LINKER_H_

#include "access_results.h"
#include "manipulation.h"
#include <pcl/common/transforms.h>
#include "pose_data.h"

class Linker {
	private:
		typedef pcl::PointNormal Point_N;
		typedef pcl::PointCloud<Point_N> Point_Cloud_N;

		/**
		  Loads T_CtoH from CSV file and saves as Eigen::Matrix<float,4,4,Eigen::DontAlign>
		  @return The T_CtoH transformation
		*/
		Eigen::Matrix<float,4,4,Eigen::DontAlign>
		get_T_CtoH(Access_Results ar);

		/**
		  Loads robot data from file (Euler format) and formats it into a Eigen::Matrix<float,4,4,Eigen::DontAlign> transformation
		  @param path The path to the data file (CSV)
		  @return The Eigen::Matrix<float,4,4,Eigen::DontAlign> transformation
		*/
		Eigen::Matrix<float,4,4,Eigen::DontAlign>
		get_robot_data_matrix(std::string path);

		/**
		  Loads robot data from file (quaternion format) and formats it into a Eigen::Matrix<float,4,4,Eigen::DontAlign> transformation
		  @param path The path to the data file (CSV)
		  @return The Eigen::Matrix<float,4,4,Eigen::DontAlign> transformation
		*/
		Eigen::Matrix<float,4,4,Eigen::DontAlign>
		get_robot_data_matrix_quaternion(std::string path);

		/**
		  Compares pose structures
		*/
		static bool
		poses_comparator(const Pose_Data& pose1, const Pose_Data& pose2);

		/**
		  Compares id structures
		*/
		static bool
		id_comparator(const ID_Data& id1, const ID_Data& id2);

  public:
		/**
		  Links pose data from the merged cloud called prev_merged_name to the merged cloud called current_merged_name
		  @param ar Access_Results object
		  @param m Manipulation object
		  @param prev_merged_name The main folder name of the previous merged cloud
		  @param current_merged_name The main folder name of the current merged cloud
		  @param model_names The model names for each cluster in the current merged cloud, as identified in the pose estimation for the previous merged cloud
		  @param view_indices The model view indices for clusters in the current merged cloud, as identified in pose estimation for the prev merged cloud
		  @param model_scores The model scores for clusters in the current merged cloud, as identified in the pose estimation for the prev merged cloud
		  @returns The merged clusters
		*/
		std::vector<Point_Cloud_N::Ptr>
		link_pose_data(Access_Results ar,
						  Manipulation m,
						  std::string prev_merged_name,
						  std::string current_merged_name,
						  std::vector<std::vector<std::string> > &model_names,
						  std::vector<std::vector<std::vector<int> > > &view_indices,
						  std::vector<std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> > > poses,
						  std::vector<std::vector<double> > inlier_fractions,
						  std::vector<std::vector<double> > accuracies);

		/**
		  Links identification data from single_name to merged_name
		  @param ar Access_Results object
		  @param m Manipulation object
		  @param single_name The main folder name of the single cloud
		  @param merged_name The main folder name of the merged cloud
		  @param models_single_in_merged The model names for each merged clusters, as identified in the linked single cluster
		  @param model_view_indices_1_in_2 The model view indices for each merged clusters, as identified in the linked single cluster
		  @param model_scores_1_in_2 The model scores for each merged clusters, as identified in the linked single cluster
		  @returns The merged clusters
		*/
		std::vector<Point_Cloud_N::Ptr>
		link_identification_data(Access_Results ar,
									Manipulation m,
									std::string single_name,
									std::string merged_name,
									std::vector<std::vector<std::string> > &models_single_in_merged,
									std::vector<std::vector<std::vector<int> > > &model_view_indices_single_in_merged,
									std::vector<std::vector<std::vector<float> > > &model_scores_single_in_merged);
    };

#endif
