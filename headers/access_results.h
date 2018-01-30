#ifndef ACCESS_RESULTS_H_
#define ACCESS_RESULTS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include "pose_data.h"
#include "id_data.h"

class Access_Results
{
	private:
		typedef pcl::PointXYZ PointT;
		typedef pcl::PointNormal Point_N;
		typedef pcl::PointXYZRGB PointT_color;
		typedef pcl::PointCloud<PointT> PointCloudT;
		typedef pcl::PointCloud<Point_N> PointCloud_N;
		typedef pcl::PointCloud<PointT_color> PointCloudT_color;
		typedef pcl::ESFSignature640 FeatureT;
		typedef pcl::PointCloud<FeatureT> FeatureCloudT;

	public:
		/**
		  Returns the path to Data in /masters_thesis
		*/
		boost::filesystem::path
		path_to_data (void);

		/**
		  Returns the path to Results in /Data
		*/
		boost::filesystem::path
		path_to_results (void);

		/**
		  Returns the path to Hint_system_results in /Results
		*/
		boost::filesystem::path
		path_to_hint_system_results (void);

		/**
		  Returns the path to Identification_results in /Results
		*/
		boost::filesystem::path
		path_to_identification_results (void);

		/**
		  Returns the path to Pose_estimation_results in /Results
		*/
		boost::filesystem::path
		path_to_pose_estimation_results (void);

		/**
		  Returns the path to Segmentation_results in /Results
		*/
		boost::filesystem::path
		path_to_segmentation_results (void);

		/**
		  Returns the path to the latest Segmentation_results in /Results
		*/
		boost::filesystem::path
		path_to_latest_segmentation_results (void);

    	/**
		  Returns the path to Calibration_results in /Results
		*/
		boost::filesystem::path
		path_to_calibration_results (void);

		/**
		  Returns the path to scene in Segmentation_results
		*/
		boost::filesystem::path
		path_to_scene_in_segmentation_results (std::string scene_name);

		/**
		  Returns the path to scene in /Identification_results
		*/
		boost::filesystem::path
		path_to_scene_in_identification_results (std::string scene_name);

		/**
		  Loads the results from identification of cluster: @param cluster_index, in scene: @param scene_name
		  @param scene_name The name of the scene
		  @param cluster_index The cluster index
		  @param models Empty vector. Adds the identified models to the vector where the most likely model is the top one (index = 0)
		  @param model_view_indices Empty vector. Adds the identified views to each model where the best view is the top one (index = 0)
		  @param model_score Empty vector. Adds the score to the identified views. A good match has low score (~0.05-0.2)
		  @param cluster Empty point cloud. Loads the point cloud of the cluster in Segmentation_results
		*/
		void
		load_identification_results (	std::string scene_name,
										int cluster_index,
										std::vector<std::string> &models,
										std::vector<std::vector<int> > &model_view_indices,
										std::vector<std::vector<float> > &model_scores,
										PointCloud_N::Ptr cluster );

    /**
      Loads the results from identification of cluster: @param cluster_index, in scene: @param scene_name, and return an ID_Data object
      @param scene_name The name of the scene
      @param cluster_index The cluster index
    	@param id_data The vector of ID_Data objects
    */
    void
    load_identification_results_id_data_format(std::string scene_name, int cluster_index, std::vector<ID_Data> &id_data);

		/**
		  Loads the latest results from identification
		  @param cluster_index The index of the cluster
		  @param models Empty vector. Adds the identified models to the vector where the most likely model is the top one (index = 0)
		  @param model_view_indices Empty vector. Adds the identified views to each model where the best view is the top one (index = 0)
		  @param model_score Empty vector. Adds the score to the identified views. A good match has low score (~0.05-0.2)
		  @param cluster Empty point cloud. Loads the point cloud of the cluster in Segmentation_results
		*/
		std::string
		load_latest_identification_results (int cluster_index,
											std::vector<std::string> &models,
											std::vector<std::vector<int> > &model_view_indices,
											std::vector<std::vector<float> > &model_scores,
											PointCloud_N::Ptr cluster );

		/**
			Loads the similar models from a given scene
			@param scene_name[in] The name of the scene
			@param cluster_index[in] The index of the cluster
			@param object The best matching object from pose estimation
			@param similar_models[out] Vector containing the similar models
		*/
		void
		load_similar_models (std::string scene_name, int cluster_index, std::string object, std::vector<std::string> &similar_models);

		/**
			Loads the similar models from the latest identitication results
			@param cluster_index[in] The index of the cluster
			@param object The best matching object from pose estimation
			@param similar_models[out] Vector containing the similar models
			@return The name of the latest scene
		*/
		std::string
		load_similar_models_latest_results (int cluster_index, std::string object, std::vector<std::string> &similar_models);

    /**
    	Loads the results from pose estimation and returns a Pose_Data object
    	@param scene_name Name of the scene to load pose data from
      @param cluster_index The index of the cluster to load pose data from
    	@param pose_data[out] Vector of Pose_Data objects
    */
    void
    load_pose_estimation_results_pose_format(std::string scene_name, int cluster_index, std::vector<Pose_Data> &pose_data);

    /**
    	Loads the results from pose estimation
    	@param scene_name Name of the scene to load pose data from
      @param cluster_index The index of the cluster to load pose data from
    	@param model_names[out] Vector of each model name
    	@param view_indices[out] Vector of vector of view indices. Outer vector is for model name, inner vector is a list of views for that model.
    	@param poses[out] Vector of poses as Eigen::Matrix<float,4,4,Eigen::DontAlign> objects
    	@param inlier_fractions[out] Vector of inlier fractions
    	@param accuracies[out] Vector of accuracies
    */
		void
		load_pose_estimation_results(std::string scene_name, int cluster_index, std::vector<std::string> &object_names, std::vector<std::vector<int> > &view_indices, std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> > &poses, std::vector<double> &inlier_fractions, std::vector<double> &accuracies);

		/**
		  Loads segmentation data
		  @param scene_name Name of the segmentation folder in Data/Results/Segmentation_results
		  @param scene_original The original scene
		  @param scene The downsampled and clustered scene
		  @param clusters Vector of clusters found in the scene
		*/
		void
		load_segmentation_results(std::string scene_name, PointCloud_N::Ptr scene_original, PointCloud_N::Ptr scene, std::vector<PointCloud_N::Ptr> &clusters);

		void 
		load_all_segmentation_results_original_scenes(std::vector<PointCloudT_color::Ptr>& original_scenes, std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> >& robot_data);
		
		
		/**
		  Loads the merged cluster in segmentation results
		  @param scene_name The name of the scene
		  @param cluster_index The cluster index
		  @param cluster The cluster point cloud
		*/
		void
		load_segmentation_merged_cluster (std::string scene_name, int cluster_index, PointCloud_N::Ptr cluster);

    /**
    	Loads the latest single segmentation plane
    	@param plane The ModelCoefficients pointer to store the plane coefficients in
    */
    void
    load_latest_single_segmentation_plane(pcl::ModelCoefficients &plane);

    /**
    	Loads the latest merged segmentation plane
    	@param plane The ModelCoefficients pointer to store the plane coefficients in
    */
    void
    load_latest_merged_segmentation_plane(pcl::ModelCoefficients &plane);

    /**
    	Loads the plane from the segmentation results of "scene_name" (note that scene_name needs to specify "single" or "merged" as well!)
    	@param scene_name The name of the scene (with single or merged subfolder specified)
    	@param plane The ModelCoefficients pointer to store the plane coefficients in
    */
    void
    load_segmentation_plane(std::string scene_name, pcl::ModelCoefficients &plane);

		/**
		  Loads latest single (non-merged) segmentation data specified by latest_path
		  @param scene_original The original scene
		  @param scene The downsampled and clustered scene
		  @param clusters Vector of clusters found in the scene
			@return The name of the latest path
		*/
		std::string
		load_latest_single_segmentation_results(PointCloud_N::Ptr scene_original, PointCloud_N::Ptr scene, std::vector<PointCloud_N::Ptr> &clusters);

		/**
		  Loads original scene (single) with certain name specified by scene_name
		  @param scene_name Name of the segmentation folder in Data/Results/Segmentation_results
		  @param scene_original The original scene
		*/
		void
		load_original_scene_single (std::string scene_name, PointCloudT_color::Ptr scene_original);

		/**
		  Loads original merged scene with certain name specified by scene_name
		  @param scene_name Name of the segmentation folder in Data/Results/Segmentation_results
		  @param merged_scene_original The original scene
		*/
		void
		load_original_merged_scene (std::string scene_name, PointCloudT_color::Ptr merged_scene_original);

		/**
		  Loads latest merged segmentation data specified by latest_path
		  @param scene_original The original scene
		  @param scene The downsampled and clustered scene
		  @param clusters Vector of clusters found in the scene
			@return The name of the latest path
		*/
		std::string
		load_latest_merged_segmentation_results(PointCloud_N::Ptr scene_original, PointCloud_N::Ptr scene, std::vector<PointCloud_N::Ptr> &clusters);

    	/**
		  Saves the identification results for each cluster in each scene.
		  @param scene The scene name
		  @param cluster_index The cluster index
		  @param model_names The identified model names
		  @param scores_vec The view-scores of the identified models
		  @param index_queries_vec The view-index of the identified models
		  @param similar_models Vector containing the similar models
		*/
		void
		save_identification_results (	std::string scene,
						int cluster_index,
						std::vector<std::string> model_names,
						std::vector<std::vector<float> > scores_vec,
						std::vector<std::vector<int> > index_queries_vec,
						std::vector<std::string> similar_models );
										
		/**
		  Comparator used to sort Pose_Data objects
		  @param pose1 The first Pose_Data object
		  @param pose2 The second Pose_Data object
		*/
		static bool
		poses_comparator(const Pose_Data& pose1, const Pose_Data& pose2); 
		
		/**
		  Loads the valid pose resutls obtained from the hint system
		  @param scene_name The name of the scene
		  @param cluster_index The cluster index
		  @param pose_data[out] Vector of Pose_Data objects
		  @return 0 if there was valid pose data for the given scene and cluster and 1 otherwise
		*/
		int
		load_hint_system_valid_poses (std::string scene_name, int cluster_index, std::vector<Pose_Data> &pose_data);
		
		/**
		  Loads the previous camera positions
		  @param scene The name of the scene
		  @return The transformation of the camera position
		*/
		Eigen::Matrix<float,4,4,Eigen::DontAlign>
		load_previous_camera_position (std::string scene);

		/**
		  Save hint system results
		  @param scene The name of the scene
		  @param cluster_index The cluster index
		  @param valid_pose_results Vector containing each line of the CSV-file for the valid pose results
		  @param invalid_pose_results Vector containing each line of the CSV-file for the invalid pose results
		  @param weighted_sum_utilities Vector containing each line of the CSV-file for the weighted, summed utilities
		  @param trajectory Vector containing each line of the CSV-file for the trajectory
		  @param next_position Vector containing each line of the CSV-file for the new position of the robot arm
		*/
		void
		save_hint_system_results (	std::string scene,
									int cluster_index,
									std::vector<std::string> valid_pose_results,
									std::vector<std::string> invalid_pose_results,
									std::vector<std::string> weighted_sum_utilities,
									std::vector<std::string> trajectory,
									std::vector<std::string> next_position );

		/**
		  Save file as CSV
		  @param file_name The name of the file
		  @param str_vec Vector containing each line in the CSV-file
		*/
		void
		save_csv_file (std::string file_name, std::vector<std::string> str_vec);

		/**
		  Loads robot data from file and formats it into a Eigen::Matrix<float,4,4,Eigen::DontAlign> transformation
		  @param path The path to the data file (CSV)
		  @return The Eigen::Matrix<float,4,4,Eigen::DontAlign> transformation
		 */
		Eigen::Matrix<float,4,4,Eigen::DontAlign> 
		get_robot_data_matrix(std::string path);

		/**
		  Loads T_CtoH from CSV file and saves as Eigen::Matrix<float,4,4,Eigen::DontAlign>
		  @return The T_CtoH transformation
		*/
		Eigen::Matrix<float,4,4,Eigen::DontAlign> 
		get_T_CtoH();
		
		/**
		  Saves the current camera position
		  @param scene_name The name of the scene
		  @param T_HtoB The transformation from robot hand to robot base
		  @param T_CtoH The transformation from camera to robot hand
		*/
		void 
		save_current_camera_position (	std::string scene_name, 
										Eigen::Matrix<float,4,4,Eigen::DontAlign> T_HtoB,
										Eigen::Matrix<float,4,4,Eigen::DontAlign> T_CtoH );
		
		/**
		  Clears all results 
		*/
		void
		clear_results ();
};

#endif
