#ifndef HINT_SYSTEM_CLASS_H
#define HINT_SYSTEM_CLASS_H

#include "access_results.h"
#include "access_model_data.h"
#include "View_graph.h"
#include "pose_data.h"
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "hint_system_data.h"

class Hint_System_Class
{
	public:
	
		/** 
		  Constructor. 
		*/
	  	Hint_System_Class (void);
	  	
	  	/**
	  	  Sets the weights used for summing up the utilities
	  	  @param weights Array containing the weights. Default is equal weights.
	  	*/
	  	void
	  	set_weights (float weights[4]);
	  	
	  	/**
		  Sets the below plane threshold
		  @param value The value of the threshold in meters
		*/
		void
		set_below_plane_threshold (double value);

		/**
		  Sets the misalignment angle threshold
		  @param value The value of the threshold in degrees
		*/
		void
		set_misalignment_angle_threshold (double value);
		
		/**
		  Sets the valid axis threshold angle which determines if a node is near a previous camera view.
		  @param value The value of the threshold in degrees
		*/
		void
		set_valid_axis_threshold (float value);
	  	
	  	/**
	  	  Set if the valid nodes int the graph should be shown
	  	  @param view_valid_nodes Set this to true to show the valid nodes
	  	*/
	  	void 
	  	set_view_valid_nodes (bool view_valid_nodes);
			  	
		/**
		  Sets if the results of the search should be shown 
		  @param view_search_results Set this to true to show the results
		  @param view_search_results Set this to true to normalize the weighted sum of the utilities
		*/
		void
		set_view_search_results (bool view_search_results, bool normalize_search_results);
		
		/**
		  Pass true to show info in terminal during execution
		  @param print_info Set this to true to show info in terminal
		*/
		void
		set_print_info (bool print_info);
		
		/**
		  Pass true to show info about the utilities in the graph
		  @param print_info Set this to true to show utilities
		*/
		void
		set_show_utilities (bool show_utilities);
		
		/**
		  Set the visualizer object to be used when visualizing
		  @param visu The visualizer pointer
		 */
		void
		set_visualizer(pcl::visualization::PCLVisualizer* visu); 
	  	
		/**
		  Finds a new view given an identified scene.  
		  @param cluster_index The index of the cluster of the scene
		  @return The vector containing all identified models with their best pose and the next position for the robot hand
		*/
		//std::pair<std::vector<Pose_Data>, Eigen::Matrix<float,4,4,Eigen::DontAlign> >
		Hint_System_Data
		find_new_view (std::string scene_name, int cluster_index);
	
	private:
	
		typedef pcl::PointXYZ PointT;
		typedef pcl::PointNormal Point_N;
		typedef pcl::PointXYZRGB PointT_color;
		typedef pcl::PointCloud<PointT> PointCloudT;
		typedef pcl::PointCloud<Point_N> PointCloud_N;
		typedef pcl::PointCloud<PointT_color> PointCloudT_color;
		
		/**
		  A structure for storing the pose estimation data 
		*/
		struct Pose_estimation_results
		{
			std::string name;
			std::vector<int> identified_views; // Current view is at index 0
			int initial_view;
			Eigen::Matrix<float,4,4,Eigen::DontAlign> pose;
			double inlier_fraction;
			double accuracy;
			PointT model_center;
			View_graph graph;
			bool invalid_pose_position;
			bool low_inlier_fraction;
			
			Pose_estimation_results () 
		    {
		    	invalid_pose_position = false;
		    	low_inlier_fraction = false;
		    }
		};
		
		bool view_valid_nodes_;
		bool view_search_results_;
		bool normalize_search_results_;
		bool print_info_;
		bool show_utilities_;
		float weights_[4];
		double BELOW_PLANE_THRESHOLD;
		double PI_CONST;
		double MISALIGNMENT_ANGLE_THRESHOLD;
		double Z_AXIS_MISALIGNMENT_THRESHOLD;
		float VALID_AXIS_THR;
		std::vector<float> view_utilities_;
		std::vector<float> feature_utilities_;
		std::vector<float> normal_utilities_;
		std::vector<float> distinguish_utilities_;
		std::vector<float> weighted_sum_utilities_;
		std::vector<int> trajectory_;
		int better_view_;
		int next_view_;
		Eigen::Matrix<float,4,4,Eigen::DontAlign> next_position_;
		Eigen::Matrix<float,4,4,Eigen::DontAlign> optimal_position_;
		std::vector<Pose_estimation_results> similar_models_;
		Eigen::Matrix<float,4,4,Eigen::DontAlign> T_HtoB;
		Eigen::Matrix<float,4,4,Eigen::DontAlign> T_CtoH;
		Eigen::Matrix<float,4,4,Eigen::DontAlign> current_position;
		std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> > previous_positions;
		pcl::visualization::PCLVisualizer* visu_ptr;
		bool visu_ptr_set;
		//PointT cluster_center;
		std::vector<std::pair<std::string, PointT> > model_centers;
		
		/**
		  Checks if the complete model has been loaded for the given model name
		  @param model_name The name of the model
		  @return A pair object where the first value indicates if the complete model has been loaded and the second value contains the center of mass of the model (if any)
		*/
		std::pair<bool, PointT>
		complete_model_loaded (std::string model_name);
		
		/**
		  Loads the complete point cloud model and computes the center of mass
		  @param model_name The name of the model
		  @param pose The estimated pose
		  @return The center of mass of the complete model
		*/
		PointT
		compute_model_center (std::string model_name, Eigen::Matrix<float,4,4,Eigen::DontAlign> pose);
		
		/**
		  Loads the pose estimation results 
		  @param scene_name The name of the scene
		  @param cluster_index The cluster index of the scene
		  @param access_results Object for Retrieving the result data
		  @param pose_results Vector containing all the data from pose estimation
		*/
		int
		load_pose_estimation_results (	std::string scene_name, 
										int cluster_index, 
										Access_Results access_results, 
										std::vector<Pose_estimation_results> &pose_results );
										
		/**
		  Loads the plane from the scene
		  @param scene_name The name of the scene
		  @return The plane coefficients
		*/
		pcl::ModelCoefficients
		load_plane (std::string scene_name);
		
		/**
		  Views the valid nodes in the graph. Valid nodes are nodes that are above the plane in the scene
		  @param scene_name The name of the scene
		  @param per The pose estimation result
		  @param plane The plane coefficients
		*/
		void
		view_valid_nodes (	std::string scene_name, 
							Pose_estimation_results per, 
							pcl::ModelCoefficients plane );
		
		/**
		  Checks if the complete model with the given transformation is below the plane
		  @param per The pose estimation result
		  @param plane The plane coefficients
		  @return True if the model is below the plane and false otherwise
		*/
		bool
		model_below_plane (Pose_estimation_results per, pcl::ModelCoefficients plane);
		
		/**
		  Aligns the camera z-axis towards the cluster center of mass
		  @param camera_center The camera center
		  @param model_center The model center of mass
		  @return The correctly aligned z-axis viewpoint
		*/
		Eigen::Vector3f
		align_axis_to_cluster_center (PointT camera_center, PointT model_center);
		
		/**
		  Finds the views that best aligns with the current and previous camera axes
		  @param per The pose estimation result
		*/
		void
		find_aligned_views (Pose_estimation_results &per);
		
		/**
		  Computes the inlier fraction of the pose. (Source code from pcl::SampleConsensusPrerejective::getFitness)
		  @param per The pose estimation result
		*/
		double
		get_inlier_fraction (PointCloud_N::Ptr source, PointCloud_N::Ptr target);
		
		/**
		  Recomputes the inlier fraction for the new views. If the recomputed inlier fraction is too low then the pose result is marked as invalid.
		  @param per The pose estimation result
		  @param scene_name The name of the scene
		  @param cluster_index The cluster index
		  @return True if the model has low inlier fraction and false otherwise
		*/
		bool
		low_inlier_fraction (Pose_estimation_results &per, std::string scene_name, int cluster_index);
		
		/**
		  Checks if identified views and poses is in agreement with the camera position and viewpoint
		  @param scene_name The name of the scene
		  @param cluster_index The cluster index
		  @param pose_results Vector containing all the data from pose estimation
		  @param valid_pose_results Vector containing all valid results from pose estimation
		  @param invalid_pose_results Vector containing all invalid results from pose estimation
		*/
		int
		camera_validation_test (	std::string scene_name,
									int cluster_index,
									std::vector<Pose_estimation_results> &pose_results, 
									std::vector<Pose_estimation_results> &valid_pose_results,
									std::vector<Pose_estimation_results> &invalid_pose_results );
									
		/**
		  Comparator used to sort Pose_Data objects
		  @param pose1 The first Pose_Data object
		  @param pose2 The second Pose_Data object
		*/
		static bool
		poses_comparator (const Pose_estimation_results& per1, const Pose_estimation_results& per2);
		
		/**
		  Comparator used to sort vectors with Pose_estimation_results objects
		  @param vec1 The first vector with Pose_estimation_results object
		  @param vec2 The second vector with Pose_estimation_results object
		*/
		static bool
		poses_comparator_2 (const std::vector<Pose_estimation_results>& vec1, const std::vector<Pose_estimation_results>& vec2);
								
		/**
		  Sorts the valid pose results according to the model names
		  @param valid_pose_results Vector containing the valid pose results
		  @param return_vec Vector with all identified models and their best poses
		  @return vector containing all the (similar) models
		*/
		std::vector<std::string>
		sort_valid_pose_results (std::vector<Pose_estimation_results> &valid_pose_results, std::vector<Pose_Data> &return_vec);

	  	/**
	  	  Normalizes the vector between 1.0 and 0.0  
	  	  @param vec The vector to normalize
	  	*/
		void 
		normalize (std::vector<float> &vec);
		
	  	/**
	  	  Combines the distinguish-utilities between similar models. Each view is assigned the maximum utility between the similar models. 
	  	  @param distinguish_utilities_vec The distinguish-utilities vector for the similar_models
	  	  @return Vector containing the combined and normalized distinguish-utilities
	  	*/
		std::vector<float> 
		combine_distinguish_utilities (std::vector<std::vector<float> > distinguish_utilities_vec);
		
		/**
		  Computes a weighted sum of all utilities. If no similar models were detected, the distinguish-utility-weight is split and added equaly 
		  to the view-utility -and feature-utility-weight.
		  @return Vector containing the summed utilities
		*/
		std::vector<float>
		sum_utilities ();
		
		/**
		  Computes the next and optimal positions in robot base coordinate system
		  @param graph The view-graph
		  @param scene_name the name of the scene
		*/
		void
		compute_new_positions (View_graph graph, std::string scene_name);
		
		/**
		  Loads the current hand position in robot base coordinates
		  @param scene_name The name of the scene
		*/
		void
		load_T_HtoB (std::string scene_name);
		
		/**
		  Loads the previous camera positions in robot base coordinates and transforms them to camera base coordinates
		  @param scene_name The name of the scene
		*/
		void
		load_previous_camera_position (std::string scene_name);
		
		/**
		  Computes the current camera position in robot base coordinates
		*/
		void
		compute_current_camera_position ();
		
		/**
		  Loads and computes the cluster center of mass
		  @param scene_name The name of the scene
		  @param cluster_index The cluster index
		/
		void 
		cluster_center_of_mass (std::string scene_name, int cluster_index);
		*/
		
		/**
		  Adds the segmentation results to the viewer
		  @param viewer The visualization object
		  @param scene_name The name of the scene
		  @param per The pose estimation results 
		*/
		void
		add_segmentation_results_to_viewer (pcl::visualization::PCLVisualizer &viewer, std::string scene_name, Pose_estimation_results per);
		
		/**
		  Views the results of the search. In the first viewer, the red node is the starting view-node and the green node is the better view-node. 
		  Blue nodes are nodes that have been visited when searching for a better node. The different utilities are shown in the second viewer.
		  A green node indicates a high utility while a red node indicates a low utility. The original scene with hint-info is shown in the third viewer.   
		  @param scene_name The name of the scene
		  @param per The pose estimation results
		*/
		void
		view_search_results (std::string scene_name, Pose_estimation_results &per);
		
		/**
		  Saves the current camera position
		*/
		void 
		save_current_camera_position (std::string scene_name);
		
		/**
		  Saves the results
		  @param scene_name The name of the scene
		  @param cluster_index The cluster index
		  @param valid_pose_results The valid pose results
		  @param invalid_pose_results The invalid pose results
		  @param flag A flag telling the function which variables to save
		*/
		void
		save_results ( 	std::string scene_name, 
						int cluster_index, 
						std::vector<Pose_estimation_results> valid_pose_results, 
						std::vector<Pose_estimation_results> invalid_pose_results,
						int flag );
	
};

#endif
