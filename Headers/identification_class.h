#ifndef IDENTIFICATION_CLASS_H_
#define IDENTIFICATION_CLASS_H_


#include <pcl/features/esf.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include "access_model_data.h"
#include "access_results.h"
#include "View_graph.h"

class Identification_Class
{
	public:
		typedef pcl::PointXYZ PointT;
		typedef pcl::PointNormal Point_N;
		typedef pcl::PointCloud<PointT> PointCloudT;
		typedef pcl::PointCloud<Point_N> PointCloud_N;
		typedef pcl::ESFSignature640 FeatureT;
		typedef pcl::ESFEstimation<Point_N, FeatureT> FeatureEstimationT;
		typedef pcl::PointCloud<FeatureT> FeatureCloudT;
		
		/**
		  Constructor
		*/
		Identification_Class (void);
		
		/**
		  Identifies an unknown cluster 
		  @param scene_name The name of the scene
		  @param cluster_index The cluster index
		  @param cluster_cloud The point cloud of the unknown cluster
		*/
		void
		identify (std::string scene_name, int cluster_index, PointCloud_N::Ptr cluster_cloud);
		
		/**
		  Set if program should show identification results
		  @param show_res True if show results, false otherwise
		*/
		void
		show_results (bool show_res);
		
		/**
		  Set if program should save identification results
		  @param save_identification_results True if save resutls, false otherwise
		*/
		void 
		set_save_results (bool save_identification_results);
	
	private:
	
		/*
		  Structure for sorting vectors with pcl::Correspondences
		*/
		struct less_than_key
		{
			inline bool operator() (const pcl::Correspondence corr1, const pcl::Correspondence corr2)
			{
				return (corr1.distance < corr2.distance);
			}
		};
		
		/*
		A structure for storing the data associated to each model
		*/
		struct model_data
		{
			std::string name;
			PointCloudT::Ptr complete_model;
			FeatureCloudT::Ptr feature_cloud;
			View_graph graph;
			std::vector<pcl::Correspondence> correspondences;

			bool operator < (const model_data &str) const
			{
				return (correspondences[0].distance < str.correspondences[0].distance);
			}
		};

		/*
		A structure for storing the data associated to the cluster
		*/
		struct cluster_data
		{
			std::string scene;
			int index;
			PointCloud_N::Ptr cloud;
			FeatureT feature;
			std::vector<std::string> similar_models;
		};
		
		bool show_res_;
		bool save_identification_results_;
		float VERY_GOOD_MATCH_;
		float GOOD_MATCH_;
		float MEDIUM_MATCH_;
		float SIMILAR_MATCH_;
		std::vector<model_data> models;
		
		/**
		  Loads all model data  
		*/
		void
		load_model_data ();
		
		/**
		  Estimates a ESF feature for a point cloud cluster
		  @param cluster The cluster object containing the point cloud
		*/
		void 
		estimate_feature (cluster_data &cluster);
		
		/**
		  Computes the l1-norm of the difference between two histograms
		  @param f1 The first histogram
		  @param f2 the second histogram 
		*/
		float 
		l1_norm (FeatureT f1, FeatureT f2);
		
		/**
		  Finds the best matching model features for a cluster
		  @param cluster The cluster object containing the point cloud
		*/
		void 
		best_matching_features (cluster_data &cluster);
		
		/**
		  Prints the identification results to terminal
		  @param cluster The cluster object containing the point cloud
		*/
		void 
		results_console (cluster_data cluster);
		
		/**
		  Shows the identification results
		  @param cluster The cluster object containing the point cloud
		*/
		void 
		results_viewer (cluster_data cluster);
		
		/**
		  Saves the identification results to file
		  @param cluster The cluster object containing the point cloud
		*/
		void
		save_results (cluster_data cluster);

};

#endif
