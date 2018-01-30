#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <signal.h>
#include "../headers/access_results.h"
#include "../headers/access_model_data.h"
#include "../headers/identification_class.h"
#include "../headers/manipulation.h"
#include "../headers/pose_class.h"
#include "../headers/linker.h"
#include "../headers/robot_data_fetcher.h"
#include "../headers/cloud_segmentation.h"
#include "../headers/hint_system_class.h"
#include "../headers/cloud_merging.h"
#include "../headers/rdf_connector.hpp"
#include "../headers/pose_data.h"
#include "../headers/config_reader.h"

typedef pcl::PointNormal Point_N;
typedef pcl::PointXYZ Point_XYZ;
typedef pcl::PointXYZRGB Point_RGBA;
typedef pcl::PointCloud<Point_N> Point_Cloud_N;
typedef pcl::PointCloud<Point_XYZ> Point_Cloud_XYZ;
typedef pcl::PointCloud<Point_RGBA> Point_Cloud_RGBA;

typedef pcl::visualization::PointCloudColorHandlerCustom<Point_N> ColorHandler_N;

int cloud_idx = 0;

int
main(int argc, char** argv) {
	Access_Results ar;
	Manipulation m;
	Access_Model_Data amd;
	Linker linker;
	bool merge = false;

	ar.clear_results();

	pcl::visualization::PCLVisualizer visu("Visu");

	Config_Reader conf;
	conf.system_load_config("../../config.ini");

	Pose_Class pc;
	pc.set_visualizer(&visu);
	pc.set_visualization_mode(conf.pose_visualization_mode);
	pc.set_print_mode(conf.pose_print_mode);

	Cloud_Segmentation cs;
	cs.set_visualizer(&visu);

	Hint_System_Class hs;
	float weights[] = {conf.hint_view_weight, conf.hint_normal_weight, conf.hint_feature_weight, conf.hint_distinguish_weight};
	hs.set_weights (weights);
	hs.set_below_plane_threshold(conf.hint_below_plane_threshold);
	hs.set_misalignment_angle_threshold(conf.hint_misalignment_angle_threshold);
	hs.set_view_search_results(conf.hint_view_search_results, conf.hint_normalize_search_results);
	hs.set_visualizer(&visu);
	hs.set_print_info(conf.hint_print_info);

	Cloud_Merging cm;
	cm.set_visualizer(&visu);

	Robot_Data_Fetcher fetcher;
	fetcher.set_auth_details(conf.web_service_username, conf.web_service_password);
	fetcher.set_url(conf.hand_web_service_url);

	RDF_Connector rdf;
	if(conf.rdf_enabled) {
		rdf.set_class_path("../../headers/rdf4j_connector.jar");
		rdf.set_class("Connector");
		rdf.initialize(conf.rdf_url, conf.rdf_repo_name);
	}

	if(boost::filesystem::exists(conf.save_path + "background.pcd")) {
		std::cout << "Background found, using it for segmentation!" << "\n";
		Point_Cloud_N::Ptr background (new Point_Cloud_N);
		m.load_cloud(conf.save_path + "background.pcd", background);
		Eigen::Matrix<float,4,4,Eigen::DontAlign> background_transformation = ar.get_robot_data_matrix(conf.save_path + "background.csv");
		Eigen::Matrix<float,4,4,Eigen::DontAlign> T_CtoH = ar.get_T_CtoH();
		cs.set_background_data(background, background_transformation, T_CtoH);
		std::cout << "Background data set!" << "\n\n";
	} else {
		std::cout << "No background found, using plane segmentation!" << "\n\n";
	}

	while(true) {
		// Wait for point cloud in share folder
		std::string scene_name = boost::lexical_cast<std::string>(cloud_idx);
		std::string original_cloud_path = conf.save_path + scene_name + ".pcd";
		std::string original_flag_path = conf.save_path + "." + scene_name + "_flag";
		std::string original_robot_data_path = conf.save_path + scene_name + ".csv";

		std::cout << "Waiting for cloud at " << original_cloud_path << '\n';
		while(!boost::filesystem::exists(original_cloud_path)) {}; // Hang until cloud is found
		while(!boost::filesystem::exists(original_flag_path)) {}; // Hang until flag is found
		std::cout << "Cloud found!" << '\n';
		std::cout << "Fetching robot data..." << '\n';
		fetcher.fetch_data(original_robot_data_path);
		std::cout << "Waiting for robot data at " << original_robot_data_path << '\n';
		while(!boost::filesystem::exists(original_robot_data_path)) {}; // Hang until robot data is found
		std::cout << "Robot data found!" << '\n';

		Eigen::Matrix<float,4,4,Eigen::DontAlign> cloud_trans = ar.get_robot_data_matrix(original_robot_data_path);
		Eigen::Matrix<float,4,4,Eigen::DontAlign> T_CtoH = ar.get_T_CtoH();
		ar.save_current_camera_position(scene_name, cloud_trans, T_CtoH);

		// Add scene and current position to RDF-DB
		if(conf.rdf_enabled) {
			std::cout << "Saving scene and current position to RDF" << std::endl;
			rdf.add_scene(cloud_idx);
			std::ostringstream oss;
			oss << cloud_trans;
			rdf.add_current_position(cloud_idx, oss.str());
		}

		// Run scene segmentation (uses background if it has been found)
		std::cout << "Segmenting cloud..." << '\n';
		Point_Cloud_RGBA::Ptr segm_cloud (new Point_Cloud_RGBA);
		m.load_cloud_rgba(original_cloud_path, segm_cloud);
		cs.set_cloud_transformation_matrix(cloud_trans);
		cs.segment(segm_cloud, scene_name, "single");

		// Copy robot data
		boost::filesystem::path latest_seg_path = ar.path_to_latest_segmentation_results();
		boost::filesystem::copy_file(original_robot_data_path, latest_seg_path.string() + "/robot_data.csv");
		std::cout << "Segmentation done!" << '\n';

		// Merge the previous scenes by supplying their saved path to scene merger, and then segment
		if(merge) {
			std::cout << "Merging clouds..." << '\n';

			std::vector<Point_Cloud_RGBA::Ptr> original_scenes;
			std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> > robot_data;
			ar.load_all_segmentation_results_original_scenes(original_scenes, robot_data);
			std::cout << "original_scenes: " << original_scenes.size() << std::endl;
			std::cout << "robot_data: " << robot_data.size() << std::endl;
			cm.merge(original_scenes, robot_data);

			std::cout << "Merging done!" << '\n';

			// Segment merged cloud
			std::cout << "Segmenting merged cloud..." << '\n';
			boost::filesystem::path seg_path = ar.path_to_segmentation_results();
			std::string merge_cloud_path = ar.path_to_segmentation_results().string() + "/" + scene_name + "/merged/scene_original.pcd";
			m.load_cloud_rgba(merge_cloud_path, segm_cloud);
			cs.segment(segm_cloud, scene_name, "merged");
		} else {
			// Copy over all data in single to merged
			boost::filesystem::create_directory(latest_seg_path.string() + "/merged");

			for(boost::filesystem::directory_iterator file(latest_seg_path.string() + "/single"); file != boost::filesystem::directory_iterator(); file++) {
				boost::filesystem::path current(file->path());
				boost::filesystem::copy_file(current, latest_seg_path.string() + "/merged/" + current.filename().string());
			}
		}

		// Load segmentation results
		Point_Cloud_N::Ptr scene_original (new Point_Cloud_N);
		Point_Cloud_N::Ptr scene (new Point_Cloud_N);
		std::vector<Point_Cloud_N::Ptr> single_clusters;
		ar.load_latest_single_segmentation_results(scene_original, scene, single_clusters);
		if(conf.rdf_enabled) {
			std::cout << "Saving clusters to RDF" << std::endl;
			for(int i = 0; i < single_clusters.size(); i++)
				rdf.add_cluster(cloud_idx, i);
		}

		// Run object identification on latest cloud
		std::cout << "Running object identification..." << '\n';
		Identification_Class ident;
		for(int i = 0; i < single_clusters.size(); i++) {
			ident.identify(scene_name, i, single_clusters[i]);
		}
		std::cout << "Object identification done!" << '\n';

		// Define the ID data for the current cloud
		std::vector<std::vector<std::string> > cluster_model_names; // cluster->model_name_index->model_name_string
		std::vector<std::vector<std::vector<int> > > cluster_model_view_indices; // cluster->model_name_index->view_indices
		std::vector<std::vector<std::vector<float> > > cluster_model_scores; // cluster->model_name_index->view_scores
		std::vector<Point_Cloud_N::Ptr> clusters; // This is always latest merged clusters

		// Define the previous pose data
		std::vector<std::vector<std::string> > prev_cluster_model_names;
		std::vector<std::vector<std::vector<int> > > prev_cluster_model_view_indices;
		std::vector<std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> > > prev_poses;
		std::vector<std::vector<double> > prev_inlier_fractions;
		std::vector<std::vector<double> > prev_accuracies;

		// Load and link identification data to merged clusters
		std::cout << "Loading latest single identification results and linking to latest merged clusters..." << '\n';

		// This is model data for the merged clusters of this cloud name
		clusters = linker.link_identification_data(ar, m, scene_name, scene_name, cluster_model_names, cluster_model_view_indices, cluster_model_scores);

		if(cloud_idx > 0) {
			std::string prev_name = boost::lexical_cast<std::string>(cloud_idx - 1);
			linker.link_pose_data(ar, m, prev_name, scene_name, prev_cluster_model_names, prev_cluster_model_view_indices, prev_poses, prev_inlier_fractions, prev_accuracies);
		}

		std::cout << "Loading and linking done!" << '\n';

		std::cout << "Identification results loaded!" << '\n';

		// Show linking results, what the single cloud has identified each merged cluster as
		// Loop through each merged cluster
		for(int c = 0; c < clusters.size(); c++) {
			std::cout << "Cluster " << c << "\n";
			// Loop through at most two model names
			std::cout << "\tLatest cloud identified this cluster as" << '\n';
			for(int idx = 0; idx < std::min((int)cluster_model_names[c].size(), 2); idx++) {
				std::cout << "\t\t" << cluster_model_names[c][idx];
				// Loop through at most two views for this model name
				for(int v = 0; v < 2; v++) {
					std::cout << ", " << cluster_model_view_indices[c][idx][v] << " (" << cluster_model_scores[c][idx][v] << ")";
				}
				std::cout << '\n';
			}

			if(merge) {
				std::cout << "\tPrevious merged cloud estimated these poses for this cluster" << '\n';
				for(int idx = 0; idx < std::min((int)prev_cluster_model_names[c].size(), 2); idx++) {
					std::cout << "\t\t" << prev_cluster_model_names[c][idx] << " (";
					for(int v = 0; v < prev_cluster_model_view_indices[c][idx].size(); v++) {
						std::cout << prev_cluster_model_view_indices[c][idx][v] << ",";
					}
					std::cout << ")" << '\n';
				}
			}

			std::cout << '\n';
		}

		// Merge views
		// Loop through each cluster
		for(int c = 0; c < clusters.size(); c++) {
			if(conf.use_centered_cluster) {
				if(!m.is_centered(clusters[c], conf.centered_threshold)) {
					std::cout << "Cluster " << c << " is not centered, skipping!" << std::endl;
					continue;
				}
			}

			std::cout << "\nFor cluster " << c << '\n';
			std::vector<Point_Cloud_N::Ptr> merged_views;
			std::vector<std::string> model_names;
			std::vector<std::vector<int> > model_view_indices;
			std::vector<Point_Cloud_N::Ptr> full_models;

			// Loop through all ID model names
			int n_total_merged_views = 0; // Keeps track of total number of views that have been merged
			for(int model_idx = 0; model_idx < cluster_model_names[c].size(); model_idx++) {
				std::string model_name = cluster_model_names[c][model_idx];
				std::cout << "\tID: " << model_name << '\n';

				// Loop through n_max_id_views ID views
				for(int v = 0; v < conf.max_id_views; v++) {
					int view_index_id = cluster_model_view_indices[c][model_idx][v];

					// If there are no pose results, this will just be skipped
					if(prev_cluster_model_names.size() == 0)
						continue;

					// Loop through all models from pose and find the ones which are same as model_name
					int n_merged_views = 0; // Counter to keep track of number of pose views that have been merged with ID views
					for(int i = 0; i < prev_cluster_model_names[c].size(); i++) {
						if(n_merged_views == conf.max_pose_views) // If enough views have been merged for this ID view, break
							break;
						if(n_total_merged_views == conf.max_merged_views) // If enough views have been merged in total, break. This will make sure no more views are merged
							break;

						if(prev_cluster_model_names[c][i] == model_name) {
							std::cout << "\t\t" << n_total_merged_views << ". View " << view_index_id << " (from ID) is merged with ";
							std::vector<int> merge_view_indices;
							merge_view_indices.push_back(view_index_id);

							// Loop through all views that are used in this pose
							for(int n = 0; n < prev_cluster_model_view_indices[c][i].size(); n++) {
								int prev_view_index = prev_cluster_model_view_indices[c][i][n];
								merge_view_indices.push_back(prev_view_index);

								std::cout << prev_view_index << "+";
							}
							std::cout << " (from pose)" << '\n';

							bool skip = false;
							for(int n = 0; n < model_view_indices.size(); n++) {
								if(model_view_indices[n] == merge_view_indices) {
									std::cout << "\t\t\tThis combination of views has already been used, skipping!" << std::endl;
									skip = true;
									break;
								}
							}
							if(skip)
								continue;

							Point_Cloud_N::Ptr merged_view (new Point_Cloud_N);
							amd.load_merged_views(model_name, merge_view_indices, merged_view);

							Point_Cloud_XYZ::Ptr full_model_xyz (new Point_Cloud_XYZ);
							Point_Cloud_N::Ptr full_model (new Point_Cloud_N);
							amd.load_complete_model(model_name, full_model_xyz);
							pcl::copyPointCloud(*full_model_xyz, *full_model);

							merged_views.push_back(merged_view);
							model_names.push_back(model_name);
							model_view_indices.push_back(merge_view_indices);
							full_models.push_back(full_model);

							n_merged_views++;
							n_total_merged_views++;
						}
					}
				}
			}

			// Not all slots have been filled
			if(merged_views.size() < conf.max_merged_views) {
				std::cout << "\n\tNot all slots have been filled, falling back to single ID views!\n" << std::endl;

				int n_total_merged_views = merged_views.size(); // Keeps track of total number of views that have been merged

				// Loop through all models from ID data
				for(int model_idx = 0; model_idx < cluster_model_names[c].size(); model_idx++) {
					std::string model_name = cluster_model_names[c][model_idx];

					// Check if model has already been merged with pose data
					bool model_name_already_merged = false;
					for(int merged_model_name_idx = 0; merged_model_name_idx < merged_views.size(); merged_model_name_idx++) {
						if(model_names[merged_model_name_idx] == model_name) {
							model_name_already_merged = true;
							break;
						}
					}

					// If model has not been merged, add single views from ID data
					if(!model_name_already_merged) {
						std::cout << "\tID: " << model_name << std::endl;
						for(int v = 0; v < conf.max_id_views; v++) {
							if(n_total_merged_views >= conf.max_merged_views)
								break;

							int view_index_id = cluster_model_view_indices[c][model_idx][v];
							std::cout << "\t\t" << n_total_merged_views << ". View " << view_index_id << " (from ID)\n";

							std::vector<int> merge_view_indices;
							merge_view_indices.push_back(view_index_id);

							Point_Cloud_N::Ptr merged_view (new Point_Cloud_N);
							amd.load_merged_views(model_name, merge_view_indices, merged_view);

							Point_Cloud_XYZ::Ptr full_model_xyz (new Point_Cloud_XYZ);
							Point_Cloud_N::Ptr full_model (new Point_Cloud_N);
							amd.load_complete_model(model_name, full_model_xyz);
							pcl::copyPointCloud(*full_model_xyz, *full_model);

							merged_views.push_back(merged_view);
							model_names.push_back(model_name);
							model_view_indices.push_back(merge_view_indices);
							full_models.push_back(full_model);

							n_total_merged_views++;
						}
					}

					if(n_total_merged_views >= conf.max_merged_views)
						break;
				}
			}

			// Visualize merged views (including ID fallback) - FOR DEBUGGING
			if(conf.visualize_merged_views) {
				for(int i = 0; i < merged_views.size(); i++) {
					std::cout << "Merged view " << i << '\n';
					visu.addPointCloud(merged_views[i], ColorHandler_N(merged_views[i], 0.0, 255.0, 0.0), "visible_object");
					visu.spin();
					visu.removeAllPointClouds();
				}
			}

			// This probably never happens?
			if(merged_views.size() == 0) {
				std::cout << "No poses merged! There was probably no linked ID data or pose data for this cluster" << '\n';
			}

			std::cout << "Estimating pose for cluster " << c << "..." << '\n';
			pc.estimate_pose_merged_views(merged_views, model_names, model_view_indices, clusters[c], scene_name, c, full_models);

			std::cout << "Running hint system..." << std::endl;
			//std::pair<std::vector<Pose_Data>, Eigen::Matrix<float,4,4,Eigen::DontAlign> > hint_results = hs.find_new_view(scene_name, c);
			Hint_System_Data hs_data = hs.find_new_view(scene_name, c);

			if (conf.rdf_enabled && hs_data.valid_results)
			{
				std::cout << "Saving pose results in RDF database..." << std::endl;
				for(int i = 0; i < hs_data.pd.size(); i++) 
				{
					Pose_Data d = hs_data.pd[i];
					std::ostringstream oss; 
					oss << d.transformation;
					rdf.add_model(cloud_idx, c, d.model_name, d.accuracy, oss.str());
				}

				std::ostringstream oss;
				oss << hs_data.next_position;
				rdf.add_next_position(cloud_idx, c, oss.str());
			}
		}

		merge = true;
		cloud_idx++;
	}
}
