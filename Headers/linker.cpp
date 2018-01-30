#include "linker.h"

bool
Linker::poses_comparator(const Pose_Data& pose1, const Pose_Data& pose2) {
	return pose1.inlier_fraction > pose2.inlier_fraction;
}

bool
Linker::id_comparator(const ID_Data& id1, const ID_Data& id2) {
	return id1.scores[0] < id2.scores[0];
}

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
std::vector<Linker::Point_Cloud_N::Ptr>
Linker::link_pose_data(Access_Results ar,
		Manipulation m,
		std::string prev_merged_name,
		std::string current_merged_name,
		std::vector<std::vector<std::string> > &model_names,
		std::vector<std::vector<std::vector<int> > > &view_indices,
		std::vector<std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> > > poses,
		std::vector<std::vector<double> > inlier_fractions,
		std::vector<std::vector<double> > accuracies) {

	std::vector<std::string> links; // Used for debugging
	Point_Cloud_N::Ptr scene_original (new Point_Cloud_N);
	Point_Cloud_N::Ptr scene (new Point_Cloud_N);
	std::vector<Point_Cloud_N::Ptr> prev_merged_clusters;
	std::vector<Point_Cloud_N::Ptr> current_merged_clusters;
	std::vector<std::vector<Pose_Data> > cluster_pose_data; // Object used to keep track of all pose data for each cluster

	// Get clusters
	ar.load_segmentation_results(prev_merged_name + "/merged", scene_original, scene, prev_merged_clusters);
	ar.load_segmentation_results(current_merged_name + "/merged", scene_original, scene, current_merged_clusters);
	model_names.resize(current_merged_clusters.size());
	view_indices.resize(current_merged_clusters.size());
	poses.resize(current_merged_clusters.size());
	inlier_fractions.resize(current_merged_clusters.size());
	accuracies.resize(current_merged_clusters.size());
	cluster_pose_data.resize(current_merged_clusters.size());

	// Get transformation matrices and compute transformation between clouds
	Eigen::Matrix<float,4,4,Eigen::DontAlign> T_CtoH = ar.get_T_CtoH();
	Eigen::Matrix<float,4,4,Eigen::DontAlign> T_prev_merged = ar.get_robot_data_matrix(ar.path_to_segmentation_results().string() + "/" + prev_merged_name + "/robot_data.csv");
	Eigen::Matrix<float,4,4,Eigen::DontAlign> T_current_merged = ar.get_robot_data_matrix(ar.path_to_segmentation_results().string() + "/" + current_merged_name + "/robot_data.csv");
	Eigen::Matrix<float,4,4,Eigen::DontAlign> T_prev_to_current = T_CtoH.inverse() * T_current_merged.inverse()*T_prev_merged * T_CtoH;

	// Loop through each cluster in prev_merged_clusters
	for(int i = 0; i < prev_merged_clusters.size(); i++) {
		// Transform the previous cluster to the current_merged_clusters coordinate system
		Point_Cloud_N::Ptr prev_cluster = prev_merged_clusters[i];
		pcl::transformPointCloud(*prev_cluster, *prev_cluster, T_prev_to_current);

		/*
		   std::vector<std::string> model_names_prev;
		   std::vector<std::vector<int> > view_indices_prev;
		   std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> > poses_prev;
		   std::vector<double> inlier_fractions_prev;
		   std::vector<double> accuracies_prev;
		   ar.load_pose_estimation_results(prev_merged_name, i, model_names_prev, view_indices_prev, poses_prev, inlier_fractions_prev, accuracies_prev);
		 */
		std::vector<Pose_Data> prev_pose_data;
		ar.load_pose_estimation_results_pose_format(prev_merged_name, i, prev_pose_data);

		// Loop through each cluster in current_merged_clusters
		for(int n = 0; n < current_merged_clusters.size(); n++) {
			// Compute inliers when fitting the previous cluster to the current cluster
			std::vector<int> inliers;
			std::vector<int> outliers;
			m.compute_inliers(current_merged_clusters[n], prev_merged_clusters[i], 0.001, &inliers, &outliers);
			double inlier_fraction = (double) inliers.size() / (double) prev_merged_clusters[i]->points.size();

			// If it fits, data for the current cluster should be equal to data for the previous cluster
			if(inlier_fraction > 0) {
				std::cout << "Linked pose data from previous cluster " << i << " to current cluster " << n << '\n';
				/*
				if(cluster_pose_data[n].size() > 0) {
					std::cout << "\tCurrent cluster " << n << " has already been merged before!!" << '\n';
				}
				*/

				cluster_pose_data[n].insert(cluster_pose_data[n].end(), prev_pose_data.begin(), prev_pose_data.end());

				/*
				   model_names[n].insert(model_names[n].end(), model_names_prev.begin(), model_names_prev.end());
				   view_indices[n].insert(view_indices[n].end(), view_indices_prev.begin(), view_indices_prev.end());
				   poses[n].insert(poses[n].end(), poses_prev.begin(), poses_prev.end());
				   inlier_fractions[n].insert(inlier_fractions[n].end(), inlier_fractions_prev.begin(), inlier_fractions_prev.end());
				   accuracies[n].insert(accuracies[n].end(), accuracies_prev.begin(), accuracies_prev.end());
				 */

				/*
				   model_names[n] = model_names_prev;
				   view_indices[n] = view_indices_prev;
				   poses[n] = poses_prev;
				   inlier_fractions[n] = inlier_fractions_prev;
				   accuracies[n] = accuracies_prev;
				 */

				std::ostringstream oss;
				oss << i << "->" << n;
				links.push_back(oss.str());

				break;
			}
		}
	}

	// Sort cluster_pose_data for each cluster based on inlier_fraction
	for(int c = 0; c < cluster_pose_data.size(); c++) {
		std::sort(cluster_pose_data[c].begin(), cluster_pose_data[c].end(), poses_comparator);
	}

	// Move data back into vectors
	for(int c = 0; c < cluster_pose_data.size(); c++) {
		std::cout << "Linked pose data from cluster " << c << '\n';
		std::vector<std::string> m_n;
		std::vector<std::vector<int> > v_i;
		std::vector<Eigen::Matrix<float,4,4,Eigen::DontAlign> > p;
		std::vector<double> i_f;
		std::vector<double> acc;
		for(int i = 0; i < cluster_pose_data[c].size(); i++) {
			std::cout << "\t" << cluster_pose_data[c][i].model_name << ",";
			for(int n = 0; n < cluster_pose_data[c][i].view_indices.size(); n++) {
				std::cout << cluster_pose_data[c][i].view_indices[n] << "+";
			}
			std::cout << "," << cluster_pose_data[c][i].inlier_fraction << "," << cluster_pose_data[c][i].accuracy << '\n';

			m_n.push_back(cluster_pose_data[c][i].model_name);
			v_i.push_back(cluster_pose_data[c][i].view_indices);
			p.push_back(cluster_pose_data[c][i].transformation);
			i_f.push_back(cluster_pose_data[c][i].inlier_fraction);
			acc.push_back(cluster_pose_data[c][i].accuracy);
		}

		model_names[c] = m_n;
		view_indices[c] = v_i;
		poses[c] = p;
		inlier_fractions[c] = i_f;
		accuracies[c] = acc;
	}

	// Write link data (for debugging purposes)
	std::ofstream ofs;
	std::string p = ar.path_to_segmentation_results().string() + "/" + current_merged_name + "/merged/pose_link_" + prev_merged_name;
	std::cout << "WRITING POSE LINK DEBUG DATA TO " << p << '\n';
	ofs.open(p.c_str());
	for(int i = 0; i < links.size(); i++) {
		ofs << links[i] << "\n";
	}
	ofs.close();

	return current_merged_clusters;
}

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
std::vector<Linker::Point_Cloud_N::Ptr>
Linker::link_identification_data(Access_Results ar,
								Manipulation m,
								std::string single_name,
								std::string merged_name,
								std::vector<std::vector<std::string> > &models_single_in_merged,
								std::vector<std::vector<std::vector<int> > > &model_view_indices_single_in_merged,
								std::vector<std::vector<std::vector<float> > > &model_scores_single_in_merged) {

	std::vector<std::string> links; // Used for debugging
	Point_Cloud_N::Ptr scene_original (new Point_Cloud_N);
	Point_Cloud_N::Ptr scene (new Point_Cloud_N);
	std::vector<Point_Cloud_N::Ptr> single_clusters;
	std::vector<Point_Cloud_N::Ptr> merged_clusters;
	std::vector<std::vector<ID_Data> > cluster_id_data; // Object used to keep track of all ID data for each cluster

	// Get clusters
	ar.load_segmentation_results(single_name + "/single", scene_original, scene, single_clusters);
	ar.load_segmentation_results(merged_name + "/merged", scene_original, scene, merged_clusters);
	models_single_in_merged.resize(merged_clusters.size());
	model_view_indices_single_in_merged.resize(merged_clusters.size());
	model_scores_single_in_merged.resize(merged_clusters.size());
	cluster_id_data.resize(merged_clusters.size());

	// Get transformation matrices and compute transformation between clouds
	Eigen::Matrix<float,4,4,Eigen::DontAlign> T_CtoH = ar.get_T_CtoH();
	Eigen::Matrix<float,4,4,Eigen::DontAlign> T_single = ar.get_robot_data_matrix(ar.path_to_segmentation_results().string() + "/" + single_name + "/robot_data.csv");
	Eigen::Matrix<float,4,4,Eigen::DontAlign> T_merged = ar.get_robot_data_matrix(ar.path_to_segmentation_results().string() + "/" + merged_name + "/robot_data.csv");
	Eigen::Matrix<float,4,4,Eigen::DontAlign> T_single_to_merged = T_CtoH.inverse() * T_merged.inverse()*T_single * T_CtoH;

	// Loop through each cluster in single_clusters
	for(int i = 0; i < single_clusters.size(); i++) {

		// Transform the single cluster to the merged_clusters coordinate system
		Point_Cloud_N::Ptr single_cluster = single_clusters[i];
		pcl::transformPointCloud(*single_cluster, *single_cluster, T_single_to_merged);

		std::vector<ID_Data> id_data;
		ar.load_identification_results_id_data_format(single_name, i, id_data);

		// Loop through each cluster in merged clusters
		for(int n = 0; n < merged_clusters.size(); n++) {
			// Compute inliers when fitting the single cluster into the merged cluster
			std::vector<int> inliers;
			std::vector<int> outliers;
			m.compute_inliers(merged_clusters[n], single_clusters[i], 0.001, &inliers, &outliers);
			double inlier_fraction = (double) inliers.size() / (double) single_clusters[i]->points.size();

			// If it fits, data for the merged cluster should be equal to data for the single cluster
			if(inlier_fraction > 0) {
				std::cout << "Linked ID data from single cluster " << i << " to merged cluster " << n << '\n';
				cluster_id_data[n].insert(cluster_id_data[n].end(), id_data.begin(), id_data.end());

				std::ostringstream oss;
				oss << i << "->" << n;
				links.push_back(oss.str());

				break;
			}
		}
	}

	// Sort ID data
	for(int c = 0; c < cluster_id_data.size(); c++) {
		std::sort(cluster_id_data[c].begin(), cluster_id_data[c].end(), id_comparator);
	}

	// Move data back into vectors
	for(int c = 0; c < cluster_id_data.size(); c++) {
		std::cout << "Linked ID data for cluster " << c << '\n';
		std::vector<std::string> m_n;
		std::vector<std::vector<int> > v_i;
		std::vector<std::vector<float> > s;
		for(int i = 0; i < cluster_id_data[c].size(); i++) {
			std::cout << "\t" << cluster_id_data[c][i].model_name << ",";
			for(int n = 0; n < cluster_id_data[c][i].view_indices.size(); n++) {
				std::cout << cluster_id_data[c][i].view_indices[n] << "," << cluster_id_data[c][i].scores[n] << ",";
			}
			std::cout << '\n';

			m_n.push_back(cluster_id_data[c][i].model_name);
			v_i.push_back(cluster_id_data[c][i].view_indices);
			s.push_back(cluster_id_data[c][i].scores);
		}

		models_single_in_merged[c] = m_n;
		model_view_indices_single_in_merged[c] = v_i;
		model_scores_single_in_merged[c] = s;
	}

	// Write link data (for debugging purposes)
	std::ofstream ofs;
	std::string p = ar.path_to_segmentation_results().string() + "/" + merged_name + "/merged/id_link_" + single_name;
	std::cout << "WRITING ID LINK DEBUG DATA TO " << p << '\n';
	ofs.open(p.c_str());
	for(int i = 0; i < links.size(); i++) {
		ofs << links[i] << "\n";
	}
	ofs.close();

	return merged_clusters;
}
