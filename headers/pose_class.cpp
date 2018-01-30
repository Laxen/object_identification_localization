#include "pose_class.h"

Pose_Class::Pose_Class() {
	LEAF_SIZE = 0.003; // Used for downsampling (0.003 for small objects, 0.010 for large)

	POSE_MAX_ITERATIONS = 10000; // THIS WAS 25000 BEFORE 2017-11-06
	POSE_MAX_CORRESPONDENCE_DISTANCE = 1.5 * LEAF_SIZE; // THIS WAS 1.1 BEFORE 2017-11-06
	POSE_CORRESPONDENCE_RANDOMNESS = 5;
	POSE_SIMILARITY_THRESHOLD = 0.8;
	POSE_INLIER_FRACTION = 0.1;
	POSE_INVERSE_INLIER_FRACTION = 0.5;

	FEATURE_RADIUS_SEARCH = 0.015; // Radius for feature estimation (was 0.010 before 2017-11-28)

	visualization_mode = 0;
	print_mode = 0;
}

/**
	Computes features for a cloud
	@param cloud The cloud to compute features for
	@return The feature cloud
 */
Pose_Class::Feature_Cloud::Ptr
Pose_Class::compute_features(Point_Cloud_N::Ptr cloud) {
	Feature_Cloud::Ptr features (new Feature_Cloud);
	pcl::FPFHEstimationOMP<Point_N, Point_N, Feature> feature_estimator;

	feature_estimator.setRadiusSearch(FEATURE_RADIUS_SEARCH);
	feature_estimator.setInputCloud(cloud);
	feature_estimator.setInputNormals(cloud);
	feature_estimator.compute(*features);

	return features;
}

/**
	Estimates the pose of an object in a scene
	@param object The object to estimate pose for
	@param scene The scene to find the object in
	@param object_features The features for the object
	@param scene_features The features for the scene
	@param transformation[out] The transformation (pose) of the object
	@param all_transformations[out] Vector with all transformations with inliers above inlier_fraction
	@return Vector of all inliers
 */
std::vector<int>
Pose_Class::estimate_pose(Point_Cloud_N::Ptr object, Point_Cloud_N::Ptr scene, Feature_Cloud::Ptr object_features, Feature_Cloud::Ptr scene_features, Eigen::Matrix<float,4,4,Eigen::DontAlign>* transformation,
		int max_iterations, int correspondence_randomness, double similarity_threshold, double max_correspondence_distance, double inlier_fraction, std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> >* all_transformations) {

	if(print_mode == 1)
		pcl::console::print_highlight("Estimating pose...\n");
	Point_Cloud_N::Ptr object_aligned (new Point_Cloud_N);

	if(print_mode == 1)
		pcl::console::print_info("\t Starting alignment...\n");
	pcl::SampleConsensusPrerejective<Point_N, Point_N, Feature> aligner;
	aligner.setInputSource(object);
	aligner.setSourceFeatures(object_features);
	aligner.setInputTarget(scene);
	aligner.setTargetFeatures(scene_features);
	aligner.setMaximumIterations(max_iterations);
	aligner.setNumberOfSamples(3);
	aligner.setCorrespondenceRandomness(correspondence_randomness);
	aligner.setSimilarityThreshold(similarity_threshold);
	aligner.setMaxCorrespondenceDistance(max_correspondence_distance);
	aligner.setInlierFraction(inlier_fraction);
	aligner.setNumberOfTransformations(10);
	if(print_mode == 1) {
		pcl::ScopeTime t("\t Alignment");
		aligner.align (*object_aligned);
	} else {
		aligner.align (*object_aligned);
	}

	aligner.getTransformations(all_transformations);

	*transformation = aligner.getFinalTransformation();
	if(aligner.hasConverged()) {
		if(print_mode == 1)
			pcl::console::print_info("\t Alignment successful!\n");
		return aligner.getInliers();
	} else {
		if(print_mode == 1)
			pcl::console::print_info("\t Alignment failed!\n");
		return aligner.getInliers();
	}
}

/**
	Matches an object to clusters OR clusters to object and returns a vector of vector of poses of the object in each cluster
	@param object The object to estimate pose of
	@param object_features The feature cloud of the object
	@param clusters Vector of clusters
	@param object_to_cluster If true, matches object to cluster, otherwise matches clusters to object
	@param transformations[out] Vector of vector of transformations for each cluster
 */
void
Pose_Class::estimate_poses_for_clusters(Point_Cloud_N::Ptr object, Feature_Cloud::Ptr object_features, std::vector<Point_Cloud_N::Ptr> clusters, bool object_to_cluster, std::vector<std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > >* transformations) {
	for(int i = 0; i < clusters.size(); i++) {
		Point_Cloud_N::Ptr cluster = clusters[i];
		if(cluster->width == 0) continue;

		if(print_mode == 1)
			pcl::console::print_highlight("\nComputing features for cluster %i\n", i);
		Feature_Cloud::Ptr cluster_features = compute_features(cluster);

		// Estimate pose for cluster
		Eigen::Matrix<float,4,4,Eigen::DontAlign> transformation = Eigen::Matrix<float,4,4,Eigen::DontAlign>::Identity();
		std::vector<int> inliers;
		std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > all_transformations;
		if(object_to_cluster) {
			inliers = estimate_pose(object, cluster, object_features, cluster_features, &transformation, POSE_MAX_ITERATIONS, POSE_CORRESPONDENCE_RANDOMNESS, POSE_SIMILARITY_THRESHOLD, POSE_MAX_CORRESPONDENCE_DISTANCE, POSE_INLIER_FRACTION, &all_transformations);
		} else {
			inliers = estimate_pose(cluster, object, cluster_features, object_features, &transformation, POSE_MAX_ITERATIONS, POSE_CORRESPONDENCE_RANDOMNESS, POSE_SIMILARITY_THRESHOLD, POSE_MAX_CORRESPONDENCE_DISTANCE, POSE_INVERSE_INLIER_FRACTION, &all_transformations);

			// Invert transformations
			for(int n = 0; n < all_transformations.size(); n++) {
				all_transformations[n].first = all_transformations[n].first.inverse().eval();
			}
			transformation = transformation.inverse().eval();
		}


		(*transformations)[i] = all_transformations;
	}
}

/**
	Match clusters to object, and match object to passed clusters
	@param object The object to match with
	@param object_features Feature cloud for the object
	@param cluster Vector of all clusters
	@param transformations[out] Vector of vector of transformations for each cluster
 */
void
Pose_Class::match_cluster_to_object_to_cluster(Point_Cloud_N::Ptr object, Feature_Cloud::Ptr object_features, std::vector<Point_Cloud_N::Ptr> clusters, std::vector<std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > >* transformations) {
	// Match clusters to object and filter out passed clusters
	std::vector<std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > > cluster_to_object_transformations (clusters.size());
	estimate_poses_for_clusters(object, object_features, clusters, false, &cluster_to_object_transformations);

	std::vector<Point_Cloud_N::Ptr> passed_clusters;
	for(int i = 0; i < cluster_to_object_transformations.size(); i++) {
		std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > cluster_transformations = cluster_to_object_transformations[i];
		if(cluster_transformations.size() > 0) {
			passed_clusters.push_back(clusters[i]);
		} else {
			Point_Cloud_N::Ptr temp_cloud (new Point_Cloud_N);
			passed_clusters.push_back(temp_cloud);
		}

		if(print_mode == 1)
			std::cout << "\nPoses in cluster " << i << ": " << cluster_transformations.size() << '\n';
	}

	// Match object to clusters
	std::vector<std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > > object_to_cluster_transformations (passed_clusters.size());
	estimate_poses_for_clusters(object, object_features, passed_clusters, true, &object_to_cluster_transformations);

	*transformations = object_to_cluster_transformations;
}

/**
	Takes transformations from each scene cluster, and puts them in pose clusters
	@param transformations Poses to be clustered. Rows = Scene clusters, Cols = (tr,inl) pairs for that scene cluster.
	@param clustered_poses[out] Rows = Pose clusters, Cols = (tr,inl) for that pose cluster. First col in each row contains (tr_avg, inl_avg) for that pose cluster.
 */
void
Pose_Class::cluster_poses(std::vector<std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > >* transformations, std::vector<std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > >* clustered_poses) {
	double closeness_threshold = 0.7;

	// Loop through all clusters
	for(int i = 0; i < transformations->size(); i++) {
		std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > cluster_transformations = (*transformations)[i];

		// Loop through transformations in this cluster
		for(int n = 0; n < cluster_transformations.size(); n++) {
			std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> pose = cluster_transformations[n];

			// Loop through all pose clusters to either add transformation to a pose cluster, or create new pose cluster
			bool added_to_cluster = false;
			for(int k = 0; k < clustered_poses->size(); k++) {
				Eigen::Matrix<float,4,4,Eigen::DontAlign> mean = (*clustered_poses)[k][0].first;
				Eigen::Matrix<float,4,4,Eigen::DontAlign> diff = pose.first - mean;

				// If this transformation is close to a cluster of poses, add it to that cluster
				if(diff.norm() < closeness_threshold) {
					(*clustered_poses)[k].push_back(pose);

					// Recompute mean
					Eigen::Matrix<float,4,4,Eigen::DontAlign> cluster_mean_pose = (*clustered_poses)[k][1].first;
					double cluster_mean_inlier_fraction = (*clustered_poses)[k][1].second;
					for(int j = 2; j < (*clustered_poses)[k].size(); j++) {
						cluster_mean_pose += (*clustered_poses)[k][j].first;
						cluster_mean_inlier_fraction += (*clustered_poses)[k][j].second;
					}
					cluster_mean_pose /= (*clustered_poses)[k].size() - 1;
					cluster_mean_inlier_fraction /= (*clustered_poses)[k].size() - 1;
					(*clustered_poses)[k][0].first = cluster_mean_pose;
					(*clustered_poses)[k][0].second = cluster_mean_inlier_fraction;

					added_to_cluster = true;
					break;
				}
			}

			// If this transformation was not added to a cluster, create a new cluster
			if(!added_to_cluster) {
				std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > temp;
				temp.push_back(pose); // Add the mean
				temp.push_back(pose); // Add the pose
				clustered_poses->push_back(temp);
			}
		}
	}
}

/**
	Takes the average pose of each pose cluster, compares it to the other average poses and computes an accuracy based on a couple of parameters between them.
	Saves the result as a new vector with ((pose_avg, inliers_avg), accuracy)
	@param clustered_poses The clustered poses (pose, inliers)
	@param filtered_poses[out] The filtered poses ((pose, inliers), accuracy)
 */
void
Pose_Class::filter_clustered_poses(	std::vector<std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > >* clustered_poses, 
		std::vector<std::pair<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double>, double> >* filtered_poses) {
	double closeness_threshold = 2.0;

	// Loop through all pose clusters
	for(int i = 0; i < clustered_poses->size(); i++) {
		// Get average pose and average inlier fraction
		Eigen::Matrix<float,4,4,Eigen::DontAlign> pose = (*clustered_poses)[i][0].first;
		double pose_inlier_fraction = (*clustered_poses)[i][0].second;

		double accuracy = 1.0;
		int n_close_clusters = 0;

		// Loop through all pose clusters except current pose cluster
		for(int n = 0; n < clustered_poses->size(); n++) {
			if(n == i) continue;

			// Check current average pose with other pose clusters average pose
			Eigen::Matrix<float,4,4,Eigen::DontAlign> diff = pose - (*clustered_poses)[n][0].first;
			if(diff.norm() < closeness_threshold) {
				// If inlier fraction is less than 85% of current pose inlier fraction, closeness doesn't count
				if((*clustered_poses)[n][0].second < pose_inlier_fraction * 0.85)
					continue;

				// Linear between 0.5 and 1 depending on how close the poses are
				accuracy += closeness_threshold / (closeness_threshold + diff.norm());

			}
		}

		// Accuracy based on inliers
		accuracy *= pose_inlier_fraction;

		// Take average pose, average inliers and pose accuracy and push it to filtered_poses
		std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> pose_inlier = std::make_pair(pose, pose_inlier_fraction);
		std::pair<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double>, double> pose_inlier_accuracy = std::make_pair(pose_inlier, accuracy);
		filtered_poses->push_back(pose_inlier_accuracy);
	}
}

/**
	Finds points with persistent features in cloud
	@param cloud The cloud to find persistent features in
	@param output_indices The indices of the points with persistent features
 */
void
Pose_Class::compute_persistent_features(Point_Cloud_N::Ptr cloud, boost::shared_ptr<std::vector<int> > output_indices) {
	pcl::MultiscaleFeaturePersistence<Point_N, Feature> feature_persistence;
	std::vector<float> scale_values;
	scale_values.push_back(0.010);
	feature_persistence.setScalesVector(scale_values);
	feature_persistence.setAlpha(1.30f);

	pcl::FPFHEstimation<Point_N, Point_N, Feature>::Ptr fpfh_estimation (new pcl::FPFHEstimation<Point_N, Point_N, Feature>());
	fpfh_estimation->setInputCloud(cloud);
	fpfh_estimation->setInputNormals(cloud);

	feature_persistence.setFeatureEstimator(fpfh_estimation);
	feature_persistence.setDistanceMetric(pcl::CS);

	Feature_Cloud::Ptr output_features (new Feature_Cloud);
	feature_persistence.determinePersistentFeatures(*output_features, output_indices);
}

/**
	Compares two filtered poses in regards to inliers (first.second is inliers)
 */
bool
Pose_Class::filtered_poses_compare(const std::pair<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double>, double>& firstElem, const std::pair<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double>, double>& secondElem) {
	return firstElem.first.second > secondElem.first.second;
}

/**
	Estimates the pose of an object in a scene using the full cluster->object->cluster pipeline, with pose clustering and pose cluster filtering
	@param object The object which pose should be estimated
	@param scene The cluster the object should fit into (CHANGE NAME TO CLUSTER)
	@return Vector of filtered poses sorted by inlier fraction
 */
std::vector<std::pair<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double>, double> >
Pose_Class::estimate_full_pose(Point_Cloud_N::Ptr object, Point_Cloud_N::Ptr cluster) {
	// Copy clouds to different objects
	Point_Cloud_N::Ptr cluster_original (new Point_Cloud_N);
	Point_Cloud_N::Ptr object_transformed (new Point_Cloud_N);
	pcl::copyPointCloud(*cluster, *cluster_original);
	pcl::copyPointCloud(*object, *object_transformed);

	std::vector<Point_Cloud_N::Ptr> clusters;
	clusters.push_back(cluster);

	if(print_mode == 1)
		pcl::console::print_info("\t Computing object features\n");
	Feature_Cloud::Ptr object_features = compute_features(object);

	std::vector<std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > > transformations (clusters.size());
	match_cluster_to_object_to_cluster(object, object_features, clusters, &transformations);

	// Rows = Pose clusters
	// Cols = Pairs of (tr, inl) for that pose cluster, first pair is (tr_avg, inl_avg) for the pose cluster
	// [(tr_avg, inl_avg), (tr, inl), (tr, inl) ...;
	//  (tr_avg, inl_avg), (tr, inl), (tr, inl) ...]
	std::vector<std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > > clustered_poses;
	cluster_poses(&transformations, &clustered_poses);

	// Each transformation together with inliers and accuracy
	// [((tr_avg,inl_avg),acc), ((tr_avg,inl_avg),acc), ((tr_avg,inl_avg),acc)]
	std::vector<std::pair<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double>, double> > filtered_poses;
	filter_clustered_poses(&clustered_poses, &filtered_poses);

	// Sort filtered poses in regards to inliers
	std::sort(filtered_poses.begin(), filtered_poses.end(), filtered_poses_compare);

	if(visualization_mode == 1 || visualization_mode == 3 || visualization_mode == 4 || visualization_mode == 5) {
		// Visualize filtered poses
		for(int i = 0; i < filtered_poses.size(); i++) {
			Eigen::Matrix<float,4,4,Eigen::DontAlign> pose = filtered_poses[i].first.first;
			double inliers = filtered_poses[i].first.second;
			double accuracy = filtered_poses[i].second;

			std::ostringstream oss;
			oss << "Pose " << i << " / " << filtered_poses.size()-1 << ", inliers: " << inliers;
			visu_ptr->setWindowName(oss.str());
			if(print_mode == 1)
				std::cout << oss.str() << '\n';

			pcl::transformPointCloudWithNormals(*object, *object_transformed, pose);

			if(visualization_mode == 3) {
				// Compute occlusion
				Point_Cloud_N::Ptr visible_object (new Point_Cloud_N);
				Point_Cloud_N::Ptr occluded_object (new Point_Cloud_N);
				std::vector<int> occluded_indices;
				Eigen::Vector3d viewpoint (0,0,0);
				manipulation.compute_occlusion(object_transformed, LEAF_SIZE, viewpoint, visible_object, occluded_object, &occluded_indices);

				// Compute points with persistent features
				boost::shared_ptr<std::vector<int> > persistent_indices (new std::vector<int>);
				compute_persistent_features(object, persistent_indices);

				// Find both occluded and persistent points
				boost::shared_ptr<std::vector<int> > occluded_persistent_indices (new std::vector<int>);
				for(int k = 0; k < occluded_indices.size(); k++) {
					for(int n = 0; n < (*persistent_indices).size(); n++) {
						if(occluded_indices[k] == (*persistent_indices)[n]) {
							occluded_persistent_indices->push_back(occluded_indices[k]);
							break;
						}
					}
				}

				// Extract occluded persistent points into cloud
				pcl::ExtractIndices<Point_N> extract_filter;
				extract_filter.setInputCloud(object_transformed);
				extract_filter.setIndices(occluded_persistent_indices);
				Point_Cloud_N::Ptr occluded_persistent_cloud (new Point_Cloud_N);
				extract_filter.filter(*occluded_persistent_cloud);

				visu_ptr->addPointCloud(visible_object, ColorHandler_N(visible_object, 0.0, 255.0, 0.0), "visible_object");
				visu_ptr->addPointCloud(occluded_object, ColorHandler_N(occluded_object, 255, 48, 48), "occluded_object");
				visu_ptr->addPointCloud(occluded_persistent_cloud, ColorHandler_N(occluded_persistent_cloud, 255, 157, 0), "occluded_persistent_cloud");
				visu_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "visible_object");
				visu_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "occluded_object");
				visu_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "occluded_persistent_cloud");
			} else if(visualization_mode == 4) {
				std::vector<int> inliers;
				std::vector<int> outliers;
				manipulation.compute_inliers(cluster_original, object_transformed, POSE_MAX_CORRESPONDENCE_DISTANCE, &inliers, &outliers);

				pcl::PointIndices::Ptr inlier_cloud (new pcl::PointIndices);
				inlier_cloud->indices = inliers;

				Point_Cloud_N::Ptr object_inliers (new Point_Cloud_N);
				Point_Cloud_N::Ptr object_outliers (new Point_Cloud_N);
				pcl::ExtractIndices<Point_N> extract;
				extract.setInputCloud(object_transformed);
				extract.setIndices(inlier_cloud);
				extract.setNegative(false);
				extract.filter(*object_inliers);
				extract.setNegative(true);
				extract.filter(*object_outliers);

				// Compute points with persistent features
				boost::shared_ptr<std::vector<int> > persistent_indices (new std::vector<int>);
				compute_persistent_features(object, persistent_indices);

				// Find both outliers and persistent points
				boost::shared_ptr<std::vector<int> > outlier_persistent_indices (new std::vector<int>);
				for(int k = 0; k < outliers.size(); k++) {
					for(int n = 0; n < (*persistent_indices).size(); n++) {
						if(outliers[k] == (*persistent_indices)[n]) {
							outlier_persistent_indices->push_back(outliers[k]);
							break;
						}
					}
				}

				// Extract outlier persistent points into cloud
				extract.setIndices(outlier_persistent_indices);
				Point_Cloud_N::Ptr outlier_persistent_cloud (new Point_Cloud_N);
				extract.setNegative(false);
				extract.filter(*outlier_persistent_cloud);

				visu_ptr->addPointCloud(object_inliers, ColorHandler_N(object_inliers, 0.0, 255.0, 0.0), "object_inliers");
				visu_ptr->addPointCloud(object_outliers, ColorHandler_N(object_outliers, 100.0, 0.0, 0.0), "object_outliers");
				visu_ptr->addPointCloud(outlier_persistent_cloud, ColorHandler_N(outlier_persistent_cloud, 255, 157, 0), "outlier_persistent_cloud");
				visu_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "object_inliers");
				visu_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "object_outliers");
				visu_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "outlier_persistent_cloud");
			} else {
				// Visualize transformation
				visu_ptr->addPointCloud(object_transformed, ColorHandler_N(object_transformed, 0.0, 255.0, 0.0), "object_transformed");
				visu_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "object_transformed");
			}

			visu_ptr->addPointCloud(cluster_original, ColorHandler_N(cluster_original, 192, 192, 192), "cluster_original");

			visu_ptr->spin();
			visu_ptr->removeAllPointClouds();
			visu_ptr->removeAllShapes();
		}
	} else if(visualization_mode == 2) {
		// Visualize all clustered poses
		if(print_mode == 1)
			std::cout << "\n\nNumber of clustered poses: " << clustered_poses.size() << '\n';
		for(int i = 0; i < clustered_poses.size(); i++) {
			std::vector<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double> > pose_cluster = clustered_poses[i];

			if(print_mode == 1)
				std::cout << "Cluster " << i << " (" << pose_cluster.size() << " poses), average inlier count: " << pose_cluster[0].second << '\n';
			for(int n = 0; n < pose_cluster.size(); n++) {
				Eigen::Matrix<float,4,4,Eigen::DontAlign> pose = pose_cluster[n].first;

				if(print_mode == 1)
					std::cout << "\tPose " << n+1 << '\n';

				pcl::transformPointCloudWithNormals(*object, *object_transformed, pose);
				visu_ptr->addPointCloud(object_transformed, ColorHandler_N(object_transformed, 0.0, 255.0, 0.0), "object_transformed");
				visu_ptr->addPointCloud(cluster_original, ColorHandler_N(cluster_original, 192, 192, 192), "cluster_original");

				std::ostringstream oss;
				for(int k = 0; k < clusters.size(); k++) {
					oss.clear();
					oss << "cluster" << k;
					visu_ptr->addPointCloud(clusters[k], ColorHandler_N(clusters[k], 0.0, 0.0, 255.0), oss.str());
					visu_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, oss.str());
				}

				visu_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "object_transformed");
				visu_ptr->spin();
				visu_ptr->removeAllPointClouds();
			}
		}
	}

	return filtered_poses;
}

/**
	Writes pose data to file
	@param scene_name The name of the scene (name of file to save)
	@param poses Pose data as a vector of pose structs
 */
void
Pose_Class::write_pose_data(std::string scene_name, int cluster_index, std::vector<Pose_Data> poses) {
	Access_Results access_results;
	std::string data_path = access_results.path_to_pose_estimation_results().string();
	data_path += "/" + scene_name + "/";
	boost::filesystem::create_directory(data_path);
	data_path += boost::lexical_cast<std::string>(cluster_index) + ".csv";
	std::cout << "Pose data saved to " << data_path << '\n';

	std::ofstream ofs;
	ofs.open(data_path.c_str());

	for(int i = 0; i < poses.size(); i++) {
		Pose_Data p = poses[i];
		ofs << p.model_name << ",";
		for(int n = 0; n < p.view_indices.size(); n++) {
			ofs << p.view_indices[n] << "+";
		}
		ofs << ",";

		for(int r = 0; r < 4; r++) {
			for(int c = 0; c < 4; c++) {
				ofs << p.transformation(r,c) << ",";
			}
		}

		ofs << p.inlier_fraction << "," << p.accuracy << "," << "\n";
	}

	ofs.close();
}

/**
	Compares the inlier fraction of two poses
*/
bool
Pose_Class::poses_comparator(const Pose_Data& pose1, const Pose_Data& pose2) {
	return pose1.inlier_fraction > pose2.inlier_fraction;
}

/**
	Estimates the pose for all views in the cluster and writes the pose data
	@param views The views to estimate the pose for (can be merged views)
	@param model_names The names for each model (needs to match with views)
	@param model_view_indices The view indices for each view (not sure how this will work in merged views)
	@param cluster The cluster to estimate the pose in
	@param scene_name The name of the original scene, name of main folder in pose estimation results
	@param cluster_index The index of the cluster, name of subfolder in pose estimation results
 */
void
Pose_Class::estimate_pose_multiple_views(	std::vector<std::vector<Point_Cloud_N::Ptr> > views,
						std::vector<std::string> model_names,
						std::vector<std::vector<int> > model_view_indices,
						Point_Cloud_N::Ptr cluster,
						std::string scene_name,
						int cluster_index,
						int max_models,
						int max_views) {
	std::vector<Pose_Data> poses;

	// Loop through all different models
	for(int i = 0; i < std::min((int)model_names.size(), max_models); i++) {
		// Loop through all views for this model
		for(int k = 0; k < std::min((int)model_view_indices.size(), max_views); k++) {
			std::vector<std::pair<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double>, double> > filtered_poses;
			filtered_poses = estimate_full_pose(views[i][k], cluster);

			// Loop through all returned poses, create pose objects and add them to the poses list
			for(int n = 0; n < filtered_poses.size(); n++) {
				Pose_Data p;
				p.model_name = model_names[i];
				std::vector<int> view_indices;
				view_indices.push_back(model_view_indices[i][k]);
				p.view_indices = view_indices;
				p.transformation = filtered_poses[n].first.first;
				p.inlier_fraction = filtered_poses[n].first.second;

				p.accuracy = filtered_poses[n].second;
				poses.push_back(p);
			}
		}
	}

	// Sort poses list after inlier fractions and write it to file
	std::sort(poses.begin(), poses.end(), poses_comparator);
	write_pose_data(scene_name, cluster_index, poses);
}

/**
	Estimates the pose for all merged views in the cluster and writes the pose data
	@param views The merged views to estimate the pose for
	@param model_names The model name for each merged view
	@param model_view_indices The view indices used to merge each view
	@param cluster The cluster to estimate the pose in
	@param scene_name The name of the original scene, name of main folder in pose estimation results
	@param cluster_index The index of the cluster, name of subfolder in pose estimation results
	@param full_models The full CAD model for each view
 */
void
Pose_Class::estimate_pose_merged_views(	std::vector<Point_Cloud_N::Ptr> views,
					std::vector<std::string> model_names,
					std::vector<std::vector<int> > model_view_indices,
					Point_Cloud_N::Ptr cluster,
					std::string scene_name,
					int cluster_index,
					std::vector<Point_Cloud_N::Ptr> full_models) {

	std::vector<Pose_Data> poses;

	for(int i = 0; i < views.size(); i++) {
		std::vector<std::pair<std::pair<Eigen::Matrix<float,4,4,Eigen::DontAlign>, double>, double> > filtered_poses;
		filtered_poses = estimate_full_pose(views[i], cluster);

		// Loop through all returned poses, create pose objects and add them to the poses list
		for(int n = 0; n < filtered_poses.size(); n++) {
			//for(int n = 0; n < std::min(1, (int)filtered_poses.size()); n++) {
			Pose_Data p;
			p.model_name = model_names[i];
			p.view_indices = model_view_indices[i];
			p.transformation = filtered_poses[n].first.first;
			p.inlier_fraction = filtered_poses[n].first.second;

			Point_Cloud_N::Ptr tr_full_model (new Point_Cloud_N);
			std::vector<int> inliers, outliers;
			pcl::transformPointCloud(*full_models[i], *tr_full_model, p.transformation);
			manipulation.compute_inliers(cluster, tr_full_model, POSE_MAX_CORRESPONDENCE_DISTANCE, &inliers, &outliers);
			double accuracy = (double) inliers.size() / (double) tr_full_model->points.size();
			p.accuracy = accuracy;

			if(visualization_mode == 5) {
				pcl::PointIndices::Ptr inlier_cloud (new pcl::PointIndices);
				inlier_cloud->indices = inliers;

				Point_Cloud_N::Ptr full_object_inliers (new Point_Cloud_N);
				Point_Cloud_N::Ptr full_object_outliers (new Point_Cloud_N);
				pcl::ExtractIndices<Point_N> extract;
				extract.setInputCloud(tr_full_model);
				extract.setIndices(inlier_cloud);
				extract.setNegative(false);
				extract.filter(*full_object_inliers);
				extract.setNegative(true);
				extract.filter(*full_object_outliers);

				std::cout << "Accuracy is " << accuracy << std::endl;
				visu_ptr->addPointCloud(cluster, ColorHandler_N(cluster, 192, 192, 192), "cluster");
				visu_ptr->addPointCloud(full_object_inliers, ColorHandler_N(full_object_inliers, 0.0, 255.0, 0.0), "inliers");
				visu_ptr->addPointCloud(full_object_outliers, ColorHandler_N(full_object_outliers, 255.0, 0.0, 0.0), "outliers");
				visu_ptr->spin();
				visu_ptr->removeAllPointClouds();
			}

			poses.push_back(p);
		}
	}

	// Sort poses list after inlier fractions and write it to file
	std::sort(poses.begin(), poses.end(), poses_comparator);
	write_pose_data(scene_name, cluster_index, poses);
}

/**
	Set the visualizer to be used when showing clouds
	@param visu Pointer to the visualizer
 */
void
Pose_Class::set_visualizer(pcl::visualization::PCLVisualizer* visu) {
	visu_ptr = visu;
}

/**
	Set the visualization mode
	0 = No visualization
	1 = Visualize clustered and filtered poses
	2 = Visualize clustered poses
	3 = Visualize clustered and filtered poses with occlusion
	4 = Visualize clustered and filtered poses with inliers
	5 = Visualize clustered and filtered poses, and full object inliers (accuracy)
	@param vis_mode The visualization mode
 */
void
Pose_Class::set_visualization_mode(int vis_mode) {
	visualization_mode = vis_mode;
}

/**
	Set the print mode
	0 = No information printed
	1 = All information printed
	@param p_mode The print mode
 */
void
Pose_Class::set_print_mode(int p_mode) {
	print_mode = p_mode;
}
