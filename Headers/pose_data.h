#ifndef POSE_DATA_H
#define POSE_DATA_H

struct Pose_Data {
  std::string model_name;
  std::vector<int> view_indices;
  Eigen::Matrix<float,4,4,Eigen::DontAlign> transformation;
  double inlier_fraction;
  double accuracy;
};

#endif
