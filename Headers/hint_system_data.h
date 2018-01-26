#ifndef HINT_SYSTEM_DATA_H
#define HINT_SYSTEM_DATA_H

struct Hint_System_Data {
  bool valid_results;
  std::vector<Pose_Data> pd;
  Eigen::Matrix<float,4,4,Eigen::DontAlign> next_position;
};

#endif
