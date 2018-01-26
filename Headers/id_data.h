#ifndef ID_DATA_H
#define ID_DATA_H

struct ID_Data {
  std::string model_name;
  std::vector<int> view_indices;
  std::vector<float> scores;
};

#endif
