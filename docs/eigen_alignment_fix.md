# Eigen alignment fix

This is used to fix alignment issues with Eigen vectors and matrices. The problem arises when one defines e.g. 
```c++
Eigen::Matrix4f
```
Different arithmetic and assignment operations will cause a segmentation fault. The solution for this is to replace it with e.g. 
```c++
Eigen::Matrix<float,4,4,Eigen::DontAlign>
```

In sample_consensus_prerejective.h and sample_consensus_prerejective.hpp DO NOT replace Eigen::Matrix4f with Eigen::Matrix<float,4,4,Eigen::DontAlign> in computeTransformation (PointCloudSource &output, const Eigen::Matrix4f& guess)

For more info see: https://eigen.tuxfamily.org/dox/group__DenseMatrixManipulation__Alignement.html
