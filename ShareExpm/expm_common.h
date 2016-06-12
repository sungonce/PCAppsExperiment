#ifndef EXPM_COMMON_H
#define EXPM_COMMON_H

#include <vector>
#include <pcl/common/common_headers.h>

typedef pcl::PointXYZ               VoxelType;
typedef pcl::PointCloud<VoxelType>  VoxelCloud;
typedef std::vector<VoxelType>      VectorVoxel;

typedef pcl::Normal                 NormalType;
typedef pcl::PointCloud<NormalType> NormalCloud;
typedef std::vector<VoxelType>      VectorNormal;   // gpu modules use PointXYZ for normal vector

#endif // EXPM_COMMON_H
