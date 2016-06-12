#ifndef FPFHMAKER_H
#define FPFHMAKER_H

#include <QDebug>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/gpu/features/features.hpp>
#include "Share/project_common.h"
#include "ShareExpm/expm_common.h"

class FpfhMaker
{
public:
    FpfhMaker();
    void ComputeFpfhByCPU(VoxelCloud::Ptr points, NormalCloud::Ptr normals);
    void ComputeFpfhByGPU(VectorVoxel& points, VectorNormal& normals);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptorsByCpu;
    std::vector<pcl::FPFHSignature33> descriptorsByGpu;
};

#endif // FPFHMAKER_H
