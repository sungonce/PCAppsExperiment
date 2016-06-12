#ifndef PRINCIPLECURVATUREMAKER_H
#define PRINCIPLECURVATUREMAKER_H

#include <QDebug>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/gpu/features/features.hpp>
#include "Share/project_common.h"
#include "ShareExpm/expm_common.h"


class PrincipleCurvatureMaker
{
public:
    PrincipleCurvatureMaker();

    void ComputeFpfhByCPU(VoxelCloud::Ptr points, NormalCloud::Ptr normals);
    void ComputeFpfhByGPU(VectorVoxel& points, VectorNormal& normals);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptorsByCpu;
    std::vector<pcl::FPFHSignature33> descriptorsByGpu;
};

#endif // PRINCIPLECURVATUREMAKER_H
