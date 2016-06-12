#ifndef SPINIMAGEMAKER_H
#define SPINIMAGEMAKER_H

#include <QDebug>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/gpu/features/features.hpp>
#include "Share/project_common.h"
#include "ShareExpm/expm_common.h"

typedef pcl::Histogram<153> SpinImage;

class SpinImageMaker
{
public:
    SpinImageMaker();

    void ComputeSpinImageByCPU(VoxelCloud::Ptr points, NormalCloud::Ptr normals);
    void ComputeSpinImageByGPU(VectorVoxel& points, VectorNormal& normals);

    pcl::PointCloud<SpinImage>::Ptr descriptorsByCpu;
    std::vector<SpinImage> descriptorsByGpu;
};

#endif // SPINIMAGEMAKER_H
