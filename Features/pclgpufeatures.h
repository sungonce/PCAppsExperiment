#ifndef PCLGPUFEATURES_H
#define PCLGPUFEATURES_H

#include <QDebug>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/gpu/features/features.hpp>
#include "Share/project_common.h"
#include "ShareExpm/expm_common.h"

namespace GpuFeature
{

class FpfhEstimator
{
    std::vector<pcl::FPFHSignature33> descriptors;

public:
    FpfhEstimator() {}

    void EstimateFpfh(VectorVoxel& points, VectorNormal& normals)
    {
        descriptors.clear();
        pcl::gpu::DeviceArray<pcl::PointXYZ> gpupoints;
        pcl::gpu::DeviceArray<pcl::PointXYZ> gpunormals;
        gpupoints.upload(points);
        gpunormals.upload(normals);

        // FPFH estimation object.
        pcl::gpu::FPFHEstimation fpfh;
        pcl::gpu::DeviceArray2D<pcl::FPFHSignature33> gpuDescriptors;
        fpfh.setInputCloud(gpupoints);
        fpfh.setInputNormals(gpunormals);
        // Search radius, to look for neighbors. Note: the value given here has to be
        // larger than the radius used to estimate the normals.
        fpfh.setRadiusSearch(0.05f, 50);
        fpfh.compute(gpuDescriptors);

        int cols=0;
        gpuDescriptors.download(descriptors, cols);

    //    {
    //        QDebug dbg = qDebug();
    //        dbg << "descriptorsByCpu GPU" << descriptors.size();
    //        for(int i=0; i<pcl::FPFHSignature33::descriptorSize(); i++)
    //            dbg << descriptors[100].histogram[i];
    //    }
    }
};


typedef pcl::Histogram<153> SpinImage;

class SpinImageEstimator
{
    std::vector<SpinImage> descriptors;

public:
    SpinImageEstimator() {}

    void EstimateSpinImage(VectorVoxel& points, VectorNormal& normals)
    {
        descriptors.clear();

        pcl::gpu::DeviceArray<pcl::PointXYZ> gpupoints;
        pcl::gpu::DeviceArray<pcl::PointXYZ> gpunormals;
        gpupoints.upload(points);
        gpunormals.upload(normals);

        // FPFH estimation object.
        pcl::gpu::SpinImageEstimation spinImage;
        pcl::gpu::DeviceArray2D<SpinImage> gpuDescriptors;
        pcl::gpu::DeviceArray<uchar> gpuMask;
        spinImage.setInputWithNormals(gpupoints, gpunormals);
    //    spinImage.setInputCloud(gpupoints);
    //    spinImage.setInputNormals(gpunormals);
        spinImage.useNormalsAsRotationAxis();
    //    spinImage.setMinPointCountInNeighbourhood(20);
        // Search radius, to look for neighbors. Note: the value given here has to be
        // larger than the radius used to estimate the normals.
        spinImage.setRadiusSearch(0.05f, 30);
        qDebug() << "ComputeSpinImageByGPU3";
        spinImage.compute(gpuDescriptors, gpuMask);
        qDebug() << "ComputeSpinImageByGPU4";

        int cols=0;
        gpuDescriptors.download(descriptors, cols);

        {
            QDebug dbg = qDebug();
            dbg << "SpinImage by GPU" << descriptors.size();
            for(int i=0; i<SpinImage::descriptorSize(); i++)
                dbg << descriptors[10000].histogram[i];
        }
    }
};

class PrincipleCurvatureEstimator
{
    std::vector<pcl::PrincipalCurvatures> descriptors;
public:
    PrincipleCurvatureEstimator() {}

    void EstimatePrincipalCurvature(VectorVoxel& points, VectorNormal& normals)
    {
        descriptors.clear();
        pcl::gpu::DeviceArray<pcl::PointXYZ> gpupoints;
        pcl::gpu::DeviceArray<pcl::PointXYZ> gpunormals;
        gpupoints.upload(points);
        gpunormals.upload(normals);

        // FPFH estimation object.
        pcl::gpu::PrincipalCurvaturesEstimation prinCurv;
        pcl::gpu::DeviceArray<pcl::PrincipalCurvatures> gpuDescriptors;
        prinCurv.setInputCloud(gpupoints);
        prinCurv.setInputNormals(gpunormals);
        // Search radius, to look for neighbors. Note: the value given here has to be
        // larger than the radius used to estimate the normals.
        prinCurv.setRadiusSearch(0.05f, 50);
        prinCurv.compute(gpuDescriptors);

        gpuDescriptors.download(descriptors);
    }
};

}


#endif // PCLGPUFEATURES_H
