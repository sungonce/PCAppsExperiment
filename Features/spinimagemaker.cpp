#include "spinimagemaker.h"

SpinImageMaker::SpinImageMaker()
    : descriptorsByCpu(new pcl::PointCloud<SpinImage>())
{
}

void SpinImageMaker::ComputeSpinImageByCPU(VoxelCloud::Ptr points, NormalCloud::Ptr normals)
{
    descriptorsByCpu->clear();
    pcl::search::KdTree<VoxelType>::Ptr kdtree(new pcl::search::KdTree<VoxelType>);
    // FPFH estimation object.
    pcl::SpinImageEstimation<VoxelType, NormalType, SpinImage> spinImage;
    spinImage.setInputCloud(points);
    spinImage.setInputNormals(normals);
    spinImage.setSearchMethod(kdtree);
    spinImage.useNormalsAsRotationAxis();
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    spinImage.setRadiusSearch(0.05f);

    spinImage.compute(*descriptorsByCpu);

    {
        QDebug dbg = qDebug();
        dbg << "SpinImage by CPU" << descriptorsByCpu->size();
        for(int i=0; i<SpinImage::descriptorSize(); i++)
            dbg << descriptorsByCpu->points[10000].histogram[i];
    }
}

void SpinImageMaker::ComputeSpinImageByGPU(VectorVoxel& points, VectorNormal& normals)
{
    descriptorsByGpu.clear();

    pcl::gpu::DeviceArray<pcl::PointXYZ> gpupoints;
    pcl::gpu::DeviceArray<pcl::PointXYZ> gpunormals;
    gpupoints.upload(points);
    gpunormals.upload(normals);
    qDebug() << "SpinImage size:" << gpupoints.size() << gpupoints.sizeBytes() << gpunormals.size() << gpunormals.sizeBytes();

    // FPFH estimation object.
    pcl::gpu::SpinImageEstimation spinImage;
    pcl::gpu::DeviceArray2D<SpinImage> gpuDescriptors;
    pcl::gpu::DeviceArray<uchar> gpuMask;
    spinImage.setInputWithNormals(gpupoints, gpunormals);
//    spinImage.setInputCloud(gpupoints);
//    spinImage.setInputNormals(gpunormals);
    spinImage.useNormalsAsRotationAxis();
//    spinImage.setImageWidth(8);
//    spinImage.setMinPointCountInNeighbourhood(20);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    spinImage.setRadiusSearch(0.05f, 30);
    qDebug() << "ComputeSpinImageByGPU3";
    spinImage.compute(gpuDescriptors, gpuMask);
    qDebug() << "ComputeSpinImageByGPU4";

    int cols=0;
    gpuDescriptors.download(descriptorsByGpu, cols);

    {
        QDebug dbg = qDebug();
        dbg << "SpinImage by GPU" << descriptorsByGpu.size();
        for(int i=0; i<SpinImage::descriptorSize(); i++)
            dbg << descriptorsByGpu[10000].histogram[i];
    }
}
