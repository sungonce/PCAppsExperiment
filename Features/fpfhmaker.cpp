#include "fpfhmaker.h"

FpfhMaker::FpfhMaker()
    : descriptorsByCpu(new pcl::PointCloud<pcl::FPFHSignature33>())
{
}

void FpfhMaker::ComputeFpfhByCPU(VoxelCloud::Ptr points, NormalCloud::Ptr normals)
{
    descriptorsByCpu->clear();
    pcl::search::KdTree<VoxelType>::Ptr kdtree(new pcl::search::KdTree<VoxelType>);
    // FPFH estimation object.
    pcl::FPFHEstimation<VoxelType, NormalType, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(points);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(kdtree);
    // Search radius, to look for neighbors. Note: the value given here has to be
    // larger than the radius used to estimate the normals.
    fpfh.setRadiusSearch(0.05f);

    fpfh.compute(*descriptorsByCpu);

//    {
//        QDebug dbg = qDebug();
//        dbg << "descriptorsByCpu CPU" << descriptorsByCpu->size();
//        for(int i=0; i<pcl::FPFHSignature33::descriptorSize(); i++)
//            dbg << descriptorsByCpu->points[100].histogram[i];
//    }
}

void FpfhMaker::ComputeFpfhByGPU(VectorVoxel& points, VectorNormal& normals)
{
    descriptorsByGpu.clear();

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
    gpuDescriptors.download(descriptorsByGpu, cols);

//    {
//        QDebug dbg = qDebug();
//        dbg << "descriptorsByCpu GPU" << descriptorsByGpu.size();
//        for(int i=0; i<pcl::FPFHSignature33::descriptorSize(); i++)
//            dbg << descriptorsByGpu[100].histogram[i];
//    }

}
