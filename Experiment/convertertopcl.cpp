#include "convertertopcl.h"

ConverterToPcl::ConverterToPcl()
    : pclPointCloud(new VoxelCloud)
    , pclNormalCloud(new NormalCloud)
{
}

void ConverterToPcl::ConvertToPCLFormat(SharedData* shdDat)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_float4* normalCloud = shdDat->ConstNormalCloud();
    const cl_uchar* nullityMap = shdDat->ConstNullityMap();

    VoxelType voxel;
    NormalType normal;
    pclPointCloud->clear();
    pclNormalCloud->clear();
    int count=0;

    for(int i=0; i<IMAGE_WIDTH*IMAGE_HEIGHT; i++)
    {
        if(nullityMap[i]>=NullID::NormalNull)
            continue;
        count++;
        voxel.getArray4fMap() << pointCloud[i].x, pointCloud[i].y, pointCloud[i].z, 0;
        pclPointCloud->push_back(voxel);
        normal.getNormalVector4fMap() << normalCloud[i].x, normalCloud[i].y, normalCloud[i].z, 0;
        pclNormalCloud->push_back(normal);
    }
//    qDebug() << "count" << count << pclPointCloud->size() << pclNormalCloud->size()
//                << "point" << pclPointCloud->points[100].x << pclPointCloud->points[100].y << pclPointCloud->points[100].z
//                   << "normal" << pclNormalCloud->points[100].normal_x << pclNormalCloud->points[100].normal_y << pclNormalCloud->points[100].normal_z;
}

void ConverterToPcl::ConvertToPCLVector(SharedData* shdDat)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_float4* normalCloud = shdDat->ConstNormalCloud();
    const cl_uchar* nullityMap = shdDat->ConstNullityMap();

    VoxelType voxel;
    VoxelType normal;
    pclPoints.clear();
    pclNormals.clear();
    int count=0;

    for(int i=0; i<IMAGE_WIDTH*IMAGE_HEIGHT; i++)
    {
        if(nullityMap[i]>=NullID::NormalNull)
            continue;
        count++;
        voxel.getArray4fMap() << pointCloud[i].x, pointCloud[i].y, pointCloud[i].z, 0;
        pclPoints.push_back(voxel);
        normal.getArray4fMap() << normalCloud[i].x, normalCloud[i].y, normalCloud[i].z, 0;
        pclNormals.push_back(normal);
    }
}
