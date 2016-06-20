#ifndef PCLCPUFEATURES_H
#define PCLCPUFEATURES_H

#include <QDebug>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/shot.h>
#include <pcl/features/narf.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/range_image/range_image_planar.h>
#include <Eigen/Eigen>
#include "Share/project_common.h"
#include "ShareExpm/expm_common.h"
#include "Share/camera_param.h"

namespace CpuFeature
{
class FpfhEstimator
{
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors;

public:
    FpfhEstimator()
        : descriptors(new pcl::PointCloud<pcl::FPFHSignature33>())
    {}

    void EstimateFpfh(VoxelCloud::Ptr points, NormalCloud::Ptr normals)
    {
        descriptors->clear();
        pcl::search::KdTree<VoxelType>::Ptr tree(new pcl::search::KdTree<VoxelType>);
        // FPFH estimation object.
        pcl::FPFHEstimation<VoxelType, NormalType, pcl::FPFHSignature33> fpfh;
        fpfh.setInputCloud(points);
        fpfh.setInputNormals(normals);
        fpfh.setSearchMethod(tree);
        // Search radius, to look for neighbors. Note: the value given here has to be
        // larger than the radius used to estimate the normals.
        fpfh.setRadiusSearch(0.05f);

        fpfh.compute(*descriptors);

    //    {
    //        QDebug dbg = qDebug();
    //        dbg << "descriptors CPU" << descriptors->size();
    //        for(int i=0; i<pcl::FPFHSignature33::descriptorSize(); i++)
    //            dbg << descriptors->points[100].histogram[i];
    //    }
    }
};


typedef pcl::Histogram<153> SpinImage;

class SpinImageEstimator
{
public:
    SpinImageEstimator()
        : descriptors(new pcl::PointCloud<SpinImage>())
    {}

    void EstimateSpinImage(VoxelCloud::Ptr points, NormalCloud::Ptr normals)
    {
        descriptors->clear();
        pcl::search::KdTree<VoxelType>::Ptr tree(new pcl::search::KdTree<VoxelType>);
        // FPFH estimation object.
        pcl::SpinImageEstimation<VoxelType, NormalType, SpinImage> spinImage;
        spinImage.setInputCloud(points);
        spinImage.setInputNormals(normals);
        spinImage.setSearchMethod(tree);
        spinImage.useNormalsAsRotationAxis();
        // Search radius, to look for neighbors. Note: the value given here has to be
        // larger than the radius used to estimate the normals.
        spinImage.setRadiusSearch(0.05f);

        spinImage.compute(*descriptors);

//        {
//            QDebug dbg = qDebug();
//            dbg << "SpinImage by CPU" << descriptors->size();
//            for(int i=0; i<SpinImage::descriptorSize(); i++)
//                dbg << descriptors->points[10000].histogram[i];
//        }
    }

    pcl::PointCloud<SpinImage>::Ptr descriptors;
};

class PrincipleCurvatureEstimator
{
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr descriptors;
public:
    PrincipleCurvatureEstimator()
        : descriptors(new pcl::PointCloud<pcl::PrincipalCurvatures>())
    {}

    void EstimatePrincipalCurvature(VoxelCloud::Ptr points, NormalCloud::Ptr normals)
    {
        pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> prinCurv;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

        prinCurv.setInputCloud(points);
        prinCurv.setInputNormals(normals);
        prinCurv.setSearchMethod(tree);
        prinCurv.setRadiusSearch(1.0);
        prinCurv.compute(*descriptors);
    }
};

class ShotEstimator
{
    pcl::PointCloud<pcl::SHOT352>::Ptr descriptors;

public:
    ShotEstimator()
        : descriptors(new pcl::PointCloud<pcl::SHOT352>())
    {}

    void EstimateShot(VoxelCloud::Ptr points, NormalCloud::Ptr normals)
    {
        descriptors->clear();
        pcl::search::KdTree<VoxelType>::Ptr tree(new pcl::search::KdTree<VoxelType>);
        // FPFH estimation object.
        pcl::SHOTEstimation<VoxelType, NormalType, pcl::SHOT352> shot;
        shot.setInputCloud(points);
        shot.setInputNormals(normals);
        shot.setSearchMethod(tree);
        // Search radius, to look for neighbors. Note: the value given here has to be
        // larger than the radius used to estimate the normals.
        shot.setRadiusSearch(0.05f);

        shot.compute(*descriptors);
    }
};

class NarfEstimator
{
    pcl::PointCloud<pcl::Narf36>::Ptr descriptors;

public:
    NarfEstimator()
        : descriptors(new pcl::PointCloud<pcl::Narf36>())
    {}

    void EstimateNarf(VoxelCloud::Ptr points, NormalCloud::Ptr normals)
    {
        descriptors->clear();

        // Convert the cloud to range image.
        int imageSizeX = IMAGE_WIDTH, imageSizeY = IMAGE_HEIGHT;
        float centerX = (IMAGE_WIDTH / 2.0f), centerY = (IMAGE_HEIGHT / 2.0f);
        float focalLengthX = FOCAL_LENGTH, focalLengthY = focalLengthX;
        float noiseLevel = 0.0f, minimumRange = 0.0f;
        Eigen::Affine3f sensorPose;
        sensorPose.setIdentity();
        pcl::RangeImagePlanar rangeImage;
        rangeImage.createFromPointCloudWithFixedSize(*points, imageSizeX, imageSizeY,
                centerX, centerY, focalLengthX, focalLengthX,
                sensorPose, pcl::RangeImage::CAMERA_FRAME,
                noiseLevel, minimumRange);

        std::vector<int> keypoints;
        keypoints.resize(points->size());
        for(int i=0; i<points->size(); ++i)
            keypoints[i] = i;
        // NARF estimation object.
        pcl::NarfDescriptor narf(&rangeImage, &keypoints);
        narf.getParameters().support_size = 0.04f;
        // If true, the rotation invariant version of NARF will be used. The histogram
        // will be shifted according to the dominant orientation to provide robustness to
        // rotations around the normal.
        narf.getParameters().rotation_invariant = true;

        narf.compute(*descriptors);
    }
};


}

#endif // PCLCPUFEATURES_H
