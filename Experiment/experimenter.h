#ifndef EXPERIMENTER_H
#define EXPERIMENTER_H

#include <QImage>
#include <QElapsedTimer>
#include <pcl/common/common_headers.h>

#include "Share/project_common.h"
#include "Share/shared_enums.h"
#include "Share/shared_data.h"
#include "Share/annotation.h"
#include "IO/drawutils.h"
#include "IO/imageconverter.h"
#include "ClUtils/cloperators.h"
#include "PCWork/radiussearch.h"
#include "PCWork/normalmaker.h"
#include "PCWork/descriptormaker.h"
#include "PCWork/descriptormakerbycpu.h"
#include "ShareExpm/expm_common.h"
#include "convertertopcl.h"
#include "Features/pclcpufeatures.h"
#include "Features/pclgpufeatures.h"

//#define COMPARE_DESC_CPU

class Experimenter
{
public:
    Experimenter();
    ~Experimenter();
    void Work(const QImage& srcColorImg, const QImage& srcDepthImg, const vecAnnot& annots, SharedData* shdDat);
    void MarkNeighborsOnImage(QImage& srcimg, QPoint pixel);
    void DrawOnlyNeighbors(SharedData& shdDat, QPoint pixel);
    void CheckDataValidity(const cl_float4* pointCloud, const cl_float4* normalCloud);

private:
    void CreateNormalAndDescriptor(SharedData* shdDat);
    cl_uchar* CreateNullityMap(SharedData* shdDat);
    void ComputePCL_CPU_descriptors(SharedData* shdDat);
    void ComputePCL_GPU_descriptors(SharedData* shdDat);

    QElapsedTimer       eltimer;
    RadiusSearch        neibSearcher;
    NormalMaker         normalMaker;
    DescriptorMaker     curvDescriptor_gpu;
    DescriptorMakerByCpu descriptorMakerCpu;

    cl_int* neighborIndices;
    cl_int* numNeighbors;
    QImage colorImg;

    ConverterToPcl pclConverter;

    CpuFeature::FpfhEstimator fpfh_cpu;
    CpuFeature::PrincipleCurvatureEstimator prinCurv_cpu;
    CpuFeature::SpinImageEstimator spinImage_cpu;
    CpuFeature::ShotEstimator shot_cpu;
    CpuFeature::NarfEstimator narf_cpu;

    GpuFeature::FpfhEstimator fpfh_gpu;
    GpuFeature::PrincipleCurvatureEstimator prinCurv_gpu;
    GpuFeature::SpinImageEstimator spinImage_gpu;

//    CpuFeature::FeatureWithNormals<pcl::FPFHEstimation, pcl::FPFHSignature33> fpfhEstimator;

    friend class PCAppsExperiment;
};

#endif // EXPERIMENTER_H
