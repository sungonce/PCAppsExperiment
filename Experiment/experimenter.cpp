#include "experimenter.h"

Experimenter::Experimenter()
    : neighborIndices(nullptr)
    , numNeighbors(nullptr)
{
}

Experimenter::~Experimenter()
{
}

void Experimenter::Work(const QImage& srcColorImg, const QImage& srcDepthImg, const vecAnnot& annots, SharedData* shdDat/*, vector<AnnotRect>& annotRects*/)
{
    shdDat->SetColorImage(srcColorImg);
    colorImg = srcColorImg;

    const cl_float4* pointCloud = ImageConverter::ConvertToPointCloud(srcDepthImg);
    shdDat->SetPointCloud(pointCloud);

    CreateNormalAndDescriptor(shdDat);

    ComputePCL_CPU_descriptors(shdDat);

    ComputePCL_GPU_descriptors(shdDat);
}

void Experimenter::CreateNormalAndDescriptor(SharedData* shdDat)
{
    const cl_float4* pointCloud = shdDat->ConstPointCloud();

    eltimer.start();
    neibSearcher.SearchNeighborIndices(pointCloud, SEARCH_RADIUS, FOCAL_LENGTH, NEIGHBORS_PER_POINT);
    neighborIndices = neibSearcher.GetNeighborIndices();
    numNeighbors = neibSearcher.GetNumNeighbors();
    qDebug() << "SearchNeighborIndices took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    normalMaker.ComputeNormal(neibSearcher.memPoints, neibSearcher.memNeighborIndices
                              , neibSearcher.memNumNeighbors, NEIGHBORS_PER_POINT);
    const cl_float4* normalCloud = normalMaker.GetNormalCloud();
    shdDat->SetNormalCloud(normalCloud);
    qDebug() << "ComputeNormal took" << eltimer.nsecsElapsed()/1000 << "us";

    const cl_uchar* nullityMap = CreateNullityMap(shdDat);
    shdDat->SetNullityMap(nullityMap);
    CheckDataValidity(pointCloud, normalCloud);
}

void Experimenter::CheckDataValidity(const cl_float4* pointCloud, const cl_float4* normalCloud)
{
    // 1. w channel of point cloud, normal cloud and descriptors must be "0"
    // 2. length of normal is either 0 or 1
    // 3. w channel of descriptors is "1" if valid, otherwise "0"
    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        assert(pointCloud[i].w==0.f);
        assert(normalCloud[i].w==0.f);
        assert(fabsf(clLength(normalCloud[i])-1.f) < 0.0001f || clLength(normalCloud[i]) < 0.0001f);
    }
}

cl_uchar* Experimenter::CreateNullityMap(SharedData* shdDat)
{
    static ArrayData<cl_uchar> nullData(IMAGE_HEIGHT*IMAGE_WIDTH);
    cl_uchar* nullityMap = nullData.GetArrayPtr();

    const cl_float4* pointCloud = shdDat->ConstPointCloud();
    const cl_float4* normalCloud = shdDat->ConstNormalCloud();

    for(int i=0; i<IMAGE_HEIGHT*IMAGE_WIDTH; i++)
    {
        nullityMap[i] = NullID::NoneNull;
        if(clIsNull(pointCloud[i]))
            nullityMap[i] = NullID::PointNull;
        else if(clIsNull(normalCloud[i]))
            nullityMap[i] = NullID::NormalNull;
    }
    return nullityMap;
}

void Experimenter::ComputePCL_CPU_descriptors(SharedData* shdDat)
{
    pclConverter.ConvertToPCLFormat(shdDat);
    qDebug() << "---------- PCL CPU descriptors ----------";

#ifdef COMPARE_DESC_CPU
    eltimer.start();
    descriptorMakerCpu.ComputeDescriptors(pointCloud, normalCloud, neighborIndices, numNeighbors, NEIGHBORS_PER_POINT);
    const DescType* descriptorsCpu = descriptorMakerCpu.GetDescriptors();
    qDebug() << "CurvatureDescriptorBy CPU took" << eltimer.nsecsElapsed()/1000 << "us";
#endif

    eltimer.start();
    fpfh_cpu.EstimateFpfh(pclConverter.pclPointCloud, pclConverter.pclNormalCloud);
    qDebug() << "FPFH CPU took" << eltimer.nsecsElapsed()/1000 << "us";

//    eltimer.start();
//    prinCurv_cpu.EstimatePrincipalCurvature(pclConverter.pclPointCloud, pclConverter.pclNormalCloud);
//    qDebug() << "PrincipalCurvature CPU took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    spinImage_cpu.EstimateSpinImage(pclConverter.pclPointCloud, pclConverter.pclNormalCloud);
    qDebug() << "SpinImage CPU took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    shot_cpu.EstimateShot(pclConverter.pclPointCloud, pclConverter.pclNormalCloud);
    qDebug() << "SHOT CPU took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    narf_cpu.EstimateNarf(pclConverter.pclPointCloud, pclConverter.pclNormalCloud);
    qDebug() << "NARF CPU took" << eltimer.nsecsElapsed()/1000 << "us";
}

void Experimenter::ComputePCL_GPU_descriptors(SharedData* shdDat)
{
    pclConverter.ConvertToPCLVector(shdDat);
    qDebug() << "---------- PCL GPU descriptors ----------";

    eltimer.start();
    curvDescriptor_gpu.ComputeDescriptor(neibSearcher.memPoints, normalMaker.memNormals
                                      , neibSearcher.memNeighborIndices, neibSearcher.memNumNeighbors, NEIGHBORS_PER_POINT);
    qDebug() << "CurvatureDescriptorBy GPU took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    fpfh_gpu.EstimateFpfh(pclConverter.pclPoints, pclConverter.pclNormals);
    qDebug() << "FPFH GPU took" << eltimer.nsecsElapsed()/1000 << "us";

    eltimer.start();
    prinCurv_gpu.EstimatePrincipalCurvature(pclConverter.pclPoints, pclConverter.pclNormals);
    qDebug() << "PrincipalCurvature GPU took" << eltimer.nsecsElapsed()/1000 << "us";

//    eltimer.start();
//    spinImage_gpu.EstimateSpinImage(pclConverter.pclPoints, pclConverter.pclNormals);
//    qDebug() << "SpinImage GPU took" << eltimer.nsecsElapsed()/1000 << "us";
}
