#ifndef EXPERIMENTER_H
#define EXPERIMENTER_H

#include <QImage>
#include <QElapsedTimer>
#include "Share/project_common.h"
#include "Share/shared_enums.h"
#include "Share/shared_data.h"
#include "Share/annotation.h"
#include "IO/drawutils.h"
#include "IO/glvertexmanager.h"
#include "IO/imageconverter.h"
#include "ClUtils/cloperators.h"
#include "PCWork/radiussearch.h"
#include "PCWork/normalmaker.h"
#include "PCWork/descriptormaker.h"
#include "PCWork/descriptormakerbycpu.h"

class Experimenter
{
public:
    Experimenter();
    ~Experimenter();
    void Work(const QImage& srcColorImg, const QImage& srcDepthImg, const vecAnnot& annots, SharedData* shdDat);
    void MarkNeighborsOnImage(QImage& srcimg, QPoint pixel);
    void DrawOnlyNeighbors(SharedData& shdDat, QPoint pixel);
    void CheckDataValidity(const cl_float4* pointCloud, const cl_float4* normalCloud
                           , const cl_float4* descriptors, const cl_float4* descriptorsCpu=nullptr);

private:
    void CreateNormalAndDescriptor(SharedData* shdDat);
    cl_uchar* CreateNullityMap(SharedData* shdDat);

    QElapsedTimer       eltimer;
    RadiusSearch        neibSearcher;
    NormalMaker         normalMaker;
    DescriptorMaker     descriptorMaker;
    DescriptorMakerByCpu descriptorMakerCpu;

    cl_int* neighborIndices;
    cl_int* numNeighbors;
    QImage colorImg;

    friend class PCAppsExperiment;
};

#endif // EXPERIMENTER_H
