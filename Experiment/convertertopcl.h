#ifndef CONVERTERTOPCL_H
#define CONVERTERTOPCL_H

#include <QDebug>
#include "Share/project_common.h"
#include "Share/shared_data.h"
#include "Share/shared_enums.h"
#include "ShareExpm/expm_common.h"

class ConverterToPcl
{
public:
    ConverterToPcl();

    void ConvertToPCLFormat(SharedData* shdDat);
    void ConvertToPCLVector(SharedData* shdDat);

    VoxelCloud::Ptr pclPointCloud;
    NormalCloud::Ptr pclNormalCloud;
    VectorVoxel pclPoints;
    VectorNormal pclNormals;
};

#endif // CONVERTERTOPCL_H
