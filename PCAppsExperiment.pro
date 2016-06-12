#-------------------------------------------------
#
# Project created by QtCreator 2016-06-06T16:25:51
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PCAppsExperiment
TEMPLATE = app

include(../PCApps/PCApps-share.pri)

# import boost
INCLUDEPATH += /home/hyukdoo/MyLibs/LibsInstalled/boost-1.61/include
LIBS += -L/home/hyukdoo/MyLibs/LibsInstalled/boost-1.61/lib \
        -lboost_thread -lboost_filesystem -lboost_iostreams -lboost_system -lboost_chrono

# import flann
INCLUDEPATH += /home/hyukdoo/MyLibs/LibsInstalled/flann-1.8.4/include
LIBS += -L/home/hyukdoo/MyLibs/LibsInstalled/flann-1.8.4/lib

# import pcl
INCLUDEPATH += /home/hyukdoo/MyLibs/LibsInstalled/pcl-1.8/include/pcl-1.8
LIBS += -L/home/hyukdoo/MyLibs/LibsInstalled/pcl-1.8/lib
PCL_LIB_FULL = $$system("find /home/hyukdoo/MyLibs/LibsInstalled/pcl-1.8/lib -name '*.so'")
for(eachlib, PCL_LIB_FULL):PCL_LIB_SO+=$$replace(eachlib, /home/hyukdoo/MyLibs/LibsInstalled/pcl-1.8/lib/libpcl, -lpcl)
for(eachlib, PCL_LIB_SO):PCL_LIB+=$$replace(eachlib, .so, )
LIBS += $$PCL_LIB
#message($$PCL_LIB)
#        -lpcl_common -lpcl_filters -lpcl_io  \
#        -lpcl_features -lpcl_cuda_features -lpcl_gpu_features   \
#        -lpcl_gpu_octree -lpcl_octree -lpcl_kdtree

SOURCES += main.cpp\
    pcappsexperiment.cpp \
    Experiment/experimenter.cpp \
    Experiment/convertertopcl.cpp

HEADERS  += \
    pcappsexperiment.h \
    Experiment/experimenter.h \
    Experiment/convertertopcl.h \
    ShareExpm/expm_common.h \
    Features/pclcpufeatures.h \
    Features/pclgpufeatures.h

FORMS    += \
    pcappsexperiment.ui

DEFINES += PCAppsExperiment_PATH=\\\"$$PWD\\\"


