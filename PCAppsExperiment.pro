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

SOURCES += main.cpp\
    Experiment/experimenter.cpp \
    pcappsexperiment.cpp

HEADERS  += \
    Experiment/experimenter.h \
    pcappsexperiment.h \
    ShareExpm/expm_common.h

FORMS    += \
    pcappsexperiment.ui

DEFINES += PCAppsExperiment_PATH=\\\"$$PWD\\\"
