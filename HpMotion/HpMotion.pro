#-------------------------------------------------
#
# Project created by QtCreator 2014-09-04T17:13:40
#
#-------------------------------------------------

QT       -= core gui

TARGET = HpMotion
TEMPLATE = lib
CONFIG += staticlib


SOURCES += \
    BVHMotionData.cpp \
    HpMotionMath.cpp \
    HuboMotionData.cpp \
    HuboVPBody.cpp \
    HuboVpController.cpp \
    IKSolver.cpp \
    Joint.cpp \
    Motion.cpp \
    MotionData.cpp

HEADERS += \
    BVHMotionData.h \
    HpMotionMath.h \
    HuboMotionData.h \
    HuboVPBody.h \
    HuboVpController.h \
    IKSolver.h \
    Joint.h \
    Motion.h \
    MotionData.h

INCLUDEPATH += ../usr/include
unix {
    target.path = ../usr/lib
    INSTALLS += target
}

macx{
INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/include/c++/4.2.1
QMAKE_MAC_SDK = macosx10.9
}

