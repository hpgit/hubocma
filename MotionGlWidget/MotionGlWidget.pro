#-------------------------------------------------
#
# Project created by QtCreator 2014-09-02T21:45:47
#
#-------------------------------------------------

QT       += core gui \
        widgets opengl

TARGET = MotionGlWidget
TEMPLATE = lib
CONFIG += staticlib

SOURCES += motionglwidget.cpp \
    camera.cpp

HEADERS += motionglwidget.h \
    camera.h
unix {
    target.path = ../usr/lib
    INSTALLS += target
}

INCLUDEPATH += ../usr/include
macx{
INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/include/c++/4.2.1
QMAKE_MAC_SDK = macosx10.9
}
