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

QMAKE_CXXFLAGS_WARN_ON = ""
QMAKE_CXXFLAGS_WARN_OFF += -Wno-unused-parameter
				-Wno-unused-variable
unix {
    target.path = ../usr/lib
    INSTALLS += target
}

INCLUDEPATH += ../usr/include
macx{
#INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/include/c++/4.2.1
QMAKE_CXXFLAGS += -stdlib=libstdc++
QMAKE_MAC_SDK = macosx10.10
}
