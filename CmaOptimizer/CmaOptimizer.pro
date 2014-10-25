#-------------------------------------------------
#
# Project created by QtCreator 2014-09-04T16:53:30
#
#-------------------------------------------------

QT       -= core gui

TARGET = CmaOptimizer
TEMPLATE = lib
CONFIG += staticlib

SOURCES += CmaOptimizer.cpp

HEADERS += CmaOptimizer.h

INCLUDEPATH += ../usr/include

QMAKE_CXXFLAGS_WARN_ON = ""
QMAKE_CXXFLAGS_WARN_OFF += -Wno-unused-parameter
				-Wno-unused-variable

unix {
    target.path = ../usr/lib
    INSTALLS += target
}

macx {
INCLUDEPATH += /usr/include/c++/4.2.1
QMAKE_CXXFLAGS += -stdlib=libstdc++
QMAKE_MAC_SDK = macosx10.10
}

