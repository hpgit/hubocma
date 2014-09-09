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

unix {
    target.path = ../usr/lib
    INSTALLS += target
}

macx {
INCLUDEPATH += /usr/include/c++/4.2.1
QMAKE_MAC_SDK = macosx10.9
}

