#-------------------------------------------------
#
# Project created by QtCreator 2014-09-02T19:26:53
#
#-------------------------------------------------

QT       += core gui \
        opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = HuboCma
TEMPLATE = app


SOURCES += main.cpp\
        hubomaincontroller.cpp \
    huboglviewer.cpp \
    huboglwidget.cpp

HEADERS  += hubomaincontroller.h \
    huboglviewer.h \
    huboglwidget.h

FORMS    += hubomaincontroller.ui \
    huboglviewer.ui

INCLUDEPATH += ../usr/include \
        ../HpMotion \
        ../CmaOptimizer \
        ../MotionGlWidget

LIBS += -L../usr/lib \
        -fopenmp \
        -lGL \
        -lHpMotion \
        -lCmaOptimizer \
        -lMotionGlWidget \
        -lexample_boundary \
        -lcma \
        -lGLU \
        -lvpLib
