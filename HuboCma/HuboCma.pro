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
    huboglwidget.cpp \
    hubotrackingviewer.cpp \
    hubotrackingmanage.cpp \
    huboikmanage.cpp \
    huboikviewer.cpp

HEADERS  += hubomaincontroller.h \
    huboglviewer.h \
    huboglwidget.h \
    hubotrackingviewer.h \
    hubotrackingmanage.h \
    huboikmanage.h \
    huboikviewer.h

FORMS    += hubomaincontroller.ui \
    huboglviewer.ui \
    hubotrackingmanage.ui \
    huboikmanage.ui

INCLUDEPATH += ../usr/include \
        ../HpMotion \
        ../CmaOptimizer \
        ../MotionGlWidget

LIBS += -L"../usr/lib" \
        -fopenmp \
        -lGL \
        -lHpMotion \
        -lCmaOptimizer \
        -lMotionGlWidget \
        -lboundary_transformation \
        -lcma \
        -lGLU \
        -lvpLib

CONFIG += console
win32{
LIBS -= -lGL
LIBS -= -lGLU
LIBS -= -fopenmp
LIBS += -lopengl32
LIBS += -lglu32

}

macx {
INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/include/c++/4.2.1
LIBS -= -L"../usr/lib"
LIBS += -L"../../usr/lib"
LIBS -= -lGL
LIBS -= -lGLU
LIBS += -L/usr/local/lib \
        -L/usr/lib \
        -L/usr/lib/system \
        -framework OpenGL
QMAKE_MAC_SDK = macosx10.9
QMAKE_INFO_PLIST = /Users/trif/Qt5.3.1/5.3/clang_64/mkspecs/macx-clang/Info.plist.app
}
