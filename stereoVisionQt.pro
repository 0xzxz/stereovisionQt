#-------------------------------------------------
#
# Project created by QtCreator 2013-07-13T15:39:09
#
#-------------------------------------------------

QT       += core gui

TARGET = stereoVisionQt
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

#INCLUDEPATH += /usr/include/opencv
INCLUDEPATH += /usr/include/opencv2
INCLUDEPATH += /usr/local/include/pcl-1.6
INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /usr/include/vtk-5.8
INCLUDEPATH += /usr/include/flann
INCLUDEPATH += /usr/include/ni
INCLUDEPATH += /usr/include/nite

LIBS += -L/usr/lib \
-lopencv_core \
-lopencv_highgui \
-lopencv_imgproc \
-lopencv_features2d \
-lopencv_calib3d \

LIBS += -L/usr/local/lib \
-lpcl_apps \
-lpcl_common \
-lpcl_features \
-lpcl_filters \
-lpcl_geometry \
-lpcl_io \
-lpcl_io_ply \
-lpcl_visualization \

LIBS += -L/usr/lib \
-lvtkCommon \
-lvtkFiltering \

LIBS += -L/usr/lib \
-lboost_thread \

LIBS += -L/usr/local/lib \
-lflann_cpp


