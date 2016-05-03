QT += core
QT -= gui

QMAKE_CXXFLAGS += -fopenmp
QMAKE_LFLAGS += -fopenmp

CONFIG += c++11

TARGET = 3DObjectsBoundriesDetection
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    normal_estimation.cpp \
    e_normal_estimation.cpp \
    ../io/pcloud_io.cpp \
    ../cloud_manip/cloud_manip.cpp \
    ../geom_op/geom_op.cpp \
    ../geom_op/vector3.cpp \
    ../interface/interface.cpp \
    ../objects/bounding_box.cpp \
    ../objects/pointbool.cpp \
    ../objects/point_xy_greyscale.cpp \
    ../objects/greyscale_image.cpp \
    ../test_lib/test_lib.cpp

HEADERS += \
    normal_estimation.h \
    e_normal_estimation.h \
    ../io/pcloud_io.h \
    ../cloud_manip/cloud_manip.h \
    ../geom_op/geom_op.h \
    ../geom_op/vector3.h \
    ../interface/interface.h \
    ../objects/bounding_box.h \
    ../objects/pointbool.h \
    ../objects/point_xy_greyscale.h \
    ../objects/greyscale_image.h \
    ../test_lib/test_lib.h

HEADERS +=

    target.path = /usr/lib
    INSTALLS += target

# -------------------- PCL --------------------
INCLUDEPATH += /usr/include/pcl-1.7
DEPENDPATH += /usr/include/pcl-1.7

# -------------------- OPENCV --------------------
INCLUDEPATH += /usr/include/OpenCV_2.4/include
DEPENDPATH += /usr/include/PC_1.7.1/include

# -------------------- Boost -------------------
INCLUDEPATH += /usr/include/boost
DEPENDPATH += /usr/include/boost
# -------------------- Eigen --------------------
INCLUDEPATH += /usr/include/eigen3
DEPENDPATH += /usr/include/eigen3
# -------------------- flann --------------------
INCLUDEPATH += /usr/include/flann
DEPENDPATH += /usr/include/flann
# -------------------- freeglut --------------------
INCLUDEPATH += /usr/include
DEPENDPATH += /usr/include

# VTK
INCLUDEPATH += /usr/include/vtk-5.8
DEPENDPATH += /usr/include/vtk-5.8

LIBS += -L/usr/lib/ -lpcl_apps
LIBS += -L/usr/lib/ -lpcl_common
LIBS += -L/usr/lib/ -lpcl_features
LIBS += -L/usr/lib/ -lpcl_filters
LIBS += -L/usr/lib/ -lpcl_io_ply
LIBS += -L/usr/lib/ -lpcl_io
LIBS += -L/usr/lib/ -lpcl_kdtree
LIBS += -L/usr/lib/ -lpcl_keypoints
LIBS += -L/usr/lib/ -lpcl_octree
LIBS += -L/usr/lib/ -lpcl_outofcore
LIBS += -L/usr/lib/ -lpcl_people
LIBS += -L/usr/lib/ -lpcl_recognition
LIBS += -L/usr/lib/ -lpcl_registration
LIBS += -L/usr/lib/ -lpcl_sample_consensus
LIBS += -L/usr/lib/ -lpcl_search
LIBS += -L/usr/lib/ -lpcl_segmentation
LIBS += -L/usr/lib/ -lpcl_surface
LIBS += -L/usr/lib/ -lpcl_tracking
LIBS += -L/usr/lib/ -lpcl_visualization

LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_atomic
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_chrono
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_date_time
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_filesystem
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_iostreams
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_regex
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_serialization
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_system
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_thread
LIBS += -L/usr/lib/x86_64-linux-gnu/ -lboost_wserialization
