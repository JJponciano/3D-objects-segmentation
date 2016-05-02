#-------------------------------------------------
#
# Project created by QtCreator 2016-02-25T11:30:04
#
#-------------------------------------------------

QT += core
QT -= gui
QT += opengl
CONFIG += C++11

TARGET = PCI
TEMPLATE = app

DEFINES += PCI_LIBRARY

SOURCES += main.cpp\
clustering.cpp\
./cloud_manip/cloud_manip.cpp\
./io/pcloud_io.cpp\
./objects/pointbool.cpp\
./objects/bounding_box.cpp\
./geom_op/geom_op.cpp\
./geom_op/vector3.cpp

HEADERS += clustering.h\
./cloud_manip/cloud_manip.h\
./io/pcloud_io.h\
./objects/pointbool.h\
./objects/bounding_box.h\
./geom_op/geom_op.h\
./geom_op/vector3.h

unix {
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
}
