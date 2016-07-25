#-------------------------------------------------
#
# Project created by QtCreator 2016-05-20T09:18:50
#
#-------------------------------------------------

QT += core gui

QMAKE_CXXFLAGS += -fopenmp
QMAKE_LFLAGS += -fopenmp
QMAKE_CFLAGS_RELEASE += -fopenmp
QMAKE_CFLAGS_DEBUG += -fopenmp

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = 3d_objects_boundries_detection_test_app
TEMPLATE = app

CONFIG += c++11

SOURCES += main.cpp\
        mainwindow.cpp \
    test_lib.cpp \
    cloud_crop_form.cpp \
    cloud_homog_form.cpp \
    normal_estimation_form.cpp \
    cloud_to_image_form.cpp \
    cont_det_form.cpp \
    ../cos_lib/src/aux_op.cpp \
    ../cos_lib/src/bounding.cpp \
    ../cos_lib/src/bounding_box.cpp \
    ../cos_lib/src/cloud_manip.cpp \
    ../cos_lib/src/clustering.cpp \
    ../cos_lib/src/image.cpp \
    ../cos_lib/src/image_greyscale.cpp \
    ../cos_lib/src/image_io.cpp \
    ../cos_lib/src/image_mixed.cpp \
    ../cos_lib/src/image_processing.cpp \
    ../cos_lib/src/image_rgb.cpp \
    ../cos_lib/src/line.cpp \
    ../cos_lib/src/lineFinding.cpp \
    ../cos_lib/src/ModelDetection.cpp \
    ../cos_lib/src/normal_estimation.cpp \
    ../cos_lib/src/plane.cpp \
    ../cos_lib/src/point_clstr.cpp \
    ../cos_lib/src/point_xy_greyscale.cpp \
    ../cos_lib/src/point_xy_mixed.cpp \
    ../cos_lib/src/point_xy_rgb.cpp \
    ../cos_lib/src/vector3.cpp \
    ../cos_lib/src/cloud_io.cpp

HEADERS  += mainwindow.h \
    test_lib.h \
    normal_estimation_form.h \
    cloud_homog_form.h \
    cloud_crop_form.h \
    cloud_to_image_form.h \
    cont_det_form.h \
    ../cos_lib/include/aux_op.h \
    ../cos_lib/include/bounding.h \
    ../cos_lib/include/bounding_box.h \
    ../cos_lib/include/cloud_manip.h \
    ../cos_lib/include/clustering.h \
    ../cos_lib/include/image.h \
    ../cos_lib/include/image_greyscale.h \
    ../cos_lib/include/image_io.h \
    ../cos_lib/include/image_mixed.h \
    ../cos_lib/include/image_processing.h \
    ../cos_lib/include/image_rgb.h \
    ../cos_lib/include/invalid_cloud_pointer.h \
    ../cos_lib/include/invalid_path.h \
    ../cos_lib/include/line.h \
    ../cos_lib/include/lineFinding.h \
    ../cos_lib/include/ModelDetection.h \
    ../cos_lib/include/normal_estimation.h \
    ../cos_lib/include/plane.h \
    ../cos_lib/include/point_clstr.h \
    ../cos_lib/include/point_xy_greyscale.h \
    ../cos_lib/include/point_xy_mixed.h \
    ../cos_lib/include/point_xy_rgb.h \
    ../cos_lib/include/vector3.h \
    ../cos_lib/include/cloud_io.h


FORMS    += mainwindow.ui \
    normal_estimation_form.ui \
    cloud_homog_form.ui \
    cloud_crop_form.ui \
    cloud_to_image_form.ui \
    cont_det_form.ui


HEADERS +=

    target.path = /usr/lib
    INSTALLS += target

# -------------------- PCL --------------------
INCLUDEPATH += /usr/include/pcl-1.7
DEPENDPATH += /usr/include/pcl-1.7

# -------------------- OPENCV --------------------
INCLUDEPATH += /usr/include/opencv
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

# VTK
INCLUDEPATH += /usr/include/vtk-5.8
DEPENDPATH += /usr/include/vtk-5.8

# linking pcl library
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

# linking boost library
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

# linking opencv library
LIBS += -L/usr/lib/i386-linux-gnu/ -lopencv_core
LIBS += -L/usr/lib/i386-linux-gnu/ -lopencv_imgproc
LIBS += -L/usr/lib/i386-linux-gnu/ -lopencv_highgui
LIBS += -L/usr/lib/i386-linux-gnu/ -lopencv_ml
LIBS += -L/usr/lib/i386-linux-gnu/ -lopencv_video
LIBS += -L/usr/lib/i386-linux-gnu/ -lopencv_features2d
LIBS += -L/usr/lib/i386-linux-gnu/ -lopencv_calib3d
LIBS += -L/usr/lib/i386-linux-gnu/ -lopencv_objdetect
LIBS += -L/usr/lib/i386-linux-gnu/ -lopencv_contrib
LIBS += -L/usr/lib/i386-linux-gnu/ -lopencv_legacy
LIBS += -L/usr/lib/i386-linux-gnu/ -lopencv_flann

LIBS += -fopenmp
