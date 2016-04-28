#include <QCoreApplication>
#include <pcl/point_cloud.h>
#include "clustering.h"
#include "../io/pcloud_io.h"
#include "../cloud_manipulation/cloud_manip.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    return a.exec();
}
