#define RANGE 0.01

#include <QCoreApplication>

#include "test_lib.h"

/* function implementation */
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

<<<<<<< HEAD
    test_normal_estimation();
=======
    test_scaling();
>>>>>>> NormalSegmentation

    return a.exec();
}

