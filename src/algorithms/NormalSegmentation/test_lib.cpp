#include "test_lib.h"

void test_geom::test_crossprod()
{
    // test cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    // test points
    pcl::PointXYZRGB pt1;
    pcl::PointXYZRGB pt2;
    pcl::PointXYZRGB pt3;

    // cross_product parameters
    geom::vectors::vector3 vect1;
    geom::vectors::vector3 vect2;

    // test cross_product
    geom::vectors::vector3 *crossprod_res;

    // initializing points
    pt1.x = 0; pt1.y = 0; pt1.z = 0;
    pt2.x = 0; pt2.y = 1; pt2.z = 0;
    pt3.x = 1; pt3.y = 0; pt3.z = 0;

    // initializing cloud
    cloud->push_back(pt1);
    cloud->push_back(pt2);
    cloud->push_back(pt3);

    // initializing vectors
    // defining the first vector
    vect1.set_x(pt1.x - pt2.x);
    vect1.set_y(pt1.y - pt2.y);
    vect1.set_z(pt1.z - pt2.z);

    // defining the second vector
    vect2.set_x(pt1.x - pt3.x);
    vect2.set_y(pt1.y - pt3.y);
    vect2.set_z(pt1.z - pt3.z);

    // calculating cross product and storing it
    crossprod_res = geom::vectors::cross_product(vect1, vect2);

    // printing information
    std::cout << "CROSS PRODUCT TEST" << std::endl << std::endl;
    std::cout << "Vector coordinates: " << std::endl;
    std::cout << "X: " << crossprod_res->get_x() << "\tY: " << crossprod_res->get_y()
                 << "\tZ: " << crossprod_res->get_z() << std::endl << std::endl;
    std::cout << "END CROSS PRODUCT TEST" << std::endl;
}
