#include <QCoreApplication>

#include "segm.h"
#include "../io/pcloud_io.h"
#include "test_lib.h"

#include <iostream>
#include <time.h>


#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

/* global variables */
// point cloud to be examined
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
std::clock_t t_start;

// pcd file path
std::string pcd_path;

/* function declaration */
// initializes the path of the pcd file
void init_path();

// initializes the point cloud
void init_cloud();

// presents the user with the test choices
void test_menu();

// tests segmentation by normal vectors with personal library
void test_ownnormest();

// tests segmentation by normal vectors
void test_pclnormest();

// prints the result of a test function
void printr();

// clears the screen
void clear_screen();

// vizualize cloud
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

/* function implementation */
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    /*
    // initializing timer
    t_start = -1;

    // initializing cloud
    init_cloud();

    // displaying the menu allowing the user to test functions
    test_menu();*/

    // init_cloud();

    /*cloud = pcloud_io::load_cloudtxt("/home/vlad-adrian/3D-objects-segmentation/data/table.txt");
    std::cout << cloud->at(5).rgb << std::endl;

    std::vector<std::pair<pcl::PointXYZRGB *, std::string>> cn = geom::estim_normals(cloud);
    std::vector<std::vector<pcl::PointXYZRGB *> *> pt_grp = segm::pts_regrp(cn);
    segm::pts_colsegm(pt_grp);

    std::cout << cloud->at(5).rgb << std::endl;

    pcloud_io::cloud_txt("/home/vlad-adrian/Documents/txt_tabtest.txt", cloud);

    std::cout << "done" << std::endl;*/

    test_geom::test_crossprod();

    return a.exec();
}

void init_path()
{
    clear_screen();
    std::cout << ".pcd file path: ";
    std::cin >> pcd_path;
    std::cout << std::endl;
}

void init_cloud()
{
    bool err;   // true if error

    do
    {
        err = false;
        init_path();

        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_path, *cloud) == -1) //* load the file
            err = true;

    } while (err);
}

void test_menu()
{
    bool err;   // true if error
    bool exit;
    int sel;    // user input

    do
    {
        clear_screen();
        err = false;
        exit = false;

        std::cout << "Test functions: " << std::endl << std::endl;
        std::cout << "\t1. test own normal estimation;" << std::endl;
        std::cout << "\t2. test pcl normal estimation;" << std::endl;
        std::cout << "\t0. quit.";
        std::cout << std::endl << std::endl;
        std::cout << "Your choice: ";

        std::cin >> sel;
        std::cout << std::endl;

        // options
        switch (sel)
        {
            case 0 :
                exit = true;
                break;
            case 1 :
                t_start = clock();
                test_ownnormest();
                break;
            case 2 :
                t_start = clock();
                test_pclnormest();
                break;
            default :
                err = true;
                break;
        }

        // printing results
        if (t_start != -1)
            printr();

        // resetting cin
        std::cin.ignore();

        // giving user time to see output
        std::cin.get();

    }   while (err || !exit);
}

void test_ownnormest()
{
    std::vector<std::pair<pcl::PointXYZRGB *, std::string>> cn = geom::estim_normals(cloud);
}

void test_pclnormest()
{
    geom::estim_normals(cloud);
}

void printr()
{
    std::cout << "Total execution time: " << (float)(clock() - t_start)/(CLOCKS_PER_SEC) << " seconds." << std::endl;
}

void clear_screen()
{
    std::cout << std::string(100, '\n');
}
