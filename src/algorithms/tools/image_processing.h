#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include "../tools/aux.h"
#include "../2d/image_greyscale.h"
#include "../2d/image_rgb.h"
#include "../2d/image_mixed.h"
#include "../2d/point_xy_greyscale.h"
#include "../2d/point_xy_rgb.h"
#include "../2d/point_xy_mixed.h"
#include "cloud_manip.h"

#include <vector>
#include <stdint.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace image_processing
{
    /** @return all the x coordinates in the parameter greyscale vector */
    std::vector<float> greyscale_vector_x_coords(std::vector<point_xy_greyscale> greyscale_vector);

    /** @return all the y coordinates in the parameter greyscale vector */
    std::vector<float> greyscale_vector_y_coords(std::vector<point_xy_greyscale> greyscale_vector);

    /** @return all of the x coordinates in the parameter rgb vector */
    std::vector<float> rgb_vector_x_coords(std::vector<point_xy_rgb> rgb_vector);

    /** @return all of the y coordinates in the parameter rgb vector */
    std::vector<float> rgb_vector_y_coords(std::vector<point_xy_rgb> rgb_vector);

    /** @return all the x coordinates in a parameter mixed vector */
    std::vector<float> mixed_vector_x_coords(std::vector<point_xy_mixed> mixed_vector);

    /** @return all the y coordinates in a parameter mixed vector */
    std::vector<float> mixed_vector_y_coords(std::vector<point_xy_mixed> mixed_vector);

    /** @return all the greyscale values in the image **/
    std::vector<unsigned short> greyscale_image_values(image_greyscale gs_img);

    /**
     * @brief greyscale_vector_to_image converts a 2D greyscale points array into a depth image
     * @param greyscale_vector is the the array to be converted
     * @param x_epsilon is used for delimiting the x cases of the image
     * @return the resulted depth image
     */
    image_greyscale greyscale_vector_to_image(std::vector<point_xy_greyscale> greyscale_vector, float x_epsilon);

    /**
     * @brief mixed_vector_to_image converts a 2D mixed points array into a mixed image
     * @param mixed_vector is the array to be converted
     * @param x_epsilon is used for delimiting the x cases of the image
     * @return the resulted mixed image
     */
    image_mixed mixed_vector_to_image(std::vector<point_xy_mixed> mixed_vector, float x_epsilon);

    /**
     * @brief mixed_image_to_rgb turns a mixed image into an rgb image
     * @param mixed_img the mixed image to be turned into rgb
     * @return the rgb image
     */
    image_rgb mixed_image_to_rgb(image_mixed mixed_img);

    /**
     * @brief mixed_image_to_greyscale turns a mixed image into a grey scale image
     * @param mixed_img the mixed image to be turned into grey scale
     * @return the grey scale image
     */
    image_greyscale mixed_image_to_greyscale(image_mixed mixed_img);

    /**
     * @brief greyscale_image_to_cloud creates a pointer to a point cloud based on a greyscale image
     * @param gs_img is the image that serves as the base for the creation of the point cloud
     * @param base_cloud_ptr is a pointer to the cloud used to create the image
     * @return a pointer to the resulted cloud
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr greyscale_image_to_cloud(image_greyscale gs_img,
                                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud_ptr);

    /**
     * @brief mixed_image_to_cloud creates a pointer to a point cloud based on a mixed image
     * @param mixed is the image that serves as the base for the creation of the point cloud
     * @param base_cloud_ptr is a pointer to the cloud used to create the image
     * @return a pointer to the resulted cloud
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mixed_image_to_cloud(image_mixed mixed_img,
                                                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_cloud_ptr);

    /**
     * @brief greyscale_image_to_mat transforms an image_greyscale object into a cv::Mat object
     * @param gs_img is the image_greyscale to be transformed into a Mat
     * @return the grey scale image as a Mat object
     */
    cv::Mat greyscale_image_to_mat(image_greyscale gs_img);

    /**
     * @brief rgb_image_to_mat transforms an image_rgb object into a cv::Mat object
     * @param rgb_img is the image_rgb to be transformed into a Mat
     * @return the rgb image as a Mat object
     */
    cv::Mat rgb_image_to_mat(image_rgb rgb_img);

    /**
     * @brief normalize augments the greyscale differences between the pixels of an image
     * @param gs_img_ptr is a pointer to the greyscale image to be normalized
     */
    void normalize(image_greyscale *gs_img_ptr);
}

#endif // IMAGE_PROCESSING_H
