#include "../objects/greyscale_image.h"
#include "../objects/point_xy_greyscale.h"

#include <vector>

namespace image_processing
{
    /** @return all the x coordinates in the parameter greyscale vector */
    std::vector<float> greyscale_x_coords(std::vector<point_xy_greyscale> greyscale_vector);

    /** @return all the y coordinates in the parameter greyscale vector */
    std::vector<float> greyscale_y_coords(std::vector<point_xy_greyscale> greyscale_vector);

    /** @return all the greyscale values in the image **/
    std::vector<unsigned short> greyscale_values(greyscale_image gs_img);

    /**
     * @brief greyscale_to_image converts an 2D greyscale points array into a depth image
     * @param greyscale_vector is the the array to be converted
     * @param x_epsilon helps delimiting the x cases of the image
     * @return the resulted depth image
     */
    greyscale_image greyscale_to_image(std::vector<point_xy_greyscale> greyscale_vector, float x_epsilon);

    /**
     * @brief normalize augments the greyscale differences between the pixels of an image
     * @param gs_img_ptr is a pointer to the greyscale image to be normalized
     */
    void normalize(greyscale_image *gs_img_ptr);
}
