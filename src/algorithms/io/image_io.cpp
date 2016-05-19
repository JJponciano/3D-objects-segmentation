#include "image_io.h"

void image_io::export_greyscale_vector(std::string path, std::vector<point_xy_greyscale> greyscale_vector)
{
    std::ofstream greyscale_file;
    std::string line;;

    // opening file
    greyscale_file.open(path, std::ios::out);

    if (!greyscale_file.is_open())
        throw "image_io::export_greyscale : Could not write file at \"" + path + "\".";

    else
    {
        if (greyscale_vector.empty())
            throw "image_io::export_greyscale : Invalid greyscale vector.";

        else
        {
            for (std::vector<point_xy_greyscale>::iterator vector_it = greyscale_vector.begin();
                 vector_it < greyscale_vector.end(); vector_it++)
            {
                line = boost::lexical_cast<std::string>((float)(vector_it->x)) + "\t"
                        + boost::lexical_cast<std::string>((float)(vector_it->y)) + "\t"
                        + boost::lexical_cast<std::string>((unsigned short)(vector_it->greyscale())) + "\t"
                        + "\n";

                greyscale_file << line;
            }
        }
    }
}

void image_io::export_greyscale_image(std::string path, unsigned short max_grey_value, image_greyscale gs_img)
{
    std::ofstream image_file;
    std::string line;
    const std::string magic_number = "P2";  // defines the format of the file

    image_file.open(path, std::ios::out);

    if (!image_file.is_open())
        throw "image_io::export_greyscale_image : Could not write file at \"" + path + "\".";

    else
    {
        if (gs_img.width() == 0 || gs_img.height() == 0)
            throw "image_io::export_greyscale_image : Invalid image.";

        else
        {
            line = magic_number;
            line.append("\n");
            image_file << line;
            line.clear();
            line = boost::lexical_cast<std::string>(gs_img.width()) + "\t"
                   + boost::lexical_cast<std::string>(gs_img.height()) + "\n";
            image_file << line;
            line.clear();
            line = boost::lexical_cast<std::string>(max_grey_value);
            image_file << line;

            for (unsigned long y = 0; y < gs_img.height(); y++)
            {
                line.clear();

                for (unsigned long x = 0; x < gs_img.width(); x++)
                {
                    line += boost::lexical_cast<std::string>(gs_img.get_grey_at(y, x)) + "\t";
                }

                line += "\n";
                image_file << line;
            }
        }
    }
}

void image_io::export_rgb_image(std::string path, unsigned int max_rgb_value, image_rgb rgb_img)
{
    std::ofstream image_file;
    std::string line;
    const std::string magic_number = "P6";  // defines the format of the file

    image_file.open(path, std::ios::out);

    if (!image_file.is_open())
        throw "image_io::export_rgb_image : Could not write file at \"" + path + "\".";

    else
    {
        if (rgb_img.width() == 0 || rgb_img.height() == 0)
            throw "image_io::export_rgb_image : Invalid image.";

        else
        {
            line = magic_number;
            line.append("\n");
            image_file << line;
            line.clear();
            line = boost::lexical_cast<std::string>(rgb_img.width()) + "\t"
                   + boost::lexical_cast<std::string>(rgb_img.height()) + "\n";
            image_file << line;
            line.clear();
            line = boost::lexical_cast<std::string>(max_rgb_value);
            image_file << line;

            for (unsigned long y = 0; y < rgb_img.height(); y++)
            {
                line.clear();

                for (unsigned long x = 0; x < rgb_img.width(); x++)
                {
                    line += boost::lexical_cast<std::string>((short)(rgb_img.get_red_at(y, x)))
                            + boost::lexical_cast<std::string>((short)(rgb_img.get_green_at(y, x)))
                            + boost::lexical_cast<std::string>((short)(rgb_img.get_blue_at(y, x)))
                            + "\t";
                }

                line += "\n";
                image_file << line;
            }
        }
    }
}
