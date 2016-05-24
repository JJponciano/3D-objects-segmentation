#include "image_io.h"

image_greyscale image_io::import_greyscale_image(std::string path)
{
    // result
    image_greyscale *gs_img;
    unsigned long height;
    unsigned long width;
    unsigned long y = 0;

    // in
    std::ifstream image_file;
    std::string line;
    int line_count = 0;

    image_file.open(path, std::ios::in);

    if (!image_file.is_open())
    {
        throw invalid_path();
    }

    while (std::getline(image_file, line))
    {
        std::istringstream iss(line);

        // ignoring comments ('#')
        if (line.at(0) != '#')
        {
            line_count++;

            // image dimensions are given in the second non commented line of the file
            if (line_count == 2)
            {
                iss >> width >> height;
                gs_img = new image_greyscale(width, height);
            }

            // line 3 is the maximum value of grey in the file and it is followed by the points
            if (line_count > 3)
            {
                int grey_value;

                // filling current row
                for (unsigned long x = 0; x < width; x++)
                {
                    iss >> grey_value;
                    gs_img->set_grey_at(y, x, grey_value);
                }

                y++;    // incrementing the next row of the image
            }
        }
    }

    return *gs_img;
}

void image_io::export_greyscale_image(std::string path, unsigned short max_grey_value, image_greyscale gs_img)
{
    if (gs_img.width() == 0 || gs_img.height() == 0)
    {
        throw std::logic_error("Invalid grey scale image.");
    }

    if (max_grey_value > 255)
        throw std::logic_error("Invalid max_grey_value (must be smaller than 255).");

    std::ofstream image_file;
    std::string line;
    const std::string magic_number = "P2";  // defines the format of the file

    image_file.open(path, std::ios::out);

    if (!image_file.is_open())
    {
        throw invalid_path();
    }

    line = magic_number;
    line.append("\n");
    image_file << line;
    line.clear();
    line = boost::lexical_cast<std::string>(gs_img.width()) + "\t"
           + boost::lexical_cast<std::string>(gs_img.height()) + "\n";
    image_file << line;
    line.clear();
    line = boost::lexical_cast<std::string>(max_grey_value) + "\n";
    image_file << line;

    for (unsigned long y = 0; y < gs_img.height(); y++)
    {
        line.clear();

        for (unsigned long x = 0; x < gs_img.width(); x++)
            line += boost::lexical_cast<std::string>(gs_img.get_grey_at(y, x)) + "\t";

        line += "\n";
        image_file << line;
    }
}

image_rgb image_io::import_rgb_image(std::string path)
{
    // result
    image_rgb *rgb_img;
    unsigned long height;
    unsigned long width;
    unsigned long y = 0;

    // in
    std::ifstream image_file;
    std::string line;
    int line_count = 0;

    image_file.open(path, std::ios::in);

    if (!image_file.is_open())
    {
        throw invalid_path();
    }

    while (std::getline(image_file, line))
    {
        std::istringstream iss(line);

        // ignoring comments ('#')
        if (line.at(0) != '#')
        {
            line_count++;

            // image dimensions are given in the second non commented line of the file
            if (line_count == 2)
            {
                iss >> width >> height;
                rgb_img = new image_rgb(width, height);
            }

            // line 3 is the maximum value of grey in the file and it is followed by the points
            if (line_count > 3)
            {
                unsigned short r, g, b;
                uint32_t rgb;

                // filling current row
                for (unsigned long x = 0; x < width; x++)
                {
                    for (unsigned short color_component = 0; color_component < 3; color_component++)
                    {
                        iss >> r >> g >> b;
                        rgb = ((uint32_t)r << 16) | (uint32_t)g << 8 | (uint32_t)b;
                        rgb_img->set_rgb_at(y, x, rgb);
                    }
                }

                y++;    // incrementing the next row of the image
            }
        }
    }

    return *rgb_img;
}

void image_io::export_rgb_image(std::string path, unsigned int max_rgb_value, image_rgb rgb_img)
{
    if (rgb_img.width() == 0 || rgb_img.height() == 0)
    {
        throw std::logic_error("Invalid rgb image.");
    }

    if (max_rgb_value > 255)
    {
        throw std::logic_error("Invalid max_rgb_value (must be smaller than 255).");
    }

    std::ofstream image_file;
    std::string line;
    const std::string magic_number = "P3";  // defines the format of the file

    image_file.open(path, std::ios::out);

    if (!image_file.is_open())
    {
        throw invalid_path();
    }

    line = magic_number;
    line.append("\n");
    image_file << line;
    line.clear();
    line = boost::lexical_cast<std::string>(rgb_img.width()) + "\t"
           + boost::lexical_cast<std::string>(rgb_img.height()) + "\n";
    image_file << line;
    line.clear();
    line = boost::lexical_cast<std::string>(max_rgb_value) + "\n";
    image_file << line;

    for (unsigned long y = 0; y < rgb_img.height(); y++)
    {
        line.clear();

        for (unsigned long x = 0; x < rgb_img.width(); x++)
        {
            line += boost::lexical_cast<std::string>((short)(rgb_img.get_red_at(y, x)))
                    + " " + boost::lexical_cast<std::string>((short)(rgb_img.get_green_at(y, x)))
                    + " " + boost::lexical_cast<std::string>((short)(rgb_img.get_blue_at(y, x)))
                    + "\t";
        }

        line += "\n";
        image_file << line;
    }
}
