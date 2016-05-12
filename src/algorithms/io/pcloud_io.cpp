#include "pcloud_io.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud_io::import_cloud(std::string path, bool is_rgb)
{
    std::string ext;    // files extension

    size_t i = path.rfind('.', path.length());

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    if (i != std::string::npos)
      ext = path.substr(i+1, path.length() - i);

    if (ext.compare("pcd") == 0)

    {
        try
        {
            pcl::io::loadPCDFile<pcl::PointXYZRGB> (path, *cloud);
        }

        catch(std::exception& e)
        {
            std::cout << "pcloud_io::load_cloud : Invalid .pcd file.";
        }
    }

    else if (ext.compare("txt") == 0)
    {
        try
        {
            cloud = pcloud_io::import_cloud_txt(path, is_rgb);
        }

        catch(char const* io_err)
        {
            throw io_err;
        }
    }

    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud_io::import_cloud_txt(std::string pathname, bool is_rgb)
{
        QFile file( QString(pathname.c_str()) );

        //if the file is not already open
        if(file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QTextStream flux(&file);
            QString  ligne; // variable contenant chaque ligne lue
            std::vector<float> px;
            std::vector<float> py;
            std::vector<float> pz;
            std::vector<float> pt_r;
            std::vector<float> pt_g;
            std::vector<float> pt_b;

            while(!flux.atEnd())
            {
                ligne= flux.readLine();
                // split the line with space as a separator character
                QStringList result =ligne.split("\t");
                //convert coordonated Qstring to float coordinates to add in vector
                if(result.size()<3) return nullptr;

                else
                {
                    //read each number and set it in the corresponding vector
                    QString r;

                    r=result.at(0);
                    px.push_back(r.toFloat());

                    r=result.at(1);
                    py.push_back(r.toFloat());

                    r=result.at(2);
                    pz.push_back(r.toFloat());

                    if (is_rgb)
                    {
                        pt_r.push_back(result.at(3).toFloat());
                        pt_g.push_back(result.at(4).toFloat());
                        pt_b.push_back(result.at(5).toFloat());
                    }

                    else
                    {
                        pt_r.push_back((float)255);
                        pt_g.push_back((float)255);
                        pt_b.push_back((float)255);
                    }
                }
            }

            //close file
            file.close();

            // create cloud point and cloud file pcl
            // Fill in the cloud data
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            for (unsigned int i = 0; i < px.size(); ++i)
            {
               pcl::PointXYZRGB pt;
               pt.x=px[i];
               pt.y=py[i];
               pt.z=pz[i];
               pt.r = pt_r[i];
               pt.g = pt_g[i];
               pt.b = pt_b[i];

               cloud->push_back(pt);
            }

            return cloud;
    }

    else throw "pcloud_io::import_cloud : Invalid .txt file.";
}


void pcloud_io::export_cloud(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
    std::ofstream cloud_file;
    std::string line;

    // cloud iterator
    pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it;

    // opening file
    cloud_file.open(path, std::ios::out);

    if (!cloud_file.is_open())
    {
        throw "pcloud_io::export_cloud : Could not write file at \"" + path + "\".";
    }

    else
    {
        if (!cloud_ptr)
            throw "pcloud_io::export_cloud : Invalid cloud.";

        else
        {
            for (cloud_it = cloud_ptr->points.begin(); cloud_it < cloud_ptr->points.end(); cloud_it++)
            {
                line = boost::lexical_cast<std::string>((float)(*cloud_it).x) + "\t"
                        + boost::lexical_cast<std::string>((float)(*cloud_it).y) + "\t"
                        + boost::lexical_cast<std::string>((float)(*cloud_it).z) + "\t"
                        + boost::lexical_cast<std::string>((short)(*cloud_it).r) + "\t"
                        + boost::lexical_cast<std::string>((short)(*cloud_it).g) + "\t"
                        + boost::lexical_cast<std::string>((short)(*cloud_it).b)
                        + "\n";

                cloud_file << line;
            }
        }
    }
}

void pcloud_io::export_greyscale_vector(std::string path, std::vector<point_xy_greyscale> greyscale_vector)
{
    std::ofstream greyscale_file;
    std::string line;;

    // opening file
    greyscale_file.open(path, std::ios::out);

    if (!greyscale_file.is_open())
    {
        throw "pcloud_io::export_greyscale : Could not write file at \"" + path + "\".";
    }

    else
    {
        if (greyscale_vector.empty())
            throw "pcloud_io::export_greyscale : Invalid greyscale vector.";

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

void pcloud_io::export_greyscale_image(std::string path, std::string magic_number, greyscale_image gs_img)
{
    std::ofstream image_file;
    std::string line;
    const std::string P_5 = "P5";
    const std::string P_2 = "P2";

    // opening file
    image_file.open(path, std::ios::out);

    if (magic_number.compare(P_5)
            && magic_number.compare(P_2))
        throw "pcloud_io::export_image : magic_number must be \"P5\" or \"P2\"";

    else
    {
        if (!image_file.is_open())
        {
            throw "pcloud_io::export_image : Could not write file at \"" + path + "\".";
        }

        else
        {
            if (gs_img.width() == 0 || gs_img.height() == 0)
                throw "pcloud_io::export_image : Invalid image.";

            else
            {
                line = magic_number.append("\n");
                image_file << line;
                line.clear();
                line = boost::lexical_cast<std::string>(gs_img.width()) + "\t"
                       + boost::lexical_cast<std::string>(gs_img.height()) + "\n";
                image_file << line;
                line.clear();
                line = boost::lexical_cast<std::string>(255);
                image_file << line;

                for (unsigned long i = 0; i < gs_img.height(); i++)
                {
                    line.clear();

                    for (unsigned long j = 0; j < gs_img.width(); j++)
                    {
                        line += boost::lexical_cast<std::string>(gs_img.get_grey_at(i, j)) + "\t";
                    }

                    line += "\n";
                    image_file << line;
                }
            }
        }
    }
}
