#include "pcloud_io.h"

void pcloud_io::export_cloud(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl)
{
    std::ofstream cloud_file;
    std::string line;

    // cloud iterator
    pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it;

    // opening file
    cloud_file.open(path, std::ios::out);

    if (!cloud_file.is_open())
    {
        throw "pcloud_io::write_cloud : Could not write file at \"" + path + "\".";
    }

    else
    {
        if (!pt_cl)
            throw "pcloud_io::write_cloud : Invalid cloud.";

        else
        {
            for (cloud_it = pt_cl->points.begin(); cloud_it < pt_cl->points.end(); cloud_it++)
            {
                line = boost::lexical_cast<std::string>((float)(*cloud_it).x) + "\t" + boost::lexical_cast<std::string>((float)(*cloud_it).y)
                       + "\t" + boost::lexical_cast<std::string>((float)(*cloud_it).z) + "\t" + boost::lexical_cast<std::string>((short)(*cloud_it).r)
                          + "\t" + boost::lexical_cast<std::string>((short)(*cloud_it).g) + "\t" + boost::lexical_cast<std::string>((short)(*cloud_it).b)
                             + "\n";

                cloud_file << line;
            }
        }
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud_io::import_cloud(std::string path, bool is_rgb)
{
    std::string ext;    // files extension

    size_t i = path.rfind('.', path.length());

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cl;

    if (i != std::string::npos)
      ext = path.substr(i+1, path.length() - i);

    if (ext.compare("pcd") == 0)

    {
        try
        {
            pcl::io::loadPCDFile<pcl::PointXYZRGB> (path, *pt_cl);
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
            pt_cl = pcloud_io::import_cloud_txt(path, is_rgb);
        }

        catch(char const* io_err)
        {
            throw io_err;
        }
    }

    return pt_cl;
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
            std::vector<uint32_t> pt_rgb;

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
                        pt_rgb.push_back((uint32_t)result.at(3).toInt() << 16 | (uint32_t)result.at(4).toInt() << 8 | (uint32_t)result.at(5).toInt());

                    else
                        pt_rgb.push_back(255);
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
               pt.rgb = pt_rgb[i];

               cloud->push_back(pt);
            }

            return cloud;
    }

    else throw "pcloud_io::load_cloud : Invalid .txt file.";
}
