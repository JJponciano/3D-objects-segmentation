#include "cloud_io.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_io::import_cloud(std::string path, bool is_rgb)
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

        catch (std::exception err)
        {
            QString err_msg;

            err_msg.append("cloud_io::import_cloud : ");
            err_msg.append(err.what());
            throw err_msg;
        }
    }

    else if (ext.compare("txt") == 0)
    {
        try
        {
            cloud = cloud_io::import_cloud_txt(path, is_rgb);
        }

        catch (std::exception err)
        {
            QString err_msg;

            err_msg.append("cloud_io::import_cloud : ");
            err_msg.append(err.what());
            throw err_msg;
        }
    }

    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_io::import_cloud_txt(std::string pathname, bool is_rgb)
{
        QFile file( QString(pathname.c_str()) );

        //if the file is not already open
        if(file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
                QString err_msg;

                err_msg.append("cloud_io::import_cloud : Invalid .txt file.");
                throw err_msg;
        }

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


void cloud_io::export_cloud(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
    std::ofstream cloud_file;
    std::string line;

    // cloud iterator
    pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it;

    // opening file
    cloud_file.open(path, std::ios::out);

    if (!cloud_file.is_open())
    {
        QString err_msg;

        err_msg.append("cloud_io::export_cloud : Could not write file at \"");
        err_msg.append(QString::fromUtf8(path.c_str()));
        err_msg.append("\".");
        throw err_msg;
    }

    if (!cloud_ptr)
    {
        QString err_msg;

        err_msg.append("cloud_io::export_cloud : Invalid cloud.");
        throw err_msg;
    }

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
