#include "cloud_io.h"

namespace ns_cos = cloud_object_segmentation;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ns_cos::io::import_cloud_txt(std::string pathname)
{
    QFile file(QString(pathname.c_str()));

    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
        throw ns_cos::except::invalid_path();

    QTextStream flux(&file);
    QString  line;  // each read line
    std::vector<float> pt_x;
    std::vector<float> pt_y;
    std::vector<float> pt_z;
    std::vector<float> pt_r;
    std::vector<float> pt_g;
    std::vector<float> pt_b;

    while(!flux.atEnd())
    {
        line = flux.readLine();
        QStringList result = line.split("\t"); // split the line with space as a separator character

        if(result.size() < 3)
            throw ns_cos::except::invalid_path();

        QString r;  // reads each coordinate

        r = result.at(0);
        pt_x.push_back(r.toFloat());

        r = result.at(1);
        pt_y.push_back(r.toFloat());

        r = result.at(2);
        pt_z.push_back(r.toFloat());

        // point cloud is rgb
        if (result.size() == 6)
        {
            pt_r.push_back(result.at(3).toFloat());
            pt_g.push_back(result.at(4).toFloat());
            pt_b.push_back(result.at(5).toFloat());
        }

        // point cloud is not rgb
        else if (result.size() < 6)
        {
            pt_r.push_back((float)255);
            pt_g.push_back((float)255);
            pt_b.push_back((float)255);
        }

        // unknown format
        else
            throw ns_cos::except::invalid_path();
    }

    file.close();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // fill in the cloud data
    for (unsigned int i = 0; i < pt_x.size(); i++)
    {
       pcl::PointXYZRGB pt;

       pt.x = pt_x[i];
       pt.y = pt_y[i];
       pt.z = pt_z[i];
       pt.r = pt_r[i];
       pt.g = pt_g[i];
       pt.b = pt_b[i];
       cloud->push_back(pt);
    }

    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ns_cos::io::import_cloud(std::string path)
{
    std::string ext;    // files extension
    size_t i = path.rfind('.', path.length());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    if (i != std::string::npos)
      ext = path.substr(i+1, path.length() - i);

    if (ext.compare("pcd") == 0)

    {
        pcl::io::loadPCDFile<pcl::PointXYZRGB> (path, *cloud);

        if (!cloud)
            throw ns_cos::except::invalid_path();
    }

    else if (ext.compare("txt") == 0)
        cloud = ns_cos::io::import_cloud_txt(path);

    return cloud;
}

void ns_cos::io::export_cloud(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{

    if (!cloud_ptr)
        throw ns_cos::except::invalid_cloud_pointer();

    std::ofstream cloud_file;
    std::string line;

    // opening file
    cloud_file.open(path, std::ios::out);

    if (!cloud_file.is_open())
        throw ns_cos::except::invalid_path();

    for (auto cloud_it = cloud_ptr->points.begin(); cloud_it < cloud_ptr->points.end(); cloud_it++)
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
