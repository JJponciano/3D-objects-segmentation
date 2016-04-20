#include "pcloud_io.h"

void pcloud_io::cloud_txt(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc)
{
    std::ofstream cloud_txt;
    std::string line;

    // cloud iterator
    pcl::PointCloud<pcl::PointXYZRGB>::iterator cloud_it;

    // opening file
    cloud_txt.open(path, std::ios::out);

    for (cloud_it = pc->points.begin(); cloud_it < pc->points.end(); cloud_it++)
    {
        line = boost::lexical_cast<std::string>((float)(*cloud_it).x) + "\t" + boost::lexical_cast<std::string>((float)(*cloud_it).y)
               + "\t" + boost::lexical_cast<std::string>((float)(*cloud_it).z) + "\t" + boost::lexical_cast<std::string>((short)(*cloud_it).r)
                  + "\t" + boost::lexical_cast<std::string>((short)(*cloud_it).g) + "\t" + boost::lexical_cast<std::string>((short)(*cloud_it).b)
                     + "\n";

        cloud_txt << line;
    }

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud_io::load_cloudtxt(std::string pathname)
{
    QFile fichier( QString(pathname.c_str()) );
    //if the file is not already open
    if(fichier.open(QIODevice::ReadOnly | QIODevice::Text))
    {
       QTextStream flux(&fichier);
       QString  ligne; // variable contenant chaque ligne lue
       std::vector<float> px;
       std::vector<float> py;
       std::vector<float> pz;
       //std::vector<int> pintensity;

       while(!flux.atEnd())
       {
           ligne= flux.readLine();
           // split the line with space as a separator character
           QStringList result =ligne.split("\t");
           //convert coordonated Qstring to float coordinates to add in vector
           if(result.size()<3) return nullptr;
           else{
               //read each number and set it in the corresponding vector
               QString r=result.at(0);
               px.push_back(r.toFloat());
               r=result.at(1);
               py.push_back(r.toFloat());
               r=result.at(2);
               pz.push_back(r.toFloat());
               //r=result.at(3);
               //pintensity.push_back(r.toInt());

           }
       }

       //close file
       fichier.close();
       // create cloud point and cloud file pcl
       // Fill in the cloud data
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

       for (int i = 0; i <px.size(); ++i)
       {
           pcl::PointXYZRGB p;
           p.x=px[i];
           p.y=py[i];
           p.z=pz[i];
//           p.r = 255;
//           p.g = 255;
//           p.b = 255;

           //p.intensity=pintensity[i];
           cloud->push_back(p);
       }
       return cloud;

    }else return nullptr;
}
