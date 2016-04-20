#include <stdint.h>
#include "segm.h"

std::vector<std::vector<pcl::PointXYZRGB *> *> segm::pts_regrp(std::vector<std::pair<pcl::PointXYZRGB *, std::string>> cloud_normals)
{
    std::vector<std::vector<pcl::PointXYZRGB *> *> gr_pts;    // contains the different groups of points by their value
    std::map<std::string, int> ptval_dict;  // contains the values added to the dictionary; auxilliary variable for creating the categories
    bool str_found; // true if value found in the dictionary
    int num_categ = 0;  // the number of the category to add the point to; represents an index in the biggest vector
    pcl::PointXYZRGB *curr_pt;   // the key of the pair currently treated
    std::string curr_val;   // the value of the pair currently treated

    // we want to add each point of the cloud to a category based on its string value
    for (auto curr_pair : cloud_normals)
    {
        str_found = false;

        // getting the values of the pair
        curr_pt = curr_pair.first;
        curr_val = curr_pair.second;

        // looking for whether the string represents a category or not
        for (auto curr_ptval : ptval_dict)
        {
            // true when the two strings are equal
            if (!curr_ptval.first.compare(curr_val))
            {
                str_found = true;
                num_categ = curr_ptval.second;
                break;
            }
        }

        // if there is already a category then we add our point to that category
        if (str_found)
            gr_pts[num_categ]->push_back(curr_pt);

        // else we create the new category and then add our point to it
        else
        {
            ptval_dict.insert(std::pair<std::string, int>(curr_val, gr_pts.size()));
            gr_pts.push_back(new std::vector<pcl::PointXYZRGB *>());
            gr_pts.back()->push_back(curr_pt);
        }
    }

    return gr_pts;
}

void segm::pts_colsegm(std::vector<std::vector<pcl::PointXYZRGB *> *> gr_pts)
{
    // colors
    uint8_t r, g, b;
    uint32_t rgb;
    std::vector<uint32_t> used_cols;
    int range = 255;
    bool used;

    // iterating through the vector of categories
    for (auto curr_cat : gr_pts)
    {
        // generating random colors different from the ones already used
        do
        {
            used = false;

            r = std::rand() % 255;
            g = std::rand() % 255;
            b = std::rand() % 255;
            rgb = (r << 16) | (g << 8) | b;

            // looking for colors already used and compairing them to the randomly generated color
            for (auto rgb_col : used_cols)
            {
                if (rgb == rgb_col)
                {
                    used = true;
                    break;
                }
            }
        } while (used);

        // ading color to used colors
        used_cols.push_back(rgb);

        // color
        for (auto curr_pt  : *curr_cat)
            curr_pt->rgb = rgb;
    }
}
