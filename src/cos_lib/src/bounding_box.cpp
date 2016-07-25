#include "../include/bounding_box.h"

// CONSTRUCTORS

cos_lib::bounding_box::bounding_box(pcl::PointCloud<point_clstr>::Ptr cloud)
{
    pcl::PointCloud<point_clstr>::iterator cloud_it;
    float xmin=99999,xmax=-99999,ymin=99999,ymax=-99999,zmin=99999,zmax=-99999;
    cloud_it++;
    for(cloud_it=cloud->begin(); cloud_it!=cloud->end(); cloud_it++)
    {
        if((*cloud_it).x < xmin) xmin = (*cloud_it).x;
        if((*cloud_it).x >= xmax) xmax = (*cloud_it).x;
        if((*cloud_it).y < ymin) ymin = (*cloud_it).y;
        if((*cloud_it).y >= ymax) ymax = (*cloud_it).y;
        if((*cloud_it).z < zmin) zmin = (*cloud_it).z;
        if((*cloud_it).z >= zmax) zmax = (*cloud_it).z;
        this->addPointIntoBox(&(*cloud_it));
    }
    this->A = new point_clstr(xmin,ymax,zmin, 0, 255, 0);
    this->B = new point_clstr(xmin,ymax,zmax, 0, 255, 0);
    this->C = new point_clstr(xmax,ymax,zmax, 0, 255, 0);
    this->D = new point_clstr(xmax,ymax,zmin, 0, 255, 0);
    this->E = new point_clstr(xmin,ymin,zmax, 0, 255, 0);
    this->F = new point_clstr(xmax,ymin,zmax, 0, 255, 0);
    this->G = new point_clstr(xmax,ymin,zmin, 0, 255, 0);
    this->H = new point_clstr(xmin,ymin,zmin, 0, 255, 0);
}

cos_lib::bounding_box::bounding_box(point_clstr* A, point_clstr* B, point_clstr* C, point_clstr* D, point_clstr* E, point_clstr* F, point_clstr* G, point_clstr* H)
{
    this->A = &(*A);
    this->B = &(*B);
    this->C = &(*C);
    this->D = &(*D);
    this->E = &(*E);
    this->F = &(*F);
    this->G = &(*G);
    this->H = &(*H);
    this->A->makeVertex();
    this->B->makeVertex();
    this->C->makeVertex();
    this->D->makeVertex();
    this->E->makeVertex();
    this->F->makeVertex();
    this->G->makeVertex();
    this->H->makeVertex();
}

// PUBLIC FUNCTIONS

void cos_lib::bounding_box::addPointIntoBox(point_clstr* point)
{
    this->points.push_back(point);
}

std::vector<cos_lib::bounding_box*> cos_lib::bounding_box::divideBox()
{
    std::vector<bounding_box*> child_boxes;

    float xmin= this->H->x, xmax=this->C->x, ymin= this->H->y, ymax=this->C->y, zmin=this->H->z, zmax=this->C->z;
    // We create all the points needed to do the child boxes
    point_clstr* I = new point_clstr(xmax-((xmax-xmin)/2), ymax, zmax-((zmax-zmin)/2), 0, 255, 0);
    point_clstr* J = new point_clstr(xmax-((xmax-xmin)/2), ymax-((ymax-ymin)/2), zmin, 0, 255, 0);
    point_clstr* K = new point_clstr(xmin, ymax-((ymax-ymin)/2), zmax-((zmax-zmin)/2), 0, 255, 0);
    point_clstr* L = new point_clstr(xmax-((xmax-xmin)/2), ymin ,zmax-((zmax-zmin)/2), 0, 255, 0);
    point_clstr* M = new point_clstr(xmax,ymax-((ymax-ymin)/2),  zmax-((zmax-zmin)/2), 0, 255, 0);
    point_clstr* N = new point_clstr(xmax-((xmax-xmin)/2), ymax-((ymax-ymin)/2), zmin, 0, 255, 0);
    point_clstr* cube_center = new point_clstr(xmax-((xmax-xmin)/2), ymax-((ymax-ymin)/2), zmax-((zmax-zmin)/2), 0, 255, 0);

    point_clstr* O = new point_clstr(xmax-((xmax-xmin)/2), ymax, zmax, 0, 255, 0);
    point_clstr* P = new point_clstr(xmax, ymax, zmax-((zmax-zmin)/2), 0, 255, 0);
    point_clstr* Q = new point_clstr(xmax-((xmax-xmin)/2), ymax, zmin, 0, 255, 0);
    point_clstr* R = new point_clstr(xmin, ymax, zmax-((zmax-zmin)/2), 0, 255, 0);
    point_clstr* S = new point_clstr(xmax, ymax-((ymax-ymin)/2), zmin, 0, 255, 0);
    point_clstr* T = new point_clstr(xmax-((xmax-xmin)/2), ymin, zmin, 0, 255, 0);
    point_clstr* U = new point_clstr(xmin, ymax-((ymax-ymin)/2), zmin, 0, 255, 0);
    point_clstr* V = new point_clstr(xmin, ymin, zmax-((zmax-zmin)/2), 0, 255, 0);
    point_clstr* W = new point_clstr(xmin, ymax-((ymax-ymin)/2), zmax, 0, 255, 0);
    point_clstr* X = new point_clstr(xmax-((xmax-xmin)/2), ymin, zmax, 0, 255, 0);
    point_clstr* Y = new point_clstr(xmax, ymax-((ymax-ymin)/2), zmax, 0, 255, 0);
    point_clstr* Z = new point_clstr(xmax, ymin, zmax-((zmax-zmin)/2), 0, 255, 0);

    // We create the child boxes accoridng to the points above
    bounding_box* cube_1 = new bounding_box(R, this->B, O, I, W, J, cube_center, K);
    this->boxes.push_back(cube_1);
    bounding_box* cube_2 = new bounding_box(I, O, this->C, P, J, Y, M, cube_center);
    this->boxes.push_back(cube_2);
    bounding_box* cube_3 = new bounding_box(this->A, R, I, Q, K, cube_center, this->E, U);
    this->boxes.push_back(cube_3);
    bounding_box* cube_4 = new bounding_box(Q, I, P, this->D, cube_center, M, S, this->E);
    this->boxes.push_back(cube_4);
    bounding_box* cube_5 = new bounding_box(K, W, J, cube_center, N, X, L, V);
    this->boxes.push_back(cube_5);
    bounding_box* cube_6 = new bounding_box(cube_center, J, Y, M, X, this->F, Z, L);
    this->boxes.push_back(cube_6);
    bounding_box* cube_7 = new bounding_box(U, K, cube_center, this->E, V, L, T, this->H);
    this->boxes.push_back(cube_7);
    bounding_box* cube_8 = new bounding_box(this->E, cube_center, M, S, L, Z, this->G, T);
    this->boxes.push_back(cube_8);

    // Sorting the points in the child boxes
    std::vector<point_clstr*>::iterator points_iterator;
    for(points_iterator = this->points.begin(); points_iterator!=this->points.end(); points_iterator++)
    {
        if((*points_iterator)->x >= xmax-((xmax-xmin)/2))
        {
            if((*points_iterator)->y >= ymax-((ymax-ymin)/2))
            {
                if((*points_iterator)->z >= zmax-((zmax-zmin)/2))
                {
                    cube_2->addPointIntoBox(*points_iterator);
                }
                else
                {
                    cube_4->addPointIntoBox(*points_iterator);
                }
            }
            else
            {
                if((*points_iterator)->z >= zmax-((zmax-zmin)/2))
                {
                    cube_6->addPointIntoBox(*points_iterator);
                }
                else
                {
                    cube_8->addPointIntoBox(*points_iterator);
                }
            }
        }
        else
        {
            if((*points_iterator)->y >= ymax-((ymax-ymin)/2))
            {
                if((*points_iterator)->z >= zmax-((zmax-zmin)/2))
                {
                    cube_1->addPointIntoBox(*points_iterator);
                }
                else
                {
                    cube_3->addPointIntoBox(*points_iterator);
                }
            }
            else
            {
                if((*points_iterator)->z >= zmax-((zmax-zmin)/2))
                {
                    cube_5->addPointIntoBox(*points_iterator);
                }
                else
                {
                    cube_7->addPointIntoBox(*points_iterator);
                }
            }
        }
    }

    // We only push the child boxes that have points
    for(bounding_box* boxes : this->boxes)
    {
        if(boxes->getNbOfPoints() > 0) child_boxes.push_back(boxes);
        else boxes->deleteBox();
    }

    this->deleteBox();
    return child_boxes;
}

void cos_lib::bounding_box::deleteBox()
{
    this->A->removeVertex();
    this->B->removeVertex();
    this->C->removeVertex();
    this->D->removeVertex();
    this->E->removeVertex();
    this->F->removeVertex();
    this->G->removeVertex();
    this->H->removeVertex();
}

std::vector<cos_lib::point_clstr*> cos_lib::bounding_box::getVertices()
{
    std::vector<point_clstr*> vertices;
    vertices.push_back(this->A);
    vertices.push_back(this->B);
    vertices.push_back(this->C);
    vertices.push_back(this->D);
    vertices.push_back(this->E);
    vertices.push_back(this->F);
    vertices.push_back(this->G);
    vertices.push_back(this->H);
    return vertices;
}
