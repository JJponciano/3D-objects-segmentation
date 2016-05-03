#include "bounding_box.h"

// CONSTRUCTORS

bounding_box::bounding_box(pcl::PointCloud<clstr::point_clstr>::Ptr cloud)
{
    pcl::PointCloud<clstr::point_clstr>::iterator cloud_it=cloud->begin();
    float xmin=(*cloud_it).x,xmax=(*cloud_it).x,ymin=(*cloud_it).y,ymax=(*cloud_it).y,zmin=(*cloud_it).z,zmax=(*cloud_it).z;
    cloud_it++;
    for(cloud_it; cloud_it!=cloud->end(); cloud_it++)
    {
        if((*cloud_it).x < xmin) xmin = (*cloud_it).x;
        if((*cloud_it).x > xmax) xmax = (*cloud_it).x;
        if((*cloud_it).y < ymin) ymin = (*cloud_it).y;
        if((*cloud_it).y > ymax) ymax = (*cloud_it).y;
        if((*cloud_it).z < zmin) zmin = (*cloud_it).z;
        if((*cloud_it).z > zmax) zmax = (*cloud_it).z;
        points.push_back(&(*cloud_it));
    }
    this->A = new clstr::point_clstr(xmin,ymax,zmin);
    this->B = new clstr::point_clstr(xmin,ymax,zmax);
    this->C = new clstr::point_clstr(xmax,ymax,zmax);
    this->D = new clstr::point_clstr(xmax,ymax,zmin);
    this->E = new clstr::point_clstr(xmin,ymin,zmax);
    this->F = new clstr::point_clstr(xmax,ymin,zmax);
    this->G = new clstr::point_clstr(xmax,ymin,zmin);
    this->H = new clstr::point_clstr(xmin,ymin,zmin);
}

bounding_box::bounding_box(clstr::point_clstr* A, clstr::point_clstr* B, clstr::point_clstr* C, clstr::point_clstr* D, clstr::point_clstr* E, clstr::point_clstr* F, clstr::point_clstr* G, clstr::point_clstr* H)
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

void bounding_box::addPointIntoBox(clstr::point_clstr* point)
{
    this->points.push_back(point);
}

std::vector<bounding_box*> bounding_box::divideBox()
{
    std::vector<bounding_box*> child_boxes;

    clstr::point_clstr* I = new clstr::point_clstr((this->D->x)/2, this->D->y, (this->C->z)/2);

    clstr::point_clstr* J = new clstr::point_clstr((this->D->x)/2, (this->D->y)/2, this->D->z);

    clstr::point_clstr* K = new clstr::point_clstr(this->H->x, (this->D->y)/2, (this->C->z)/2);

    clstr::point_clstr* L = new clstr::point_clstr((this->D->x)/2, this->H->y ,(this->C->z)/2);

    clstr::point_clstr* M = new clstr::point_clstr(this->D->x,(this->D->y)/2,  (this->C->z)/2);

    clstr::point_clstr* N = new clstr::point_clstr((this->D->x)/2, (this->D->y)/2, this->D->z);

    clstr::point_clstr* O = new clstr::point_clstr((this->D->x)/2, this->D->y, this->C->z);

    clstr::point_clstr* P = new clstr::point_clstr(this->D->x, this->D->y, (this->C->z)/2);

    clstr::point_clstr* Q = new clstr::point_clstr((this->D->x)/2, this->D->y, this->D->z);

    clstr::point_clstr* R = new clstr::point_clstr(this->H->x, this->D->y, (this->C->z)/2);

    clstr::point_clstr* S = new clstr::point_clstr(this->D->x, (this->D->y)/2, this->D->z);

    clstr::point_clstr* T = new clstr::point_clstr((this->D->x)/2, this->H->y, this->H->z);

    clstr::point_clstr* U = new clstr::point_clstr(this->H->x, (this->D->y)/2, this->H->z);

    clstr::point_clstr* V = new clstr::point_clstr(this->H->x, this->H->y, (this->C->z)/2);

    clstr::point_clstr* W = new clstr::point_clstr(this->H->x, (this->D->y)/2, this->C->z);

    clstr::point_clstr* X = new clstr::point_clstr((this->D->x)/2, this->H->y, this->C->z);

    clstr::point_clstr* Y = new clstr::point_clstr(this->D->x, (this->D->y)/2, this->C->z);

    clstr::point_clstr* Z = new clstr::point_clstr(this->D->x, this->H->y, (this->C->z)/2);

    clstr::point_clstr* cube_center = new clstr::point_clstr((this->D->x)/2, (this->D->y)/2, (this->C->z)/2);

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

    // Sorting the points in their boxes
    std::vector<clstr::point_clstr*>::iterator points_iterator;
    for(points_iterator = this->points.begin(); points_iterator!=this->points.end(); points_iterator++)
    {
        if((*points_iterator)->x >= (this->C->x)/2)
        {
            if((*points_iterator)->y >= (this->C->y)/2)
            {
                if((*points_iterator)->z >= (this->C->z)/2)
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
                if((*points_iterator)->z >= (this->C->z)/2)
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
            if((*points_iterator)->y >= (this->C->y)/2)
            {
                if((*points_iterator)->z >= (this->C->z)/2)
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
                if((*points_iterator)->z >= (this->C->z)/2)
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

    for(bounding_box* boxes : this->boxes)
    {
        if(boxes->getNbOfPoints() == 0) child_boxes.push_back(boxes);
        else boxes->deleteBox();
    }

    return child_boxes;
}

void bounding_box::deleteBox()
{
    this->A->removeVertex();
    this->B->removeVertex();
    this->C->removeVertex();
    this->D->removeVertex();
    this->E->removeVertex();
    this->F->removeVertex();
    this->G->removeVertex();
    this->H->removeVertex();
    this->~bounding_box();
}

bounding_box::~bounding_box()
{

}
