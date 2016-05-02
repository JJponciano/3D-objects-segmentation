#include "bounding_box.h"

// CONSTRUCTORS

bounding_box::bounding_box(pcl::PointCloud<clstr::PointBool>::Ptr cloud)
{
    float xmin=9999,xmax=-9999,ymin=9999,ymax=-9999,zmin=9999,zmax=-9999;
    pcl::PointCloud<clstr::PointBool>::iterator cloud_it;
    for(cloud_it=cloud->begin(); cloud_it!=cloud->end(); cloud_it++)
    {
        if((*cloud_it).x < xmin) xmin = (*cloud_it).x;
        if((*cloud_it).x < xmax) xmax = (*cloud_it).x;
        if((*cloud_it).y < ymin) ymin = (*cloud_it).y;
        if((*cloud_it).y < ymax) ymax = (*cloud_it).y;
        if((*cloud_it).z < zmin) zmin = (*cloud_it).z;
        if((*cloud_it).z < zmax) zmax = (*cloud_it).z;
    }
}

bounding_box::bounding_box(clstr::PointBool* A, clstr::PointBool* B, clstr::PointBool* C, clstr::PointBool* D, clstr::PointBool* E, clstr::PointBool* F, clstr::PointBool* G, clstr::PointBool* H)
{
    this->A = &(*A);
    this->B = &(*B);
    this->C = &(*C);
    this->D = &(*D);
    this->E = &(*E);
    this->F = &(*F);
    this->G = &(*G);
    this->H = &(*H);
    bounding_box* ptr = this;
    this->A->makeVertexOf(ptr);
    this->B->makeVertexOf(ptr);
    this->C->makeVertexOf(ptr);
    this->D->makeVertexOf(ptr);
    this->E->makeVertexOf(ptr);
    this->F->makeVertexOf(ptr);
    this->G->makeVertexOf(ptr);
    this->H->makeVertexOf(ptr);
}

// PUBLIC FUNCTIONS

std::vector<bounding_box*> bounding_box::divideBox()
{
    std::vector<bounding_box*> child_boxes;

    clstr::PointBool* I = new clstr::PointBool((this->D->x)/2, this->D->y, (this->C->z)/2);

    clstr::PointBool* J = new clstr::PointBool((this->D->x)/2, (this->D->y)/2, this->D->z);

    clstr::PointBool* K = new clstr::PointBool(this->H->x, (this->D->y)/2, (this->C->z)/2);

    clstr::PointBool* L = new clstr::PointBool((this->D->x)/2, this->H->y ,(this->C->z)/2);

    clstr::PointBool* M = new clstr::PointBool(this->D->x,(this->D->y)/2,  (this->C->z)/2);

    clstr::PointBool* N = new clstr::PointBool((this->D->x)/2, (this->D->y)/2, this->D->z);

    clstr::PointBool* O = new clstr::PointBool((this->D->x)/2, this->D->y, this->C->z);

    clstr::PointBool* P = new clstr::PointBool(this->D->x, this->D->y, (this->C->z)/2);

    clstr::PointBool* Q = new clstr::PointBool((this->D->x)/2, this->D->y, this->D->z);

    clstr::PointBool* R = new clstr::PointBool(this->H->x, this->D->y, (this->C->z)/2);

    clstr::PointBool* S = new clstr::PointBool(this->D->x, (this->D->y)/2, this->D->z);

    clstr::PointBool* T = new clstr::PointBool((this->D->x)/2, this->H->y, this->H->z);

    clstr::PointBool* U = new clstr::PointBool(this->H->x, (this->D->y)/2, this->H->z);

    clstr::PointBool* V = new clstr::PointBool(this->H->x, this->H->y, (this->C->z)/2);

    clstr::PointBool* W = new clstr::PointBool(this->H->x, (this->D->y)/2, this->C->z);

    clstr::PointBool* X = new clstr::PointBool((this->D->x)/2, this->H->y, this->C->z);

    clstr::PointBool* Y = new clstr::PointBool(this->D->x, (this->D->y)/2, this->C->z);

    clstr::PointBool* Z = new clstr::PointBool(this->D->x, this->H->y, (this->C->z)/2);

    return child_boxes;
}
