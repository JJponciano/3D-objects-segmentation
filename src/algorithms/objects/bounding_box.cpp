#include "bounding_box.h"

// CONSTRUCTORS

bounding_box::bounding_box(pcl::PointCloud<clstr::PointBool>::Ptr cloud)
{
    pcl::PointCloud<clstr::PointBool>::iterator cloud_it=cloud->begin();
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
    }
    this->A = new clstr::PointBool(xmin,ymax,zmin);
    this->B = new clstr::PointBool(xmin,ymax,zmax);
    this->C = new clstr::PointBool(xmax,ymax,zmax);
    this->D = new clstr::PointBool(xmax,ymax,zmin);
    this->E = new clstr::PointBool(xmin,ymin,zmax);
    this->F = new clstr::PointBool(xmax,ymin,zmax);
    this->G = new clstr::PointBool(xmax,ymin,zmin);
    this->H = new clstr::PointBool(xmin,ymin,zmin);
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

    clstr::PointBool* cube_center = new clstr::PointBool((this->D->x)/2, (this->D->y)/2, (this->C->z)/2);

    bounding_box* cube_1 = new bounding_box(R, this->B, O, I, W, J, cube_center, K);
    child_boxes.push_back(cube_1);

    bounding_box* cube_2 = new bounding_box(I, O, this->C, P, J, Y, M, cube_center);
    child_boxes.push_back(cube_2);

    bounding_box* cube_3 = new bounding_box(this->A, R, I, Q, K, cube_center, this->E, U);
    child_boxes.push_back(cube_3);

    bounding_box* cube_4 = new bounding_box(Q, I, P, this->D, cube_center, M, S, this->E);
    child_boxes.push_back(cube_4);

    bounding_box* cube_5 = new bounding_box(K, W, J, cube_center, N, X, L, V);
    child_boxes.push_back(cube_5);

    bounding_box* cube_6 = new bounding_box(cube_center, J, Y, M, X, this->F, Z, L);
    child_boxes.push_back(cube_6);

    bounding_box* cube_7 = new bounding_box(U, K, cube_center, this->E, V, L, T, this->H);
    child_boxes.push_back(cube_7);

    bounding_box* cube_8 = new bounding_box(this->E, cube_center, M, S, L, Z, this->G, T);
    child_boxes.push_back(cube_8);

    return child_boxes;
}
