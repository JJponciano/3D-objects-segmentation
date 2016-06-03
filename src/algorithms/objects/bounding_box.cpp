#include "bounding_box.h"

// CONSTRUCTORS

bounding_box::bounding_box(pcl::PointCloud<clstr::point_clstr>::Ptr cloud)
{
    pcl::PointCloud<clstr::point_clstr>::iterator cloud_it;
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
    this->A = new clstr::point_clstr(xmin,ymax,zmin, 0, 255, 0);
    this->B = new clstr::point_clstr(xmin,ymax,zmax, 0, 255, 0);
    this->C = new clstr::point_clstr(xmax,ymax,zmax, 0, 255, 0);
    this->D = new clstr::point_clstr(xmax,ymax,zmin, 0, 255, 0);
    this->E = new clstr::point_clstr(xmin,ymin,zmax, 0, 255, 0);
    this->F = new clstr::point_clstr(xmax,ymin,zmax, 0, 255, 0);
    this->G = new clstr::point_clstr(xmax,ymin,zmin, 0, 255, 0);
    this->H = new clstr::point_clstr(xmin,ymin,zmin, 0, 255, 0);
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

    float xmin= this->H->x, xmax=this->C->x, ymin= this->H->y, ymax=this->C->y, zmin=this->H->z, zmax=this->C->z;
    // We create all the points needed to do the child boxes
    clstr::point_clstr* I = new clstr::point_clstr(xmax-((xmax-xmin)/2), ymax, zmax-((zmax-zmin)/2), 0, 255, 0);
    clstr::point_clstr* J = new clstr::point_clstr(xmax-((xmax-xmin)/2), ymax-((ymax-ymin)/2), zmin, 0, 255, 0);
    clstr::point_clstr* K = new clstr::point_clstr(xmin, ymax-((ymax-ymin)/2), zmax-((zmax-zmin)/2), 0, 255, 0);
    clstr::point_clstr* L = new clstr::point_clstr(xmax-((xmax-xmin)/2), ymin ,zmax-((zmax-zmin)/2), 0, 255, 0);
    clstr::point_clstr* M = new clstr::point_clstr(xmax,ymax-((ymax-ymin)/2),  zmax-((zmax-zmin)/2), 0, 255, 0);
    clstr::point_clstr* N = new clstr::point_clstr(xmax-((xmax-xmin)/2), ymax-((ymax-ymin)/2), zmin, 0, 255, 0);
    clstr::point_clstr* cube_center = new clstr::point_clstr(xmax-((xmax-xmin)/2), ymax-((ymax-ymin)/2), zmax-((zmax-zmin)/2), 0, 255, 0);

    clstr::point_clstr* O = new clstr::point_clstr(xmax-((xmax-xmin)/2), ymax, zmax, 0, 255, 0);
    clstr::point_clstr* P = new clstr::point_clstr(xmax, ymax, zmax-((zmax-zmin)/2), 0, 255, 0);
    clstr::point_clstr* Q = new clstr::point_clstr(xmax-((xmax-xmin)/2), ymax, zmin, 0, 255, 0);
    clstr::point_clstr* R = new clstr::point_clstr(xmin, ymax, zmax-((zmax-zmin)/2), 0, 255, 0);
    clstr::point_clstr* S = new clstr::point_clstr(xmax, ymax-((ymax-ymin)/2), zmin, 0, 255, 0);
    clstr::point_clstr* T = new clstr::point_clstr(xmax-((xmax-xmin)/2), ymin, zmin, 0, 255, 0);
    clstr::point_clstr* U = new clstr::point_clstr(xmin, ymax-((ymax-ymin)/2), zmin, 0, 255, 0);
    clstr::point_clstr* V = new clstr::point_clstr(xmin, ymin, zmax-((zmax-zmin)/2), 0, 255, 0);
    clstr::point_clstr* W = new clstr::point_clstr(xmin, ymax-((ymax-ymin)/2), zmax, 0, 255, 0);
    clstr::point_clstr* X = new clstr::point_clstr(xmax-((xmax-xmin)/2), ymin, zmax, 0, 255, 0);
    clstr::point_clstr* Y = new clstr::point_clstr(xmax, ymax-((ymax-ymin)/2), zmax, 0, 255, 0);
    clstr::point_clstr* Z = new clstr::point_clstr(xmax, ymin, zmax-((zmax-zmin)/2), 0, 255, 0);

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
    std::vector<clstr::point_clstr*>::iterator points_iterator;
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
}

std::vector<clstr::point_clstr*> bounding_box::getVertices()
{
    std::vector<clstr::point_clstr*> vertices;
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
