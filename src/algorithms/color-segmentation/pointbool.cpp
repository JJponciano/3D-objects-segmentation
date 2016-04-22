#include "pointbool.h"

clstr::PointBool::PointBool()
{

}

void clstr::PointBool::addNeighbour(clstr::PointBool* neighbour)
{
    this->neighbours.push_back(neighbour);
}
