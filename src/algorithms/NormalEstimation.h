#ifndef MODELDETECTION_H
#define MODELDETECTION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>

namespace normalEstimation{
     /**
     * @brief Get the normal of 3 points
     */
float[3] normale(float x,float y,float z,float x2,float y2,float z2,float x3,float y3,float z3) {
	 // Calcul de 2 vecteurs à partir des 3 points
    float v1x = x - x2;
    float v1y = y - y2;
    float v1z = z - z2;
    float v2x = x2 -x3;
    float v2y = y2 - y3;
    float v2z = z2 - z3;

    
    // calcul du produit vectoriel
    float xn=(v1y * v2z - v1z * v2y);
    float yn=(v1z * v2x - v1x * v2z);
    float zn=(v1x * v2y - v1y * v2x);

    // Calcul de la norme du vecteur                
   float length = (float) sqrt((xn * xn) + (yn * yn) + (zn * zn));

    if (length == 0.0f) length = 1.0f; //évite une violente erreur
    xn /= length;
    yn /= length;
    zn /= length;
	float normal[3]={xn,yn,zn};
}

#endif // MODELDETECTION_H
