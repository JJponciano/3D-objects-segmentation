# 3D-objects-segmentation
This project aims to locate and isolated in a point cloud every object presenting different surface characteristics.

Three algorithms are in the course of implementations.

## Segmentation by normal

The objective is to make a segmentation based on the normal. For it it is necessary to realize at first a function allowing to estimate the normal in every point to be able to allocate a color, then make a grouping of the wealthy points normal one similar, in other words, allocate the same color to a zone having normal one common.

## Detection and definition of line

The objective is to recognize lines in a cloud of points. We shall allocate a color to every point belonging to a line. Furthermore, when two lines cross, it will be necessary to determine the formed angle. Two functions are expected, the first one allowing to detect lines, second allowing to estimate the angle formed by two lines.

## Clustering

The objective is to cut a cloud of points in several sub-clouds, according to the color of every point as well as their distance. Every generated cloud has to correspond to a surface of the cloud of origin.
