/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package pointcloud.model.base;

import java.awt.Color;
import java.io.Serializable;


public class Point3D extends Point implements Serializable {

    protected Color color;

    public Point3D(float x, float y, float z) {
        super(x, y, z);
        this.color = Color.WHITE;
    }

    public Point3D(float x, float y, float z, Color color) {
        super(x, y, z);
        this.color = color;
    }

    /**
     *
     * @param pointarray
     */
    public Point3D(float[] pointarray) {
        super(0, 0, 0);
        this.color = Color.WHITE;
        this.set(pointarray);
    }

    /**
     * create point at the origin.
     */
    public Point3D() {
        super(0, 0, 0);
        this.color = Color.WHITE;
    }

    /**
     * Set a point from float array at the format xyz
     *
     * @param xyz array which the first value corresponding to the x coordinate,
     * the second value corresponding to the y coordinate and the last value
     * corresponding to the z value.
     */
    public void set(float[] xyz) {
        if (xyz.length == 3) {
            this.x = xyz[0];
            this.y = xyz[1];
            this.z = xyz[2];
        }
    }

    /**
     * Get the point at the array shape
     *
     * @return array contained x,y,z coordinates in this sequences.
     */
    public float[] get() {
        float[] xyz = {this.x, this.y, this.z};
        return xyz;
    }

    public Color getColor() {
        return color;
    }

    public void setColor(Color color) {
        this.color = color;
    }

    @Override
    public String toString() {
        String s = x + "\t" + y + "\t" + z;
        return s;
    }

    @Override
    public double distance(Point p) {
        return Math.sqrt(Math.pow(x - p.getX(), 2) + Math.pow(y - p.getY(), 2) + Math.pow(z - p.getZ(), 2));
    }

    /**
     * Get the point calculate by the linear interpolation.
     *
     * @param p next extremity.
     * @param t value of the interpolation(usualy between 0 and 1).
     * @return The point at the specifique value in the linear interpolation.
     */
    @Override
    public Point linearInterpolation(Point p, double t) {
        float px = (float) ((1.f - t) * this.x + t * p.getX());
        float py = (float) ((1.f - t) * this.y + t * p.getY());
        float pz = (float) ((1.f - t) * this.z + t * p.getZ());
        return new Point3D(px, py, pz);
    }

}
