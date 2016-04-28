/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package pointcloud.model.base;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import pointcloud.model.Picloud3D;


public abstract class PiCloud implements IPicloud {

    protected List<Point3D> points;
    protected final String ext;

    public PiCloud() {
        this.ext = "pic";
        this.points = new ArrayList<>();
    }

    /**
     * add a new point at the cloud.
     *
     * @param point
     */
    public void addPoint(Point3D point) {
        this.points.add(point);
    }

    @Override
    public void loadFromTXT(String pathfile) throws FileNotFoundException {
        this.fireStartLoading();
        //read the file
        File fileio = new File(pathfile);
        try (BufferedReader reader = Files.newBufferedReader(fileio.toPath(),
                StandardCharsets.UTF_8)) {
            String line = null;
            while ((line = reader.readLine()) != null) {
                String[] split = line.split("\\s");
                Point3D p = new Point3D(Float.parseFloat(split[0]), Float.parseFloat(split[1]), Float.parseFloat(split[2]));
                if (split.length == 6) {
                    p.setColor(new Color(Integer.parseInt(split[3]), Integer.parseInt(split[4]), Integer.parseInt(split[5])));
                }
                this.points.add(p);
            }
            //send signal
            this.fireEndLoading();
            this.fireCloudChange();

        } catch (IOException ex) {
            Logger.getLogger(Picloud3D.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    @Override
    public abstract void loadBin(String pathfile) throws FileNotFoundException;

    @Override
    public abstract void saveBin(String pathfile);

    @Override
    public void saveTXT(String pathfile) throws FileNotFoundException {
        String ext = "txt";
        //add the extension
        if (ext.isEmpty()) {
            ext = "unknown";
        } else {
            //test if the extension is already added
            int lastIndexOf = pathfile.lastIndexOf(ext);
            //if the extension is not already added
            if (lastIndexOf < pathfile.length() - ext.length()) {
                if (ext.charAt(0) != '.') {
                    pathfile += ".";
                }

                pathfile += ext;
            }
        }
        //save the list of point
        String txt = this.toString();

        File fileio = new File(pathfile);
        Charset charset = Charset.forName("UTF8");
        try (BufferedWriter writer = Files.newBufferedWriter(fileio.toPath(), charset)) {
            writer.write(txt, 0, txt.length());
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }
    }

    @Override
    public boolean setCloudArray(float[] cloud) {
        if ((cloud.length) % 3 != 0) {
            return false;
        }
        //for each value of the cloud
        for (int i = 0; i < cloud.length - 2; i++) {
            //get the coordinate of the point
            float x = (float) cloud[i];
            i++;
            float y = (float) cloud[i];
            i++;
            float z = (float) cloud[i];
            //add the point
            this.points.add(new Point3D(x, y, z));
        }
        this.fireCloudChange();

        return true;
    }

    @Override
    public float[] getCloudArray() {
        float[] arraycloud = new float[this.points.size() * 3];
        int j = 0;//index of the array cloud.
        //for each point
        for (int i = 0; i < this.points.size(); i++) {
            //add each coordinate at the array cloud.
            arraycloud[j] = this.points.get(i).getX();
            j++;
            arraycloud[j] = this.points.get(i).getY();
            j++;
            arraycloud[j] = this.points.get(i).getZ();
            j++;
        }
        return arraycloud;
    }

    /**
     * Say at all listener that the cloud are changed
     */
    public abstract void fireCloudChange();

    @Override
    public String toString() {
        StringBuffer buff = new StringBuffer();
        for (Point3D point : points) {
            buff.append(point.toString()).append("\n");
        }
        return buff.toString();
    }

    /**
     * Remove all point in the cloud.
     *
     * @deprecated cause runtime error in point cloud viewer.
     */
    public void clear() {
        this.points.clear();
        this.fireCloudChange();
    }

    /**
     * Center the cloud
     */
    @Override
    public void centerFast() {
        if (this.points.size() > 0) {
            float[] orig = this.points.get(0).get();

            for (int i = 1; i < this.points.size(); i++) {
                this.points.get(i).setX(this.points.get(i).getX() - orig[0]);
                this.points.get(i).setY(this.points.get(i).getY() - orig[1]);
                this.points.get(i).setZ(this.points.get(i).getZ() - orig[2]);

            }  //send signal
            this.fireCloudChange();
        }
    }

    @Override
    public void scale(float x, float y, float z) {
        for (int i = 0; i < this.points.size(); i++) {
            this.points.get(i).setX(this.points.get(i).getX() * x);
            this.points.get(i).setY(this.points.get(i).getY() * y);
            this.points.get(i).setZ(this.points.get(i).getZ() * z);

        }  //send signal
        this.fireCloudChange();
    }

    @Override
    public void switchYZ() {
        for (int i = 0; i < this.points.size(); i++) {
            float y = this.points.get(i).getY();
            this.points.get(i).setY(this.points.get(i).getZ());
            this.points.get(i).setZ(y);

        }  //send signal
        this.fireCloudChange();
    }

    public void switchZX() {
        for (int i = 0; i < this.points.size(); i++) {
            float z = this.points.get(i).getZ();
            this.points.get(i).setZ(this.points.get(i).getX());
            this.points.get(i).setX(z);

        }  //send signal
        this.fireCloudChange();
    }

    protected abstract void fireStartLoading();

    protected abstract void fireEndLoading();

    protected abstract void fireStartSaving();

    protected abstract void fireEndSaving();

    /**
     * Returns the number of point in the point cloud.
     *
     * @return Rhe number of points.
     */
    public int size() {
        return this.points.size();
    }

    /**
     * Returns the element at the specified position in this point cloud.
     *
     * @param index index of the element to return
     * @return the element at the specified position in this point cloud
     */
    public Point3D get(int index) {
        if (index < this.size() && index >= 0) {
            return this.points.get(index);
        } else {
            return null;
        }
    }

    /**
     * Get both points representing the extremum of the cloud.
     *
     * @return The first point is the minimum of every point contained and the
     * second point is the maximum of every point contained.
     */
    public Point3D[] getMinMax() {
        if (this.size() > 0) {
            float maxX = this.points.get(0).getX();
            float maxY = this.points.get(0).getY();
            float maxZ = this.points.get(0).getZ();
            float minX = this.points.get(0).getX();
            float minY = this.points.get(0).getY();
            float minZ = this.points.get(0).getZ();
            for (int i = 1; i < this.size(); i++) {
                //compare the coordinate value
                if (maxX < this.points.get(i).getX()) {
                    maxX = this.points.get(i).getX();
                } else if (minX > this.points.get(i).getX()) {
                    minX = this.points.get(i).getX();
                }

                if (maxY < this.points.get(i).getY()) {
                    maxY = this.points.get(i).getY();
                } else if (minY > this.points.get(i).getY()) {
                    minY = this.points.get(i).getY();
                }

                if (maxZ < this.points.get(i).getZ()) {
                    maxZ = this.points.get(i).getZ();
                } else if (minZ > this.points.get(i).getZ()) {
                    minZ = this.points.get(i).getZ();
                }
            }
            return new Point3D[]{new Point3D(minX, minY, minZ), new Point3D(maxX, maxY, maxZ)};
        }
        return new Point3D[]{new Point3D(), new Point3D()};
    }

    public void add(Picloud3D pointcloud) {
        for (int i = 0; i < pointcloud.size(); i++) {
            this.addPoint(pointcloud.get(i));
        }
        this.fireCloudChange();
    }

    public List<Point3D> getPoints() {
        return points;
    }

    /**
     * Set the cloud
     *
     * @param cloud new cloud.
     */
    public void setCloud(PiCloud cloud) {
        if (cloud != null) {
            this.clear();
            this.points = cloud.getPoints();
            this.fireCloudChange();
        }
    }

}
