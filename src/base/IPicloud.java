/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package pointcloud.model.base;

import java.io.FileNotFoundException;


public interface IPicloud {

    /**
     * load Load a cloud from a TXT file
     *
     * @param pathfile path and name of the file contained the cloud.
     * @throws java.io.FileNotFoundException if the file cannot be found
     */
    public void loadFromTXT(String pathfile) throws FileNotFoundException;

    /**
     * load Load a cloud from a PIC file
     *
     * @param pathfile path and name of the file contained the cloud.
     * @throws java.io.FileNotFoundException if the file cannot be found
     */
    public void loadBin(String pathfile) throws FileNotFoundException;

    /**
     * saveBin Save the cloud in a PIC file
     *
     * @param pathfile path and name of the file to will saveBin the cloud.
     *
     */
    public void saveBin(String pathfile);

    /**
     * saveBin Save the cloud in a TXT file
     *
     * @param pathfile path and name of the file to will saveBin the cloud.
     * @throws java.io.FileNotFoundException if the file cannot be found
     *
     */
    public void saveTXT(String pathfile) throws FileNotFoundException;

    /**
     * setCloudArray set the point cloud from a float array.
     *
     * @param cloud array contained each value of each point. \warning {The
     * first value of the array is the array length} For exemple, for the first
     * point: cloud[0] corresponding to the array length. cloud[1] corresponding
     * to the X point coordinate. cloud[2] corresponding to the Y point
     * coordinate. cloud[3] corresponding to the Z point coordinate. cloud[4]
     * corresponding to the X point coordinate...
     * @return true if the cloud could be set, false otherside.
     */
    public boolean setCloudArray(float[] cloud);

    /**
     * getCloudArray transform the cloud in float array
     *
     * @return cloud array contained each value of each point.
     * @warning {The first value of the array is the array length}
     */
    public float[] getCloudArray();

    /**
     *Centralise the cloud
     */
    public void centerFast();
    /**
     * Scale the cloud 
     * @param x scale in x
     * @param y scale in y
     * @param z scale in z
     */
    public void scale(float x, float y, float z);

    /**
     *Switch coordinate y and z
     */
    public void switchYZ();
          public void switchZX();

}
