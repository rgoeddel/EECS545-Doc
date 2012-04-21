package kinect.kinect;

import java.util.*;
import java.awt.*;
import java.awt.image.BufferedImage;

/**  Features to implement:
 **  - measure of sharpness
 **  - relative difference
 **     - bounding box, color, distance, etc
 **/
public class FeatureVec
{
    public ArrayList<Double> features;

    public FeatureVec(ArrayList<double[]> points)
    {
        features = getFeatureVec(points);
    }

    /** Retrieve a feature vector for the given set of data points.  All new
     ** features should be added in here in order to be added into the feature
     ** vector.  Right now includes average and variance for RGB and HSV and
     ** length and width of object
     ** @return feture vector
     **/
    public static ArrayList<Double> getFeatureVec(ArrayList<double[]> points)
    {
        ArrayList<Double> features = new ArrayList<Double>();
        addArray(features, avgRGB(points));
        //addArray(features, varRGB(points));
        addArray(features, avgHSV(points));
        //addArray(features, varHSV(points));
        //addArray(features, lwh(points));
        return features;
    }

    /** Return a string of the features for the given set of points (used for
        liblinear in the 545 project.**/
    public static String featureString(ArrayList<double[]> points)
    {
        ArrayList<Double> features = getFeatureVec(points);
        String s = "[";
        for(Double d : features){
            s += d + " ";
        }
        s += "]";
        return s;
    }

    /** Find the average red, green, and blue values for a group of pixels.
     ** pixels are assumed to have four coordinates, (x, y, z, rgb).
     ** @return [r,g,b] averages.
     **/
    public static double[] avgRGB(ArrayList<double[]> points)
    {
        double[] avg = new double[3];
        for(double[] p:points){
            Color bgr = new Color((int)p[3]);
            avg[2] += bgr.getRed();
            avg[1] += bgr.getGreen();
            avg[0] += bgr.getBlue();
        }
        divideEquals(avg, 255.0*points.size());
        return avg;
    }


    /** Calculate the variance of red, green, blue values in a group of pixels.
     ** Pixels are assumed to have four coordinates: (x, y, z, rgb).
     ** @return [r,g,b,] variances
     **/
    public static double[] varRGB(ArrayList<double[]> points)
    {
        double[] avg = avgRGB(points);
        return varRGB(points, avg);
    }


    /** Calculate the variance of red, green, blue values in a group of pixels.
     ** Pixels are assumed to have four coordinates: (x, y, z, rgb).
     ** @return [r,g,b,] variances
     **/
    public static double[] varRGB(ArrayList<double[]> points, double[] avg)
    {
        double[] var = new double[3];
        for(double[] p:points){
            Color bgr = new Color((int)p[3]);
            var[2] += (bgr.getRed()-avg[0])*(bgr.getRed()-avg[0]);
            var[1] += (bgr.getGreen()-avg[1])*(bgr.getGreen()-avg[1]);
            var[0] += (bgr.getBlue()-avg[2])*(bgr.getBlue()-avg[2]);
        }
        divideEquals(var, points.size());
        return var;
    }


    /** Find the average hue, saturation, and values for a group of pixels.
     ** pixels are assumed to have four coordinates, (x, y, z, rgb).
     ** @return [h,s,v] averages.
     **/
    public static double[] avgHSV(ArrayList<double[]> points)
    {
        double[] avg = new double[3];
        float[] hsv  = new float[3];
        for(double[] p:points){
            Color rgb = new Color((int)p[3]);
            Color.RGBtoHSB(rgb.getBlue(), rgb.getGreen(), rgb.getRed(), hsv);
            for(int i=0; i<avg.length; i++){
                avg[i] += (double) hsv[i];
            }
        }
        divideEquals(avg, points.size());
        return avg;
    }


    /** Calculate the variance of hue, saturation, and values in a group of pixels.
     ** Pixels are assumed to have four coordinates: (x, y, z, rgb).
     ** @return [h,s,v] variances
     **/
    public static double[] varHSV(ArrayList<double[]> points)
    {
        double[] avg = avgHSV(points);
        return varHSV(points, avg);
    }


    /** Calculate the variance of hue, saturation, and values in a group of pixels.
     ** Pixels are assumed to have four coordinates: (x, y, z, rgb).
     ** @return [h,s,v] variances
     **/
    public static double[] varHSV(ArrayList<double[]> points, double[] avg)
    {
        double[] var = new double[3];
        float[] hsvV = new float[3];
        for(double[] p:points){
            Color rgb = new Color((int)p[3]);
            Color.RGBtoHSB(rgb.getBlue(), rgb.getGreen(), rgb.getRed(), hsvV);
            for(int i=0; i<avg.length; i++){
                double hsv = (double) hsvV[i];
                var[i] += (hsv-avg[i])*(hsv-avg[i]);
            }
        }
        divideEquals(var, points.size());
        return var;
    }


    /** Find the bounding box for a group of pixels by finding the extreme values
     ** in all directions of the points.  This may not be the best way/may be
     ** implemented elsewhere.
     ** @return [xmin, ymin, zmin, xmax, ymax, zmax]
     */
    public static double[] boundingBox(ArrayList<double[]> points)
    {
        double[] max = new double[]{-1000, -1000, -1000};
        double[] min = new double[]{1000, 1000, 1000};
        for(double[] p: points){
            for(int i=0; i<3; i++){
                if(p[i] < min[i]) min[i] = p[i];
                if(p[i] > max[i]) max[i] = p[i];
            }
        }
        return new double[]{min[0], min[1], min[2], max[0], max[1], max[2]};
    }


    /** Get the length, width, and height of the object by first getting the
     ** bounding box of an object.  Length is in the x direction, width is in the
     ** y direction, and height is in z.
     ** @return [length, width, height] from bounds.
     **/
    public static double[] lwh(ArrayList<double[]> points)
    {
        return lwh(boundingBox(points));
    }


    /** Get the length, width, and height of the object (as perceived). Length is
     ** in the x direction, width is in the y direction, and height is in z.
     ** @return [length, width, height] from bounds.
     **/
    public static double[] lwh(double[] bounds)
    {
        assert(bounds.length == 6);
        double[] lwh = new double[3];
        lwh[0] = Math.abs(bounds[3] - bounds[0]);
        lwh[1] = Math.abs(bounds[4] - bounds[1]);
        lwh[2] = Math.abs(bounds[5] - bounds[2]);
        return lwh;
    }
    
    public static String getShapeFeature(BufferedImage image){
    	double[] features = PCA.getFeatures(image, 7);
    	String shapeInput = "[";
	    for (double f : features)
	    	shapeInput += f + " ";
	    shapeInput += "]";
	    return shapeInput;
    }

    private static void divideEquals(double[] values, double divisor)
    {
        assert(divisor !=0);
        for(int i=0;i<values.length; i++){
            values[i] /= divisor;
        }
    }

    private static void addArray(ArrayList<Double> list, double[] additions)
    {
        for(double d:additions){
            list.add(d);
        }
    }
}
