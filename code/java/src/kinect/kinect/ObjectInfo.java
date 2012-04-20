package kinect.kinect;

import april.vis.*;
import april.jmat.*;
import april.util.UnionFindSimple;

import kinect.lcmtypes.*;
import lcm.lcm.*;

import java.io.*;
import java.nio.*;
import javax.swing.*;
import java.awt.*;
import java.util.*;
import java.awt.image.*;


public class ObjectInfo{

    double numPoints;
    int color;
    int repID;
    int ufsID;
    double[] sumPoints;
    int[] sumColor;
    double leftmost;
    double rightmost;
    double uppermost;
    double lowermost;

    ArrayList<Double> features;
    ArrayList<double[]> points;
    ArrayList<Color> colors;

    public ObjectInfo(){
        features = new ArrayList<Double>();
    }

    /** Create a new object with info about it. Objects begin with a single point.**/
    public ObjectInfo(int color, int repID, double[] point)
    {
        Random r = new Random();
        this.repID = r.nextInt();
        features = new ArrayList<Double>();
        this.numPoints = 1;
        this.color = color;
        this.ufsID = repID;
        // XXXX - Need to transform everything with respect to the ground
        // plane and the kinect
        this.leftmost = point[0];
        this.rightmost = point[0];
        this.uppermost = point[1];
        this.lowermost = point[1];

        sumPoints = new double[]{point[0], point[1], point[2]};
        Color c = new Color((int) point[3]);
        sumColor = new int[]{c.getRed(), c.getBlue(), c.getGreen()};

        this.points = new ArrayList<double[]>();
        this.points.add(point);
        
        this.colors = new ArrayList<Color>();
        this.colors.add(color);
    }

    /** Set the features of this object. **/
    public void setFeatures(ArrayList<Double> f)
    {
        features = f;
    }

    /** Return a list of the features that have been assigned to this object. **/
    public double[] getFeatures()
    {
        double[] f = new double[features.size()];
        for(int i=0; i<f.length; i++){
            f[i] = features.get(i);
        }
        return f;
    }

    /** Add a new point to this object. **/
    public void update(double[] point){
        if(this.leftmost > point[0])
            this.leftmost = point[0];
        if(this.rightmost < point[0])
            this.rightmost = point[0];
        if(this.uppermost > point[1])
            this.uppermost = point[1];
        if(this.lowermost < point[1])
            this.lowermost = point[1];

        this.numPoints ++;
        for(int i=0; i<sumPoints.length; i++){
            sumPoints[i] += point[i];
        }
        Color c = new Color((int)point[3]);
        sumColor[0] += c.getRed();
        sumColor[1] += c.getBlue();
        sumColor[2] += c.getGreen();

        this.points.add(point);
        this.colors.add(c);
    }

    /** Get the center of the object (mean x, y,z). **/
    public double[] getCenter()
    {
        double[] center = new double[sumPoints.length];
        for(int i=0; i<sumPoints.length; i++){
            center[i] = sumPoints[i]/numPoints;
        }
        return center;
    }

    /** Get the average color of the object as an array [r, g, b]. **/
    public double[] avgColor()
    {
        double[] avg = new double[sumColor.length];
        for(int i=0; i<avg.length; i++){
            avg[i] = sumColor[i]/numPoints;
        }
        return avg;
    }

    /** Given a hashmap of objects, find the one that is most similar to this
        object. The most similar one will be the object that has the closest
        center and with a mean color that is within a threshold of this object.**/
    // XXX - probably want to take complete feature vector into account, not only colors
    public int mostSimilar(HashMap<Integer, ObjectInfo> objects)
    {
        int best = -1;
        double minDist = 100000;
        double minColorDist = 30;

        Collection c = objects.values();
        for(Iterator itr = c.iterator(); itr.hasNext(); ){
            ObjectInfo obj2 = (ObjectInfo)itr.next();
            double centerDist = LinAlg.distance(getCenter(), obj2.getCenter());
            double colorDist = LinAlg.distance(avgColor(), obj2.avgColor());
            if (centerDist < minDist && colorDist < minColorDist){
                minDist = centerDist;
                best = obj2.ufsID;
            }
        }
        return best;
    }

    /** Say this object is the same as a past one by giving it the old object's
        ID. Eventually this method might allow us to store the old object in the
        history of this object? **/
    public void equateObject(int newID, int newColor)
    {
        repID = newID;
        color = newColor;
    }
    
//    public BufferedImage getImage(){
//    	int[][] pixels = new int[points.size()][];
//    	for(int i = 0; i < points.size(); i++){
//    		double[] pixel = KUtils.getPixel(points.get(i));
//    		pixels[i] = new int[]{(int)pixel[0], (int)pixel[1]};
//    	}
//    	
//    	
//    	
//    	
//    }
//    
    
    
    
}
