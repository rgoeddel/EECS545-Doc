package kinect.kinect;

import april.vis.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.util.UnionFindSimple;

import lcm.lcm.*;

import java.io.*;
import java.nio.*;
import javax.swing.*;
import java.awt.*;
import java.util.*;
import java.awt.image.*;


public class Segment
{
    DataAggregator da;
    Boolean colorSegments;
    int width, height;
    double OBJECT_THRESH = 250;

    static Random rand = new Random(56436337);
    static double[] t = new double[] { -0.0254, -0.00013, -0.01218 }; // Added .01 to t[2] here rather than below
    double[] floorPlane;
    boolean floorFound;
    // Set up some "Random" colors to draw the segments
    static int[] colors = new int[]{0xff3300CC, 0xff9900CC, 0xffCC0099, 0xffCC0033,
                                       0xff0033CC, 0xff470AFF, 0xff7547FF, 0xffCC3300,
                                       0xff0099CC, 0xffD1FF47, 0xffC2FF0A, 0xffCC9900,
                                       0xff00CC99, 0xff00CC33, 0xff33CC00, 0xff99CC00};

    public Segment(DataAggregator dataAg, Boolean colorSegments_)
    {
        da = dataAg;
        colorSegments = colorSegments_;
        width = da.WIDTH;
        height = da.HEIGHT;
        floorPlane = new double[4];
        floorFound = false;
    }

    /** First segment the frame into objects and then get the features
     ** for each object. **/
    public void segmentFrame()
    {
        unionFind();
        getFeatures();
    }

    /** Get features for each of the objects identified in the frame.**/
    public void getFeatures()
    {
        Collection c = da.objects.values();
        for(Iterator itr = c.iterator(); itr.hasNext(); ){
            ObjectInfo d = (ObjectInfo)itr.next();
            ArrayList<double[]> points = d.points;
            d.setFeatures(FeatureVec.getFeatureVec(points));
        }
    }

    /** Given the representative ID of an object, return its features.
     ** @param ID of object in question
     ** @return corresponding feature vector
     **/
    public double[] getObjectFeatures(int id)
    {
        double[] features = null;
        Collection c = da.objects.values();
        for(Iterator itr = c.iterator(); itr.hasNext(); ){
            ObjectInfo d = (ObjectInfo)itr.next();
            if (d.repID == id){
                features = d.getFeatures();
                break;
            }
        }
        return features;
    }

    /** Get the difference in the z-direction of two pixels. **/
    private double depthDiff(double[] p1, double[] p2)
    {
        return Math.abs(p1[2] - p2[2]);
    }

    /** Get the difference in the z-direction of two pixels. **/
    private double dist(double[] p1, double[] p2)
    {
        double dx = p1[0]-p2[0];
        double dy = p1[1]-p2[1];
        double dz = p1[2]-p2[2];
        return Math.sqrt(dx*dx + dy*dy + dz*dz);
    }

    /** Find the Euclidean distance between two colors. **/
    private double colorDiff(double color1, double color2)
    {
        Color c1 = new Color((int)color1);
        Color c2 = new Color((int)color2);
        int rDiff = c1.getRed() - c2.getRed();
        int gDiff = c1.getGreen() - c2.getGreen();
        int bDiff = c1.getBlue() - c2.getBlue();
        double diff = Math.sqrt(rDiff*rDiff + bDiff*bDiff + gDiff*gDiff);
        return diff;
    }

    /** union find- for each pixel, compare with pixels around it and merge if
     ** they are close enough. **/
    public void unionFind()
    {
        removeFloorPoints();
        da.ufs = new UnionFindSimple(da.currentPoints.size());

        //create unions of pixels that are close together spatially
        for(int y=0; y<height; y++){
            for(int x=0; x<width; x++){
                int loc1 = y*width + x;
                double[] p1 = da.currentPoints.get(loc1);
                // Look at all surrounding pixels
                if(!Arrays.equals(p1, new double[4])){
                    int loc2 = y*width + x + 1;
                    int loc3 = (y+1)*width + x;
                    if (loc2>=0 && loc2<width*height){
                        double[] p2 = da.currentPoints.get(loc2);
                        if(!Arrays.equals(p2, new double[4])
                           && (dist(p1, p2) < da.unionThresh
                               || colorDiff(p1[3], p2[3]) < da.colorThresh)){
                            da.ufs.connectNodes(loc1, loc2);
                        }
                    }

                    if (loc3>=0 && loc3<width*height){
                        double[] p2 = da.currentPoints.get(loc3);
                        if(!Arrays.equals(p2, new double[4])
                           && (depthDiff(p1, p2) < da.unionThresh
                               || colorDiff(p1[3], p2[3]) < da.colorThresh)){
                            da.ufs.connectNodes(loc1, loc3);
                        }
                    }
                }
            }
        }

        //collect data on all the objects segmented by the union find algorithm in the previous step
        ObjectInfo info;
        da.prevObjects = da.objects;
        da.prevMap = da.map;
        da.objects = new HashMap<Integer, ObjectInfo>();
        da.map = new HashMap<Integer, Integer>();

        // Make new objectInfos
        for(int i = 0; i < da.currentPoints.size(); i++){
            double[] point = da.currentPoints.get(i);
            //if(!Arrays.equals(point, new double[4])){
            if(Math.abs(point[0]-0)>.0001 &&Math.abs(point[2]-0)>.0001 && Math.abs(point[2]-0)>.0001){

                if(da.ufs.getSetSize(i) > OBJECT_THRESH){
                    int repID = da.ufs.getRepresentative(i);
                    Object repColor = da.map.get(repID);
                    if(repColor != null){
                        info = (ObjectInfo)da.objects.get(repID);
                        info.update(point);
                    }
                    else{
                        int color = colors[i%colors.length];
                        da.map.put(repID, color);
                        info = new ObjectInfo(color, repID, point);
                        da.objects.put(repID, info);
                    }
                    Integer color = da.map.get(repID);
                    da.coloredPoints.add(new double[]{point[0], point[1], point[2], color});
                }
            }
        }

        // Pair up objects from this frame with last frame
        if(da.prevObjects.size() > 0){
            Collection cNew = da.objects.values();
            for(Iterator itr = cNew.iterator(); itr.hasNext(); ){
                ObjectInfo obj = (ObjectInfo)itr.next();
                int mostSim = obj.mostSimilar(da.prevObjects);
                if(mostSim != -1){
                    int newID = da.prevObjects.get(mostSim).repID;
                    int newColor = da.prevObjects.get(mostSim).color;
                    obj.equateObject(newID, newColor);
                    da.prevObjects.remove(mostSim);
                }
                for(double[] p : obj.points){
                    da.coloredPoints.add(new double[]{p[0], p[1], p[2], obj.color});
                }
            }
        }
    }

    /** "Remove" points that are too close to the floor by setting them to
     ** empty arrays.
     ** @return whether a plane was found and points were removed
     **/
    private boolean removeFloorPoints()
    {
        // Only calculate the floor plane once XXX - maybe do multiple times and average?
        if(floorFound == false){
            floorPlane = estimateFloor(400);
            floorFound = true;
        }

        if (Arrays.equals(floorPlane, new double[4])) return false;

        for(int i=0; i<da.currentPoints.size(); i++){
            double[] point = da.currentPoints.get(i);
            double[] p = new double[]{point[0], point[1], point[2]};
            if(pointToPlaneDist(p, floorPlane) < da.ransacThresh)
                da.currentPoints.set(i, new double[4]);
            if(belowPlane(p, floorPlane))
                da.currentPoints.set(i, new double[4]);
        }
        return true;
    }

    /** Check if a given poinjt is on the other side of the ground plane as
     ** the camera is (this might mean we want to delete them).**/
    private boolean belowPlane(double[] p, double[] coef)
    {
        double eval = coef[0]*p[0] + coef[1]*p[1] + coef[2]*p[2] + coef[3];
        if(eval < 0 && coef[3] > 0) return true;
        else if(eval > 0 && coef[3] < 0) return true;
        return false;
    }

    /** Given a point and the coefficients for a plane, find the distance
     ** between them.
     ** @param point is the point we want a distance to
     ** @param coef is the pqrs coeficients of a plane (px+qy+rz+s=0)
     ** @return distance
     **/
    private double pointToPlaneDist(double[] point, double[] coef)
    {
        assert(point.length == 3 && coef.length == 4);

        double ax = coef[0]*point[0];
        double by = coef[1]*point[1];
        double cz = coef[2]*point[2];
        double a2 = coef[0]*coef[0];
        double b2 = coef[1]*coef[1];
        double c2 = coef[2]*coef[2];

        return Math.abs(ax+ by + cz + coef[3])/ Math.sqrt(a2 + b2 + c2);
    }


    /** Estimate the floor plane using RANSAC algorithm (assumes that the major
        plane in the image is the "floor").
        @param iterations is the number of iterations RANSAC is run for
        @return The characterizing coefficients for the floor plane
    **/
    public double[] estimateFloor(int iterations)
    {
        if(da.currentPoints.size() == 0)
            return null;

        int numPoints = da.currentPoints.size();
        double bestPlane[] = new double[4];  // Parameters of plane equation
        int bestFit = 0;                     // Most points that fit a guess plane
        int numSamples = (int)Math.floor(numPoints*da.ransacPercent);

        // Perform specified number of iterations
        for(int i=0; i<iterations; i++){
            int numFit = 0;
            // Choose three random points
            double[] r1 = da.currentPoints.get(rand.nextInt(numPoints));
            double[] r2 = da.currentPoints.get(rand.nextInt(numPoints));
            double[] r3 = da.currentPoints.get(rand.nextInt(numPoints));

            double[] p1 = new double[]{r1[0], r1[1], r1[2]};
            double[] p2 = new double[]{r2[0], r2[1], r2[2]};
            double[] p3 = new double[]{r3[0], r3[1], r3[2]};

            // Derive plane through all three points
            double[] p2p1 = LinAlg.subtract(p2, p1);
            double[] p3p1 = LinAlg.subtract(p3, p1);
            double[] pqr = LinAlg.crossProduct(p2p1, p3p1);
            double s = -(pqr[0]*r1[0] + pqr[1]*r1[1] + pqr[2]*r1[2]);
            double[] plane = new double[]{pqr[0], pqr[1], pqr[2], s};

            // Check whether a sample of points is within a threshold of the plane.
            for(int j = 0; j < numSamples; j++){
                double[] p = da.currentPoints.get(rand.nextInt(numPoints));
                if(Math.abs(p[0]) < t[2])
                    continue;
                if (pointToPlaneDist(new double[]{p[0], p[1], p[2]}, plane) < da.ransacThresh)
                    numFit ++;
            }

            // Compare new plane against last best, saving it if better
            if(numFit > bestFit && numFit > (numSamples/6)){
                bestFit = numFit;
                bestPlane = LinAlg.copy(plane);
            }
        }

        double percent = bestFit / (double) numSamples;

        if(bestFit == 0)
            return new double[4];
        return bestPlane;
    }
}