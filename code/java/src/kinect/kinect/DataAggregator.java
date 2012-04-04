package kinect.kinect;

import april.util.*;
import lcm.lcm.*;
import java.util.*;

public class DataAggregator{
    int WIDTH = 640;
    int HEIGHT = 480;

    float[] depthLookUp = new float[2048];          //holds depth conversions so we only have to calculate them once

    ArrayList<double[]> currentPoints;                //array of all depth points mapped to xyz grid
    ArrayList<double[]> coloredPoints;
    HashMap<Integer, ObjectInfo> objects;           //map of all objects found in current frame to their data
    HashMap<Integer, ObjectInfo> prevObjects;       //map of all objects found in previous frame
    HashMap<Integer, Integer> map;                  //map of object ID to color
    HashMap<Integer, Integer> prevMap;              // "                    "   for previous frame

    UnionFindSimple ufs;                            //Union Find class for keeping track of union find algo

    double colorThresh;
    double unionThresh;
    double ransacPercent;
    double ransacThresh;

    double[] t;

    public DataAggregator(boolean record){
        t = new double[] { -0.0254, -0.00013, -0.00218 }; //-2.8831221109805402e-02, -1.6603848659438847e-06, -6.5479395637054133e-04
        objects = new HashMap<Integer, ObjectInfo>();
        prevObjects = new HashMap<Integer, ObjectInfo>();
        map = new HashMap<Integer, Integer>();
        prevMap = new HashMap<Integer, Integer>();
    }

}
