package kinect.kinect;

import april.util.*;
import lcm.lcm.*;
import java.util.*;

public class DataAggregator{
	public int WIDTH = 640;
	public int HEIGHT = 480;

	public float[] depthLookUp = new float[2048];          //holds depth conversions so we only have to calculate them once

	public ArrayList<double[]> currentPoints;                //array of all depth points mapped to xyz grid
	public ArrayList<double[]> coloredPoints;
	public HashMap<Integer, ObjectInfo> objects;           //map of all objects found in current frame to their data
	public HashMap<Integer, ObjectInfo> prevObjects;       //map of all objects found in previous frame
	public HashMap<Integer, Integer> map;                  //map of object ID to color
	public HashMap<Integer, Integer> prevMap;              // "                    "   for previous frame

	public UnionFindSimple ufs;                            //Union Find class for keeping track of union find algo

    public double colorThresh;
    public double unionThresh;
    public double ransacPercent;
    public double ransacThresh;

    public double[] t;

    public DataAggregator(boolean record){
        t = new double[] { -0.0254, -0.00013, -0.00218 }; //-2.8831221109805402e-02, -1.6603848659438847e-06, -6.5479395637054133e-04
        objects = new HashMap<Integer, ObjectInfo>();
        prevObjects = new HashMap<Integer, ObjectInfo>();
        map = new HashMap<Integer, Integer>();
        prevMap = new HashMap<Integer, Integer>();
    }

}
