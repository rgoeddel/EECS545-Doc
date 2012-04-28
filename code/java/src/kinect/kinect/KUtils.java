package kinect.kinect;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

import kinect.lcmtypes.*;
import april.jmat.*;

public class KUtils
{

    final static int WIDTH = 640;
    final static int HEIGHT = 480;

    public static double Frgbx = 5.2549146934887904e+02; // focal length
    public static double Frgby = 5.2564205413114996e+02; //
    public static double Crgbx = 3.1961505746870336e+02; // camera center in pixels
    public static double Crgby = 2.5073442780176708e+02; //
    public static double[] Krgb = {2.7376758102455484e-01,-1.0924409942510054e+00,
                                   1.1055834733919326e-04,-3.6043261204609631e-04,1.6539945052733636e+00};

    // parameters for IR depth camera
    public static double Firx = 5.7191759217862204e+02; // focal length
    public static double Firy = 5.8760489958891026e+02; //

    public static double Cirx = 3.37025048258259540e+02; // larger -> moves right
    public static double Ciry = 2.4675008449138741e+02; // larger -> moves down

//    public static double Cirx = 3.2525048258259540e+02; // camera center in pixels
//    public static double Ciry = 2.4275008449138741e+02; //
    public static double[] Kir = {-1.8177169518802491e-01,1.1695212447715055e+00,
                                  -4.4512701863162500e-03,4.9033335891042291e-03,-2.7471289416375160e+00};

    // Rigid Body Transformation between IR and RGB camera courtesy of Nicolas Burrus
    // adding negative sign infront of x translation term since IR camera in -x direction from rgb
    public static double[][] Transirtorgb = new double[][]
    {{9.9992722787002031e-01, 3.9846949567157140e-03, -1.1386885890292265e-02,2.5455174253462186e-02},
     {3.9640245034303703e-03, 9.9999045541141818e-01, 1.8372794563272670e-03,2.1449990961662204e-04},
     {1.1394098205334914e-02, -1.7920078589010067e-03,9.9993347940446564e-01,-3.1850123170360141e-04},
     {0,0,0,1}};

    public static float[] depthLookup;          //holds depth conversions so we only have to calculate them once


    public static double[][] kinectToWorldXForm;
    static{
    	try{
    		BufferedReader in = new BufferedReader(new FileReader("/home/rgoeddel/class/EECS545-Doc/code/java/kinect.calib"));
        	kinectToWorldXForm = new double[4][4];
    		for(int i = 0; i < 4; i++){
    			for(int j = 0; j < 4; j++){
    				kinectToWorldXForm[i][j] = Double.parseDouble(in.readLine());
    			}
    		}
    		in.close();
    	} catch (IOException e){
    		kinectToWorldXForm = new double[][]{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    	}
    }

    /** Converts a point in the kinect coordinate frame to world coordinates **/
    public static double[] getWorldCoordinates(double[] kinectCoordinates){
    	double[] pt = new double[]{kinectCoordinates[0], kinectCoordinates[1], kinectCoordinates[2], 1};
    	double[] wc = LinAlg.matrixAB(pt, kinectToWorldXForm);
    	return new double[]{wc[0], wc[1], wc[2]};
    }


    /** Create depth look-up table for later use. **/
    public static float[] createDepthMap()
    {
        depthLookup = new float[2048];
        for (int i = 0; i < depthLookup.length; i++){
            depthLookup[i] = rawDepthToMeters(i);
        }
        return depthLookup;
    }

    // This function come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
    // via PointCloud.java by Daniel Shiffman (https://github.com/shiffman).
    // Apparently the kinect describes depth using integer between 0 and 2047. They can
    // be transformed into depth in meters.
    private static float rawDepthToMeters(int rawDepth)
    {
        if (rawDepth < 2047){
            // From Stephane Magnenat (used in E568 project)
            double k1 = 1.1863;
            double k2 = 2842.5;
            float k3 = (float) 0.1236;
            float result =   (float) (Math.tan(rawDepth / k2 + k1));
            return k3 * result;
            //return (float)(1.0 / ((double)(rawDepth) * -0.0030711016 + 3.3309495161));
	    }
        return 0.0f;
    }

    public static double[] getPixel(double[] kinectPt){
    	double[] pixel = new double[]{0, 0};
    	pixel[0] = kinectPt[0] * Firx / kinectPt[2] + Cirx;
    	pixel[1] = kinectPt[1] * Firy / kinectPt[2] + Ciry;
    	return pixel;
    }


    public static double[] getRegisteredXYZRGB(int pixel_x, int pixel_y,
                                               kinect_status_t kstat)
    {

        int i = pixel_y*kstat.WIDTH+pixel_x;
        int d = ((kstat.depth[2*i + 1]&0xff) << 8) | (kstat.depth[2*i]&0xff);
        double depth = d / 1000.0;
        int c = 0xff000000 |
            ((kstat.rgb[3*i+0]&0xff) << 0) |
            ((kstat.rgb[3*i+1]&0xff) << 8) |
            ((kstat.rgb[3*i+2]&0xff) << 16);



        double[] xyzc = new double[4];
        xyzc[0] = (pixel_x - Cirx) * depth / Firx;
        xyzc[1] = (pixel_y - Ciry) * depth / Firy;
        xyzc[2] = depth;
        xyzc[3] = c;
        return xyzc;
    }

    public static double[] getXYZRGB(int x, int y, double depth, kinect_status_t ks)
    {
        // Code from E568 Project
        double[] xyz = new double[3];
        xyz[0] = (x - Cirx) * depth / Firx;
        xyz[1] = (y - Ciry) * depth / Firy;
        xyz[2] = depth;

        // rotation transformation to transform from IR frame to RGB frame
        double[] cxyz = LinAlg.transform(Transirtorgb, xyz);

        // project 3D point into rgb image frame
        int cx = (int) ((cxyz[0] * Frgbx / cxyz[2]) + Crgbx);
        int cy = (int) ((cxyz[1] * Frgby / cxyz[2]) + Crgby);

        if ((cx < 0) || (cx >= WIDTH) || (cy < 0) || (cy >= HEIGHT)) {
            return new double[]{xyz[0], xyz[1], xyz[2], 0xff000000};
        }

        int rgb = getPixelColor(ks, cx, cy);


        return new double[]{xyz[0], xyz[1], xyz[2], rgb};

    }

    public static int getPixelColor(kinect_status_t ks, int x, int y)
    {
        int i = y*ks.WIDTH + x;
        int rgb =  0xff000000 |
                   ((ks.rgb[3*i+2]&0xff) << 16) |
                   ((ks.rgb[3*i+1]&0xff) << 8) |
                   (ks.rgb[3*i+0]&0xff);
        return rgb;
    }
}
