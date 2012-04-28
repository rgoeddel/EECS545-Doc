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
    public static double Cirx = 3.3125048258259540e+02; // larger -> moves right
    public static double Ciry = 2.2875008449138741e+02; // larger -> moves down
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
    
    public static double[][] kinectToWorldXForm;
    static{
    	try{
    		BufferedReader in = new BufferedReader(new FileReader("/home/bolt/mlbolt/code/java/kinect.calib"));
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
    
/*
    static double fx_d = 1 / 5.9421434211923247e+02;
    static double fy_d = 1 / 5.9104053696870778e+02;
    static double cx_d = 3.3930780975300314e+02;
    static double cy_d = 2.4273913761751615e+02;
    static double[] t = new double[] {-1.5e-02, 2.5073334719943473e-03,-1.2922411623995907e-02 };//from http://www.ros.org/wiki/kinect_calibration/technical
*/

    /** Create depth look-up table for later use. **/
    public static float[] createDepthMap()
    {
        float[] depthLookUp = new float[2048];
        for (int i = 0; i < depthLookUp.length; i++){
            depthLookUp[i] = rawDepthToMeters(i);
        }
        return depthLookUp;
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

/*
        int argb = 0xff000000 | ((0x00ff0000 & (vt.pic[cy*WIDTH*5 + cx*5 + 2] << 16))
                      + (0x0000ff00 & (vt.pic[cy*WIDTH*5 + cx*5 + 1] << 8))
                      + (0x000000ff & vt.pic[cy*WIDTH*5 + cx*5 + 0]));
*/

        //int argb = frame.argb[cy * Constants.WIDTH + cx];

        return new double[]{xyz[0], xyz[1], xyz[2], rgb};

/*
        double result[] = new double[3];
        result[0] = (float)((x-cx_d) * depth * fx_d + t[0]);
        result[1] = (float)((y-cy_d) * depth * fy_d + t[1]);
        result[2] = (float)(depth + t[2]);
        return result;
*/
    }

    public static int getPixelColor(kinect_status_t ks, int x, int y)
    {
        int i = y*ks.WIDTH + x;
        int rgb =  0xff000000 |
                   ((ks.rgb[3*i+2]&0xff) << 16) |
                   ((ks.rgb[3*i+1]&0xff) << 8) |
                   (ks.rgb[3*i+0]&0xff);
        return rgb;

        /*
        return 0xff000000 | ((0x00ff0000 & (vt.pic[y*WIDTH*5 + x*5 + 2] << 16))
                             + (0x0000ff00 & (vt.pic[y*WIDTH*5 + x*5 + 1] << 8))
                             + (0x000000ff & vt.pic[y*WIDTH*5 + x*5 + 0]));*/
    }
}
