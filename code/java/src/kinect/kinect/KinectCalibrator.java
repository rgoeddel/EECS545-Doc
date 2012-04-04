package kinect.kinect;

import april.vis.*;
import april.jmat.*;
import april.jmat.geom.GRay3D;
import april.util.*;

import lcm.logging.*;
import kinect.lcmtypes.*;


import java.io.*;
import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.image.*;


class KinectCalibrator // implements LCMSubscriber
{
	private enum Mode {ORIGIN, X, Y, TEST};
    final int FRAME_WIDTH = 800;
    final int FRAME_HEIGHT = 600;
    final int MIN_POINTS = 250;
    
    final int KINECT_WIDTH = kinect_status_t.WIDTH;
    final int KINECT_HEIGHT = kinect_status_t.HEIGHT;
    
    float[] depthLookUp = KUtils.createDepthMap();
    

    VisWorld visWorld;
    VisLayer imageLayer;
    Log log;
    
    String calibFilename;
    
    Mode curMode = Mode.ORIGIN;
    double[] originLocation = null;
    double[] xLocation = null;
    double[] yLocation = null;
    double[] testLocation = null;

    // The most recently accessed kinect status
    kinect_status_t ks = null;

    public KinectCalibrator(GetOpt opt)
    {
    	// Calibration variables
        calibFilename = "kinect.calib";

        try{
            log = new Log(opt.getString("log"), "r");
        }catch(Exception e){
            System.err.println("Error: "+e);
        }
        
        // Setup Frame
        JFrame frame = new JFrame("Calibrate Kinect");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLayout(new BorderLayout());
        
        // Initialize the image frame and canvas
        visWorld = new VisWorld();
        
        imageLayer = new VisLayer(visWorld);
        VisCanvas visCanvas = new VisCanvas(imageLayer);
        
        //Set up initial camera view
        imageLayer.cameraManager.uiLookAt(new double[] {KINECT_WIDTH/2, KINECT_HEIGHT/2, 400},// Camera position
                                  new double[] {KINECT_WIDTH/2, KINECT_HEIGHT/2, 0.0},// Point looking at
                                  new double[] {0.0, 1.0, 0.0},// Up
                                  false);
        
    	imageLayer.addEventHandler(new ClickEventHandler());

        getKinectData();
        redrawImage();

        frame.add(visCanvas, BorderLayout.CENTER);
    	

        // Set up labels to choose from and buttons for skipping/commiting
        ParameterGUI paramGUI = new ParameterGUI();
        paramGUI.addButtons("ido", "Choose Origin", "idx", "Choose X Direction", "idy", "Choose Y Direction", "test", "Test Point", "createCalib", "Create Calibration File");
        paramGUI.addListener(new ParameterListener() {
            public void parameterChanged(ParameterGUI pg, String name) {
                if (name.equals("createCalib")) {
                    createCalibFile(calibFilename);
                } else if(name.equals("ido")){
                	curMode = Mode.ORIGIN;
                } else if(name.equals("idx")){
                	curMode = Mode.X;
                } else if(name.equals("idy")){
                	curMode = Mode.Y;
                } else if(name.equals("test")){
                	curMode = Mode.TEST;
                }
            }
        });
        frame.add(paramGUI, BorderLayout.SOUTH);

       

        // Finalize JFrame
        frame.setSize(FRAME_WIDTH, FRAME_HEIGHT);
        frame.setVisible(true);
    }
    
    protected class ClickEventHandler extends VisEventAdapter{
    	public boolean mouseClicked(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
    		double[] intersect = ray.intersectPlaneXY();
    		double x = intersect[0];
    		double y = intersect[1];
    		double[] location = new double[3];
    		
    		if(x < 1 || x >= KINECT_WIDTH-1 || y < 1 || y >= KINECT_HEIGHT-1){
    			// Out of bounds
    			return false;
    		}

    		location[0] = intersect[0];
    		location[1] = intersect[1];
    		location[2] = 1; // In front of the image for the buffer
    		
    		if(curMode == Mode.ORIGIN){
    	    	System.out.println("---- ORIGIN ----");
    			originLocation = location;
    		} else if(curMode == Mode.X){
    	    	System.out.println("---- X Point ----");
    			xLocation = location;
    		} else if(curMode == Mode.Y){
    			System.out.println("---- Y Point ---");
    			yLocation = location;
    		} else {
    			testLocation = location;
    		}
    		
    		redrawImage();
    		
            return false;
        }
    }
    
    private String v2s(double[] v){
    	return String.format("(%f, %f, %f)", v[0], v[1], v[2]);
    }
    
    /** Writes the kinect calibration information into the file **/
    public void createCalibFile(String filename){
    	if(xLocation == null || originLocation == null || yLocation == null){
    		System.out.println("!!! kinect.calib not saved - needs the origin, x, and y specified !!!");
    		return;
    	}
    	// Find the ends of the vector to be treated as x
    	double[] originPoint = getKinectPoint((int)originLocation[0], (int)originLocation[1]);
    	double[] xPoint = getKinectPoint((int)xLocation[0], (int)xLocation[1]);
    	double[] yPoint = getKinectPoint((int)yLocation[0], (int)yLocation[1]);
    	
    	System.out.println("O: " + v2s(originPoint));
    	System.out.println("X: " + v2s(xPoint));
    	System.out.println("Y: " + v2s(yPoint));
    	
    
    	// Find the world Matrix in kinect coordinates
    	double[] worldX = LinAlg.normalize(LinAlg.subtract(xPoint, originPoint));
    	double[] worldY = LinAlg.normalize(LinAlg.subtract(yPoint,  originPoint));
    	double[] worldZ = LinAlg.crossProduct(worldY, worldX);
    	worldY = LinAlg.crossProduct(worldX, worldZ);
    	

    	System.out.println("O: " + v2s(worldX));
    	System.out.println("X: " + v2s(worldY));
    	System.out.println("Y: " + v2s(worldZ));
    	
    	// Translates kinect coordinates to the origin of the world coordinate system
    	double[][] k2wTranslate = new double[][]{
    			{1, 0, 0, 0},
    			{0, 1, 0, 0},
    			{0, 0, 1, 0},
    			{-originPoint[0], -originPoint[1], -originPoint[2], 1}
    	};
    	// Transform from world basis to kinect basis
    	double[][] w2kTransform = new double[][]{
    			{worldX[0], worldX[1], worldX[2], 0},
    			{worldY[0], worldY[1], worldY[2], 0},
    			{worldZ[0], worldZ[1], worldZ[2], 0},
    			{0, 0, 0, 1}
    	};
    	// Translates in world coordinates from the base to the origin of the arm
    	double[][] wTranslate = new double[][]{
    			{1, 0, 0, 0},
    			{0, 1, 0, 0},
    			{0, 0, 1, 0},
    			{0, 0, 0, 1}
    	};
    	
    	// Overall transform is k2wTranslate * inv(w2kTransform) * wTranslate
    	// (translate to the world origin, then transform to the world basis, then translate within the world)
    	double[][] k2wTransform = LinAlg.matrixAB(LinAlg.matrixAB(k2wTranslate, LinAlg.inverse(w2kTransform)), wTranslate);
    	KUtils.kinectToWorldXForm = k2wTransform;
    	
    	// Write the file out 
    	try{
    		BufferedWriter out = new BufferedWriter(new FileWriter(calibFilename));
    		System.out.println("Writing transformation matrix");
    		for(int i = 0; i < 4; i++){
    			for(int j = 0; j < 4; j++){
    				out.write(k2wTransform[i][j] + "\n");
    			}
    			System.out.println(String.format("|%5f, %5f, %5f, %5f|", k2wTransform[i][0], k2wTransform[i][1], 
    					k2wTransform[i][2], k2wTransform[i][3]));
    		}
    		out.close();
    	} catch (IOException e){
    		System.out.println("Couldn't write kinect.calib");
    	}
    	
    	// Tests, should come out to unit vectors
    	/*
    	double[] a = KUtils.getWorldCoordinates(LinAlg.add(worldX, originPoint));
    	System.out.println(String.format("(%f, %f, %f)", a[0], a[1], a[2]));
    	a = KUtils.getWorldCoordinates(LinAlg.add(worldY, originPoint));
    	System.out.println(String.format("(%f, %f, %f)", a[0], a[1], a[2]));
    	a = KUtils.getWorldCoordinates(LinAlg.add(worldZ, originPoint));
    	System.out.println(String.format("(%f, %f, %f)", a[0], a[1], a[2]));
    	*/
    }
    
    /** Takes a point in image coordinates and returns a point in the space of the kinect **/
    public double[] getKinectPoint(int x, int y){
		int index = (KINECT_HEIGHT - y)*KINECT_WIDTH + x;
		int depth = ((ks.depth[2*index+1]&0xff) << 8) | (ks.depth[2*index+0]&0xff);
		double[] xyzrgb = KUtils.getXYZRGB(x, y, depthLookUp[depth], ks);
    	
    	System.out.println("Depth: " + depthLookUp[depth]);
    	//System.out.println(String.format("(%f, %f, %f)", xyzrgb[0], xyzrgb[1], xyzrgb[2]));
    	return new double[]{xyzrgb[0], xyzrgb[1], xyzrgb[2]};
    }

    /** Load in the next frame from the log..**/
    public void getKinectData()
    {    	
        try{
            ks = new kinect_status_t(log.readNext().data);
        } catch(Exception e){
            System.err.println(e.getMessage());
        }
    }
    
    /** Draws the image on the screen with circles where the user has clicked **/
    public void redrawImage(){
    	BufferedImage image = new BufferedImage(KINECT_WIDTH, KINECT_HEIGHT, BufferedImage.TYPE_3BYTE_BGR);
        byte[] buf = ((DataBufferByte)(image.getRaster().getDataBuffer())).getData();
        for (int i = 0; i < buf.length; i+=3) {
            buf[i] = ks.rgb[i+2];	// B
            buf[i+1] = ks.rgb[i+1];	// G
            buf[i+2] = ks.rgb[i];	// R
        }

        // Render the kinect image
        VisWorld.Buffer visBuffer = visWorld.getBuffer("backgroundImage");
        visBuffer.addBack(new VzImage(image, VzImage.FLIP));
        
        // Draw a circle at the base and head locations
        if(originLocation != null){
            VzCircle circle = new VzCircle(4, new VzLines.Style(Color.red, 2));
            VisChain vch = new VisChain(LinAlg.translate(originLocation), circle);
            visBuffer.addBack(vch);
        }
        if(xLocation != null){
            VzCircle circle = new VzCircle(4, new VzLines.Style(Color.green, 2));
            VisChain vch = new VisChain(LinAlg.translate(xLocation), circle);
            visBuffer.addBack(vch);
        }
        if(yLocation != null){
            VzCircle circle = new VzCircle(4, new VzLines.Style(Color.yellow, 2));
            VisChain vch = new VisChain(LinAlg.translate(yLocation), circle);
            visBuffer.addBack(vch);
        }
        if(testLocation != null){
            VzCircle circle = new VzCircle(4, new VzLines.Style(Color.blue, 2));
            VisChain vch = new VisChain(LinAlg.translate(testLocation), circle);
            visBuffer.addBack(vch);

        	double[] testPt = getKinectPoint((int)testLocation[0], (int)testLocation[1]);
        	double[] a = KUtils.getWorldCoordinates(testPt);
        	System.out.println(String.format("(%f, %f, %f)", a[0], a[1], a[2]));
        }
        visBuffer.swap();
    }
   

    public static void main(String[] args)
    {
        // Define possible commandline arguments
        GetOpt opts = new GetOpt();
        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('l', "log", "stdout", "Log file to get the image from");

        if (!opts.parse(args)) {
            System.err.println("ERR: Opts error - " + opts.getReason());
            System.exit(1);
        }
        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(1);
        }

        //segment = new Segment(da, true);
        KinectCalibrator kc = new KinectCalibrator(opts);
    }
}
