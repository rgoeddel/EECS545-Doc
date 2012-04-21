package kinect.ispy;

import april.vis.*;
import april.jmat.*;
import april.util.*;

import lcm.lcm.*;
import kinect.lcmtypes.*;
import kinect.kinect.*;

import kinect.classify.*;

import java.io.*;
import java.nio.*;
import javax.swing.*;

import java.awt.*;
import java.util.*;
import java.awt.image.*;

/* To Do:
 *  - Add features for shape recognition to FeatureVec
 */


public class ISpy extends JFrame implements LCMSubscriber
{
	final static int WIDTH = 800;
	final static int HEIGHT = 600;

    static int initialColorThresh = 13;
    static double initialUnionThresh = 0.5;
    static double  initialRansacThresh = .02;
    static double  initialRansacPercent = .1;

    // Subset of the image used 
    final static int[] viewBorders = new int[]{75, 150, 575, 400};
    final static Rectangle viewRegion = new Rectangle(viewBorders[0], viewBorders[1], viewBorders[2] - viewBorders[0], viewBorders[3] - viewBorders[1]);
	
	private VisWorld visWorld;
	private VisLayer displayLayer;
	private VisCanvas displayCanvas;
	
    static LCM lcm = LCM.getSingleton();
	
	private kinect_status_t kinectData = null;
	private DataAggregator da;
	private Segment segmenter;
	
	private Map<Integer, SpyObject> objects;
	private KNN colorKNN;
	private KNN shapeKNN;
	
	
	public ISpy(DataAggregator da, Segment segmenter){
		super("ISpy");
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLayout(new GridBagLayout());
        
        // Set up vis world
        visWorld = new VisWorld();
        displayLayer = new VisLayer(visWorld);
        displayCanvas = new VisCanvas(displayLayer);
        
        // Set up camera
        displayLayer.cameraManager.uiLookAt(new double[]{WIDTH/2, HEIGHT/2, WIDTH/2},  //Position
        								    new double[]{WIDTH/2, HEIGHT/2, 0}, // Lookat
        								    new double[]{0, 1, 0}, false); // Up 
        
        this.da = da;
        this.segmenter = segmenter;
        objects = new HashMap<Integer, SpyObject>();

        lcm.subscribe("KINECT_STATUS", this);
        
        colorKNN = new KNN(30, 6, "color_features.dat");
        shapeKNN = new KNN(10, 15, "shape_features.dat");
        colorKNN.loadData();
        shapeKNN.loadData();

        this.add(displayCanvas);
        this.setSize(WIDTH, HEIGHT);
        this.setVisible(true);
	}
	
    /** Upon recieving a message from the Kinect, translate each depth point into
     ** x,y,z space and find the pixel color for it.  Then draw it.
     **/
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
        	kinectData = new kinect_status_t(ins);
        }catch (IOException e){
            e.printStackTrace();
            return;
        }
        
        extractPointCloudData();
        BufferedImage image = getKinectImage();
        updateObjects();
        drawDisplayLayer(image);
    }
    
    private void extractPointCloudData(){
    	da.currentPoints = new ArrayList<double[]>();
        da.coloredPoints = new ArrayList<double[]>();

        for(int y= (int)viewRegion.getMinY(); y<viewRegion.getMaxY(); y++){
            for(int x=(int)viewRegion.getMinX(); x<viewRegion.getMaxX(); x++){
                int i = y*kinectData.WIDTH + x;
                int d = ((kinectData.depth[2*i+1]&0xff) << 8) |
                        (kinectData.depth[2*i+0]&0xff);
                double[] pKinect = KUtils.getXYZRGB(x, y, da.depthLookUp[d], kinectData);
                da.currentPoints.add(pKinect);
            }
        }
    }
    
    private BufferedImage getKinectImage(){
    	if(kinectData == null){
    		return new BufferedImage(10, 10, BufferedImage.TYPE_3BYTE_BGR);
    	}
    	BufferedImage image = new BufferedImage(kinectData.WIDTH, kinectData.HEIGHT, BufferedImage.TYPE_3BYTE_BGR);
		byte[] buf = ((DataBufferByte)(image.getRaster().getDataBuffer())).getData();
		for (int i = 0; i < buf.length; i+=3) {
		    buf[i] = kinectData.rgb[i+2];   // B
			buf[i+1] = kinectData.rgb[i+1]; // G
			buf[i+2] = kinectData.rgb[i];   // R
		}
		return image;
    }
    
    private void updateObjects(){
    	if(da.currentPoints.size() <= 0){ return; }
    	ArrayList<ObjectInfo> objs = new ArrayList<ObjectInfo>();
    	segmenter.unionFind();
    	
    	Set<Integer> objsToRemove = new HashSet<Integer>();
    	for(Integer id : objects.keySet()){
    		objsToRemove.add(id);
    	}
    	
    	for(ObjectInfo obj : da.objects.values()){
            double[] bb = FeatureVec.boundingBox(obj.points);
            
            double[] min = KUtils.getWorldCoordinates(new double[]{bb[0], bb[1], bb[2]});
            double[] max = KUtils.getWorldCoordinates(new double[]{bb[3], bb[4], bb[5]});    
            double[] pos = new double[]{0, 0, 0};
            for(int i = 0; i < 3; i++){
            	pos[i] = (min[i] + max[i])/2;
            }
            
            String colorFeatures = FeatureVec.featureString(obj.points);
            String shapeFeatures = FeatureVec.getShapeFeature(obj.getImage());
            String color = colorKNN.classify(colorFeatures);
            String shape = shapeKNN.classify(shapeFeatures);
            
            ArrayList<String> labels = new ArrayList<String>();
            labels.add(color);
            labels.add(shape);
           
            int id = obj.repID;
            SpyObject spyObject;
            if(objects.containsKey(id)){
            	spyObject = objects.get(id);
            	objsToRemove.remove(id);
            } else {
            	spyObject = new SpyObject(id);
            	objects.put(id, spyObject);
            }
        	spyObject.updateLabels(labels);
            spyObject.pos = pos;
            spyObject.min = min;
            spyObject.max = max;
    	}
    	
    	for(Integer id : objsToRemove){
    		objects.remove(id);
    	}
    }
       
    public void drawObjects(VisWorld.Buffer worldBuffer){
    	
    }
        
    public void drawDisplayLayer(BufferedImage kinectImage){
        VisWorld.Buffer worldBuffer = visWorld.getBuffer("displayImage");
        worldBuffer.addBack(new VzImage(kinectImage, VzImage.FLIP));
        
        drawObjects(worldBuffer);        
        
        worldBuffer.swap();
    }
    
    public static void main(String args[])
    {
        // Set up data aggregator and segmenter
        DataAggregator da = new DataAggregator(false);
        da.colorThresh = initialColorThresh;
        da.unionThresh = initialUnionThresh;
        da.ransacThresh = initialRansacThresh;
        da.ransacPercent = initialRansacPercent;
        da.depthLookUp = KUtils.createDepthMap();
        da.WIDTH = (int)ISpy.viewRegion.getWidth();
        da.HEIGHT = (int)ISpy.viewRegion.getHeight();
        boolean colorSegments = true;
        Segment segment = new Segment(da, colorSegments);

        ISpy ispy = new ISpy(da, segment);
    }
}
