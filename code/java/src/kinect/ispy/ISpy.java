package kinect.ispy;

import april.vis.*;
import april.jmat.*;
import april.jmat.geom.GRay3D;
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
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.geom.AffineTransform;
import java.awt.image.*;

/* To Do:
 *  - Add features for shape recognition to FeatureVec
 */


public class ISpy extends JFrame implements LCMSubscriber
{
	final static int WIDTH = 800;
	final static int HEIGHT = 600;
	final static int K_WIDTH = kinect_status_t.WIDTH;
	final static int K_HEIGHT = kinect_status_t.HEIGHT;

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
	
	private JLabel ispyLabel;
	private JTextField inputField;
	private JButton addColorButton;
	private JButton addShapeButton;
	
    static LCM lcm = LCM.getSingleton();
	
	private kinect_status_t kinectData = null;
	private DataAggregator da;
	private Segment segmenter;
	
	private Map<Integer, SpyObject> objects;
	private KNN colorKNN;
	private KNN shapeKNN;
	
	
	
	
	public ISpy(DataAggregator da, Segment segmenter){
		super("ISpy");
        this.setSize(WIDTH, HEIGHT);
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        this.setLayout(new GridBagLayout());
        GridBagConstraints gbc = new GridBagConstraints();
        
        addColorButton = new JButton("Add Color");
        gbc.fill = GridBagConstraints.BOTH;
        gbc.weightx = .5;
        gbc.weighty = .05;
        gbc.gridx = 0;
        gbc.gridy = 0;
        this.add(addColorButton, gbc);
        
        
        // Set up vis world
        visWorld = new VisWorld();
        displayLayer = new VisLayer(visWorld);
        displayCanvas = new VisCanvas(displayLayer);
        
        // Set up camera
        displayLayer.cameraManager.uiLookAt(new double[]{viewRegion.getCenterX(), kinect_status_t.HEIGHT-viewRegion.getCenterY(), viewRegion.width},  //Position
        								    new double[]{viewRegion.getCenterX(), kinect_status_t.HEIGHT-viewRegion.getCenterY(), 0}, // Lookat
        								    new double[]{0, 1, 0}, false); // Up 
    	displayLayer.addEventHandler(new DisplayClickEventHandler());

        gbc.fill = GridBagConstraints.BOTH;
        gbc.gridwidth = 2;
        gbc.weighty = .95;
        gbc.weightx = 1;
        gbc.gridx = 0;
        gbc.gridy = 1;
        this.add(displayCanvas, gbc);
        
        ispyLabel = new JLabel("I spy something");
        gbc.ipadx = 20;
        gbc.gridwidth = 1;
        gbc.weighty = .05;
        gbc.weightx = .5;
        gbc.gridx = 0;
        gbc.gridy = 2;
        gbc.insets = new Insets(20, 20, 10, 20);
        this.add(ispyLabel, gbc);
        
        inputField = new JTextField();
        gbc.weightx = .5;
        gbc.gridx = 1;
        gbc.insets = new Insets(10, 20, 10, 20);
        this.add(inputField, gbc);
        inputField.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent arg0) {
				findObject(inputField.getText());
				inputField.setText("");
			}
        });
        
        this.da = da;
        this.segmenter = segmenter;
        objects = new HashMap<Integer, SpyObject>();

        lcm.subscribe("KINECT_STATUS", this);
        
        colorKNN = new KNN(30, 6, "color_features.dat");
        shapeKNN = new KNN(10, 15, "shape_features.dat");
        colorKNN.loadData();
        shapeKNN.loadData();
        

        this.setVisible(true);
	}
	
	public void findObject(String desc){
		ArrayList<String> labels = new ArrayList<String>();
		String[] splitDesc = desc.toLowerCase().trim().split(" ");
		for(int i = 0; i < splitDesc.length; i++){
			if(splitDesc[i].trim().equals("") || splitDesc[i].trim().equals("and")){
				continue;
			}
			labels.add(splitDesc[i]);
		}
		for(SpyObject obj : objects.values()){
			if(obj.matches(labels)){
				System.out.println("FOUND IT!");
				return;
			}
		}
		System.out.println("NO MATCH");
		
	}
	
	protected class DisplayClickEventHandler extends VisEventAdapter{
    	public boolean mouseClicked(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
    		double[] intersect = ray.intersectPlaneXY();
    		double x = intersect[0];
    		double y = K_HEIGHT - intersect[1];
    		for(SpyObject obj : objects.values()){
    			if(obj.bbox.contains(x, y)){
    				System.out.println("CLICKED");
    				break;
    			}
    		}
    		return false;
        }
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
			int x = (i/3)%kinectData.WIDTH;
			int y = (i/3)/kinectData.WIDTH;
			if(viewRegion.contains(x, y)){
			    buf[i] = kinectData.rgb[i+2];   // B
				buf[i+1] = kinectData.rgb[i+1]; // G
				buf[i+2] = kinectData.rgb[i];   // R
			}
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
    		Rectangle projBBox = obj.getProjectedBBox();
            double[] pos = new double[]{projBBox.getCenterX(), projBBox.getCenterY()};
            
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
            spyObject.bbox = projBBox;
            spyObject.lastObject = obj;
    	}
    	
    	for(Integer id : objsToRemove){
    		objects.remove(id);
    	}
    }
       
    public void drawObjects(VisWorld.Buffer worldBuffer){
    	for(SpyObject obj : objects.values()){  
//    		// Get image a 
//    		BufferedImage img = obj.lastObject.getImage();
//    		AffineTransform tx = AffineTransform.getScaleInstance(1, -1);
//    		tx.translate(0, -img.getHeight(null));
//    		AffineTransformOp op = new AffineTransformOp(tx, AffineTransformOp.TYPE_NEAREST_NEIGHBOR);
//    		img = op.filter(img, null);
//    		
    		//VisChain vch = new VisChain(LinAlg.translate(obj.bbox.getMinX(), K_HEIGHT-obj.bbox.getMaxY()), new VzImage(img));
    		//worldBuffer.addBack(vch);

    		double x = obj.pos[0];
    		double y = K_HEIGHT-obj.pos[1];
    		
    		
    		
    		VzRectangle rect = new VzRectangle(obj.bbox.getWidth(), obj.bbox.getHeight(), new VzLines.Style(Color.WHITE, 2));
    		VisChain vch2 = new VisChain(LinAlg.translate(x, y), rect);
    		
    		String labelString = "";
    		for(String label : obj.labels){
    			labelString += label + "\n";
    		}
    		VzText text = new VzText(labelString);
            VisChain vch3 = new VisChain(LinAlg.translate(obj.bbox.getMaxX(), K_HEIGHT - obj.bbox.getMaxY()),
            		LinAlg.scale(1), text);
    		
    		
    		
    		worldBuffer.addBack(vch2);
    		worldBuffer.addBack(vch3);
    	}
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
