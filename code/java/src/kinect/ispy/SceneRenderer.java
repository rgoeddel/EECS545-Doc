package kinect.ispy;

import april.vis.*;
import april.jmat.*;
import april.jmat.geom.GRay3D;
import lcm.lcm.*;
import kinect.lcmtypes.*;
import kinect.kinect.*;

import kinect.classify.*;

import java.io.*;
import javax.swing.*;

import java.awt.*;
import java.util.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.image.*;

/* To Do:
 *  - Add features for shape recognition to FeatureVec
 */

public class SceneRenderer extends VisLayer
{
	final static int K_WIDTH = kinect_status_t.WIDTH;
	final static int K_HEIGHT = kinect_status_t.HEIGHT;

    // Subset of the image used 
    final static int[] viewBorders = new int[]{75, 150, 575, 400};
    final static Rectangle viewRegion = new Rectangle(viewBorders[0], viewBorders[1], viewBorders[2] - viewBorders[0], viewBorders[3] - viewBorders[1]);
	
	private final ISpy ispy;
	private VisCanvas canvas;
	
	
	public SceneRenderer(VisWorld world, ISpy ispy){
		super(world);
		this.ispy = ispy;
		canvas = new VisCanvas(this);
    
    // Set up camera
    this.cameraManager.uiLookAt(new double[]{viewRegion.getCenterX(), kinect_status_t.HEIGHT-viewRegion.getCenterY(), viewRegion.width},  //Position
        								    new double[]{viewRegion.getCenterX(), kinect_status_t.HEIGHT-viewRegion.getCenterY(), 0}, // Lookat
        								    new double[]{0, 1, 0}, false); // Up 
  	this.addEventHandler(new DisplayClickEventHandler());
	}
	
	public VisCanvas getCanvas(){
		return canvas;
	}
	
	protected class DisplayClickEventHandler extends VisEventAdapter{
    	@Override
			public boolean mouseClicked(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
	    		double[] intersect = ray.intersectPlaneXY();
	    		double x = intersect[0];
	    		double y = K_HEIGHT - intersect[1];
	  			ispy.mouseClicked(x, y);
	    		return false;
        }
    }
    
    private BufferedImage getKinectImage(kinect_status_t kinectData){
    	if(kinectData == null){
    		return new BufferedImage(10, 10, BufferedImage.TYPE_3BYTE_BGR);
    	}
    	BufferedImage image = new BufferedImage(kinect_status_t.WIDTH, kinect_status_t.HEIGHT, BufferedImage.TYPE_3BYTE_BGR);
			byte[] buf = ((DataBufferByte)(image.getRaster().getDataBuffer())).getData();
			for (int i = 0; i < buf.length; i+=3) {
				int x = (i/3)%kinect_status_t.WIDTH;
				int y = (i/3)/kinect_status_t.WIDTH;
				if(viewRegion.contains(x, y)){
				  buf[i] = kinectData.rgb[i+2];   // B
					buf[i+1] = kinectData.rgb[i+1]; // G
					buf[i+2] = kinectData.rgb[i];   // R
				}
			}
			return image;
    }
       
    public void drawObjects(VisWorld.Buffer worldBuffer, Map<Integer, SpyObject> objects){
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

    		VzRectangle rect = new VzRectangle(obj.bbox.getWidth(), obj.bbox.getHeight(), new VzLines.Style(obj.getBoxColor(), 2));
    		VisChain vch2 = new VisChain(LinAlg.translate(x, y), rect);
    		
    		String labelString = "";
    		labelString += String.format("<<monospaced,black>>%s:%.2f\n", obj.getColor(), obj.getColorConfidence());
    		labelString += String.format("<<monospaced,black>>%s:%.2f\n", obj.getShape(), obj.getShapeConfidence());
    		VzText text = new VzText(labelString);
            VisChain vch3 = new VisChain(LinAlg.translate(obj.bbox.getMaxX(), K_HEIGHT - obj.bbox.getMaxY()),
            		LinAlg.scale(1), text);
    		
    		
    		
    		worldBuffer.addBack(vch2);
    		worldBuffer.addBack(vch3);
    	}
    }
    
    public void drawScene(kinect_status_t kinectData, Map<Integer, SpyObject> objects, DataAggregator da){
      VisWorld.Buffer worldBuffer = this.world.getBuffer("displayImage");
      
    	BufferedImage background = getKinectImage(kinectData);
    	for(int y= (int)viewRegion.getMinY(); y<viewRegion.getMaxY(); y++){
        for(int x=(int)viewRegion.getMinX(); x<viewRegion.getMaxX(); x++){
            int i = y*kinect_status_t.WIDTH + x;
            int d = ((kinectData.depth[2*i+1]&0xff) << 8) |
                    (kinectData.depth[2*i+0]&0xff);
            double[] pKinect = KUtils.getXYZRGB(x, y, da.depthLookUp[d], kinectData);
            double[] pixel = KUtils.getPixel(pKinect);
    				Color c =  new Color((int)pKinect[3]);
    				Color rc = new Color(c.getBlue(), c.getGreen(), c.getRed());
    				background.setRGB((int)pixel[0], (int)pixel[1], rc.getRGB());
        }
    }
      worldBuffer.addBack(new VzImage(background, VzImage.FLIP));
      
      drawObjects(worldBuffer, objects);

      worldBuffer.swap();
    	
    }
}
