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
import java.awt.geom.AffineTransform;
import java.awt.image.*;

/* To Do:
 *  - Add features for shape recognition to FeatureVec
 */

public class SceneRenderer extends VisLayer
{
	final static int K_WIDTH = kinect_status_t.WIDTH;
	final static int K_HEIGHT = kinect_status_t.HEIGHT;

	private final ISpy ispy;
	private VisCanvas canvas;


	public SceneRenderer(VisWorld world, ISpy ispy){
		super(world);
		this.ispy = ispy;
		canvas = new VisCanvas(this);

        // Set up camera
        this.cameraManager.uiLookAt(new double[]{ISpy.viewRegion.getCenterX(), kinect_status_t.HEIGHT-ISpy.viewRegion.getCenterY(), ISpy.viewRegion.width},  //Position
                                    new double[]{ISpy.viewRegion.getCenterX(), kinect_status_t.HEIGHT-ISpy.viewRegion.getCenterY(), 0}, // Lookat
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
            if(ISpy.viewRegion.contains(x, y)){
                buf[i] = kinectData.rgb[i+2];   // B
                buf[i+1] = kinectData.rgb[i+1]; // G
                buf[i+2] = kinectData.rgb[i];   // R
            }
        }
        return image;
    }

    public void drawObjects(VisWorld.Buffer buffer, Map<Integer, SpyObject> objects){
    	double width = ISpy.viewRegion.getWidth() / 2 + 80;
    	double height = ISpy.viewRegion.getHeight() / 2 + 60;

    	double theta = 0;
    	for(SpyObject obj : objects.values()){
    		VzImage img = new VzImage(obj.lastObject.getImage());
    		buffer.addBack(new VisChain(LinAlg.translate(obj.bbox.getMinX(), K_HEIGHT - obj.bbox.getMinY()), LinAlg.scale(1, -1, 1), img));




    		double x, y;
    		theta = Math.atan2(obj.bbox.getCenterY() - ISpy.viewRegion.getCenterY(), obj.bbox.getCenterX() - ISpy.viewRegion.getCenterX());
    		double vert = (Math.sin(theta) == 0) ? Double.MAX_VALUE : height / Math.abs(Math.sin(theta));
    		double horiz = (Math.cos(theta) == 0) ? Double.MAX_VALUE : width / Math.abs(Math.cos(theta));
    		if(vert < horiz){
    			x = ISpy.viewRegion.getCenterX() - 40 + Math.cos(theta) * vert;
    			y = ISpy.viewRegion.getCenterY() + 30 + Math.sin(theta) * vert;
    		} else {
    			x = ISpy.viewRegion.getCenterX() - 40 + Math.cos(theta) * horiz;
    			y = ISpy.viewRegion.getCenterY() + 30 + Math.sin(theta) * horiz;
    		}

    		String labelString = "";

    		String tf="<<monospaced,white,dropshadow=false>>";
    		labelString += String.format("%s%d\n", tf, obj.id);
    		labelString += String.format("%s%s:%.2f\n", tf, obj.getColor(), obj.getColorConfidence());
    		labelString += String.format("%s%s:%.2f\n", tf, obj.getShape(), obj.getShapeConfidence());
    		labelString += String.format("%s%s:%.2f\n", tf, obj.getSize(), obj.getSizeConfidence());
    		VzText text = new VzText(labelString);
            VisChain vch3 = new VisChain(LinAlg.translate(x, K_HEIGHT - y), LinAlg.scale(1.2), text);
            buffer.addBack(vch3);

    		VisVertexData line = new VisVertexData();
    		double[] objPos = new double[]{obj.bbox.getCenterX(),K_HEIGHT-obj.bbox.getCenterY()};
    		double minDist = Double.MAX_VALUE;
    		double bestX = 0, bestY = 0;
    		for(int i = 0; i <= 1; i ++){
    			for(int j = 0; j <= 1; j ++){
    				double cx = x + i * 100;
    				double cy = y - j * 70;
    				double dist = LinAlg.normF(new double[]{cx - objPos[0], cy - objPos[1]});
    				if(dist < minDist){
    					minDist = dist;
    					bestX = cx;
    					bestY = cy;
    				}
    			}
    		}
    		line.add(objPos);
    		line.add(new double[]{bestX,K_HEIGHT-bestY});
    		buffer.addBack(new VzLines(line, VzLines.LINES,new VzLines.Style(Color.WHITE, 3)));

    		VzRectangle rect = new VzRectangle(obj.bbox.getWidth(), obj.bbox.getHeight(), new VzLines.Style(Color.WHITE, 3));
    		buffer.addBack(new VisChain(LinAlg.translate(obj.bbox.getCenterX(), K_HEIGHT - obj.bbox.getCenterY()), rect));
    	}
    }

    public void drawScene(kinect_status_t kinectData, Map<Integer, SpyObject> objects){
        VisWorld.Buffer worldBuffer = this.world.getBuffer("displayImage");

    	BufferedImage background = getKinectImage(kinectData);
		double[] pt = new double[2];
		for(int y = (int)ISpy.viewRegion.getMinY(); y < ISpy.viewRegion.getMaxY(); y++){
			for(int x = (int)ISpy.viewRegion.getMinX(); x < ISpy.viewRegion.getMaxX(); x++){
	            int i = y*kinect_status_t.WIDTH + x;
	            int d = ((kinectData.depth[2*i+1]&0xff) << 8) |
	                    (kinectData.depth[2*i+0]&0xff);
	            double[] pKinect = KUtils.getXYZRGB(x, y, KUtils.depthLookup[d], kinectData);
	            double[] pixel = KUtils.getPixel(pKinect);
	    				Color c =  new Color((int)pKinect[3]);
	    				Color rc = new Color(c.getBlue(), c.getGreen(), c.getRed());
	    				background.setRGB((int)pixel[0], (int)pixel[1], rc.getRGB());
	        }
	    }
        worldBuffer.addBack(new VzImage(background, VzImage.FLIP));
        worldBuffer.setDrawOrder(-10);
        worldBuffer.swap();

        VisWorld.Buffer textBuffer = this.world.getBuffer("textInfo");
        drawObjects(textBuffer, objects);
        textBuffer.setDrawOrder(10);
        textBuffer.swap();
    }
}
