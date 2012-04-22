package kinect.ispy;

import java.awt.Color;
import java.awt.Rectangle;
import kinect.classify.*;
import java.util.ArrayList;
import java.util.Queue;
import java.util.LinkedList;
import java.util.Collections;

import kinect.kinect.ObjectInfo;

public class SpyObject {
	public double[] pos;
	public Rectangle bbox;
    
    public int id;
    public ObjectInfo lastObject;
	
    public Queue<ConfidenceLabel> shapeConLabels;
    public Queue<ConfidenceLabel> colorConLabels;
    double shapeConfidence;
    double colorConfidence;
    String bestColor;
    String bestShape;
    public Color boxColor = Color.white;
    
    public SpyObject(int id)
    {
		colorConLabels = new LinkedList<ConfidenceLabel>();
		shapeConLabels = new LinkedList<ConfidenceLabel>();
		this.colorConfidence = 0.0;
		this.shapeConfidence = 0.0;
		this.bestColor = "unknown";
		this.bestShape = "unknown";
		this.id = id;
    }
    
	public String getColor()
	    {
		return bestColor;
	    }
	
	public String getShape()
	    {
		return bestShape;
    }
	
	public double getColorConfidence(){
		return colorConfidence;
	}

	public double getShapeConfidence(){
		return shapeConfidence;
	}
	
	public Color getBoxColor(){
		// Add 5 to each color channel up to the max of 255
		boxColor = new Color(boxColor.getRed() < 245 ? boxColor.getRed() + 10 : 255,
				boxColor.getGreen() < 245 ? boxColor.getGreen() + 10 : 255,
				boxColor.getBlue() < 245 ? boxColor.getBlue() + 10 : 255);
		return boxColor;				
	}
	
    public double updateColorConfidence(ConfidenceLabel cl)
    {
	colorConLabels.offer(cl);
	if (colorConLabels.size() > 10)
	    colorConLabels.remove();
	double sum = 0;
	int count = 0;
	ArrayList<String> bestS = new ArrayList<String>();
	ArrayList<Integer> bestCount = new ArrayList<Integer>();
	int max = 0;
	int index;
	for (ConfidenceLabel c : colorConLabels)
	{
	    String label = c.getLabel();
	    
	    int cnt = 0;
	    if ((index = bestS.indexOf(label)) >= 0)
	    {
		cnt = bestCount.get(index) + 1;
		bestCount.set(index, cnt);
	    }
	    else
	    {
		cnt = 1;
		bestS.add(label);
		bestCount.add(cnt);
	    }
	    if (cnt > max)
		max = cnt;
	    sum+= c.getConfidence();
	    count++;
	}
	//best label
	if ((index = bestCount.indexOf(max)) >= 0)
	{
	    bestColor = bestS.get(index);
	}	
	colorConfidence = sum/(double)count * (double)max/(double)count; 
	return colorConfidence;
    }

    public double updateShapeConfidence(ConfidenceLabel cl)
    {
	shapeConLabels.offer(cl);
	if (shapeConLabels.size() > 10)
	    shapeConLabels.remove();
	double sum = 0;
	int count = 0;
	ArrayList<String> bestS = new ArrayList<String>();
	ArrayList<Integer> bestCount = new ArrayList<Integer>();
	int max = 0;
	int index;
	for (ConfidenceLabel c : shapeConLabels)
	{
	    String label = c.getLabel();
	    
	    int cnt = 0;
	    if ((index = bestS.indexOf(label)) >= 0)
	    {
		cnt = bestCount.get(index) + 1;
		bestCount.set(index, cnt);
	    }
	    else
	    {
		cnt = 1;
		bestS.add(label);
		bestCount.add(cnt);
	    }
	    if (cnt > max)
		max = cnt;
	    sum+= c.getConfidence();
	    count++;
	}
	//best label
	if ((index = bestCount.indexOf(max)) >= 0)
	{
	    bestShape = bestS.get(index);
	}	
	shapeConfidence = sum/(double)count * (double)max/(double)count; 
	return shapeConfidence;
    }
    
    public boolean matches(ArrayList<String> labels){
		for(String label : labels){
			if(bestColor.equals(label)){
				continue;
			} else if(bestShape.equals(label)){
				continue;
			} 
			return false;
		}
		return true;
	}
}
