package kinect.ispy;

import java.awt.Color;
import java.awt.Rectangle;
import kinect.classify.*;
import java.util.ArrayList;
import java.util.Queue;
import java.util.LinkedList;
import java.util.Collections;

import kinect.kinect.ObjectInfo;

public class SpyObject implements Comparable<SpyObject>{
    public double[] pos;
    public Rectangle bbox;
    
    public int id;
    public ObjectInfo lastObject;
	
    public Queue<ConfidenceLabel> shapeConLabels;
    public Queue<ConfidenceLabel> colorConLabels;
    public Queue<ConfidenceLabel> sizeConLabels;
    double shapeConfidence;
    double colorConfidence;
    double sizeConfidence;
    
    String bestColor;
    String bestShape;
    String bestSize;
    
    String secondBestColor;
    String secondBestShape;
    double shapeThreshold;
    public Color boxColor = Color.white;
    ArrayList<String> shapeList;
    public SpyObject(int id)
    {
	colorConLabels = new LinkedList<ConfidenceLabel>();
	shapeConLabels = new LinkedList<ConfidenceLabel>();
	sizeConLabels = new LinkedList<ConfidenceLabel>();
	shapeList = new ArrayList<String>();
	this.colorConfidence = 0.0;
	this.shapeConfidence = 0.0;
	this.bestColor = "unknown";
	this.bestShape = "unknown";
	this.bestSize = "unknown";
	this.secondBestColor = "";
	this.secondBestShape = "";
	this.shapeThreshold = 0.5;
	this.id = id;
    }
    
	public String getColor()
	    {
		return bestColor;
	    }
	public String getSize()
	    {
		return bestSize;
	    }
	
	public String getShape()
	    {
		return bestShape;
    }
	
	public double getColorConfidence(){
		return colorConfidence;
	}
	public double getSizeConfidence(){
		return sizeConfidence;
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
	int max2 = 0;
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
	    {
		max2 = max;
		max = cnt;
	    }
	    sum+= c.getConfidence();
	    count++;
	}
	//best label
	if ((index = bestCount.indexOf(max)) >= 0)
	{
	    bestColor = bestS.get(index);
	}
	//second best
	if ((max2 > 0) && ((index = bestCount.indexOf(max2)) >= 0))
	{
	    secondBestColor = bestS.get(index);
	}
	colorConfidence = sum/(double)count * (double)max/(double)count; 
	return colorConfidence;
    }

    public double updateSizeConfidence(ConfidenceLabel cl)
    {
	sizeConLabels.offer(cl);
	if (sizeConLabels.size() > 10)
	    sizeConLabels.remove();
	double sum = 0;
	int count = 0;
	ArrayList<String> bestS = new ArrayList<String>();
	ArrayList<Integer> bestCount = new ArrayList<Integer>();
	int max = 0;
	int max2 = 0;
	int index;
	for (ConfidenceLabel c : sizeConLabels)
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
	    {
		max2 = max;
		max = cnt;
	    }
	    sum+= c.getConfidence();
	    count++;
	}
	//best label
	if ((index = bestCount.indexOf(max)) >= 0)
	{
	    bestSize = bestS.get(index);
	}
	//second best
	/*
	if ((max2 > 0) && ((index = bestCount.indexOf(max2)) >= 0))
	{
	    secondBestColor = bestS.get(index);
	}
	*/
	sizeConfidence = sum/(double)count * (double)max/(double)count; 
	return sizeConfidence;
    }

    public double updateShapeConfidence(ConfidenceLabel cl, 
	ArrayList<ConfidenceLabel> confidenceThresholds)
    {
	shapeConLabels.offer(cl);
	if (shapeConLabels.size() > 15)
	    shapeConLabels.remove();
	double sum = 0;
	int count = 0;
	ArrayList<String> bestS = new ArrayList<String>();
	ArrayList<Integer> bestCount = new ArrayList<Integer>();
	int max = 0;
	int max2 = 0;
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
	    {
		max2 = max;
		max = cnt;
	    }
	    sum+= c.getConfidence();
	    count++;
	}
	//best label
	if ((index = bestCount.indexOf(max)) >= 0)
	{
	    bestShape = bestS.get(index);
	}
	//second best
	if ((max2 > 0) && ((index = bestCount.indexOf(max2)) >= 0))
	{
	    secondBestShape = bestS.get(index);
	}
	
	for (ConfidenceLabel thresh : confidenceThresholds)
	{
	    if (!shapeList.contains(thresh.getLabel()))
	    {
		//System.out.println("ADDED SHAPE : " + thresh.getLabel());
		shapeList.add(thresh.getLabel());
	    }
	    if (bestShape.equals(thresh.getLabel()))
	    {
		shapeThreshold = thresh.getConfidence();
		break;
	    }
	}
	shapeConfidence = sum/(double)count * (double)max/(double)count; 
	return shapeConfidence;
    }
    
    public boolean matchesOneAndSecondBest(ArrayList<String> labels, int numbest)
    {
	int cntbest = 0;
	for(String label : labels){
	    if (bestColor.equals(label)) {
		cntbest++;
		continue;
	    } else if (bestShape.equals(label)) {
		cntbest++;
		continue;
	    } else if (secondBestColor.equals(label)) {
		continue;
	    } else if (secondBestShape.equals(label)) {
		continue;
	    } 
	    return false;
	}
	if (cntbest >= numbest)
	    return true;
	return false;
    }
    
    //TODO terrible temporary hack
    public boolean isShape(String label)
    {
    	return (label.equals("blue") ||
    			label.equals("red") ||
    			label.equals("yellow") ||
    			label.equals("green") ||
    			label.equals("purple") ||
    			label.equals("orange"));
    }
    public boolean matchesOrUnconfident(ArrayList<String> labels){
	//double shapeThreshold = 0.5;
	
	for(String label : labels)
	{	    		
	    if (isShape(label) && ((bestColor.equals(label)) || colorConfidence < 0.4)){
		continue;
	    } else if (!isShape(label) && ((bestShape.equals(label)) || shapeConfidence < 0.7)) {
		continue;
	    } 
	    return false;
	}
	return true;
    }
    
    public boolean matchesBestColor(ArrayList<String> labels)
    {	
	for(String label : labels)
	{
	    if(bestColor.equals(label)){
		return true;
	    } 
	}
	return false;
    }
    public boolean matchesBestShape(ArrayList<String> labels)
    {	
	for(String label : labels)
	{
	    if(bestShape.equals(label)){
		return true;
	    } 
	}
	return false;
    }

    public boolean matches(ArrayList<String> labels){
	for(String label : labels){
	    if(bestColor.equals(label)){
		continue;
	    } else if(bestShape.equals(label)){
		continue;
	    } else if(bestSize.equals(label)){
		continue;
	    } 
	    return false;
	}
	return true;
    }
    
    @Override
    public int compareTo(SpyObject obj)
    {
	double diff = (this.colorConfidence + this.shapeConfidence - this.shapeThreshold) - 
	    (obj.colorConfidence + obj.shapeConfidence - obj.shapeThreshold);
	    
	if (diff < 0)
	    return (-1);
	else if (diff > 0)
	    return (1);
	else
	    return 0;
    }
}
