package kinect.ispy;

import kinect.classify.*;
import java.util.ArrayList;
import java.util.Queue;
import java.util.LinkedList;
import java.util.Collections;

public class SpyObject {
    public double[] min;
    public double[] max;
    public double[] pos;
    
    public int id;
	
    public ArrayList<String> labels;
    public Queue<ConfidenceLabel> shapeConLabels;
    public Queue<ConfidenceLabel> colorConLabels;
    double shapeConfidence;
    double colorConfidence;
    String bestColor;
    String bestShape;
    public SpyObject(int id)
    {
	labels = new ArrayList<String>();	
	shapeConLabels = new LinkedList<ConfidenceLabel>();
	shapeConLabels = new LinkedList<ConfidenceLabel>();
	this.colorConfidence = 0.0;
	this.shapeConfidence = 0.0;
	this.bestColor = "unknown";
	this.bestShape = "unknown";
	this.id = id;
    }
    
    public void updateLabels(ArrayList<String> newLabels){
	labels = newLabels;
	Collections.sort(labels);
    }
    public String getColor()
    {
	return bestColor;
    }
    public String getShape()
    {
	return bestShape;
    }
    
    public double updateColorConfidence(ConfidenceLabel cl)
    {
	colorConLabels.offer(cl);
	if (colorConLabels.size() > 5)
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
	if (shapeConLabels.size() > 5)
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
}
