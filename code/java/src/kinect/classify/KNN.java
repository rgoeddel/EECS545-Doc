package kinect.classify;

import java.io.Console;
import java.lang.*;
import java.io.*;
import java.util.*;
import java.util.regex.Pattern;
import java.util.regex.Matcher;

/*
 * @author James Kirk
 */
class Point {
    
    int dim;
    List<Double> points;
    
    public Point(double [] points, int dim)
    {
	this.dim = dim;
	this.points = new ArrayList<Double>(dim);
	for (int i = 0; i < dim; i++)
	    this.points.add(points[i]);
    }
    public Point(List<Double> points, int dim)
    {
	this.dim = dim;
	this.points = new ArrayList<Double>(dim);
	for (int i = 0; i < dim; i++)
	    this.points.add(points.get(i));
    }
    public Point(Point p)
    {
	this(p.getPoints(), p.getDimensions());
    }
        
    public List<Double> getPoints()
    {
	return points;
    }
    public int getDimensions()
    {
	return dim;
    }
    public double distance(Point p)
    {
	List<Double> points2 = p.getPoints();
	double sum = 0;
	for (int i = 0; i < dim; i++)
	    sum+= Math.pow(points.get(i) - points2.get(i), 2);
	
	return Math.sqrt(sum);
    }

}

class TrainingSample
{
    Point p;
    String label;
    
    public TrainingSample(Point p, String label)
    {
	this.p = new Point(p);
	this.label = label;
    }
    public Point getPoint()
    {
	return p;
    }
    public String getLabel()
    {
	return label;
    }

    public double distance(TrainingSample ts)
    {
	return p.distance(ts.p);
    }
    public double distance(Point pt)
    {
	return p.distance(pt);
    }
}

class ConfidenceLabel implements Comparable<ConfidenceLabel>
{
    double confidence;
    String label;
    
    public ConfidenceLabel(double confidence, String label)
    {
	this.confidence = confidence;
	this.label = label;
    }
    
    public double getConfidence()
    {
	return confidence;
    }
    public String getLabel()
    {
    	return label;
    }
    
    @Override
    public int compareTo(ConfidenceLabel cl)
    {
	//ConfidenceLabel cl = (ConfidenceLabel) o;
	double diff = this.confidence - cl.confidence;
	if (diff < 0)
	    return (1);
	else if (diff > 0)
	    return (-1);
	else
	    return 0;
    }
    
}


public class KNN {
    
    List<TrainingSample> data;
String datafile;
    //thresholds and respective labes
    List<Double> thresholds;
    List<String> labels;
    int Km;
    int dim;
    public KNN(int Km, int dim, String datafile)
    {
	this.Km = Km;
	this.datafile = datafile;
	this.dim = dim;
	thresholds = new ArrayList<Double>();
	labels = new ArrayList<String>();
	data = new ArrayList<TrainingSample>();
    }
    
    public void add(Point p, String label)
    {
	data.add(new TrainingSample(p, label));
	if (!labels.contains(label))
	{
	    labels.add(label);
	    thresholds.add(0.05); //TODO magic number default threshold
	}
    }
    
    public void add(List<Double> p, String label)
    {
	data.add(new TrainingSample(new Point(p, dim), label));
	if (!labels.contains(label))
	{
	    labels.add(label);
	    thresholds.add(0.05); //TODO magic number default threshold
	}
    }
    public String getLabelFromString(String datainput)
    {
	String reg = "\\{.*\\}";
	Pattern p = Pattern.compile(reg);
	
	Matcher m = p.matcher(datainput);
	String label;
	if (m.find())
	{
	    label = m.group();
	    label = label.replace("{", "");
	    label = label.replace("}", "");
	    return label;
	}
	else
	{
	    System.out.println("ERROR: no label found on training data");
	    return "unknown";
	}
    }
    //public static Point getLabelFromString(String datainput)
    //{
    public static Point getPointFromString(String datainput, int dim)
    {
	String reg = "[0-9]{1,}+(\\.[0-9]{1,})";
	Pattern p = Pattern.compile(reg);
	
	Matcher m = p.matcher(datainput);
	int i = 0;
	
	//get first 12 features- RGB vvv HSV vvv
	List<Double> pt = new ArrayList<Double>();
	while (m.find() && i < dim)
	{
	    pt.add(Double.valueOf(m.group()).doubleValue());
	    i++;
	}
	if (i != dim)
	{
	    System.out.println("ERROR: datainput has only " + i + 
			       " features and K=" + dim);
	    return null;
	}
	
	return new Point(pt, dim);
    }
    public void add(String datainput)
    {
	String label = getLabelFromString(datainput);
	String reg = "[0-9]{1,}+(\\.[0-9]{1,})";
	Pattern p = Pattern.compile(reg);
	
	Matcher m = p.matcher(datainput);
	int i = 0;
	
	//get first 12 features- RGB vvv HSV vvv
	List<Double> pt = new ArrayList<Double>();
	while (m.find() && i < dim)
	{
	    pt.add(Double.valueOf(m.group()).doubleValue());
	    i++;
	}
	if (i != dim)
	{
	    System.out.println("ERROR: datainput has only " + i + 
			       " features and K=" + dim);
	    return;
	}
      
	data.add(new TrainingSample(new Point(pt, dim), label));
	if (!labels.contains(label))
	{
	    System.out.println("New label: " + label);
	    labels.add(label);
	    thresholds.add(0.05); //TODO magic number default threshold
	}
	
    }
    public void add(TrainingSample ts)
    {
	data.add(ts);
	String label = ts.getLabel();
	if (!labels.contains(label))
	{
	    labels.add(label);
	    thresholds.add(0.05); //TODO magic number default threshold
	}
    }
    public void adjustThreshold(String label, int direction)
    {
	int index;
	if ((direction != 1) && (direction != -1))
	{
	    System.out.println("ERROR: illegal adjustment");
	    return;
	}
	if ((index = labels.indexOf(label)) >= 0)
	{
	    double threshold = thresholds.get(index);
	    
	    //TODO magic numbers
	    if (direction > 0)
		threshold = threshold + Math.pow(0.9 - threshold, 2);
	    else
		threshold = threshold - Math.pow(threshold - 0.5, 2);
	    //System.out.println(threshold);
	    thresholds.set(index, threshold);
	}
	else
	{
	    System.out.println("ERROR: label not found)");
	}
    }

    public String getNearestNeighbor(Point p)
    {
	double min = 10000;
	String nearestLabel = "none";
	
	for (TrainingSample ts : data)
	{
	    double dist = ts.distance(p);
	    if (dist < min)
	    {
		min = dist;
		nearestLabel = ts.getLabel();
	    }
	}
	
	return nearestLabel;
    }
    
    public String getNewLabel(int k, Point p)
    {
	List<ConfidenceLabel> results = getMostConfidentLabels(k, p);
	int index = -1;
	ConfidenceLabel best = results.get(0);
	String nearestLabel = best.getLabel();
	double confidence = best.getConfidence();
	
	if ((index = labels.indexOf(nearestLabel)) >= 0)
	{
	    if (confidence < thresholds.get(index))
		nearestLabel = "Uncertain";
	}
	
	return nearestLabel;
    }
    public List<TrainingSample> getKNearestNeighbors(int k, Point p)
    {
	List<TrainingSample> nearest = new ArrayList<TrainingSample> ();
		
	for (TrainingSample ts : data)
	{
	    if (nearest.size() < k)
	    {
		nearest.add(ts);
		continue;
	    }
	    
	    double dist = ts.distance(p);
	    int replace = -1;
	    
	    // find the farthest point that is father than ts
	    // if there is such a point and if so replace
	    for (int i = 0; i < nearest.size(); i++)
	    {
		TrainingSample cur = nearest.get(i);
	    
		if (dist < cur.distance(p))
		{
		    replace = i;
		    dist = cur.distance(p);
		}
	    }
	    if (replace >= 0)
		nearest.set(replace, ts);
	}

	return nearest;
    }
    
    public List<ConfidenceLabel> getMostConfidentLabels(int k, Point p)
    {
	List<TrainingSample> nearest = getKNearestNeighbors(k, p);
	List<ConfidenceLabel> cl = new ArrayList<ConfidenceLabel> ();
	
	if (nearest.isEmpty())
	{
	    return null;
	}
	
	List<String> answers = new ArrayList<String> ();
	for (int i = 0; i < nearest.size(); i++)
	{
	    TrainingSample ts = nearest.get(i);
	    

	    String label = ts.getLabel();
	    if (answers.contains(label))
		continue;
	    
	    answers.add(label);
	    
	    int count = 1;
	    for (int j = 0; j < nearest.size(); j++)
	    {
		if (i == j)
		{
		    continue;
		}
		TrainingSample ts2 = nearest.get(j);
    
		if (label.equals(ts2.getLabel()))
		{
		    count++;
		}
	    }
	    
	    cl.add(new ConfidenceLabel(((double) count)/((double) nearest.size()), label));
	}
	Collections.sort(cl);
	return cl;	
    }
    
    public void loadData()
    {
	try
	{    	    
	    FileInputStream fstream = 
		new FileInputStream(this.datafile);
	    DataInputStream in = new DataInputStream(fstream);
	    BufferedReader br = new BufferedReader(new InputStreamReader(in));
	    String strLine;
	    //Read File Line By Line
	    int i = 0;
	    while ((strLine = br.readLine()) != null)
	    {
		add(strLine);
	    }
	    in.close();
	}
	catch (Exception e)
	{
	    System.err.println("Error: " + e.getMessage());
	} 
    }
    public String classify(String inputdata)
    {
	Point test = getPointFromString(inputdata, this.dim);
	List<ConfidenceLabel> cl = getMostConfidentLabels(this.Km, test);
	ConfidenceLabel c = cl.get(0);
	String ans = c.getLabel();
	//ans = ans + " " + c.getConfidence();
	//return getNewLabel(10, test);
	return ans;
    }
    /*
    public static void main(String []args)
    {
	KNN k = new KNN(15,"");
	k.loadData();
	String label = k.classifyShape("[0.145120 0.058048 0.406336 0.406336 0.087072 0.029024 0.174144 0.261216 0.145120 0.087072 0.058048 0.058048 0.087072 0.261216 ]");
	System.out.println(label);
    }
    */
    
}
