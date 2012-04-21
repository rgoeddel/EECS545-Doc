package kinect.ispy;

import java.util.ArrayList;
import java.util.Collections;

public class SpyObject {
	public double[] min;
	public double[] max;
	public double[] pos;
	
	public int id;
	
	public ArrayList<String> labels;
	
	public SpyObject(int id){
		labels = new ArrayList<String>();
		this.id = id;
	}
	
	public void updateLabels(ArrayList<String> newLabels){
		labels = newLabels;
		Collections.sort(labels);
	}
}
