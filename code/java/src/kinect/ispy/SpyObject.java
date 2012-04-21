package kinect.ispy;

import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.Collections;

import kinect.kinect.ObjectInfo;

public class SpyObject {
	public Rectangle bbox;
	public double[] pos;
	
	public int id;
	
	public ArrayList<String> labels;
	public ObjectInfo lastObject = null;
	
	public SpyObject(int id){
		labels = new ArrayList<String>();
		this.id = id;
	}
	
	public boolean matches(ArrayList<String> lbls){
		for(String label : lbls){
			int i = 0;
			for(i = 0; i < labels.size(); i++){
				if(labels.get(i).equals(label)){
					break;
				}
			}
			if(i == labels.size()){
				return false;
			}
		}
		return true;
	}
	
	public void updateLabels(ArrayList<String> newLabels){
		labels = newLabels;
		Collections.sort(labels);
	}
}
