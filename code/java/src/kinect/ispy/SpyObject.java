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
    public ObjectInfo object;
    public ObjectInfo lastObject;

    public Queue<ConfidenceLabel> shapeConLabels;
    public Queue<ConfidenceLabel> colorConLabels;
    public Queue<ConfidenceLabel> sizeConLabels;

    public double shapeConfidence;
    public double colorConfidence;
    public double sizeConfidence;

    public String bestColor;
    public String bestShape;
    public String bestSize;

    //keep track of thresholds for best labels
    double shapeThreshold;
    double colorThreshold;
    double sizeThreshold;

    public Color boxColor = Color.white;

    public SpyObject(int id)
    {
        colorConLabels = new LinkedList<ConfidenceLabel>();
        shapeConLabels = new LinkedList<ConfidenceLabel>();
        sizeConLabels = new LinkedList<ConfidenceLabel>();

        this.colorConfidence = 0.0;
        this.shapeConfidence = 0.0;
        this.bestColor = "unknown";
        this.bestShape = "unknown";
        this.bestSize = "unknown";


        this.sizeThreshold = 0.5;
        this.shapeThreshold = 0.5;
        this.colorThreshold = 0.5;

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

    public double updateColorConfidence(ConfidenceLabel cl,
                                        ArrayList<ConfidenceLabel> confidenceThresholds)
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
        //get threshold for this new best label
        for (ConfidenceLabel thresh : confidenceThresholds)
        {
            if (bestColor.equals(thresh.getLabel()))
            {
                colorThreshold = thresh.getConfidence();
                break;
            }
        }

        colorConfidence = sum/(double)count * (double)max/(double)count;
        return colorConfidence;
    }

    public double updateSizeConfidence(ConfidenceLabel cl,
                                       ArrayList<ConfidenceLabel> confidenceThresholds)
    {
        sizeConLabels.offer(cl);
        if (sizeConLabels.size() > 10)
            sizeConLabels.remove();
        double sum = 0;
        int count = 0;
        ArrayList<String> bestS = new ArrayList<String>();
        ArrayList<Integer> bestCount = new ArrayList<Integer>();
        int max = 0;

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
        //get threshold for this new best label
        for (ConfidenceLabel thresh : confidenceThresholds)
        {
            if (bestSize.equals(thresh.getLabel()))
            {
                sizeThreshold = thresh.getConfidence();
                break;
            }
        }

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
        //get threshold for this new best label
        for (ConfidenceLabel thresh : confidenceThresholds)
        {
            if (bestShape.equals(thresh.getLabel()))
            {
                shapeThreshold = thresh.getConfidence();
                break;
            }
        }
        shapeConfidence = sum/(double)count * (double)max/(double)count;
        return shapeConfidence;
    }

    public double shapeConfidenceThresholdDif()
    {
    	System.out.println(bestShape + " comparing thresh:" +
                           this.shapeThreshold + " and conf:" +
                           this.shapeConfidence);
        return this.shapeConfidence - this.shapeThreshold;
    }


    public double colorConfidenceThresholdDif()
    {
        System.out.println(bestColor + " comparing thresh:" +
                       this.colorThreshold + " and conf:" +
                           this.colorConfidence);
        return this.colorConfidence - this.colorThreshold;
    }

    public double sizeConfidenceThresholdDif()
    {
        System.out.println(bestSize + " comparing thresh:" +
                           this.sizeThreshold + " and conf:" +
                           this.sizeConfidence);
        return this.sizeConfidence - this.sizeThreshold;
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

    public boolean matchesBestSize(ArrayList<String> labels)
    {
        for(String label : labels)
        {
            if(bestSize.equals(label)){
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
}
