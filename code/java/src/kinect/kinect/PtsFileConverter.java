package kinect.kinect;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.*;
import java.util.*;

import kinect.classify.FeatureExtractor;

import april.util.*;

/** Take in a pts file and output a feature vector file */
public class PtsFileConverter
{
	enum FeatureType{
		COLOR, SHAPE
	}
    public PtsFileConverter(GetOpt opts)
    {
        FileInputStream fin;
        DataInputStream ins = null;
        FileWriter fout = null;
        try {
        	
        	String inFileString = opts.getString("infile");
        	String[] inFiles = inFileString.split(",");
            fout = new FileWriter(opts.getString("outfile"), true);
        	for(int i = 0; i < inFiles.length; i++){
                fin = new FileInputStream(inFiles[i]);
                ins = new DataInputStream(fin);

                FeatureType type;
                if(opts.getString("type").equals("color")){
                	type = FeatureType.COLOR;
                } else {
                	type = FeatureType.SHAPE;
                }
                
                if (ins != null && fout != null) {
                    convertFile(ins, fout, type);
                }
                ins.close();
                fin.close();
        	}

        } catch (Exception ex) {
            System.err.println("ERR: "+ex);
            ex.printStackTrace();
        }

    }

    private void convertFile(DataInputStream ins, FileWriter fout, FeatureType type)
    {
        BinaryStructureReader bsr = new BinaryStructureReader(ins);
        PrintWriter pwout;
        try {
            pwout = new PrintWriter(fout);
        } catch (Exception ioex) {
            ioex.printStackTrace();
            return;
        }

        try {
            while (true) {
                // Read in data
                int nlabels = bsr.readInt();
                bsr.blockBegin();
                ArrayList<String> labels = new ArrayList<String>();
                for (int i = 0; i < nlabels; i++) {
                    String label = bsr.readString();
                    labels.add(label);
                }
                bsr.blockEnd();

                int npoints = bsr.readInt();
                ArrayList<double[]> points = new ArrayList<double[]>();
                bsr.blockBegin();
                for (int i = 0; i < npoints; i++) {
                    points.add(bsr.readDoubles());
                }
                bsr.blockEnd();

                ArrayList<Double> features;
                if(type == FeatureType.COLOR){
                	String colorString = "red,orange,yellow,green,blue,purple";
                	String[] colors = colorString.split(",");
                	Set<String> colorSet = new HashSet<String>();
                	features = FeatureExtractor.getColorFeatures(points);
                    for(int i = 0; i < colors.length; i++){
                		colorSet.add(colors[i]);
                	}
                    // Write back out to FV file
                    pwout.printf("[");
                    for (int i = 0; i < features.size(); i++) {
                        pwout.printf("%f ", features.get(i));
                    }
                    pwout.printf("] {");
                    for (int i = 0; i <  labels.size(); i++) {
                    	if(colorSet.contains(labels.get(i))){
                            pwout.printf("%s", labels.get(i));
                    	}
                    }
                    pwout.printf("}\n");
                    pwout.flush();
                } else {
                	BufferedImage img = ObjectInfo.getImage(points, null);
                	features = FeatureExtractor.getShapeFeatures(img);
                	
                	String shapeString = "arch,rectangle,triangular,rectangular,triangle,square,cylinder,t-shaped,l-shaped,half-cylinder";
                	String[] shapes = shapeString.split(",");
                	Set<String> shapeSet = new HashSet<String>();
                	for(int i = 0; i < shapes.length; i++){
                		shapeSet.add(shapes[i]);
                	}
                    // Write back out to FV file
                    pwout.printf("[");
                    for (int i = 0; i < features.size(); i++) {
                        pwout.printf("%f ", features.get(i));
                    }
                    pwout.printf("] {");
                    for (int i = 0; i <  labels.size(); i++) {
                		String label = labels.get(i).toLowerCase();
                    	if(shapeSet.contains(label)){
                    		if(label.equals("rectangular")){
                    			label = "rectangle";
                    		} else if(label.equals("triangular")){
                    			label = "triangle";
                    		}
                            pwout.printf("%s", label);
                    	}
                    }
                    pwout.printf("}\n");
                    pwout.flush();
                	
                }
                

            }
        } catch (Exception ex) {
        }
    }

    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();

        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('i', "infile", null, "Input .pts file");
        opts.addString('o', "outfile", null, "Output feature vector file");
        opts.addString('t', "type", "shape", "Type of features to extract: {color, shape}");

        if (!opts.parse(args)) {
            System.err.println("ERR: "+opts.getReason());
            System.exit(1);
        }
        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(1);
        }

        PtsFileConverter pfc = new PtsFileConverter(opts);
    }
}
